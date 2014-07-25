#include "deformer.h"

#include <algorithm>
#include <limits.h>
#include <iostream>
#include <stdio.h>

Deformer::Deformer(Mesh *_mesh, OpenMesh::VPropHandleT<Mesh::Point> _origin_handle)
	: mesh(_mesh), origin_h(_origin_handle)  {
		k_stretch = 1.0f;
		k_bend = 0.0f;
		mesh->add_property(vweight_h);
		mesh->add_property(eweight_h);
		mesh->add_property(displace_h);
		mesh->add_property(stretch_h);
		mesh->add_property(bend_h);
		
		lap_solver = new Solver();
		bilap_solver = new Solver();
}

Deformer::~Deformer() {
	delete lap_solver;
	delete bilap_solver;
}

void Deformer::Preprocess(const std::vector<Vec3f> &origin_constraints) {
	CalculateWeight();
	Map(origin_constraints);
	PreprocessSolver();
}

void	Deformer::CalculateWeight()
{
	Mesh::VertexIter        v_it, v_end(mesh->vertices_end());
	Mesh::EdgeIter          e_it, e_end(mesh->edges_end());
	Mesh::VertexFaceIter    vf_it;
	Mesh::FaceVertexIter    fv_it;
	Mesh::HalfedgeHandle    h0, h1, h2;
	Mesh::VertexHandle      v0, v1;
	Mesh::Point             p0, p1, p2, d0, d1;
	Mesh::Scalar            w, area;

	for (e_it=mesh->edges_begin(); e_it!=e_end; ++e_it)
	{
		w  = 0.0;

		h0 = mesh->halfedge_handle(e_it.handle(), 0);
		v0 = mesh->to_vertex_handle(h0);
		p0 = mesh->property(origin_h, v0);

		h1 = mesh->halfedge_handle(e_it.handle(), 1);
		v1 = mesh->to_vertex_handle(h1);
		p1 = mesh->property(origin_h, v1);

		h2 = mesh->next_halfedge_handle(h0);
		p2 = mesh->property(origin_h, mesh->to_vertex_handle(h2));
		d0 = (p0 - p2).normalize();
		d1 = (p1 - p2).normalize();
		w += 1.0 / tan(acos(std::min(0.99f, std::max(-0.99f, (d0|d1)))));

		h2 = mesh->next_halfedge_handle(h1);
		p2 = mesh->property(origin_h, mesh->to_vertex_handle(h2));
		d0 = (p0 - p2).normalize();
		d1 = (p1 - p2).normalize();
		w += 1.0 / tan(acos(std::min(0.99f, std::max(-0.99f, (d0|d1)))));

		w = std::max(0.0f, w);
		mesh->property(eweight_h,e_it) = w;
	}


	for (v_it=mesh->vertices_begin(); v_it!=v_end; ++v_it)
	{
		area = 0.0;

		for (vf_it=mesh->vf_iter(v_it); vf_it; ++vf_it)
		{
			fv_it = mesh->fv_iter(vf_it);

			const Mesh::Point& P = mesh->property(origin_h, fv_it);  ++fv_it;
			const Mesh::Point& Q = mesh->property(origin_h, fv_it);  ++fv_it;
			const Mesh::Point& R = mesh->property(origin_h, fv_it);

			area += ((Q-P)%(R-P)).norm() * 0.5f * 0.3333f;
		}

		mesh->property(vweight_h,v_it) = 1.0 / (2.0 * area);
	}
}

Deformer & Deformer::Config(float _k_stretch, float _k_bend) {
	k_stretch = _k_stretch;
	k_bend = _k_bend;
	return *this;
}

void Deformer::Map(const std::vector<Vec3f> &constraints) {
	hard_constraints.clear();
	soft_constraints.clear();

	std::vector<Mesh::VHandle> *vertex_constraints = &hard_constraints;
	for(int i=0; i<constraints.size(); i++) {
		Vec3f cur = constraints[i];
		Mesh::VertexHandle closest_h;
		float closest_distance = FLT_MAX;
		for(Mesh::VIter v_it=mesh->vertices_begin(); v_it!=mesh->vertices_end(); v_it++) {
			float dis = (mesh->point(v_it) - cur).length();
			if(dis < closest_distance) {
				closest_distance = dis;
				closest_h = v_it;
			}
		}
		vertex_constraints->push_back(closest_h);
	}
	assert(vertex_constraints->size() == constraints.size());

	for(Mesh::VIter v_it=mesh->vertices_begin(); v_it!=mesh->vertices_end(); v_it++) {
		bool should_add = true;
		for(int i=0; i<hard_constraints.size(); i++) {
			if(hard_constraints[i] == v_it) {
				should_add = false;
				break;
			}
		}
		if(should_add)
			soft_constraints.push_back(v_it);
	}
	assert(mesh->n_vertices() == (soft_constraints.size() + hard_constraints.size()));
}

namespace {
	/*****TODO: Optimize writing to matrix if possible.*****/
	void m_set(Deformer::Matrix *M, int i, int j, float x, float y, float z) {
		M->coeffRef(i*3, j*3) = x;
		M->coeffRef(i*3+1, j*3+1) = y;
		M->coeffRef(i*3+2, j*3+2) = z;
	}

	void m_set(Deformer::Matrix *M, int i, int j, float v) {
		m_set(M, i, j, v, v, v);
	}

	void m_add(Deformer::Matrix *M, int i, int j, float x, float y, float z) {
		M->coeffRef(i*3, j*3) += x;
		M->coeffRef(i*3+1, j*3+1) += y;
		M->coeffRef(i*3+2, j*3+2) += z;
	}

	void m_add(Deformer::Matrix *M, int i, int j, float v) {
		m_add(M, i, j, v, v, v);
	}

	void dump_matrix(const Deformer::Matrix &M, const std::string &filename) {
		FILE * file = fopen(filename.c_str(), "w");
		assert(file);
		for(int i=0; i<M.rows(); i++) {
			for(int j=0; j<M.cols(); j++) {
				fprintf(file, "%4.2f\t", M.coeff(i, j));
			}
			fprintf(file, "\n");
		}
		fclose(file);
	}

	void dump_vec(const Eigen::VectorXf &V, const std::string &filename) {
		FILE * file = fopen(filename.c_str(), "w");
		assert(file);
		for(int i=0; i<V.size(); i++) {
			fprintf(file, "%4.2f\t", V[i]);
		}
		fclose(file);
	}
}

void Deformer::SetupLaplace(const std::unordered_map<int, int> &map, 
			Mesh::VHandle cur, int i, Deformer::Matrix *M, float s) {
	float sum = 0.0f;
	float v_w = mesh->property(vweight_h, cur);

	for(Mesh::VOHIter voh_it=mesh->voh_begin(cur); voh_it!=mesh->voh_end(cur); voh_it++) {
		Mesh::EHandle e_h = mesh->edge_handle(voh_it);
		float w = mesh->property(eweight_h, e_h);
		sum += w;
		int j = map.at(mesh->to_vertex_handle(voh_it).idx());
		m_set(M, i, j, v_w*w*s);
	}

	m_set(M, i, map.at(cur.idx()), -v_w*sum*s);
}

void Deformer::AddLaplace(const std::unordered_map<int, int> &map, 
			Mesh::VHandle cur, int i, Deformer::Matrix *M, float s) {
	float v_w = mesh->property(vweight_h, cur);
	float sum = 0.0f;

	for(Mesh::VOHIter voh_it=mesh->voh_begin(cur); voh_it!=mesh->voh_end(cur); voh_it++) {
		Mesh::EHandle e_h = mesh->edge_handle(voh_it);
		float w = mesh->property(eweight_h, e_h);
		sum += w;
		int j = map.at(mesh->to_vertex_handle(voh_it).idx());
		m_add(M, i, j, v_w*w*s);
	}

	m_add(M, i, map.at(cur.idx()), -v_w*sum*s);
}

void Deformer::PreprocessSolver() {
	//Build a Handle to index map.
	std::unordered_map<int, int> map;
	for(int i=0; i<hard_constraints.size(); i++) {
		map[hard_constraints[i].idx()] = i;
	}
	for(int i=0; i<soft_constraints.size(); i++) {
		map[soft_constraints[i].idx()] = i+hard_constraints.size();
	}

	int n = 3 * mesh->n_vertices();

	/************** Membrane Energy **************/
	Matrix M_mem;
	{
		Matrix &M = M_mem;
		M.resize(n,n);
		M.setZero();

		//hard constraints
		for(int _i=0; _i<hard_constraints.size(); _i++) {
			int i = _i;
			m_set(&M, i, i, 1.0f);
		}


		//soft constraints
		for(int _i=0; _i<soft_constraints.size(); _i++) {
			int i = hard_constraints.size() + _i;
			Mesh::VHandle cur = soft_constraints[_i];
			SetupLaplace(map, cur, i, &M);
		}
	
		M.makeCompressed();
		lap_solver->analyzePattern(M);
		lap_solver->factorize(M);
	}

	/************** Bend Energy **************/
	Matrix M_bend;
	{
		Matrix &M = M_bend;
		M.resize(n,n);
		M.setZero();
		//hard constraint
		for(int i=0; i<hard_constraints.size(); i++) {
			m_set(&M, i, i, 1.0f);
		}

		//soft constraints
		for(int _i=0; _i<soft_constraints.size(); _i++) {
			int i = hard_constraints.size() + _i;
			Mesh::VHandle cur = soft_constraints[_i];
			float v_wi = mesh->property(vweight_h, cur);
			float sum = 0.0f;
			for(Mesh::VOHIter voh_it=mesh->voh_begin(cur); voh_it!=mesh->voh_end(cur); voh_it++) {
				Mesh::VHandle ov_h = mesh->to_vertex_handle(voh_it);
				Mesh::EHandle e_h = mesh->edge_handle(voh_it);
				float e_wj = mesh->property(eweight_h, e_h);
				int j = map[mesh->to_vertex_handle(voh_it).idx()];
				sum += e_wj;
				AddLaplace(map, ov_h, i, &M, v_wi*e_wj);
			}
			AddLaplace(map, cur, i, &M, -v_wi*sum);
		}
		M.makeCompressed();
		bilap_solver->analyzePattern(M);
		bilap_solver->factorize(M);
	}
}

Deformer::DisplaceProp Deformer::Exec(const std::vector<Vec3f> &displace) {
	int n = 3 * mesh->n_vertices();

	/**************Solve for membrane energy**************/
	{
		Solver &solver = *lap_solver;

		Eigen::VectorXf X(n), B(n);
		B.setZero();
		for(int i=0; i<hard_constraints.size(); i++) {
			B[i*3] = displace[i][0];
			B[i*3+1] = displace[i][1];
			B[i*3+2] = displace[i][2];
		}

		X  = solver.solve(B);

		for(int i=0; i<hard_constraints.size(); i++) {
			Mesh::VHandle cur_h = hard_constraints[i];
			Mesh::Point &p = mesh->property(stretch_h, cur_h);
			p[0] = X[i*3];
			p[1] = X[i*3+1];
			p[2] = X[i*3+2];
		}

		for(int _i=0; _i<soft_constraints.size(); _i++) {
			int i = _i+hard_constraints.size();
			Mesh::VHandle cur_h = soft_constraints[_i];
			Mesh::Point &p = mesh->property(stretch_h, cur_h);
			p[0] = X[i*3];
			p[1] = X[i*3+1];
			p[2] = X[i*3+2];
		}
	}
	/**************Solve for thin plate energy**************/
	{
		Solver &solver = *bilap_solver;
		
		Eigen::VectorXf X(n), B(n);
		B.setZero();
		//hard constraint
		for(int i=0; i<hard_constraints.size(); i++) {
			B[i*3] = displace[i][0];
			B[i*3+1] = displace[i][1];
			B[i*3+2] = displace[i][2];
		}

		X  = solver.solve(B);

		for(int i=0; i<hard_constraints.size(); i++) {
			Mesh::VHandle cur_h = hard_constraints[i];
			Mesh::Point &p = mesh->property(bend_h, cur_h);
			p[0] = X[i*3];
			p[1] = X[i*3+1];
			p[2] = X[i*3+2];
		}

		for(int _i=0; _i<soft_constraints.size(); _i++) {
			int i = _i+hard_constraints.size();
			Mesh::VHandle cur_h = soft_constraints[_i];
			Mesh::Point &p = mesh->property(bend_h, cur_h);
			p[0] = X[i*3];
			p[1] = X[i*3+1];
			p[2] = X[i*3+2];
		}
	}

	//weight the stretch term and bend term
	for(Mesh::VertexIter v_it=mesh->vertices_begin(); v_it!=mesh->vertices_end(); v_it++) {
		Mesh::Point &final = mesh->property(displace_h, v_it);
		const Mesh::Point &stretch = mesh->property(stretch_h, v_it);
		const Mesh::Point &bend = mesh->property(bend_h, v_it);
		
		final = k_stretch * stretch + k_bend * bend; 
	}

	return displace_h;		
}
