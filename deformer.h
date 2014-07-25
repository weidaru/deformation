#ifndef DEFORMER_H_
#define DEFORMER_H_

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <vector>
#include <unordered_map>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>


class Deformer {
public:
	typedef Eigen::SparseMatrix<float, Eigen::ColMajor> Matrix;


private:
	typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
	typedef OpenMesh::Vec3f Vec3f;
	typedef Eigen::SparseLU<Matrix, Eigen::COLAMDOrdering<Matrix::Index>> Solver;

public:
	typedef OpenMesh::VPropHandleT<Mesh::Point> DisplaceProp;

public:
	Deformer(Mesh *_mesh, OpenMesh::VPropHandleT<Mesh::Point> _origin_handle);

	~Deformer();

	Deformer &Config(float _k_stretch, float _k_bend);

	float GetStretchCoefficient() { return k_stretch; }

	float GetBendCoefficient() { return k_bend; }

	void Preprocess(const std::vector<Vec3f> &origin_constraints);

	DisplaceProp Exec(const std::vector<Vec3f> &constraints_displace);

private:
	void CalculateWeight();
	void Map(const std::vector<Vec3f> &constraints);
	void PreprocessSolver();
	void SetupLaplace(const std::unordered_map<int, int> &map, 
								Mesh::VHandle cur, int i, Matrix *M, float scale = 1.0f);
	void AddLaplace(const std::unordered_map<int, int> &map, 
								Mesh::VHandle cur, int i, Matrix *M, float scale = 1.0f);

private:
	Mesh *mesh;
	OpenMesh::VPropHandleT<Mesh::Point> origin_h;
	OpenMesh::VPropHandleT<Mesh::Point> displace_h;
	OpenMesh::VPropHandleT<Mesh::Point> stretch_h;
	OpenMesh::VPropHandleT<Mesh::Point> bend_h;
	OpenMesh::VPropHandleT<Mesh::Scalar> vweight_h;
	OpenMesh::EPropHandleT<Mesh::Scalar> eweight_h;
	
	float k_stretch, k_bend;

	std::vector<Mesh::VHandle> hard_constraints;
	std::vector<Mesh::VHandle> soft_constraints;

	Solver *lap_solver;
	Solver *bilap_solver;
};


#endif			//DEFORMER_H_