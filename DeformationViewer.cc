//=============================================================================
//
//  CLASS DeformationViewer - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "DeformationViewer.hh"
#include <vector>
#include <float.h>
#include <limits.h>

#include "deformer.h"



//== IMPLEMENTATION ========================================================== 


DeformationViewer::
DeformationViewer(const char* _title, int _width, int _height)
: MeshViewer(_title, _width, _height), new_constraints(false), deformer(0)
{ 
  	mesh_.add_property(orig_point_);
	deformer = new Deformer(&mesh_, orig_point_);
	deformer->Config(0.5f, 0.5f);
  	init();
	set_mode(MOVE);
}


//-----------------------------------------------------------------------------


DeformationViewer::
~DeformationViewer()
{
	delete deformer;
}


//-----------------------------------------------------------------------------


static const float kRatioStep = 0.1f;

namespace {
	inline float clamp(float val, float low, float up) {
		val = val > up ? up : val;
		val = val < low ? low : val;
		return val;
	}
}

void
DeformationViewer::
keyboard(int key, int x, int y)
{
	switch (key)
	{
    	case ' ':
    	{
      		std::cout << "Deforming..." << std::flush;
			deform_mesh();
      		glutPostRedisplay();
      		std::cout << "done\n";
      		break;
    	}
			
		case 'm':
		{
			set_mode(MOVE);
			glutPostRedisplay();
			break;
		}
			
		case 'p':
		{
			set_mode(PICK);
			glutPostRedisplay();
			break;
		}
			
		case 'd':
		{
			set_mode(DRAG);
			glutPostRedisplay();
			break;
		}
			
		case 'r': // reset
		{
			set_mode(MOVE);

			Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
			for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
				mesh_.set_point(v_it, orig_point(v_it));

			mesh_.update_normals();

			moved_constraints_ = orig_constraints_;
			
			glutPostRedisplay();
			break;
		}
			
		default:
		{
			MeshViewer::keyboard(key, x, y);
			break;
		}
	}
}


//-----------------------------------------------------------------------------


void
DeformationViewer::
set_mode( Mode _mode )
{
	switch(mode_ = _mode)
	{
		case MOVE:
			glutSetCursor( GLUT_CURSOR_LEFT_ARROW );
			break;

		case PICK:
			glutSetCursor( GLUT_CURSOR_CROSSHAIR );
			break;

		case DRAG:
			glutSetCursor( GLUT_CURSOR_INFO );
			break;
	}	
}


//-----------------------------------------------------------------------------


void
DeformationViewer::
glText(int x, int y, const std::string& _text)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// set raster pos
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0.0, (GLfloat) viewport[2], 0.0, (GLfloat) viewport[3]);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2i(x, y);

	// draw characters
	std::string::const_iterator s_it(_text.begin()), s_end(_text.end());
	for (; s_it!=s_end; ++s_it)
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *s_it);
	
	// restore matrices
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}


//-----------------------------------------------------------------------------


bool
DeformationViewer::
open_mesh(const char* _filename)
{
	// load mesh
	if (MeshViewer::open_mesh(_filename))
	{
		// store original vertex positions
		Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
		for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
			orig_point(v_it) = mesh_.point(v_it);
		return true;
  }
  return false;
}


//-----------------------------------------------------------------------------

static const double kConstraintRadiusFalloff = 0.05;

void 
DeformationViewer::
draw(const std::string& _draw_mode)
{
	// draw mesh
  	MeshViewer::draw(_draw_mode);
	
	
	// draw spheres
	GLfloat mat_sphere[4] = {1.0f, 0.0f, 0.0f, 1.0f};
	GLfloat active_sphere[4] = {0.0f, 1.0f, 0.0f, 1.0f};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat_sphere);
	
	for (int i=0; i<moved_constraints_.size(); ++i)
	{
		glPushMatrix();
		glTranslatef( moved_constraints_[i][0],
					  moved_constraints_[i][1],
					  moved_constraints_[i][2] );
		if(i == active_sphere_)
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   active_sphere);
		else
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat_sphere);
		glutSolidSphere(kConstraintRadiusFalloff*radius_, 20, 20);
		glPopMatrix();
	}

	glBegin( GL_LINES );
	for (int i=0; i<moved_constraints_.size(); ++i)
	{
		GL::glVertex( orig_constraints_[i] );
		GL::glVertex( moved_constraints_[i] );
	}
	glEnd();
	
	GLfloat mat_mesh[4] = {0.4, 0.4, 0.4, 1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_mesh);

	
	// draw text
	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	switch (mode_)
	{
		case MOVE:
			glText(10, 10, "Move");
			break;

		case PICK:
			glText(10, 10, "Pick");
			break;

		case DRAG:
			glText(10, 10, "Drag");
			break;
	}
}


//-----------------------------------------------------------------------------

namespace {
	void mouse_pick(int x, int y, float *z, double pos[3]) {
		GLdouble  modelview[16], projection[16];
		GLint     viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		glGetIntegerv(GL_VIEWPORT, viewport);


		// read depth buffer value at (x, y_new)
		double y_new = viewport[3] - y; // in OpenGL y is zero at the 'bottom'
		glReadPixels(x, y_new, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, z );


		// reverse projection to get 3D point
		gluUnProject(x, y_new, *z, 
			modelview, 
			projection, 
			viewport, 
			&pos[0], &pos[1], &pos[2]);
	}
}


void 
DeformationViewer::
mouse(int button, int state, int x, int y)
{
	switch (mode_)
	{
		// move the mesh
		case MOVE:
		{
			MeshViewer::mouse(button, state, x, y);
			break;
		}
		
		// create a new handle point 
		case PICK:
		{
			if (state == GLUT_DOWN)
			{	
				float z;
				double pos[3];
				mouse_pick(x, y, &z, pos);
				
				if (z != 1.0f)
				{
					Mesh::Point p((float)pos[0],(float)pos[1],(float)pos[2]);
					orig_constraints_.push_back(p);
					moved_constraints_.push_back(p);
					new_constraints = true;
					std::cout << "Create new handle sphere at " << p << std::endl;
				}
				
				glutPostRedisplay();
			}

			break;
		}
			
		case DRAG:
		{
			if(state == GLUT_DOWN) {
				float z;
				double pos[3];
				mouse_pick(x, y, &z, pos);
				Vec3f pos_vec;
				pos_vec[0] = pos[0];
				pos_vec[1] = pos[1];
				pos_vec[2] = pos[2];
				//Do linear search in constraints, find the closest constraint.
				int closest_index = -1;
				float closest_distance = FLT_MAX;
				for(int i=0; i<moved_constraints_.size(); i++) {
					const Vec3f &cons = moved_constraints_[i];
					float distance = (cons - pos_vec).length();
					if(distance < closest_distance) {
						closest_index = i;
						closest_distance = distance;
					}
				}
				if(closest_distance < kConstraintRadiusFalloff * 1.5 * radius_) {
					active_sphere_ =  closest_index;
				}
				else {
					active_sphere_ = -1;
				}
			}
			break;
		}
	}
}


//-----------------------------------------------------------------------------


void 
DeformationViewer::
motion(int x, int y)
{
	switch (mode_)
	{
		case MOVE:
		{
			MeshViewer::motion(x, y);
			break;
		}

			
		case DRAG:
		{
			assert(orig_constraints_.size() == moved_constraints_.size());
			if(active_sphere_<0 || active_sphere_ > moved_constraints_.size()-1)
				break;

			GLdouble  modelview[16], projection[16];
			GLint     viewport[4];
			glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
			glGetDoublev(GL_PROJECTION_MATRIX, projection);
			glGetIntegerv(GL_VIEWPORT, viewport);
			double y_new = viewport[3] - y; // in OpenGL y is zero at the 'bottom'


			GLdouble ox, oy, oz, wx, wy, wz;

			gluProject( moved_constraints_[active_sphere_][0],
						moved_constraints_[active_sphere_][1],
						moved_constraints_[active_sphere_][2],
						modelview,
						projection,
						viewport,
						&wx, &wy, &wz );
			
			gluUnProject( x, y_new, wz,
						  modelview, 
						  projection, 
						  viewport, 
						  &ox, &oy, &oz );
			
			moved_constraints_[active_sphere_] = Mesh::Point(ox, oy, oz);

			// compute deformation on the fly...
			if(!new_constraints) 
				deform_mesh();

			glutPostRedisplay();
			break;
		}
	}
}


//-----------------------------------------------------------------------------


void 
DeformationViewer::deform_mesh()
{
	if(mesh_.n_vertices() == 0) {
		std::cout<<"Load a valid mesh before deformation.";
		return;
	}

	if(new_constraints) {
		deformer->Preprocess(orig_constraints_);
		new_constraints = false;
	}

	std::vector<Vec3f> constraints_displace;
	constraints_displace.resize(orig_constraints_.size());
	for(int i=0; i<orig_constraints_.size(); i++) {
		constraints_displace[i] = moved_constraints_[i]-orig_constraints_[i];
	}
	Deformer::DisplaceProp p_h = deformer->Exec(constraints_displace);
	for(Mesh::VertexIter v_it=mesh_.vertices_begin(); v_it!=mesh_.vertices_end(); v_it++) {
		mesh_.point(v_it) = mesh_.property(orig_point_, v_it) + mesh_.property(p_h, v_it);
	}
	mesh_.update_normals();
}


//=============================================================================
