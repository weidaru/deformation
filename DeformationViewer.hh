//=============================================================================
//
//  CLASS DeformationViewer
//
//=============================================================================


#ifndef DEFORMATION_VIEWER_HH
#define DEFORMATION_VIEWER_HH


//== INCLUDES =================================================================


#include "MeshViewer.hh"


//== CLASS DEFINITION =========================================================

class Deformer;

class DeformationViewer : public MeshViewer
{
public:
   
	/// default constructor
	DeformationViewer(const char* _title, int _width, int _height);

	// destructor
	~DeformationViewer();

	/// open mesh
	virtual bool open_mesh(const char* _filename);

private:

	virtual void keyboard(int key, int x, int y);
	virtual void draw(const std::string& _draw_mode);
	virtual void motion(int x, int y);
	virtual void mouse(int button, int state, int x, int y);

	
	enum Mode { MOVE, PICK, DRAG } mode_;
	void set_mode( Mode _mode );

	void glText(int x, int y, const std::string& _text);
	
	Mesh::Point& orig_point(Mesh::VertexHandle _vh) 
	{ return mesh_.property(orig_point_, _vh); }

	void deform_mesh();

private:

	OpenMesh::VPropHandleT<Mesh::Point>  orig_point_;

	std::vector<Vec3f>  orig_constraints_, moved_constraints_;
	int                 active_sphere_;

	bool new_constraints;

	Deformer *deformer;
};


//=============================================================================
#endif // CURVVIEWERWIDGET_HH defined
//=============================================================================

