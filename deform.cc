#include "DeformationViewer.hh"

int main(int argc, char **argv)
{
	glutInit(&argc, argv);

	DeformationViewer window("Deformation Viewer", 512, 512);

	if (argc>1)
		window.open_mesh(argv[1]);

	glutMainLoop();
}
