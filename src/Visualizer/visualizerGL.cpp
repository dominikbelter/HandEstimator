#include "../include/Visualizer/visualizerGL.h"
#include <memory>
#include <stdexcept>

using namespace handest;

/// A single instance of VisualizerGL
VisualizerGL::Ptr visualizer;

VisualizerGL::VisualizerGL(void) {
    // angle of rotation for the camera direction
    //angle=0;
    // actual vector representing the camera's direction
    lx=0.0f,ly = 0.0f, lz=-1.0f;
    // XYZ position of the camera
    //x=0.0f,y = 0.0f, z=0.0f;
    dist = 0.5f;
	eyex = 2;
	eyey = 0;
	eyez = -2;
	centerx = 0;
	centery = 0;
	centerz = 0.5f;
	keydown = false;
	transx = 0;
	transy = 0;
}

void VisualizerGL::addCloud(Point3D::Cloud& cloud, RGBA& colour) {
	std::cout << "Rozmiar: " << cloud.size() << std::endl;
	for(int i=0; i< cloud.size(); i++)
	{
		cloud[i].colour = colour;
		myPointCloud.push_back(cloud[i]);

	}
	std::cout << "Dodano punkty" << std::endl;
}


void VisualizerGL::show(void) const {
    std::cout << "fs1\n";
    char *argv[1] = {(char*) "soomething"};
    int tmp;
    glutInit(&tmp,argv);
    std::cout << "fs2\n";
	char key;
	std::cout << "OPENGL" << std::endl << "Sterowanie: " << std::endl
		<< " *myszka (lewy klawisz obrot, prawy przesuniecie)" << std::endl
		<< " *klawiatura (WSAD)" << std::endl
		<< "Uruchomic OpenGL? (Y or N)";
	std::cin >> key;
	if(key == 'Y' || key =='y')
	{
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(50,25);
	glutInitWindowSize(480,480);
	glutCreateWindow("OpenGL");
	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glutDisplayFunc(Draw);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseClick);
	//glutPassiveMotionFunc(MouseMove);
    glutMotionFunc(MouseMove);
    //glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	//glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutMainLoop();

	}

}

void VisualizerGL::Draw()
{
	visualizer->CreateHand();
}
void VisualizerGL::CreateHand()
{
	glClearColor(1.0,1.0,1.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glColor3f(0,0,0);
	//gluLookAt( eyex, eyey, eyez, transx, transy, 0, 0, 1, 0 );
	//camNormx = eyex - centerx;
	//camNormy = eyey - centery;
	//camNormz = eyez - centerz;
    gluLookAt(	centerx+lx, centery+ly, centerz+lz, centerx, centery,  centerz, 0.0f, 1.0f,  0.0f);
	visualizer->DrawGlobalAxis();
	//visualizer->DrawGrid();

    glPointSize(2.0);
	glBegin(GL_POINTS);
	for(int i=0; i< myPointCloud.size(); i++)
	{
		glColor3f(myPointCloud[i].colour.r,myPointCloud[i].colour.g,myPointCloud[i].colour.b);
		glVertex3f(myPointCloud[i].position.x,  myPointCloud[i].position.y, myPointCloud[i].position.z);
	}
	glEnd();
	glFlush();
	glutSwapBuffers();
	//std::cout << "Draw";
}
void VisualizerGL::clear(void) {
	myPointCloud.clear();
}
void VisualizerGL::Reshape(int width, int height)
{
	glViewport( 0, 0, width, height );
	glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    // rzutowanie perspektywiczne
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//std::cout << "Resize: " << width << ", " << height;
    Draw();
}
void VisualizerGL::Keyboard(unsigned char key, int x, int y)
{
	int max;
	double boost = 0.1;
	float fraction = 0.1;
	switch( key )
    {
        case 'a' :
			visualizer->angle -= 0.01;
			visualizer->lx = sin(visualizer->angle);
			visualizer->lz = -cos(visualizer->angle);

			std::cout << visualizer->centerx+visualizer->lx << ", ";
			std::cout << visualizer->centery+visualizer->ly << ", ";
			std::cout << visualizer->centerz+visualizer->lz << std::endl;
			break;
		case 'd' :
			visualizer->angle += 0.01;
			visualizer->lx = sin(visualizer->angle);
			visualizer->lz = -cos(visualizer->angle);
			std::cout << visualizer->centerx+visualizer->lx << ", ";
			std::cout << visualizer->centery+visualizer->ly << ", ";
			std::cout << visualizer->centerz+visualizer->lz << std::endl;
			break;
		case '+' :

			//sualizer->centerx = visualizer->centerx +0.1;
			//sualizer->centery = visualizer->centery +0.1;
			visualizer->centerz = visualizer->centerz +0.1;
			break;
		case '-' :
			//visualizer->centerx = visualizer->centerx -0.1;
			//visualizer->centery = visualizer->centery -0.1;
			visualizer->centerz = visualizer->centerz -0.1;
			break;
//    case 'w':
//    case 's':
//		max = visualizer->eyex;
//		if(visualizer->eyey > max)
//			max = visualizer->eyey;
//		if(visualizer->eyez > max)
//			max = visualizer->eyez;
//		break;
//	}
//
//	switch( key )
//    {
//    case 'w':
//			visualizer->eyex -= boost*visualizer->eyex/max;
//			visualizer->eyey -= boost*visualizer->eyey/max;
//			visualizer->eyez -= boost*visualizer->eyez/max;
//
//		break;
//    case 's':
//
//		visualizer->eyex += boost*visualizer->eyex/max;
//		visualizer->eyey += boost*visualizer->eyey/max;
//		visualizer->eyez += boost*visualizer->eyez/max;
//
//		break;
//    case 'a':
//		if(visualizer->eyey>0)
//			visualizer->eyex += 1*boost;
//		else
//			visualizer->eyex -= 1*boost;
//		if(visualizer->eyex < 0)
//			visualizer->eyey += 1*boost;
//		else
//			visualizer->eyey -= 1*boost;
//        break;
//    case 'd':
//		if(visualizer->eyey>0)
//			visualizer->eyex -= 1*boost;
//		else
//			visualizer->eyex += 1*boost;
//		if(visualizer->eyex < 0)
//			visualizer->eyey -= 1*boost;
//		else
//			visualizer->eyey += 1*boost;
//        break;
//	case '+':
//         visualizer->eyez += 1*boost;
//		 visualizer->eyey += 1*boost;
//		 visualizer->eyex += 1*boost;
//		 break;
//	case '-':
//         visualizer->eyez -= 1*boost;
//		 visualizer->eyey -= 1*boost;
//		 visualizer->eyex -= 1*boost;
//		 break;
//
	}

	Reshape( glutGet( GLUT_WINDOW_WIDTH ), glutGet( GLUT_WINDOW_HEIGHT ) );
}
void VisualizerGL::MouseClick(int button, int state,int x, int y)
{
	if( GLUT_DOWN == state )
	{
        visualizer->keydown = true;
		std::cout << "Down \n";
		if( GLUT_LEFT_BUTTON == button)
			visualizer->currentKey = eKey::Left;
		else if(GLUT_RIGHT_BUTTON == button)
			visualizer->currentKey = eKey::Right;
		MouseMove(x, y);
    }
	if( GLUT_UP == state)
	{
		visualizer->keydown = false;
		std::cout << "Up \n";
	}
	if (button == 3) // mouse wheel
	{
		Keyboard('+', x, y);
		std::cout << "WheelUp \n";
	}
	if ((button == 4)) // mouse wheel
	{
		Keyboard('-', x, y);
		std::cout << "WheelDown \n";
	}

}
void VisualizerGL::MouseMove(int x, int y)
{
	GLdouble prevx = visualizer->currentx;
	GLdouble prevy = visualizer->currenty;
	visualizer->currentx = x;
	visualizer->currenty = y;
	double boost = 0.1f;
	if(prevx == visualizer->currentx && prevy == visualizer->currenty)
		return;
	if(visualizer->keydown)
	{
		if(visualizer->currentx > prevx && visualizer->currentKey == eKey::Left)
			Keyboard('a', x, y);
		else if(visualizer->currentKey == eKey::Left)
			Keyboard('d', x, y);
		if(visualizer->currentx > prevx && visualizer->currentKey == eKey::Right)
			visualizer->transy -= boost*1;
		else if(visualizer->currentx < prevx && visualizer->currentKey == eKey::Right)
			visualizer->transy += boost*1;
		if(visualizer->currenty > prevy && visualizer->currentKey == eKey::Right)
			visualizer->transx -= boost*1;
		else if(visualizer->currenty < prevy && visualizer->currentKey == eKey::Right)
			visualizer->transx += boost*1;

		Reshape( glutGet( GLUT_WINDOW_WIDTH ), glutGet( GLUT_WINDOW_HEIGHT ) );
	}

}
void VisualizerGL::DrawGlobalAxis()
{
	glPushMatrix();
	glLineWidth(3);
	glBegin(GL_LINES);
	//oœ X (czerowny)
	glColor3f(255,0,0);
	glVertex3f(0,0,0);
	glVertex3f(2,0,0);
	//oœ Y (zielony)
	glColor3f(0,255,0);
	glVertex3f(0,0,0);
	glVertex3f(0,2,0);
	//oœ Z (niebieski)
	glColor3f(0,0,255);
	glVertex3f(0,0,0);
	glVertex3f(0,0,2);
	glEnd();
	glPopMatrix();
}
void VisualizerGL::DrawGrid()
{
	glPushMatrix();
	glBegin(GL_LINES);
	glColor3f(0.8,0.8,0.8);
	glLineWidth(2);
	for(int i=0; i<=400; i+=40)
	{
		for(int j=0; j<=400; j+=40)
		{
		glVertex3f(0+i,0+j, -3);
		glVertex3f(40+i,0+j, -3);

		glVertex3f(40+i,0+j, -3);
		glVertex3f(40+i,40+j, -3);

		glVertex3f(40+i,40+j, -3);
		glVertex3f(0+i,40+j, -3);

		glVertex3f(0+i,40+j, -3);
		glVertex3f(0+i,0+j, -3);
		}
	}
	glEnd();
	glPopMatrix();
}
handest::Visualizer* handest::createVisualizerGL(void) {
	visualizer.reset(new VisualizerGL());
	return visualizer.get();
}
