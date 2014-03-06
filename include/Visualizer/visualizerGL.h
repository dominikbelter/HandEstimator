/** @file visualizerGL.h
 *
 * implementation - OpenGL Visualizer
 *
 */

#ifndef VISUALIZERGL_H_INCLUDED
#define VISUALIZERGL_H_INCLUDED

#include "visualizer.h"
#include <iostream>
#include <memory>
#include <cmath>
#include <GL/glut.h>


namespace handest {
	/// create a single visualizer (OpenGL)
	Visualizer* createVisualizerGL(void);
};

using namespace handest;

/// Visualizer implementation
class VisualizerGL : public Visualizer {
private:
	Point3D::Cloud myPointCloud;
	GLdouble eyex;
	GLdouble eyey;
	GLdouble eyez;
	GLdouble centerx;
	GLdouble centery;
	GLdouble centerz;
	GLdouble currentx;
	GLdouble currenty;
	GLdouble transx;
	GLdouble transy;
	// angle of rotation for the camera direction
    GLdouble angle;
    // actual vector representing the camera's direction
    GLdouble lx,ly,lz;
    GLdouble dist;
    // XZ position of the camera
    GLdouble x,y,z;
    GLdouble camNormx,camNormy,camNormz;
	bool keydown;
	void CreateHand();
	void DrawGlobalAxis();
	void DrawGrid();
	enum eKey
	{
		Right,
		Left
	};
	int currentKey;

public:
	/// Draw function for glut
	static void Draw();
	static void Reshape(int width, int height);
	static void Keyboard(unsigned char key, int x, int y);
	static void MouseClick(int button, int state,int x, int y);
	static void MouseMove(int x, int y);
	/// Pointer
	typedef std::unique_ptr<VisualizerGL> Ptr;

	/// Construction
	VisualizerGL(void);

	///Add Points
	virtual void addCloud(Point3D::Cloud& cloud, RGBA& colour) ;

	///Show Points
	virtual void show() const;

	///Clear Points
	virtual void clear();

};

#endif // VISUALIZERGL_H_INCLUDED
