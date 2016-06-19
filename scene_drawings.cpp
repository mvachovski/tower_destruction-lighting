/*
 * scene_drawings.cpp
 *
 *  Created on: 11.10.2015 г.
 *      Author: martin
 */

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/glext.h>

#include "scene_drawings.h"

#include <cstdio>

namespace opengl_scene {


struct
{
	float radius;
	float phi;
	float theta;
}ViewerCoords;

GLfloat normal_up[] = {.0, 1., 0};
GLfloat normal_down[] = {.0, -1., 0};
GLfloat normal_back[] = {.0, 0., -1};
GLfloat normal_front[] = {.0, 0., 1};
GLfloat normal_left[] = {-1, 0., 0};
GLfloat normal_right[] = {1, 0., 0};

GLfloat mat_ground_specular[] = { 0., 0.63, 0.011, 1.0 };
GLfloat mat_ground_ambient[] = { 0., 0.63, 0.011, 1.0 };
GLfloat mat_ground_diffuse[] = { 0., 0.63, 0.011, 1.0 };
GLfloat mat_ground_emission[] = { .0, .0, .0, 1.0 };
GLfloat mat_ground_shininess[] = { 50.0 };

GLfloat mat_brick_specular[] = { 0.7490, 0.1764, 0., 1.0 };
GLfloat mat_brick_ambient[] = { 0.7490, 0.1764, 0., 1.0 };
GLfloat mat_brick_diffuse[] = { 0.7490, 0.1764, 0., 1.0 };
GLfloat mat_brick_emission[] = { .0, .0, .0, 1.0 };
GLfloat mat_brick_shininess[] = { 50.0 };

GLfloat mat_ball_diffuse[] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat mat_ball_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat mat_ball_specular[] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat mat_ball_emission[] = { 1.0, .051, .051, 1.0 };
GLfloat mat_ball_shininess[] = { 50.0 };

GLfloat light_position[] = { 0.0, 20.0, 15.0, 0.0 };
GLfloat light_diffuse[] = { 1.0, 1.0, 1., 0.0 };

void initGLScene()
{
	glutInitContextVersion(4, 3);
	glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition (100, 100);
	glutCreateWindow("Building collapse");
	glutDisplayFunc(drawScene);
	glutReshapeFunc(resize);
	glutKeyboardFunc(keyInput);
	glutSpecialFunc(specialKeyInput);
//	glutTimerFunc(500, timer_function, 1);
	glutIdleFunc(idle_function);

	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);

	glewExperimental = GL_TRUE;
	glewInit();

//	glDisable(GL_CULL_FACE_MODE);

//	glMaterialfv(GL_BACK, GL_SPECULAR, mat_specular);
//	glMaterialfv(GL_BACK, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);

	glShadeModel (GL_SMOOTH);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glClearColor(0., 0., 0.2941, 0.0);
//	glDepthFunc(GL_NEVER);
	//		setup();

	ViewerCoords.radius = 75;
	ViewerCoords.theta = M_PI / 3;
	ViewerCoords.phi = M_PI / 6;

}

void drawScene()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	double cos_theta = cos(ViewerCoords.theta);
	double sin_theta = sin(ViewerCoords.theta);
	double cos_phi = cos(ViewerCoords.phi);
	double sin_phi = sin(ViewerCoords.phi);

//	gluLookAt(10.0, vert, 30.0, 0. , 10.0, 0. , 0.0, 1.0, 0.0);
	gluLookAt(ViewerCoords.radius * sin_theta * sin_phi,
		ViewerCoords.radius * cos_theta + 10,
		ViewerCoords.radius * sin_theta * cos_phi,
		0. , 10.0, 0. , 0.0, 1.0, 0.0);

	for (int i=0; i < world.m_dynamicsWorld->getNumCollisionObjects(); i++)
	{
		btCollisionObject* obj = world.m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		btScalar m[16];
		if (body && body->getMotionState())
		{
			body->getMotionState()->getWorldTransform(trans);
		}
		else
		{
			trans = obj->getWorldTransform();
		}

//		printf("Object[%d]: X=%f Y=%f Z=%f\n", i,
//				trans.getOrigin().getX(),
//				trans.getOrigin().getY(),
//				trans.getOrigin().getZ());

		if(i == 0)		// base
		{
			glColor3f(0.1, 0.1, 0.1);

			glTranslatef(	trans.getOrigin().getX(),
							trans.getOrigin().getY(),
							trans.getOrigin().getZ());

			glMaterialfv(GL_FRONT, GL_SPECULAR, mat_ground_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, mat_ground_shininess);
			glMaterialfv(GL_FRONT, GL_EMISSION, mat_ground_emission);
			glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ground_ambient);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_ground_diffuse);

			glBegin(GL_TRIANGLES);
			glNormal3fv(normal_up);
			glVertex3f(0, 0, 0);
			glVertex3f(100, 0, -100);
			glVertex3f(-100, 0, -100);
			glVertex3f(0, 0, 0);
			glVertex3f(100, 0, 100);
			glVertex3f(-100, 0, 100);
			glVertex3f(0, 0, 0);
			glVertex3f(100, 0, -100);
			glVertex3f(100, 0, 100);
			glVertex3f(0, 0, 0);
			glVertex3f(-100, 0, -100);
			glVertex3f(-100, 0, 100);
			glEnd();

			glTranslatef(	-trans.getOrigin().getX(),
							-trans.getOrigin().getY(),
							-trans.getOrigin().getZ());
			continue;
		}

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		trans.getOpenGLMatrix(m);

		glMultMatrixf((GLfloat*)m);

		drawPrimitive(obj);

		glPopMatrix();
	}

//	drawCoordinateSystem();

	glutSwapBuffers();
}

void drawPrimitive(btCollisionObject *p)
{
	btRigidBody* body = btRigidBody::upcast(p);

	// the shape is a sphere
	btSphereShape *pSphere = dynamic_cast<btSphereShape *>(body->getCollisionShape());
	if(pSphere)
	{
//		glColor3f(1.0, 0.0, 0.0);

		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_ball_specular);
		glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ball_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_ball_diffuse);
		glMaterialfv(GL_FRONT, GL_EMISSION, mat_ball_emission);
		glMaterialfv(GL_FRONT, GL_SHININESS, mat_ball_shininess);

		glutSolidSphere(pSphere->getRadius(), 20, 10);
		return;
	}

	// the shape is a cylinder
	btCylinderShape *pCyl = dynamic_cast<btCylinderShape *>(body->getCollisionShape());
	static GLUquadric *quadric = NULL;
	if(pCyl)
	{
		if(!quadric)
		{
			quadric = gluNewQuadric();
		}

		glColor3f(.0, .0, 1.0);
		gluCylinder(quadric, 1, 1, 6, 10, 10);
		return;
	}

	btBoxShape *pBox= dynamic_cast<btBoxShape *>(body->getCollisionShape());
	if(pBox)
	{
		btVector3 extensions = pBox->getHalfExtentsWithoutMargin();
//		glutWireCube(extensions.getX() * 2);

//		printf("box extensions: %f %f %f\n", extensions.getX(), extensions.getY(), extensions.getZ());

		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_brick_specular);
		glMaterialfv(GL_FRONT, GL_AMBIENT, mat_brick_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_brick_diffuse);
		glMaterialfv(GL_FRONT, GL_SHININESS, mat_brick_shininess);
		glMaterialfv(GL_FRONT, GL_EMISSION, mat_brick_emission);

		glBegin(GL_QUADS);
		// upper and lower walls

//		// left and right walls
		for(int i = -1; i < 2; i+=2)
		{
			glColor3f(0, i > 0 ? 1 : 0.5, 0);
			glNormal3fv(i < 0 ? normal_left : normal_right);
//			glBegin(GL_LINE_LOOP);
			glVertex3f(i * extensions.getX(), -extensions.getY(), -extensions.getZ());
			glVertex3f(i * extensions.getX(), -extensions.getY(), extensions.getZ());
			glVertex3f(i * extensions.getX(), extensions.getY(), extensions.getZ());
			glVertex3f(i * extensions.getX(), extensions.getY(), -extensions.getZ());
//			glEnd();
		}

		for(int i = -1; i < 2; i+=2)
		{
//			glColor3f(i > 0 ? 1 : 0.5, 0, 0);
			glNormal3fv(i < 0 ? normal_down : normal_up);
//			glBegin(GL_LINE_LOOP);
			glVertex3f(-extensions.getX(), i * extensions.getY(), -extensions.getZ());
			glVertex3f(extensions.getX(), i * extensions.getY(), -extensions.getZ());
			glVertex3f(extensions.getX(), i * extensions.getY(), extensions.getZ());
			glVertex3f(-extensions.getX(), i * extensions.getY(), extensions.getZ());
//			glEnd();
		}
//
//		// back and forth walls
		for(int i = -1; i < 2; i+=2)
		{
//			glBegin(GL_LINE_LOOP);
			glColor3f(0, 0, i > 0 ? 1 : 0.5);
			glNormal3fv(i < 0 ? normal_back : normal_front);
			glVertex3f(-extensions.getX(), extensions.getY(), i * extensions.getZ());
			glVertex3f(extensions.getX(), extensions.getY(), i * extensions.getZ());
			glVertex3f(extensions.getX(), -extensions.getY(), i * extensions.getZ());
			glVertex3f(-extensions.getX(), -extensions.getY(), i * extensions.getZ());
//			glEnd();
		}
		glEnd();

		return;
	}

}

void resize (int w, int h)
{
	glViewport (0, 0, w, h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(120.0, (float)w/(float)h, 1.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
}

void keyInput(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'W':
	case 'w':
		if(ViewerCoords.theta > 0.1 * M_PI)
			ViewerCoords.theta -= 0.05 * M_PI;
		break;
	case 'S':
	case 's':
		if(ViewerCoords.theta < 0.9 * M_PI)
			ViewerCoords.theta += 0.05 * M_PI;
		break;
	case 'A':
	case 'a':
		ViewerCoords.phi -= .05 * M_PI;
		break;
	case 'D':
	case 'd':
		ViewerCoords.phi += .05 * M_PI;
		break;
	case '+':
		ViewerCoords.radius += 1;
		break;
	case '-':
		if(ViewerCoords.radius > 2)
			ViewerCoords.radius -= 1;
		break;
	case 'B':
	case 'b':
		world.launchBall();
		break;
	case 'P':
	case 'p':
		printf(world.togglePause() ? "PAUSE\n" : "RESUME\n");
		break;
	default:
		break;
	}
}

void specialKeyInput(int key, int x, int y)
{
//   if (key == GLUT_KEY_RIGHT)


//   if (key == GLUT_KEY_LEFT);

   glutPostRedisplay();
}

void timer_function(int value)
{
//	printf("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	glutTimerFunc(1000, timer_function, 1);
}

void idle_function()
{
//	static int count = 0;
//	printf("%s %s %d count=%d\n", __FILE__, __FUNCTION__, __LINE__, count++);

	world.integrate();
	drawScene();

}

void drawCoordinateSystem()
{
	glBegin(GL_LINES);

	glColor3f(0.0, 0.0, 0.0);
	for(int i = 1; i < 15; i++)
	{
		glVertex3f(-20, i, 0);
		glVertex3f(20, i, 0);
		glVertex3f(-20, -i, 0);
		glVertex3f(20, -i, 0);
	}

	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(-20, 0, 0);
	glVertex3f(20, 0, 0);

	glEnd();
}

}
