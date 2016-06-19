/*
 * BulletSim.cpp, renamed to PhysicalWorld.cpp
 *
 *  Created on: 17 Oct 2015
 *      Author: martin
 */

#include "PhysicalWorld.h"

#include <cmath>

#define		SIZE_A		(5. * 1.3)
#define		SIZE_B		(1. * 1.3)

#define		PARALLELEPIPED_SIZE_X		2 * SIZE_B
#define		PARALLELEPIPED_SIZE_Y		2 * SIZE_B
#define		PARALLELEPIPED_SIZE_Z		2 * SIZE_A

#define		STATIC_PLANE_Y			-15



PhysicalWorld::PhysicalWorld()
{
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	m_overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solver = new btSequentialImpulseConstraintSolver;
//	m_solver = new

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	m_groundTransform.setIdentity();
	m_groundTransform.setOrigin(btVector3(0,-56,0));

	m_isPaused = false;

	populatePhysicalWorld();

}

void PhysicalWorld::populatePhysicalWorld()
{

	{
		btScalar mass(0.);
		btCollisionShape *pStaticPlaneShape = new btStaticPlaneShape(btVector3(0, 1., 0), 0);
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(btVector3(0, STATIC_PLANE_Y, 0));

		btVector3 localInertia(0,0,0);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,pStaticPlaneShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		m_dynamicsWorld->addRigidBody(body);
	}

//	{
//		btCollisionShape* colShape = new btCylinderShape(btVector3(1, 3, 1));
//		m_collisionShapes.push_back(colShape);
//
//		btTransform startTransform;
//		startTransform.setIdentity();
//
//		btScalar	mass(10.f);
//
//		bool isDynamic = (mass != 0.f);
//
//		btVector3 localInertia(0,0,0);
//		if (isDynamic)
//			colShape->calculateLocalInertia(mass,localInertia);
//
//		double theta = M_PI * .93;
//		btQuaternion q(cos(theta / 2), -sin(theta / 2), -sin(theta / 2), -sin(theta) / 2);
//
//		startTransform.setRotation(q);
//		startTransform.setOrigin(btVector3(0, 5, 0));
//
//		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//
//		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
//		btRigidBody* body = new btRigidBody(rbInfo);
//
//		m_dynamicsWorld->addRigidBody(body);
//	}
//

	buildWall(50, 15);
//	buildTower(50);
//	buildTower(30);

//	addConstrainedSystem();

}

void PhysicalWorld::integrate()
{
	if(!m_isPaused)
		m_dynamicsWorld->stepSimulation(1.f/60.f, 20);
}

PhysicalWorld::~PhysicalWorld()
{

	//remove the rigidbodies from the dynamics world and delete them
	for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	delete m_dynamicsWorld;
	delete m_solver;
	delete m_overlappingPairCache;
	delete m_dispatcher;
	delete m_collisionConfiguration;
	m_collisionShapes.clear();

}

void PhysicalWorld::buildTower(int nFloors)
{
	btVector3 halfDims(SIZE_B, SIZE_B, SIZE_A);
	for(int i = 0; i < nFloors; i++)
	{
		float x = (i % 2 == 0) ? -SIZE_A + SIZE_B: 0;
		float z = (i % 2) ? -SIZE_A + SIZE_B : 0;

		btVector3 originLeft(x, (1 + 2 * i) * SIZE_B + STATIC_PLANE_Y, z);
		btVector3 originRight(-x, (1 + 2 * i) * SIZE_B + STATIC_PLANE_Y, -z);

		m_dynamicsWorld->addRigidBody(createRigidBox(originLeft, halfDims, (i % 2) * M_PI_2));
		m_dynamicsWorld->addRigidBody(createRigidBox(originRight, halfDims, (i % 2) * M_PI_2));
	}
}

#define BRICK_HSIZE_X	2.5
#define BRICK_HSIZE_Y	2
#define BRICK_HSIZE_Z	3.5


void PhysicalWorld::buildWall(int nRows, int nBricksPerRow)
{
//	PARALLELEPIPED_SIZE_X
	btVector3 halfDims(BRICK_HSIZE_X, BRICK_HSIZE_Y, BRICK_HSIZE_Z);
	for(int row = 0; row < nRows; row++)
	{
		int nBricks = nBricksPerRow + (row + 1) % 2;
//		float start_x = -BRICK_HSIZE_X * nBricks;
		float start_z = -BRICK_HSIZE_Z * nBricks;

		for(int i = 0; i < nBricks; i++)
		{
//			float x = start_x + i * PARALLELEPIPED_SIZE_X;
			float x = .0;
			float z = start_z + i * 2 * BRICK_HSIZE_Z;
			btVector3 origin(x, (1 + 2 * row) * BRICK_HSIZE_Y + STATIC_PLANE_Y, z);
			m_dynamicsWorld->addRigidBody(createRigidBox(origin, halfDims, 0 * M_PI_2, 50));
		}



//		float x = (row % 2 == 0) ? -SIZE_A + SIZE_B: 0;
//		float z = (row % 2) ? -SIZE_A + SIZE_B : 0;
//
//		btVector3 originLeft(x, (1 + 2 * row) * SIZE_B + STATIC_PLANE_Y, z);
//		btVector3 originRight(-x, (1 + 2 * row) * SIZE_B + STATIC_PLANE_Y, -z);
//
//		m_dynamicsWorld->addRigidBody(createRigidBox(originLeft, halfDims, (row % 2) * M_PI_2));
//		m_dynamicsWorld->addRigidBody(createRigidBox(originRight, halfDims, (row % 2) * M_PI_2));
	}
}

void PhysicalWorld::launchBall()
{
	btCollisionShape* colShape = new btSphereShape(btScalar(2.));
	m_collisionShapes.push_back(colShape);

	btScalar	mass(3.f);

	btVector3 localInertia(0, 0, 0);
	colShape->calculateLocalInertia(mass,localInertia);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(40, 4, 4));

	btDefaultMotionState* motionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,colShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setLinearVelocity(btVector3(-80, 10, 0));
	body->setRestitution(1);

	m_dynamicsWorld->addRigidBody(body);
}

bool PhysicalWorld::togglePause()
{
	return m_isPaused = !m_isPaused;
}

void PhysicalWorld::addConstrainedSystem()
{
	btVector3 origin1(0, SIZE_A + STATIC_PLANE_Y + 10, 20);
	btVector3 origin2(-20, 3 * SIZE_A + STATIC_PLANE_Y, 0);
	btVector3 halfDims(SIZE_A, SIZE_A, SIZE_A);

	btRigidBody *pBody1 = createRigidBox(origin1, halfDims, 0);
//	btRigidBody *pBody2 = createRigidBox(origin2, halfDims, 0);

	btPoint2PointConstraint *pConstraint = new btPoint2PointConstraint(*pBody1,
		btVector3(0, 0, 0));

	m_dynamicsWorld->addConstraint(pConstraint, false);

	m_dynamicsWorld->addRigidBody(pBody1);
//	m_dynamicsWorld->addRigidBody(pBody2);

}

btRigidBody* PhysicalWorld::createRigidBox(btVector3& origin,
	btVector3& halfDimensions, double angle, float mass)
{
	btBoxShape *pBoxShape = new btBoxShape(halfDimensions);
//	m_collisionShapes.push_back(colShapeBox2);

	btTransform transform;
	transform.setIdentity();
	btVector3 rotationAxis(0, -1, 0);

	btVector3 localInertia(0,0,0);
	pBoxShape->calculateLocalInertia(mass,localInertia);

	transform.setOrigin(origin);
	transform.setRotation(btQuaternion(rotationAxis, angle));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState, pBoxShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

//	m_dynamicsWorld->addRigidBody(body);

	return body;
}
