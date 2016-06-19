/*
 * BulletSim.h
 *
 *  Created on: 17 Oct 2015
 *      Author: martin
 */

#ifndef PHYSICALWORLD_H_
#define PHYSICALWORLD_H_

#include "btBulletDynamicsCommon.h"

class PhysicalWorld {
public:
	PhysicalWorld();
	void integrate();

	void launchBall();
	bool togglePause();

	virtual ~PhysicalWorld();

private:
	void populatePhysicalWorld();

	void buildTower(int nFloors);
	void buildWall(int nRows, int nBricksPerRow);

	void addConstrainedSystem();

	btRigidBody* createRigidBox(btVector3 &origin, btVector3 &halfDimensions, double angle, float mass = 10.0);

public:
	btDiscreteDynamicsWorld* m_dynamicsWorld;

private:
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher* m_dispatcher;
	btBroadphaseInterface* m_overlappingPairCache;
//	btSequentialImpulseConstraintSolver* m_solver;
	btConstraintSolver* m_solver;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btTransform m_groundTransform;
	bool m_isPaused;
};

#endif /* PHYSICALWORLD_H_ */
