/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <iostream>

#include "RayCastBug.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include <stdio.h> //printf debugging


#include "LinearMath/btAlignedObjectArray.h"

///RayCastBug shows how to use the btCollisionWorld::rayTest feature

#include "../CommonInterfaces/CommonRigidBodyBase.h"


class RayCastBug : public CommonRigidBodyBase
{
	
	
public:
	
	RayCastBug(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper)
	{
	}
	virtual ~RayCastBug()
	{
	}
	virtual void	initPhysics();
	
	virtual void	exitPhysics();
	
	void	castRays(); 
	
	virtual void stepSimulation(float deltaTime);
	
	virtual void resetCamera()
	{
		float dist = 4;
		float pitch = -45;
		float yaw = 35;
		float targetPos[3]={ 0.0, 0.0, 0.0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
	
};

void RayCastBug::castRays() 
{
	static int frame = 0;
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		
		m_dynamicsWorld->updateAabbs();
		m_dynamicsWorld->computeOverlappingPairs();
		
		btVector3 red(1,0,0);
		btVector3 blue(0,0,1);
		btVector3 magenta(1,0,1);

		///first hit
		{
			btScalar theta(0.04); // ray angle in the measured from the x axis
			btScalar range(25.0); // ray length
			btScalar x = range*btCos(theta);
			btScalar y = range*btSin(theta);

			btVector3 from(-1.0,0.0,0.0);
			btVector3 to(x-1,y,0.0);
			m_dynamicsWorld->getDebugDrawer()->drawLine(from,to,red);

			btCollisionWorld::ClosestRayResultCallback closestResults(from,to);

			// Computed the ray cast return twice and report
			if(frame == 11)
				closestResults.m_flags |= btTriangleRaycastCallback::kF_UseSubSimplexConvexCastRaytest;
			else
				closestResults.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
			
			m_dynamicsWorld->rayTest(from,to,closestResults);

			if (closestResults.hasHit())
			{
				btScalar point_radius = 0.05;

				btVector3 p = from.lerp(to,closestResults.m_closestHitFraction);
				m_dynamicsWorld->getDebugDrawer()->drawSphere(p,point_radius,magenta);
				m_dynamicsWorld->getDebugDrawer()->drawSphere(from,point_radius,magenta);
				m_dynamicsWorld->getDebugDrawer()->drawSphere(to,point_radius,magenta);
				m_dynamicsWorld->getDebugDrawer()->drawLine(p,p+closestResults.m_hitNormalWorld,magenta);

				// Only print on frames 11 and 12 (after all other messages were printed).
				if(frame == 11 || frame == 12) {
					std::cout << std::endl;
					std::cout << "***********************************************************************" << std::endl;
					if(frame == 11){
						std::cout << "** Computation performed with \"kF_UseSubSimplexConvexCastRaytest\":   **" << std::endl;
					}else {
						std::cout << "** Computation performed with \"kF_UseGjkConvexCastRaytest\":          **" << std::endl;
					}

					std::cout << "   Box hit at: ";
					std::cout << p.getX() << " " << p.getY() << " " << p.getZ() << " " << std::endl;
				}

			}
		}
	}
	++frame;
}


 void RayCastBug::stepSimulation(float deltaTime)
{
	castRays();
	CommonRigidBodyBase::stepSimulation(deltaTime);
}





void	RayCastBug::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	// Create a box. Inputs are half extents
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(0.5),btScalar(1.0),btScalar(0.5)));
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0.5,0.0,0.0));

	// We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setRollingFriction(1);
		body->setFriction(1);
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void	RayCastBug::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
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
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;
	
	delete m_solver;
	m_solver = 0;
	
	delete m_broadphase;
	m_broadphase = 0;
	
	delete m_dispatcher;
	m_dispatcher = 0;
	
	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;
	
}

class CommonExampleInterface*    RayCastBugCreateFunc(struct CommonExampleOptions& options)
{
	return new RayCastBug(options.m_guiHelper);
}


