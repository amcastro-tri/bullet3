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

#include "DemoEntries.h"

#include "../CcdPhysicsDemo/CcdPhysicsDemo.h"
#include "../BspDemo/BspDemo.h"
#include "../BasicDemo/BasicDemo.h"
#include "../ConcaveDemo/ConcaveDemo.h"
#include "../ConvexDecompositionDemo/ConvexDecompositionDemo.h"
#include "../RagdollDemo/RagdollDemo.h"
#include "../GimpactTestDemo/GimpactTestDemo.h"
#include "../Raytracer/Raytracer.h"
#include "../GjkConvexCastDemo/LinearConvexCastDemo.h"
#include "../VehicleDemo/VehicleDemo.h"
#include "../ConstraintDemo/ConstraintDemo.h"


btDemoEntry g_demoEntries[] =
{
		{"RagdollDemo",RagdollDemo::Create},
		{"ConvexDecomposition",ConvexDecompositionDemo::Create},
		{"CcdPhysicsDemo", CcdPhysicsDemo::Create},
		{"BasicDemo", BasicDemo::Create},
		{"BspDemo", BspDemo::Create},
		{"ConcaveDemo",ConcaveDemo::Create},
		{"Gimpact Test", GimpactConcaveDemo::Create},
		{"Raytracer Test",Raytracer::Create},
		{"GjkConvexCast",LinearConvexCastDemo::Create},
		{"VehicleDemo",VehicleDemo::Create},
		{"ConstraintDemo",ConstraintDemo::Create},
		{0, 0}


};
