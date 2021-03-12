#pragma once

#include <assimp/scene.h>
#include "ModelHelper.h"

class IKSolver
{
public:
	void ccdSolve(Bone& cur, Bone& end, aiVector3D& target, float limit_angle);

	
};

