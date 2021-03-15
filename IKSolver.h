#pragma once

#include <assimp/scene.h>
#include "ModelHelper.h"

class IKSolver
{
public:
	enum class EndEffector
	{
		HEAD, LEFT_FORE_FOOT, RIGHT_FORE_FOOT, LEFT_REAR_FOOT, RIGHT_REAR_FOOT
	};

	IKSolver();
	
	void setBoneChain(EndEffector end);
	void setContext();
	bool traverseBones(const aiNode* cur);
	void solve();
	void ccdSolve(Bone& cur, Bone& end);
	void updateBonePos(int index);
	void applyRotation(Mesh& mesh);

	const aiScene* scene;
	ModelHelper* helper;
	std::string start, end;
	std::vector<Bone> bones;
	aiVector3D offset;
	aiVector3D target;
	float angle_limit{AI_MATH_PI_F / 6.f};
	float epsilon{1e-3f};
	int max_iter{20};
	bool showIkResult{false};

private:
	bool isApproximatelyEqual(const aiVector3D& a, const aiVector3D& b);
};

