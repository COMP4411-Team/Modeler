#include "IKSolver.h"


IKSolver::IKSolver()
{
	setBoneChain(EndEffector::HEAD);
}

void IKSolver::setContext()
{
	bones.clear();
	target = mesh->getBone(end).end + offset;
	traverseBones(scene->mRootNode);
}

bool IKSolver::traverseBones(const aiNode* cur)
{
	string name(Mesh::processBoneName(cur->mName.data));
	if (name == end)
	{
		bones.push_back(mesh->getBone(name));
		return true;
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
	{
		if (traverseBones(cur->mChildren[i]))
		{
			bones.push_back(mesh->getBone(name));
			if (name != start) return true;
			else return false;
		}
	}
	return false;
}

void IKSolver::solve()
{
	if (bones.empty()) return;
	for (auto& bone : bones)
		bone.rotation = aiQuaternion();		// reuse the rotation
	
	Bone& end = bones[0];
	for (int iter = 0; iter < max_iter; ++iter)
		for (int i = 0; i < bones.size(); ++i)
		{
			ccdSolve(bones[i], end);
			updateBonePos(i);
			if (isApproximatelyEqual(end.end, target))
				return;
		}
}

// Use CCD to solve IK
// Reference: https://blog.csdn.net/gamesdev/article/details/14110875
void IKSolver::ccdSolve(Bone& cur, Bone& end)
{
	aiVector3D cur2end = end.end - cur.start;
	aiVector3D cur2target = target - cur.start;

	aiQuaternion inverse_rotation = cur.global_rotation;
	inverse_rotation.Conjugate();

	aiVector3D local_cur2end = inverse_rotation.Rotate(cur2end).Normalize();
	aiVector3D local_cur2target = inverse_rotation.Rotate(cur2target).Normalize();

	float angle = acos(local_cur2target * local_cur2end);

	if (isnan(angle) || abs(angle) < epsilon)
		return;

	angle = max(angle, -angle_limit);
	angle = min(angle, angle_limit);
	
	aiVector3D axis = local_cur2end ^ local_cur2target;

	aiQuaternion rotation(axis, angle);
	
	cur.rotation = cur.rotation * rotation;
	cur.global_rotation = cur.global_rotation * rotation;
}

void IKSolver::updateBonePos(int index)
{
	for (int i = index; i >= 0; --i)
	{
		Bone& bone = bones[i];
		if (i != bones.size() - 1) 
			bone.start = bones[i + 1].end;

		aiVector3D dir = bone.global_rotation.Rotate(aiVector3D(0, 1, 0));
		dir.Normalize();
		
		bone.end = bone.start + dir * bone.length;
	}
}

void IKSolver::applyRotation(Mesh& mesh)
{
	aiVector3D scaling(1, 1, 1), position(0, 0, 0);		// dummy
	for (auto& bone : bones)
		for (auto& ori_bone : mesh.bones)
		{
			if (bone.name != ori_bone.name)
				continue;
			ori_bone.local_transformation = aiMatrix4x4t<float>(scaling, bone.rotation, position);
			break;
		}
}

bool IKSolver::isApproximatelyEqual(const aiVector3D& a, const aiVector3D& b)
{
	return (a - b).Length() < epsilon;
}

void IKSolver::setBoneChain(EndEffector end_effector)
{
	switch (end_effector)
	{
	case EndEffector::HEAD:
		start = "neck";
		end = "head";
		break;
	case EndEffector::LEFT_FORE_FOOT:
		start = "foreLimpLeft1";
		end = "foreLimpLeft3";
		break;
	case EndEffector::LEFT_REAR_FOOT:
		start = "rearLimpLeft1";
		end = "rearLimpLeft3";
		break;
	case EndEffector::RIGHT_FORE_FOOT:
		start = "foreLimpRight1";
		end = "foreLimpRight3";
		break;
	case EndEffector::RIGHT_REAR_FOOT:
		start = "rearLimpRight1";
		end = "rearLimpRight3";
		break;
	}
}
