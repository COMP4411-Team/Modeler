#include "IKSolver.h"


// Use CCD to solve IK
// Reference: https://blog.csdn.net/gamesdev/article/details/14110875
void IKSolver::ccdSolve(Bone& cur, Bone& end, aiVector3D& target, float limit_angle)
{
	aiVector3D end2target = end.end - cur.start;
	aiVector3D cur2target = target - cur.start;

	aiQuaternion inverse_rotation = cur.global_rotation.Conjugate();

	aiVector3D local_end2target = inverse_rotation.Rotate(end2target).Normalize();
	aiVector3D local_cur2target = inverse_rotation.Rotate(cur2target).Normalize();

	float angle = acos(local_cur2target * local_end2target);
	
	aiVector3D axis = local_cur2target ^ local_end2target;

	aiQuaternion rotation(axis, angle);
	
	cur.rotation = cur.rotation * rotation;
	cur.global_rotation = cur.global_rotation * rotation;
}
