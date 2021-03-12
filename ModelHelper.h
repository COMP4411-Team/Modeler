#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <map>
#include "mat.h"
#include "vec.h"

class Vertex
{
public:
	aiVector3D original_pos;
	aiVector3D world_pos;
	aiVector3D tex_coords;
	std::vector<int> bone_index;
	std::vector<float> bone_weight;
};

class Bone
{
public:
	// Matrices used for rendering the mesh
	aiMatrix4x4t<float> final_transformation;	// final matrix applied to vertices
	aiMatrix4x4t<float> local_transformation;	// all the user specified transformations
	aiMatrix4x4t<float> offset;					// transform vertex from local to bone space

	// Info for rendering the bone and IK
	float length;
	aiVector3D start, end;			// world space coords for end points
	aiQuaternion rotation;			// rotation from parent to local space
	aiQuaternion global_rotation;	// from world to bone space, for IK
};

class Mesh
{
public:
	const aiMesh* data;
	std::vector<Vertex> vertices;
	std::vector<Bone> bones;
	std::map<std::string, int> bone_map;
	aiVector3D aabb_min, aabb_max;

	bool applyTranslate(const std::string& bone_name, const aiVector3D& translation);

	// angle in degrees
	bool applyRotationX(const std::string& bone_name, float angle);
	bool applyRotationY(const std::string& bone_name, float angle);
	bool applyRotationZ(const std::string& bone_name, float angle);
	
	bool applyScaling(const std::string& bone_name, const aiVector3D& scale);
	bool restoreIdentity(const std::string& bone_name);
	bool applyMatrix(const std::string& bone_name, const aiMatrix4x4t<float>& mat);

	void printBoneHierarchy(const aiNode* cur, int depth);
	Bone& getBone(const std::string& name);
	
	static std::string processBoneName(const std::string& name);
};

class ModelHelper
{
public:
	void loadModel(const std::string& model, const std::string& bone);
	void preprocess();
	void calBoneTransformation(const aiQuaternion& global_rotation, const aiNode* cur);
	void parseBoneInfo(Mesh& mesh, const std::string& filename);
	void loadTexture(const std::string& filename);

	static aiVector3D calSphericalCoords(const aiVector3D& vec);		// unused
	static aiMatrix4x4t<float> calTrafoMatrix(const aiVector3D& vec);	// unused
	static aiMatrix4x4t<float> calViewingTransformation(Vec3f& eye, Vec3f& at, Vec3f& up);
	
	Assimp::Importer importer;
	const aiScene* scene{nullptr};
	std::vector<Mesh> meshes;
	
	unsigned char* tex{nullptr};
	int tex_height, tex_width;
	bool tex_loaded{false};
};

