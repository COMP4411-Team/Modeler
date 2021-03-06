#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <map>
#include "mat.h"

class Vertex
{
public:
	aiVector3D original_pos;
	aiVector3D world_pos;
	std::vector<int> bone_index;
	std::vector<float> bone_weight;
};

class Bone
{
public:
	aiMatrix4x4t<float> final_transformation;	// final matrix applied to vertices
	aiMatrix4x4t<float> local_transformation;	// all the user specified transformations
	aiMatrix4x4t<float> offset;					// transform vertex from local to bone space
};

class Mesh
{
public:
	const aiMesh* data;
	std::vector<Vertex> vertices;
	std::vector<Bone> bones;
	std::map<std::string, int> bone_map;

	bool applyTranslate(const std::string& bone_name, aiVector3D& translation);

	// angle in degrees
	bool applyRotationX(const std::string& bone_name, float angle);
	bool applyRotationY(const std::string& bone_name, float angle);
	bool applyRotationZ(const std::string& bone_name, float angle);
	
	bool applyScaling(const std::string& bone_name, aiVector3D& scale);
	bool restoreIndentity(const std::string& bone_name);
	bool applyMatrix(const std::string& bone_name, aiMatrix4x4t<float>& mat);

	void printBoneHierarchy(const aiNode* cur, int depth);

	static std::string processBoneName(const std::string name);
};

class ModelHelper
{
public:

	bool loadModel(const char* filename);
	bool preprocess();
	
	Assimp::Importer importer;
	const aiScene* scene{nullptr};
	std::vector<Mesh> meshes;
};

