#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <map>

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
	aiMatrix4x4t<float> final_transformation;
	aiMatrix4x4t<float> offset;
};

class Mesh
{
public:
	const aiMesh* data;
	std::vector<Vertex> vertices;
	std::vector<Bone> bones;
	std::map<std::string, int> bone_map;
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

