#include "ModelHelper.h"

#include <iostream>

using namespace Assimp;
using namespace std;

bool ModelHelper::loadModel(const char* filename)
{
	auto* new_scene = importer.ReadFile(filename, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
	                                    aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

	if (new_scene == nullptr)
		return false;
	
	delete scene;
	scene = new_scene;
	return preprocess();
}

bool ModelHelper::preprocess()
{
	meshes.resize(scene->mNumMeshes);
	for (int i = 0; i < scene->mNumMeshes; ++i)
	{
		auto* mesh = scene->mMeshes[i];
		meshes[i].data = mesh;
		auto& vertices = meshes[i].vertices;
		auto& bones = meshes[i].bones;
		auto& bone_map = meshes[i].bone_map;

		vertices.resize(mesh->mNumVertices);
		for (int j = 0; j < mesh->mNumVertices; ++j)
		{
			vertices[j].original_pos = (mesh->mVertices[j]);
		}

		bones.resize(mesh->mNumBones);
		for (int j = 0; j < mesh->mNumBones; ++j)
		{
			auto* bone = mesh->mBones[j];
			bone_map[Mesh::processBoneName(string(bone->mName.data))] = j;
			bones[j].offset = bone->mOffsetMatrix;

			for (int k = 0; k < bone->mNumWeights; ++k)
			{
				int vertex_id = bone->mWeights[k].mVertexId;
				vertices[vertex_id].bone_index.push_back(j);
				vertices[vertex_id].bone_weight.push_back(bone->mWeights[k].mWeight);
			}
		}
	}
	return true;
}

bool Mesh::applyTranslate(const std::string& bone_name, aiVector3D& translation)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Translation(translation, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationX(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationX(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationY(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationY(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyRotationZ(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationZ(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::applyScaling(const std::string& bone_name, aiVector3D& scale)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Scaling(scale, mat);
	return applyMatrix(bone_name, mat);
}

bool Mesh::restoreIndentity(const std::string& bone_name)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = aiMatrix4x4t<float>();
	return true;
}

bool Mesh::applyMatrix(const std::string& bone_name, aiMatrix4x4t<float>& mat)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = mat * bones[index].local_transformation;
	return true;
}

void Mesh::printBoneHierarchy(const aiNode* cur, int depth)
{
	string name = processBoneName(cur->mName.data);
	if (bone_map.find(name) != bone_map.end())
	{
		string prefix(depth, '\t');
		cout << prefix << name << endl;
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		printBoneHierarchy(cur->mChildren[i], depth + 1);
}

std::string Mesh::processBoneName(const std::string name)
{
	int pos = name.find('_');
	if (pos != name.npos)
		return name.substr(pos + 1);
	return name;
}
