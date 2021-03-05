#include "ModelHelper.h"

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
			bone_map[string(bone->mName.data)] = j;
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
