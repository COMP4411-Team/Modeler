#include "ModelHelper.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include "bitmap.h"

using namespace Assimp;
using namespace std;

void ModelHelper::loadModel(const string& model, const string& bone)
{
	auto* new_scene = importer.ReadFile(model, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
	                                    aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

	if (new_scene == nullptr)
		throw runtime_error("failed to parse model file");
	
	delete scene;
	scene = new_scene;
	preprocess();
	if (!bone.empty())
		parseBoneInfo(meshes[0], bone);
	calBoneTransformation(aiMatrix4x4t<float>(), scene->mRootNode);
}

void ModelHelper::preprocess()
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
			vertices[j].original_pos = (mesh->mVertices[j]);

		if (mesh->HasTextureCoords(0))
		{
			for (int j = 0; j < mesh->mNumVertices; ++j)
				vertices[j].tex_coords = (mesh->mTextureCoords[0][j]);
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
}

void ModelHelper::calBoneTransformation(const aiMatrix4x4t<float>& transformation, const aiNode* cur)
{
	string name = Mesh::processBoneName(cur->mName.data);
	auto cur_transformation = transformation;

	for (auto& mesh : meshes)
	{
		if (mesh.bone_map.find(name) == mesh.bone_map.end())
			continue;

		auto& bone = mesh.bones[mesh.bone_map[name]];

		aiVector3D vec_world = bone.end - bone.start;			// the bone in world space

		try
		{
			Bone& p = mesh.getBone(cur->mParent->mName.data);
			if (p.end == bone.start)
			{
				aiMatrix4x4t<float> translation;
				aiMatrix4x4t<float>::Translation({0.f, 0.f, -p.spherical_coords.z}, translation);
				cur_transformation = translation * transformation;
			}
		} catch (...) { }
		
		aiVector3D vec_local = cur_transformation * vec_world;		// the bone in parent's space
		
		bone.spherical_coords = calSphericalCoords(vec_local);
		cur_transformation = calTrafoMatrix(vec_local) * cur_transformation;
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		calBoneTransformation(cur_transformation, cur->mChildren[i]);
}

void ModelHelper::parseBoneInfo(Mesh& mesh, const string& filename)
{
	ifstream fs(filename);
	if (!fs.is_open())
		throw runtime_error("failed to open bone info");

	string bone_name;
	while (fs >> bone_name)
	{
		if (mesh.bone_map.find(bone_name) == mesh.bone_map.end())
			throw runtime_error("bone info file contains unknown bone names");

		Bone& bone = mesh.getBone(bone_name);
		
		float x, y, z;
		fs >> x >> y >> z;
		bone.start = {x, y, z};

		fs >> x >> y >> z;
		bone.end = {x, y, z};
	}
}

void ModelHelper::loadTexture(const std::string& filename)
{
	int height, width;
	auto* new_tex = readBMP(const_cast<char*>(filename.c_str()), width, height);
	if (new_tex == nullptr)
		throw runtime_error("failed to load texture");
	delete tex;
	tex = new_tex;
	tex_height = height;
	tex_width = width;
	tex_loaded = false;		// not loaded or updated in opengl
}

aiVector3D ModelHelper::calSphericalCoords(const aiVector3D& vec)
{
	float rho = vec.Length();
	float theta = atan2(vec.y, vec.x);
	float phi = acos(vec.z / rho);
	return aiVector3D(theta, phi, rho);
}

aiMatrix4x4t<float> ModelHelper::calTrafoMatrix(const aiVector3D& vec)
{
	float theta = -atan2(vec.y, vec.x);		// the inverse
	float phi = -acos(vec.z / vec.Length());
	aiMatrix4x4t<float> out1, out2;
	aiMatrix4x4t<float>::RotationZ(theta, out1);
	aiMatrix4x4t<float>::RotationY(phi, out2);
	return out2 * out1;		// the inverse
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

bool Mesh::restoreIdentity(const std::string& bone_name)
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

Bone& Mesh::getBone(const std::string& name)
{
	if (bone_map.find(name) != bone_map.end())
		return bones[bone_map[name]];

	throw invalid_argument("bone does not exist");
}

std::string Mesh::processBoneName(const std::string& name)
{
	int pos = name.find('_');
	if (pos != name.npos)
		return name.substr(pos + 1);
	return name;
}
