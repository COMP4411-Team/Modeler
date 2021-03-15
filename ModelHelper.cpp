#include "ModelHelper.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include "bitmap.h"

using namespace Assimp;
using namespace std;

void ModelHelper::loadModel(const string& model, const string& bone)
{
	const auto* new_scene = importer.ReadFile(model, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
	                                    aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
	if (new_scene == nullptr)
		throw runtime_error("failed to parse model file");
	
	delete scene;
	scene = new_scene;
	preprocess();
	if (!bone.empty())
	{
		parseBoneInfo(bone);
		calBoneTransformation(aiQuaternion(), scene->mRootNode);
	}
}

void ModelHelper::preprocess()
{
	meshes.resize(scene->mNumMeshes);
	for (int i = 0; i < scene->mNumMeshes; ++i)
	{
		auto* mesh = scene->mMeshes[i];
		auto& vertices = meshes[i].vertices;
		meshes[i].data = mesh;
		meshes[i].name = mesh->mName.data;
		meshes[i].parent = this;

		vertices.resize(mesh->mNumVertices);
		for (int j = 0; j < mesh->mNumVertices; ++j)
			vertices[j].original_pos = (mesh->mVertices[j]);

		if (mesh->HasTextureCoords(0))
		{
			for (int j = 0; j < mesh->mNumVertices; ++j)
				vertices[j].tex_coords = (mesh->mTextureCoords[0][j]);
		}

		for (int j = 0; j < mesh->mNumBones; ++j)
		{
			auto* bone = mesh->mBones[j];
			string name = processBoneName(string(bone->mName.data));

			if (bone_map.find(name) == bone_map.end())
			{
				bones.emplace_back();
				bone_map[name] = bones.size() - 1;
				meshes[i].offset_map[name] = bone->mOffsetMatrix;
			}

			int bone_id = bone_map[name];
			
			for (int k = 0; k < bone->mNumWeights; ++k)
			{
				int vertex_id = bone->mWeights[k].mVertexId;
				vertices[vertex_id].bone_index.push_back(bone_id);
				vertices[vertex_id].bone_weight.push_back(bone->mWeights[k].mWeight);
			}
		}
	}
}

// transformation can transform bones from world space to parent space
void ModelHelper::calBoneTransformation(const aiQuaternion& global_rotation, const aiNode* cur)
{
	string name = processBoneName(cur->mName.data);
	aiQuaternion new_global_rotation = global_rotation;
	
	if (bone_map.find(name) != bone_map.end())
	{
		auto& bone = bones[bone_map[name]];
		bone.node = cur;
		bone.name = name;

		auto trafo = cur->mTransformation;
		aiQuaternion rotation;
		aiVector3D position;
		trafo.DecomposeNoScaling(rotation, position);
		
		bone.rotation = rotation;
		bone.global_rotation = global_rotation * rotation;
		new_global_rotation = bone.global_rotation;
		bone.length = (bone.end - bone.start).Length();
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		calBoneTransformation(new_global_rotation, cur->mChildren[i]);
}

void ModelHelper::parseBoneInfo(const string& filename)
{
	ifstream fs(filename);
	if (!fs.is_open())
		throw runtime_error("failed to open bone info");

	string bone_name;
	while (fs >> bone_name)
	{
		if (bone_map.find(bone_name) != bone_map.end())
		{
			Bone& bone = getBone(bone_name);
			float x, y, z;
			
			fs >> x >> y >> z;
			bone.start = {x, y, z};

			fs >> x >> y >> z;
			bone.end = {x, y, z};	
		}
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

void ModelHelper::printMeshInfo(bool showBoneHierarchy)
{
	for (int i = 0; i < meshes.size(); ++i)
	{
		std::cout << "mesh " << i << ": " << meshes[i].name << std::endl
			<< "mNumVertices: " << scene->mMeshes[i]->mNumVertices << std::endl
			<< "mNumFaces: " << scene->mMeshes[i]->mNumFaces << std::endl
			<< "mNumBones: " << scene->mMeshes[i]->mNumBones << std::endl;
	}
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

aiMatrix4x4t<float> ModelHelper::calViewingTransformation(Vec3f& eye, Vec3f& at, Vec3f& up)
{
	
	Vec3f viewDir = eye - at, upDir = up - eye;
	viewDir.normalize();
	upDir.normalize();

	// up cross view, somehow the vec header does not implement cross product
	float x = upDir[1] * viewDir[2] - upDir[2] * viewDir[1];
	float y = -(upDir[0] * viewDir[2] - upDir[2] * viewDir[0]);
	float z = upDir[0] * viewDir[1] - upDir[1] * viewDir[0];
	Vec3f leftNormal(x, y, z);
	leftNormal.normalize();

	// recalculate up by viewDir cross leftNormal
	x = viewDir[1] * leftNormal[2] - viewDir[2] * leftNormal[1];
	y = -(viewDir[0] * leftNormal[2] - viewDir[2] * leftNormal[0]);
	z = viewDir[0] * leftNormal[1] - viewDir[1] * leftNormal[0];

	upDir = {x, y, z};
	aiMatrix4x4t<float> mat;

	mat.a1 = leftNormal[0];
	mat.b1 = leftNormal[1];
	mat.c1 = leftNormal[2];
	
	mat.a2 = upDir[0];
	mat.b2 = upDir[1];
	mat.c2 = upDir[2];

	mat.a3 = viewDir[0];
	mat.b3 = viewDir[1];
	mat.c3 = viewDir[2];
	
	mat.a4 = -(leftNormal * Vec3f(eye[0], eye[1], eye[2]));
	mat.b4 = -(upDir * Vec3f(eye[0], eye[1], eye[2]));
	mat.c4 = -(viewDir * Vec3f(eye[0], eye[1], eye[2]));
	mat.d4 = 1.f;
	return mat;
}

bool ModelHelper::applyTranslate(const std::string& bone_name, const aiVector3D& translation)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Translation(translation, mat);
	return applyMatrix(bone_name, mat);
}

bool ModelHelper::applyRotationX(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationX(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool ModelHelper::applyRotationY(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationY(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool ModelHelper::applyRotationZ(const std::string& bone_name, float angle)
{
	angle = AI_MATH_PI_F * angle / 180.f;
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::RotationZ(angle, mat);
	return applyMatrix(bone_name, mat);
}

bool ModelHelper::applyScaling(const std::string& bone_name, const aiVector3D& scale)
{
	aiMatrix4x4t<float> mat;
	aiMatrix4x4t<float>::Scaling(scale, mat);
	return applyMatrix(bone_name, mat);
}

bool ModelHelper::restoreIdentity(const std::string& bone_name)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = aiMatrix4x4t<float>();
	return true;
}

bool ModelHelper::applyMatrix(const std::string& bone_name, const aiMatrix4x4t<float>& mat)
{
	if (bone_map.find(bone_name) == bone_map.end())
		return false;
	int index = bone_map[bone_name];
	bones[index].local_transformation = mat * bones[index].local_transformation;
	return true;
}

void ModelHelper::printBoneHierarchy(const aiNode* cur, int depth)
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

Bone& ModelHelper::getBone(const std::string& name)
{
	if (bone_map.find(name) != bone_map.end())
		return bones[bone_map[name]];

	throw invalid_argument("bone does not exist");
}

std::string ModelHelper::processBoneName(const std::string& name)
{
	int pos = name.find_last_of('_');
	if (pos != name.npos)
		return name.substr(pos + 1);
	return name;
}
