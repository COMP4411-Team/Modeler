#include <iostream>

#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>
#include <gl/GLU.h>

#include "modelerglobals.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "ModelHelper.h"
#include "bitmap.h"
#include "camera.h"
#include "modelerui.h"

using namespace std;
using namespace Assimp;
using Matrix4f = aiMatrix4x4t<float>;

ModelHelper helper;		// simply use global variable for now
Matrix4f global_inverse;
float tick = 0.f;
float cur_fov = 30.f;
float cur_zfar = 100.f;

// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView 
{
public:
    SampleModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) { }

    virtual void draw();
	//void ChangeLight0();
	//void ChangeLight1();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{ 
    return new SampleModel(x,y,w,h,label); 
}


inline float degree2Radian(float degree)
{
	return degree / 180.f * AI_MATH_PI_F;
}

inline float radian2Degree(float radian)
{
	return radian / AI_MATH_PI_F * 180.f;
}


void drawTriangle(const aiVector3D& v1, const aiVector3D& v2, const aiVector3D& v3)
{
	drawTriangle(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
}

void drawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
{
	drawTriangle(v1.world_pos, v2.world_pos, v3.world_pos);
}

void applyAiMatrix(const aiMatrix4x4t<float>& mat)
{
	float m[16];
	m[0] = mat.a1; m[1] = mat.b1; m[2] = mat.c1; m[3] = mat.d1;
	m[4] = mat.a2; m[5] = mat.b2; m[6] = mat.c2; m[7] = mat.d2;
	m[8] = mat.a3; m[9] = mat.b3; m[10] = mat.c3; m[11] = mat.d3;
	m[12] = mat.a4; m[13] = mat.b4; m[14] = mat.c4; m[15] = mat.d4;
	glMultMatrixf(m);
}

//control of light 0 and 1
//void SampleModel::ChangeLight0() {
	
//}

//void SampleModel::ChangeLight1() {
//	ModelerView::moveLight1(VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1));
//}

// column to row order
aiMatrix4x4t<float> array2Mat(float* a)
{
	aiMatrix4x4t<float> mat;
	mat.a1 = a[0]; mat.b1 = a[1]; mat.c1 = a[2]; mat.d1 = a[3];
	mat.a2 = a[4]; mat.b2 = a[5]; mat.c2 = a[6]; mat.d2 = a[7];
	mat.a3 = a[8]; mat.b3 = a[9]; mat.c3 = a[10]; mat.d3 = a[11];
	mat.a4 = a[12]; mat.b4 = a[13]; mat.c4 = a[14]; mat.d4 = a[15];
	return mat;
}


void adjustCameraPos(aiVector3D center, Camera* camera, float radius)
{
	camera->minDolly = -radius;
	Vec3f at{center.x, center.y, center.z};
	float cur_len = (at - camera->mPosition).length();

	if (cur_len < radius)
	{
		camera->mDolly = -radius * 2;
		Vec3f dir = camera->mPosition - camera->mLookAt;
		dir.normalize();
		dir *= radius;
		camera->mUpVector += dir;
		camera->mPosition += dir;
	}
}


// If current point is not visible, adjust fov
void adjustCamera(aiVector3D point, float aspect)
{
	if (point.z < 0)
		return;
	
	float fov_needed = atan(abs(point.y / point.z)) / AI_MATH_PI_F * 180.f;
	fov_needed = max(fov_needed, atan(abs(point.x * aspect / point.z)) / AI_MATH_PI_F * 180.f);

	fov_needed = max(fov_needed, 30.f);
	fov_needed = max(cur_fov, fov_needed);
	float zfar_needed = max(abs(point.z), cur_zfar);

	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fov_needed, aspect,1.0,zfar_needed);
				
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

// Frame all
void frameAll()
{
	ModelerView* view = ModelerApplication::Instance()->m_ui->m_modelerView;
	auto* camera = view->m_camera;
	float aspect = (float)view->w() / view->h();
	
	float modelview[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

	auto mat = array2Mat(modelview);
	auto aabb_max = mat * helper.meshes[0].aabb_max;
	auto aabb_min = mat * helper.meshes[0].aabb_min;
	auto mid = (helper.meshes[0].aabb_max + helper.meshes[0].aabb_min) / 2.f;

	camera->setLookAt({mid.x, mid.y, mid.z});
	camera->applyViewingTransform();
	// adjustCameraPos((aabb_min + aabb_max) / 2.f, camera, (aabb_min - aabb_max).Length() / 2);
	
	auto point = aabb_max;
	adjustCamera(point, aspect);

	point = aabb_min;
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_min.x, aabb_max.y, aabb_max.z};
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_max.x, aabb_min.y, aabb_max.z};
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_max.x, aabb_max.y, aabb_min.z};
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_min.x, aabb_min.y, aabb_max.z};
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_min.x, aabb_max.y, aabb_min.z};
	adjustCamera(point, aspect);

	point = aiVector3D{aabb_max.x, aabb_min.y, aabb_min.z};
	adjustCamera(point, aspect);

	camera->applyViewingTransform();
}

// Animation
void animate()
{
	auto& mesh = helper.meshes[0];
	float left1 = cos(tick) * 15;
	float right1 = sin(tick) * 15;
	float left2 = cos(tick) * 30;
	float right2 = sin(tick) * 30;
	float left3 = -cos(cos(tick)) * 35;
	float right3 = -sin(sin(tick)) * 35;
	float head = sin(tick) * 2;
	float neck = cos(tick) * 2;
	float tail = cos(tick) * 2;
	float main = sin(tick) * 0.1f;

	mesh.applyTranslate("main", aiVector3D(0, 0, main));

	mesh.applyRotationX("neck", neck);

	mesh.applyRotationZ("head", head);

	mesh.applyRotationX("tail", tail);
	
	mesh.applyRotationX("foreLimpLeft1", left1);
	mesh.applyRotationX("foreLimpRight1", right1);
	mesh.applyRotationZ("rearLimpLeft1", left1);
	mesh.applyRotationZ("rearLimpRight1", right1);

	mesh.applyRotationX("foreLimpLeft2", left2);
	mesh.applyRotationX("foreLimpRight2", right2);
	mesh.applyRotationZ("rearLimpLeft2", left2);
	mesh.applyRotationZ("rearLimpRight2", right2);

	mesh.applyRotationX("foreLimpLeft3", left3);
	mesh.applyRotationX("foreLimpRight3", right3);
	mesh.applyRotationZ("rearLimpLeft3", left3 * 0.5);
	mesh.applyRotationZ("rearLimpRight3", right3 * 0.5);

	tick += 0.5f;
	if (tick > 1e4f * AI_MATH_PI_F)
		tick = 0.f;
}


// Apply all the user controls to meshes in one place
void applyMeshControls()
{
	auto& mesh = helper.meshes[0];

	mesh.restoreIdentity("main");
	mesh.applyRotationZ("main", VAL(ROTATE_ALL));

	float x = VAL(XPOS), y = VAL(YPOS), z = VAL(ZPOS);

	mesh.applyTranslate("main", aiVector3D(x, y, z));

	//===========================================================

	mesh.restoreIdentity("neck");
	mesh.applyRotationX("neck", VAL(NECK_PITCH));
	mesh.applyRotationZ("neck", VAL(NECK_YAW));
	mesh.applyRotationY("neck", VAL(NECK_ROLL));

	mesh.restoreIdentity("head");
	mesh.applyRotationZ("head", VAL(HEAD_PITCH));
	mesh.applyRotationX("head", VAL(HEAD_YAW));
	mesh.applyRotationY("head", VAL(HEAD_ROLL));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft1");
	mesh.applyRotationX("foreLimpLeft1", VAL(LEFT_FORELIMP_1));

	mesh.restoreIdentity("foreLimpRight1");
	mesh.applyRotationX("foreLimpRight1", VAL(RIGHT_FORELIMP_1));

	mesh.restoreIdentity("rearLimpLeft1");
	mesh.applyRotationZ("rearLimpLeft1", VAL(LEFT_REARLIMP_1));

	mesh.restoreIdentity("rearLimpRight1");
	mesh.applyRotationZ("rearLimpRight1", VAL(RIGHT_REARLIMP_1));

	//============================================================

	mesh.restoreIdentity("foreLimpLeft2");
	mesh.applyRotationX("foreLimpLeft2", VAL(LEFT_FORELIMP_2));

	mesh.restoreIdentity("foreLimpRight2");
	mesh.applyRotationX("foreLimpRight2", VAL(RIGHT_FORELIMP_2));

	mesh.restoreIdentity("rearLimpLeft2");
	mesh.applyRotationZ("rearLimpLeft2", VAL(LEFT_REARLIMP_2));

	mesh.restoreIdentity("rearLimpRight2");
	mesh.applyRotationZ("rearLimpRight2", VAL(RIGHT_REARLIMP_2));

	//=============================================================

	mesh.applyRotationZ("foreLimpLeft2", VAL(LEFT_FORELIMP_2_YAW));

	mesh.applyRotationZ("foreLimpRight2", VAL(RIGHT_FORELIMP_2_YAW));

	mesh.applyRotationX("rearLimpLeft2", VAL(LEFT_REARLIMP_2_YAW));

	mesh.applyRotationX("rearLimpRight2", VAL(RIGHT_REARLIMP_2_YAW));

	//=============================================================

	mesh.restoreIdentity("foreLimpLeft3");
	mesh.applyRotationX("foreLimpLeft3", VAL(LEFT_FORELIMP_3));

	mesh.restoreIdentity("foreLimpRight3");
	mesh.applyRotationX("foreLimpRight3", VAL(RIGHT_FORELIMP_3));

	mesh.restoreIdentity("rearLimpLeft3");
	mesh.applyRotationZ("rearLimpLeft3", VAL(LEFT_REARLIMP_3));

	mesh.restoreIdentity("rearLimpRight3");
	mesh.applyRotationZ("rearLimpRight3", VAL(RIGHT_REARLIMP_3));

	//==============================================================

	mesh.restoreIdentity("tail");
	mesh.applyRotationX("tail", VAL(TAIL_PITCH));
	mesh.applyRotationZ("tail", VAL(TAIL_YAW));
}


// aiNode* is a node in the bone hierarchy, it contains the name, its own transformation, and
// pointers to its parent and children
void traverseBoneHierarchy(Mesh& mesh, const aiNode* cur, const Matrix4f& parent_transformation)
{
	if (cur == nullptr) return;
	string bone_name(Mesh::processBoneName(cur->mName.data));

	// cur->mTransformation transforms the node from its local space to its parent's space
	Matrix4f cur_transformation = cur->mTransformation;
	Matrix4f global_transformation = parent_transformation * cur_transformation;

	// In case some node doesn't represent a bone, we check whether we can find the bone_name in the map
	if (mesh.bone_map.find(bone_name) != mesh.bone_map.end())
	{
		int bone_index = mesh.bone_map[bone_name];

		global_transformation = global_transformation * mesh.bones[bone_index].local_transformation; 

		// final_transformation is used to transform the vertices from local space to world space
		// any other transformation should be right-multiplied to global_transformation
		mesh.bones[bone_index].final_transformation = 
			global_inverse * global_transformation * mesh.bones[bone_index].offset;
	}

	// Recursively visit its children
	for (int i = 0; i < cur->mNumChildren; ++i)
	{
		traverseBoneHierarchy(mesh, cur->mChildren[i], global_transformation);
	}
}


// Update the position in world space for each vertex
void processVertices(Mesh& mesh)
{
	for (auto& vertex : mesh.vertices)
	{
		auto& bone = mesh.bones[vertex.bone_index[0]];
		Matrix4f transformation = bone.final_transformation * vertex.bone_weight[0];

		for (int i = 1; i < vertex.bone_index.size(); ++i)
		{
			auto& bone = mesh.bones[vertex.bone_index[i]];
			transformation = transformation + bone.final_transformation * vertex.bone_weight[i];
		}
		vertex.world_pos = transformation * vertex.original_pos;
	}
}


void renderMesh(Mesh& mesh)
{
	auto* ai_mesh = mesh.data;
	for (int i = 0; i < ai_mesh->mNumFaces; ++i)
	{
		const aiFace& face = ai_mesh->mFaces[i];
		//auto& vertices = mesh.vertices;
		//int v1 = face.mIndices[0];
		//int v2 = face.mIndices[1];
		//int v3 = face.mIndices[2];
		
		// drawTriangle(vertices[v1], vertices[v2], vertices[v3]);
		drawTriangle(mesh, face);
	}

	// Recalculate aabb for the mesh
	mesh.aabb_min = mesh.aabb_max = mesh.vertices[0].world_pos;
	for (auto& vt : mesh.vertices)
	{
		aiVector3D& pos = vt.world_pos;
		mesh.aabb_min.x = min(mesh.aabb_min.x, pos.x);
		mesh.aabb_min.y = min(mesh.aabb_min.y, pos.y);
		mesh.aabb_min.z = min(mesh.aabb_min.z, pos.z);

		mesh.aabb_max.x = max(mesh.aabb_max.x, pos.x);
		mesh.aabb_max.y = max(mesh.aabb_max.y, pos.y);
		mesh.aabb_max.z = max(mesh.aabb_max.z, pos.z);
	}
}


void renderBones(Mesh& mesh, const aiNode* cur)
{
	glPushMatrix();

	string name = Mesh::processBoneName(cur->mName.data);
	if (mesh.bone_map.find(name) != mesh.bone_map.end())
	{
		Bone& bone = mesh.getBone(name);

		float theta = radian2Degree(bone.spherical_coords.x);
		float phi = radian2Degree(bone.spherical_coords.y);

		string p_name = Mesh::processBoneName(cur->mParent->mName.data);

		try
		{
			Bone& p = mesh.getBone(p_name);
			if (p.end == bone.start)
				glTranslatef(0, 0, p.spherical_coords.z);
			
		} catch (...) { }

		applyAiMatrix(bone.local_transformation);

		glRotatef(theta, 0, 0, 1);
		glRotatef(phi, 0, 1, 0);
		
		drawCylinder(bone.spherical_coords.z, 0.3, 0.01);	// cylinder for now
	}

	for (int i = 0; i < cur->mNumChildren; ++i)
		renderBones(mesh, cur->mChildren[i]);

	glPopMatrix();
}

//void adjustLight

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{
	// frameAll();
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
	//ModelerView::openLight0(VAL(LIGHT0_ENABLE));
	//ModelerView::openLight1(VAL(LIGHT1_ENABLE));
	//ModelerView::moveLight0(VAL(LIGHTX_0), VAL(LIGHTY_0), VAL(LIGHTZ_0));
	//ModelerView::moveLight1(VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1));
    ModelerView::draw();
	if (VAL(LIGHT0_ENABLE)) {
		glEnable(GL_LIGHT0);
		GLfloat changedLightPosition0[] = { VAL(LIGHTX_0), VAL(LIGHTY_0), VAL(LIGHTZ_0),0 };
		glLightfv(GL_LIGHT0, GL_POSITION, changedLightPosition0);
	}
	else {
		glDisable(GL_LIGHT0);
	}

	if (VAL(LIGHT1_ENABLE)) {
		glEnable(GL_LIGHT1);
		GLfloat changedLightPosition1[] = { VAL(LIGHTX_1), VAL(LIGHTY_1), VAL(LIGHTZ_1),0 };
		glLightfv(GL_LIGHT1, GL_POSITION, changedLightPosition1);
	}
	else {
		glDisable(GL_LIGHT1);
	}

	//const auto* glVersion = glGetString(GL_VERSION);
	//const auto* glRenderer = glGetString(GL_RENDERER);
	//const auto* glVendor = glGetString(GL_VENDOR);
	//const auto* gluVersion = glGetString(GLU_VERSION);
	//printf("version %s\n", glVersion);
	//printf("renderer %s\n", glRenderer);
	//printf("vendor %s\n", glVendor);
	//printf("glu version %s\n", gluVersion);

	drawSphere(0.1);
	drawCylinder(1, 0.1, 0.01);
	
	// Init texture
	glEnable(GL_TEXTURE_2D);
	if (!helper.tex_loaded)
	{
		unsigned int texture_id;
		glGenTextures(1, &texture_id);
		glBindTexture(GL_TEXTURE_2D, texture_id);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, helper.tex_width, helper.tex_height,
		0, GL_RGB, GL_UNSIGNED_BYTE, helper.tex);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		helper.tex_loaded = true;
	}
	
	
	// Setup env and pose
	setAmbientColor(0.75f, 0.75f, 0.75f);
	setDiffuseColor(0.75f, 0.75f, 0.75f);
	glScaled(0.5, 0.5, 0.5);
	glRotated(-90, 1, 0, 0);
	glRotated(180, 0, 0, 1);
	glTranslated(0, 0, -5);

	// Initialization
	auto& mesh = helper.meshes[0];
	auto* scene = helper.scene;
	global_inverse = scene->mRootNode->mTransformation.Inverse();

	// Apply user controls to meshes here
	applyMeshControls();

	if (ModelerApplication::Instance()->m_animating)
		animate();

	// Apply controls to bones and render them
	renderBones(mesh, scene->mRootNode);

	// Avoid overlapping bones and meshes

	glTranslated(0, 5, 0);
	glRotated(180, 1, 0, 0);

	// Render the meshes
	traverseBoneHierarchy(mesh, scene->mRootNode, Matrix4f());
	processVertices(mesh);
	renderMesh(mesh);
}

int main()
{
	helper.loadModel("./models/lowpolydeer_uv.dae", "./models/lowpolydeer_bone_modified.txt");

	helper.loadTexture("./models/wood_texture.bmp");
	
	auto* scene = helper.scene;
	std::cout << "import done, mNumMeshes: " << scene->mNumMeshes << std::endl;

	std::cout << "mesh0 mNumVertices: " << scene->mMeshes[0]->mNumVertices
		<< "\t mNumFaces: " << scene->mMeshes[0]->mNumFaces
		<< "\t mNumBones: " << scene->mMeshes[0]->mNumBones << std::endl;
	
	helper.meshes[0].printBoneHierarchy(scene->mRootNode, 0);	// print out the bones

	
	
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
    controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
    controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
    controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);

	controls[LIGHT0_ENABLE] = ModelerControl("Open Light source 0?", 0, 1, 1, 1);
	controls[LIGHTX_0] = ModelerControl("Light0 X Position", 0, 8, 0.1f, 4);
	controls[LIGHTY_0] = ModelerControl("Light0 Y Position", -2, 6, 0.1f, 2);
	controls[LIGHTZ_0] = ModelerControl("Light0 Z Position", -8, 0, 0.1f,-4);

	controls[LIGHT1_ENABLE] = ModelerControl("Open Light source 1?", 0, 1, 1, 1);
	controls[LIGHTX_1] = ModelerControl("Light1 X Position", -6, 2, 0.1f, -2);
	controls[LIGHTY_1] = ModelerControl("Light1 Y Position", -3, 5, 0.1f, 1);
	controls[LIGHTZ_1] = ModelerControl("Light1 Z Position", 0, 10, 0.1f, 5);

	controls[ROTATE_ALL] = ModelerControl("Rotate All", -180, 180, 1, 0);

	controls[NECK_PITCH] = ModelerControl("Neck Pitch", -45, 45, 1, 0);
	controls[NECK_YAW] = ModelerControl("Neck Yaw", -45, 45, 1, 0);
	controls[NECK_ROLL] = ModelerControl("Neck Roll", -45, 45, 1, 0);

	controls[HEAD_PITCH] = ModelerControl("Head Pitch", -45, 45, 1, 0);
	controls[HEAD_YAW] = ModelerControl("Head Yaw", -30, 30, 1, 0);
	controls[HEAD_ROLL] = ModelerControl("Head Roll", -30, 30, 1, 0);

	controls[LEFT_FORELIMP_1] = ModelerControl("Left Fore Thigh", -30, 30, 1, 0);
	controls[RIGHT_FORELIMP_1] = ModelerControl("Right Fore Thigh", -30, 30, 1, 0);
	controls[LEFT_REARLIMP_1] = ModelerControl("Left Rear Thigh", -30, 30, 1, 0);
	controls[RIGHT_REARLIMP_1] = ModelerControl("Right Rear Thigh", -30, 30, 1, 0);

	controls[LEFT_FORELIMP_2] = ModelerControl("Left Fore Limp1 Pitch", -30, 30, 1, 0);
	controls[RIGHT_FORELIMP_2] = ModelerControl("Right Fore Limp1 Pitch", -30, 30, 1, 0);
	controls[LEFT_REARLIMP_2] = ModelerControl("Left Rear Limp1 Pitch", -30, 30, 1, 0);
	controls[RIGHT_REARLIMP_2] = ModelerControl("Right Rear Limp1 Pitch", -30, 30, 1, 0);

	controls[LEFT_FORELIMP_2_YAW] = ModelerControl("Left Fore Limp1 Yaw", -10, 10, 1, 0);
	controls[RIGHT_FORELIMP_2_YAW] = ModelerControl("Right Fore Limp1 Yaw", -10, 10, 1, 0);
	controls[LEFT_REARLIMP_2_YAW] = ModelerControl("Left Rear Limp1 Yaw", -10, 10, 1, 0);
	controls[RIGHT_REARLIMP_2_YAW] = ModelerControl("Right Rear Limp1 Yaw", -10, 10, 1, 0);

	controls[LEFT_FORELIMP_3] = ModelerControl("Left Fore Limp2", -30, 30, 1, 0);
	controls[RIGHT_FORELIMP_3] = ModelerControl("Right Fore Limp2", -30, 30, 1, 0);
	controls[LEFT_REARLIMP_3] = ModelerControl("Left Rear Limp2", -30, 30, 1, 0);
	controls[RIGHT_REARLIMP_3] = ModelerControl("Right Rear Limp2", -30, 30, 1, 0);

	controls[TAIL_PITCH] = ModelerControl("Tail Pitch", -30, 30, 1, 0);
	controls[TAIL_YAW] = ModelerControl("Tail Yaw", -15, 15, 1, 0);

    ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
