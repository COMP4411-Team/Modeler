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

using namespace std;
using namespace Assimp;
using Matrix4f = aiMatrix4x4t<float>;

ModelHelper helper;		// simply use global variable for now
Matrix4f global_inverse;


// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView 
{
public:
    SampleModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) { }

    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label)
{ 
    return new SampleModel(x,y,w,h,label); 
}


void drawTriangle(const aiVector3D& v1, const aiVector3D& v2, const aiVector3D& v3)
{
	drawTriangle(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
}

void drawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
{
	drawTriangle(v1.world_pos, v2.world_pos, v3.world_pos);
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
		auto& vertices = mesh.vertices;
		int v1 = face.mIndices[0];
		int v2 = face.mIndices[1];
		int v3 = face.mIndices[2];
		
		drawTriangle(vertices[v1], vertices[v2], vertices[v3]);
	}
}


// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{	
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();

	/*
	const auto* glVersion = glGetString(GL_VERSION);
	const auto* glRenderer = glGetString(GL_RENDERER);
	const auto* glVendor = glGetString(GL_VENDOR);
	const auto* gluVersion = glGetString(GLU_VERSION);
	printf("version %s\n", glVersion);
	printf("renderer %s\n", glRenderer);
	printf("vendor %s\n", glVendor);
	printf("glu version %s\n", gluVersion);
	 */

	// draw the floor
	setAmbientColor(0.75f, 0.75f, 0.75f);
	setDiffuseColor(0.75f, 0.75f, 0.75f);
	glScaled(1.0, 1.0, 1.0);

	auto& mesh = helper.meshes[0];
	auto* scene = helper.scene;
	global_inverse = scene->mRootNode->mTransformation.Inverse();


	// Test the transformations
	//mesh.restoreIdentity("main");
	//mesh.applyRotationX("main", VAL(ROTATE));

	mesh.restoreIdentity("neck");
	mesh.applyRotationZ("neck", VAL(ROTATE));
	
	traverseBoneHierarchy(mesh, scene->mRootNode, Matrix4f());
	processVertices(mesh);
	renderMesh(mesh);
	
	//glPushMatrix();
	//glTranslated(-5,0,-5);
	//drawBox(10,0.01f,10);
	//glPopMatrix();

	//// draw the sample model
	//setAmbientColor(.1f,.1f,.1f);
	//setDiffuseColor(COLOR_GREEN);
	//glPushMatrix();
	//glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));

	//	glPushMatrix();
	//	glTranslated(-1.5, 0, -2);
	//	glScaled(3, 1, 4);
	//	drawBox(1,1,1);
	//	glPopMatrix();

	//	// draw cannon
	//	glPushMatrix();
	//	glRotated(VAL(ROTATE), 0.0, 1.0, 0.0);
	//	glRotated(-90, 1.0, 0.0, 0.0);
	//	drawCylinder(VAL(HEIGHT), 0.1, 0.1);

	//	glTranslated(0.0, 0.0, VAL(HEIGHT));
	//	drawCylinder(1, 1.0, 0.9);

	//	glTranslated(0.0, 0.0, 0.5);
	//	glRotated(90, 1.0, 0.0, 0.0);
	//	drawCylinder(4, 0.1, 0.2);
	//	glPopMatrix();

	//glPopMatrix();
}

int main()
{
	// Test assimp
	helper.loadModel("./models/lowpolydeer.dae");

	auto* scene = helper.scene;
	std::cout << "import done\nmeshes: " << scene->mNumMeshes << std::endl;
	std::cout << "bones: " << scene->mMeshes[0]->mNumBones << std::endl;
	helper.meshes[0].printBoneHierarchy(scene->mRootNode, 0);	// print out the bones
	
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
    controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
    controls[YPOS] = ModelerControl("Y Position", 0, 5, 0.1f, 0);
    controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);
    controls[HEIGHT] = ModelerControl("Height", 1, 2.5, 0.1f, 1);
	controls[ROTATE] = ModelerControl("Rotate", -135, 135, 1, 0);

    ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
