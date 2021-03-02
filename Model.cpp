// The sample model.  You should build a file
// very similar to this for when you make your model.
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

const aiScene* scene;

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

void drawTriangle(const aiVector3D* v1, const aiVector3D* v2, const aiVector3D* v3)
{
	drawTriangle(v1->x, v1->y, v1->z, v2->x, v2->y, v2->z, v3->x, v3->y, v3->z);
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
	glScaled(0.01, 0.01, 0.01);
	const aiMesh* mesh = scene->mMeshes[0];

	for (int i = 0; i < mesh->mNumBones; ++i)
	{
		auto* bone = *(mesh->mBones + i);
		auto name = bone->mName;
		for (int j = 0; j < bone->mNumWeights; ++j)
		{
			auto* weight = bone->mWeights + j;
		}
	}

	
	for (int i = 0; i < mesh->mNumFaces; ++i)
	{
		const aiFace& face = mesh->mFaces[i];
		const aiVector3D* vertices[3];
		for (int j = 0; j < 3; ++j)
		{
			auto idx = face.mIndices[j];
			vertices[j] = &(mesh->mVertices[idx]);
		}
		drawTriangle(vertices[0], vertices[1], vertices[2]);
	}
	
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
	Assimp::Importer importer;
	scene = importer.ReadFile("./models/lowpolydeer.dae", 
		aiProcess_CalcTangentSpace |
		aiProcess_Triangulate |
		aiProcess_JoinIdenticalVertices |
		aiProcess_SortByPType);
	std::cout << "import done\nmeshes: " << scene->mNumMeshes << std::endl;
	std::cout << "bones: " << scene->mMeshes[0]->mNumBones << std::endl;
	
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
