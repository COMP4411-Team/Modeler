#ifndef _MODELER_GLOBALS_H
#define _MODELER_GLOBALS_H

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif

// This is a list of the controls for the SampleModel
// We'll use these constants to access the values 
// of the controls from the user interface.
enum SampleModelControls
{ 
	XPOS, YPOS, ZPOS, ROTATE_ALL,
	LIGHT0_ENABLE, LIGHT1_ENABLE,
	LIGHTX_0, LIGHTY_0, LIGHTZ_0,
	LIGHTX_1, LIGHTY_1, LIGHTZ_1,
	NECK_PITCH, NECK_YAW, NECK_ROLL,
	HEAD_PITCH, HEAD_YAW, HEAD_ROLL,
	LEFT_FORELIMP_1, RIGHT_FORELIMP_1, LEFT_REARLIMP_1, RIGHT_REARLIMP_1, // the four thighs
	LEFT_FORELIMP_2, RIGHT_FORELIMP_2, LEFT_REARLIMP_2, RIGHT_REARLIMP_2,
	LEFT_FORELIMP_2_YAW, RIGHT_FORELIMP_2_YAW, LEFT_REARLIMP_2_YAW, RIGHT_REARLIMP_2_YAW,
	LEFT_FORELIMP_3, RIGHT_FORELIMP_3, LEFT_REARLIMP_3, RIGHT_REARLIMP_3,
	TAIL_PITCH, TAIL_YAW,
	NUMCONTROLS
};

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))

#endif