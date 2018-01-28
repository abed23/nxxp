/*
 *	steering.cpp
 *
 *	Contains funcitons reponsible for servo operation 
 */
 
#include "settings.h"	

 
 
// Clip lower/higher outputs to max settings of servo calibration
void updateServo(float SteeringOutput) {
	
	if (SteeringOutput < MAX_TURN_LEFT)
		SteeringOutput = MAX_TURN_LEFT;
	else if ( SteeringOutput > MAX_TURN_RIGHT)
		SteeringOutput = MAX_TURN_RIGHT;
	
	TFC_SetServo(0, SteeringOutput);
	
}

