/*
 *	speed.cpp
 *
 *	Contains funcitons reponsible for speed method selection and calculation
 */
 
 #include "settings.h"	

/** Calculates speed output in relation to position error
 *
 *  For low error values, the speed is set to the maximum value defined in settings.h,
 *  otherwise it is adjusted proportionally to the error value. 
 *
 *  @param LeftWheelSpeed pointer
 *  @param RightWheelSpeed pointer
 *  @param Error passed by value to avoid double pointer and possible memory access issues
 */
void updateSpeedRelative(float* SpeedOutputLeft, float* SpeedOutputRight, float positionLineError ) {
	
		// Car turns left, set right wheel to be faster and right slower
		// to improve turning angle with additional wheelpower
		// position error should be lower than negative relative error threshold
		if (positionLineError < (RELATIVE_MIN_ERR * -1.0)) {
			*SpeedOutputLeft = SPEED_MAX - (SPEED_MAX - SPEED_MIN) * ( (float) positionLineError * 2 / CAM_DATA_LEN );
			*SpeedOutputRight = SPEED_MIN;
		} 
		// Car turns right, set left wheel to be slower and right faster
		// to improve turning angle with additional wheelpower
		// position error should be higher than positive relative error threshold
		else if (positionLineError > RELATIVE_MIN_ERR) {    
			*SpeedOutputLeft = SPEED_MIN;
			*SpeedOutputRight = SPEED_MAX - (SPEED_MAX - SPEED_MIN) * ( (float) positionLineError * 2 / CAM_DATA_LEN );
		}
		// If error is withing -/+ range of relative error threshold, full speed to improve
		// lap time on the track
		else {
			*SpeedOutputLeft = SPEED_MAX;
			*SpeedOutputRight = SPEED_MAX;
		}
}



/** Selects desired calculation method 
 * 	for cars' speed output(SPEED_MODE)
 *
 *  @param LeftWheelSpeed pointer
 *  @param RightWheelSpeed pointer
 *  @param Error variable pointer
 */
void updateSpeed(float positionLineError) {
	 
	float SpeedOutputLeft(0);			// Left motor speed
	float SpeedOutputRight(0);			// Right motor speed
	 
	 // Update speed
	switch (SPEED_MODE) {
		
		// Crazy fast mode - risky
		case 0:
			SpeedOutputLeft = SPEED_MAX;
			SpeedOutputRight = SPEED_MAX;
		break;
		
		// Slow mode
		case 1: 
			SpeedOutputLeft = SPEED_MIN;
			SpeedOutputRight = SPEED_MIN;
		break;
		
		// Relative speed mode
		case 2:
			updateSpeedRelative(&SpeedOutputLeft, &SpeedOutputRight, positionLineError );
		break;
	}
	
	// Set values
	TFC_SetMotorPWM(SpeedOutputLeft, SpeedOutputRight);
 }
 








