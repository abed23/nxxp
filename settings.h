/**
 *	Settings.h
 *
 *	Stores program settings and global variables/objects. 
 *
*/

#include "TFC.h"													// Shield library and mbed OS	


/* ------------------------------- *
	HARDWARE SETUP VALUES
 * ------------------------------- */
#define MIN_AIN				0										// Min analog input value
#define MAX_AIN				4096									// Max analog input value
#define CAM_DATA_LEN 		128										// Camera pixels
#define NUM_TFC_TICKERS		4										// Number of tickers
#define TICKER_INTERVAL_US	2000									// Ticker interval us (scan interval)

/* ------------------------------- *
	SETTINGS
 * ------------------------------- */
#define RACE_MODE_TICKS		20										// Race mode update delay
#define	MOTOR_OFF			0										// No motor
#define COM_DEMO			0										// 0 - switch setup for servo calibration, 1 - COM port servo

/* ------------------------------- *
	CALCULATION CONSTANTS
 * ------------------------------- */
#define PI				3.14159f
#define EXP				2.71828f
#define CONST_MULT		0.03978f

/* ------------------------------- *
	DEBUG VIA COM PORT PRINTS
 * ------------------------------- */
// State
#define PRINT_STATE			0										// Show system state

// Camera
#define	SNAP_RAW_STREAM		1										// Raw data stream from the camera input
#define	SNAP_GAUSSIAN_FLT	2										// Gaussian filtered snapshot
#define	SNAP_GRADIENT_FLT	3										// Gradient snapshot
#define SNAP_NON_MAX_SUP	4										// Non-max supression snapshot
#define PRINT_SNAPSHOT		0										// Print snapshot

// Lines
#define PRINT_EDGES_VIS		0										// Edge printout
#define PRINT_EDGES_DET		0										// Value and pos of edge 
#define PRINT_LINE_VIS		0										// Edge printout
#define PRINT_PROCESSING	0										// show line position
#define PRINT_PID			0										// Print what car will do next


/* ------------------------------- *
	SPEED SETUP
 * ------------------------------- */
#define SPEED_MODE			2										// Speed mode selection
#define SPEED_SLOW			0.35									// Testing purposes
#define SPEED_MIN			0.3										// Slow speed for race
#define SPEED_MAX			0.55									// Risky TOP SPEED
#define RELATIVE_MIN_ERR	10										// Minimum error thres for relative wheel speed -/+ pix10
#define PID_DATA_LEN		CAM_DATA_LEN - 4						// Take away filtered out pixels

/* ------------------------------- *
	STEERING SETUP
 * ------------------------------- */
#define MAX_TURN_LEFT		-0.54									// Max left turn to avoid blocking wheels
#define MAX_TURN_RIGHT		0.36									// Max right turn to avoid blocking wheels
#define SERVO_ERROR			-0.09									// Servo error - center wheel position
#define	CENTER_POS			0										// 0.0
#define CENTER_CAMERA		63										// int midpoint
#define	MIN_NO_LINE			0										// If cannot find line in 3 tries = 60mS, pos get updated to 0.0
#define MAX_NO_LINE			30										// Max no line statuses before stopping the car - out of the track
#define ERROR_BUFF_LEN		3										//  Remember last 100ms to prevent u turn fallouts

/* ------------------------------- *
	POSITION SETUP
 * ------------------------------- */
#define MAX_EDGES						6							// Max number of edges of each type
#define MAX_WIDTH						12							// Max line width
#define MIN_WIDTH						4							// Min line width
#define CALCULATE_THRESHOLDS			0							// Calculate gradient thresholds on each scan
#define FIXED_GRADIENT_THRESHOLD		100							// Use fixed value of gradient threshold 


/** 
* 	Establish serial comm with usb
*   USBTX = PTA2,
*   USBRX = PTA1,
*	The baudrate of the serial port (default = 9600).
*	format(int bits=8, Parity parity=Serial::None, int stop_bits=1);
*/
static Serial pc(USBTX,USBRX);	

//State
typedef enum {
	LOST,
	LINE,
	LINE_LEFT,
	LINE_RIGHT,
	BOTH_LINES, 													// not possible with this setup , still considered here
	STOP
} TrackState_t;
