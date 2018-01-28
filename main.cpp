#include "settings.h"												// Car settings
#include "improc.h"													// Image processing library	
#include "speed.h"													// Speed library
#include "steering.h"												// Steering library

// Edges
static uint8_t    		negEdges[MAX_EDGES]={0};					// Index of position storing negative edge
static uint8_t	 		posEdges[MAX_EDGES]={0};					// Index of central position of positiove edge
static TrackState_t		CarState;									// Stores state of the car
static uint16_t 		lostTrack;	

// Line position
static float 			CurrentLinePosition(0);
static float 			OldLinePosition(0);

// PID ERRORS
static float 			CurrentPosError(0);
static float 			OldPosError(0);
static float 			IntegralError(0);
static float 			DerivativeError(0);
static float 			OutputPID(SERVO_ERROR);


// Define operation modes
enum MODE {
	MODE_IDLE,							// 0 - Idle/incorrect mode
	MODE_CHECK_STEERING,				// 1 - Check and calibrate servo
	MODE_CHECK_SPEED,					// 2 - Speed check
	MODE_CHECK_CAMERA,					// 3 - Camera data stream
	MODE_RUN							// 4 - Race mode
};


// Control of the car position error
float		positionLine;											// Store position of the line
float 		positionLineTemp;										// Helper for floor visible case
float		positionLineOld;										// Store previous position of the line
float 		positionLineError;										// Error
float 		positionLineErrorOld;									// Error Old
float 		positionLineErrorAvg;									// Avg error to improve u turn detection
float		positionLineErrorSign;
float		SteeringOutput(SERVO_ERROR);							// Wheels straight on each initialization
float 		positionErrorBuffer[ERROR_BUFF_LEN];					// error buffer to improve detection of u-turns
uint16_t	noLineCnt(0);
float 		manSteer(0);
float 		manMot(0);
int16_t 	motorTest(0);


// Main variables
uint8_t 			t(0);
uint8_t 			dipswitch(0);
uint8_t 			dipswitch_old(0);
static bool 		transition(1);		
static bool			offTheTrack(0);
int16_t				cameraSnapshotPeaks[CAM_DATA_LEN];				// Peak values filtered with threshold
static float 		bestLeft(0);
static float 		bestRight(0);

// Ticker init
Ticker TFC_Clock;	

//	Array of tickers to use within a program
volatile uint32_t TFC_Ticker[NUM_TFC_TICKERS];


void lightLine() {
	
	switch (CarState) {
		
		case LINE_LEFT:
			TFC_BAT_LED0_ON;
			TFC_BAT_LED1_OFF;
			TFC_BAT_LED2_OFF;
			TFC_BAT_LED3_OFF;
		break;
		
		case LINE_RIGHT:
			TFC_BAT_LED0_OFF;
			TFC_BAT_LED1_OFF;
			TFC_BAT_LED2_OFF;
			TFC_BAT_LED3_ON;
		break;
		
		case LINE:
			if (bestLeft == 0  && bestRight != 127) {
				(bestRight < 33) ? TFC_BAT_LED0_ON : TFC_BAT_LED0_OFF;
				(bestRight < 64) ? TFC_BAT_LED1_ON : TFC_BAT_LED1_OFF;
				TFC_BAT_LED2_ON;
				TFC_BAT_LED3_ON;
			}
			else if (bestRight == 127 && bestLeft > 0) {
				TFC_BAT_LED0_ON;
				TFC_BAT_LED1_ON;
				(bestLeft > 63) ? TFC_BAT_LED2_ON : TFC_BAT_LED2_OFF;
				(bestLeft > 95) ? TFC_BAT_LED3_ON : TFC_BAT_LED3_OFF;
			}
		break;
		
		default:
			TFC_BAT_LED0_OFF;
			TFC_BAT_LED1_OFF;
			TFC_BAT_LED2_OFF;
			TFC_BAT_LED3_OFF;
		break;
		
	}
}

void pid() {
	
	// Target pos
	float dleft(0), dright(0);
	float target = 0;  // we want to stick to the left a bit
	
	
	// Gains
	// KP = MOVEMENT_SPAN / PIXELS AVAILABLE
	// MIN width is minimum possible non-zero (line pos will never be less than min width so it cuts pixels too)
	float kP = (float) ( MAX_TURN_RIGHT - MAX_TURN_LEFT ) / ( CAM_DATA_LEN - MIN_WIDTH );
	float kI = 0; //KI_TUN;
	float kD = 0; //KD_TUN;
	
	kP = kP * abs(TFC_ReadPot(0) * 2.5);
	kD = 0 ;
	if (PRINT_PID) pc.printf("KP:\t%3.3f \tKD: \t %3.3f \r\n", abs(TFC_ReadPot(0) * 2.5), kD); 
	
	
	// Calculate the distance between two lines
	if (CarState == LINE_LEFT) {
		bestLeft = CurrentLinePosition;
		bestRight = CAM_DATA_LEN - 1;
		CurrentPosError =  -CurrentLinePosition;
		if (PRINT_PID) pc.printf("New best left:\t%3.3f \r\n", bestLeft);
	} else if (CarState == LINE_RIGHT) {
		bestRight = CurrentLinePosition;
		bestLeft = 0;
		CurrentPosError = (CAM_DATA_LEN - CurrentLinePosition);
		if (PRINT_PID) pc.printf("New best right:\t%3.3f \r\n", bestRight);
	}
	else if (CarState == LINE) {
		
		if (bestLeft == bestRight) {
			if (CurrentLinePosition > 63.5) {
				bestRight = CurrentLinePosition;
			} else {
				bestLeft = CurrentLinePosition;
				bestRight = CAM_DATA_LEN-1;
			}
			if (PRINT_PID) pc.printf("Select \tLeft:\t%3.3f \tRight:\t%3.3f \r\n", bestLeft, bestRight);
		}
		
		// Calculate distances before out best known positions
		dleft = abs(CurrentLinePosition - bestLeft);
		dright = abs(CurrentLinePosition - bestRight);
		if (PRINT_PID) pc.printf("Select2 \tdLeft:\t%3.3f \tdRight:\t%3.3f \r\n", dleft, dright);
		
		// Last known right line seem to be closer to newly found line,
		// because left line is further from the llast known position.
		// We can assume that the line is still on the right
		if ( dleft > dright) {
			bestRight = CurrentLinePosition;
			bestLeft = 0;
			CurrentPosError = (CAM_DATA_LEN - CurrentLinePosition);
		} 
		// ifl left is a better choice, take it!
		else if (dright > dleft) {
			bestLeft = CurrentLinePosition;
			bestRight = CAM_DATA_LEN -1;
			CurrentPosError =  -CurrentLinePosition;
		}
	} else {
				
		//LPF
		CurrentPosError = 0.9 * OldPosError + (1 - 0.9)*CurrentPosError;	
	}
	
	CurrentPosError = CurrentPosError - target;
	
	if (PRINT_PID) pc.printf("Current pos error:\t%3.3f \tCarstate:\t%d \r\n", CurrentPosError, CarState);

	// Integral Error
	// 0.02 is derivation time - 20ms (0.02s)
	// Integral error rise when line on right
	// falls when life on the left side
	IntegralError = IntegralError + ( CurrentPosError * 0.02);
	
	// Derivative Error
	DerivativeError = (CurrentPosError - OldPosError) * 0.02;
	
	// PID output
	// Includes servo mounting error correction
	OutputPID = kP*CurrentPosError /*+ kI*IntegralError */ + kD*DerivativeError + SERVO_ERROR;
	
	// Store old line pos
	OldPosError = CurrentPosError;
	
	if (PRINT_PID)
		pc.printf("IN:\t%3.3f\t ERR:\t%3.3f\tDeriv:\t%5.3f\tOut:\t%3.3f\t\r\n",CurrentLinePosition ,CurrentPosError, DerivativeError, OutputPID);
	
}

// Increment tickers of each clock cycle
void updateTickers() {
    for(int i=0; i<NUM_TFC_TICKERS; i++)
        if(TFC_Ticker[i]<0xFFFFFFFF) TFC_Ticker[i]++;
}


// Reset system parameters 
void resetAll() {
	TFC_HBRIDGE_DISABLE;    					// Motor OFF
    TFC_SetMotorPWM(0.0, 0.0 ); 				// SP Left/Right Motor = 0.0
    TFC_SetServo(0, SERVO_ERROR);       		// Center servo
	TFC_SetBatteryLED_Level(0);					// Battery leds OFF
	motorTest = 0;								// Reset motor test demo
	lostTrack = 0;
	offTheTrack = 0;
	
	if (dipswitch < 4) 
		TFC_SetBatteryLED_Level(dipswitch);		// Set corresp batt level
	
	if (PRINT_STATE)  {
		switch(dipswitch) {
			case MODE_IDLE:
				pc.printf("[MODE][%d]: IDLE\r\n", dipswitch);
			break;
			
			case MODE_CHECK_STEERING:
				pc.printf("[MODE][%d]: CHECK STEERING\r\n", dipswitch);
			break;
			
			case MODE_CHECK_SPEED:
				pc.printf("[MODE][%d]: CHECK SPEED\r\n", dipswitch);
			break;
					
			case MODE_CHECK_CAMERA:
				pc.printf("[MODE][%d]: CHECK CAMERA\r\n", dipswitch);
			break;
			
			case MODE_RUN:
				pc.printf("[MODE][%d]: CHECK CAMERA\r\n", dipswitch);
			break;
		}
	}
	
}

// Enables com parameter adjustment via pc terminal
void comParam( float* val, float min, float max ) {
	float temp(*val);
	char c;
	do {
		c = pc.getc();
		switch (c) {
			case '+':
				(temp+0.05 > max) ? temp=max : temp+=0.05;
			break;
			
			case '-':
				(temp-0.05 < min) ? temp=min : temp-=0.05;
			break;
			
			case 'c':
				temp = (min+max)/2;
			break;
			
			case 'x':
				pc.printf("Operation cancelled.\r\n");
			break;
						
			case 's':
				*val = temp; 
			break;
			
			default:
				pc.printf("Incorrect key input.\r\n");
			break;
		}
		
		if (c != 'x' ) pc.printf("Value updated:\t%1.2f \r\n", temp);
	} while (c!='s' && c != 'x');
	
	
}

// Main program 
int main() {
		// Initialize helper library
		// for shield used in the project
		// pc.baud(115200 - if need more data fluency
		TFC_Init();
	
		// Attach clock to TFC_TickerUpdate function
		// so the function gets executed on each clock update
		// effectively updating the ticker array
		// 20ms ticks
		TFC_Clock.attach_us(&updateTickers, TICKER_INTERVAL_US);
	
		// Reset system params before loop
		resetAll();
	
		// Generate gaussian distribution kernel
		computeKernel();

    for(;;) {      
		// Remember previous state
		dipswitch_old = dipswitch;
	
		// Get dipswitch configuration
		dipswitch = TFC_GetDIP_Switch() & 0x07;
		
		// Reset all parameters on state transition
		(dipswitch_old != dipswitch) ? transition = 1 : transition = 0;
	
		// 2 dipswitches are defining the system
		switch(dipswitch) {
			
			// SERVO DEMONSTRATION VIA COM AND SERVO CALIBRATION
			case MODE_CHECK_STEERING:
				if(TFC_Ticker[0] >= 10) { 	// 20ms scan intervals
					TFC_Ticker[0] = 0;
				
					// Update system state and reset
					if (transition) 
						resetAll();
					
					// COM port demonstration
					if (COM_DEMO) {
						
						// Welcome instruction
						pc.printf("Adjust servo value using '+' and '-', then press\r\n 's' to save or 'c' to center and 'x' to exit:\r\n");
						
						// UI handling
						comParam(&manSteer, MAX_TURN_LEFT, MAX_TURN_RIGHT );
						
						// Update the value
						updateServo(manSteer);
					} else {
						TFC_SetServo(0,TFC_ReadPot(0));
						pc.printf("[SERVO(pot)]: %1.2f\r\n", TFC_ReadPot(0));
					}		
				}
			
			break;
				
			// SPEED DEMONSTRATION VIA COM
			case MODE_CHECK_SPEED:	
				if(TFC_Ticker[1] >= 10) { 	// 20ms scan intervals
					TFC_Ticker[1] = 0;
						
					if (transition) 
						resetAll();
						
					if (motorTest <= 0 && COM_DEMO ) {
						
						// Reset motors 
						TFC_HBRIDGE_DISABLE;
						
						// Welcome instruction
						pc.printf("Adjust speed value using '+' and '-', then press\r\n 's' to start or 'c' to reset and 'x' to exit:\r\n");
						
						// UI handling
						comParam(&manMot, -1.0f , 1.0f );
						TFC_SetMotorPWM(manMot, manMot);
						
						// Ask to start
						pc.printf("Press 's' to start the drive test (2s) or 'x' to exit:\r\n");
						char c = pc.getc();
						
						// Set timer
						if (c == 's') 
							motorTest = 100;
						
					} else {
							TFC_HBRIDGE_ENABLE;
							motorTest--;
					}							

			}						
			break;
				
			// CAMERA DATA DEMONSTRATION VIA COM
			case MODE_CHECK_CAMERA:
				if(TFC_Ticker[2] >= 10 && TFC_LineScanImageReady>0) {	// 20ms scan intervals
					
					// Reset ticker and flags
					TFC_Ticker[2] = 0;
					TFC_LineScanImageReady=0;
					
					if (transition)
						resetAll();
					
					printCamera(&TFC_LineScanImage0[0], 0, 127);
				}							   	
			break;
						
			// RUN MODE
			case MODE_RUN:
				if (TFC_LineScanImageReady > 0 && TFC_Ticker[3] > RACE_MODE_TICKS ) {	// Intervals 40/50ms [20-25 ticks] are best
					
					TFC_LineScanImageReady=0;
					TFC_Ticker[3] = 0;
					
					if (transition) {
						resetAll();
						offTheTrack = 0;
					}
					// Enable motor operation
					if (!MOTOR_OFF && !offTheTrack) 
						TFC_HBRIDGE_ENABLE;
					else
						TFC_HBRIDGE_DISABLE;
					
					// Processes camera data and calclates vehicle's position
					// in relation to the track center. 
					imageprocessing(&cameraSnapshotPeaks[0]);
					
					// Detect edges
					detectEdges(&cameraSnapshotPeaks[0], &posEdges[0], &negEdges[0]);
					
					// Process edges
					process_edges(&CarState, &CurrentLinePosition, &OldLinePosition, &posEdges[0], &negEdges[0], &OldPosError);

					// Set servo output with kP controller based on servo error and position
					pid();
					
					// Update servo (servo is mounted the opposite way so it's negative error),L<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll:????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>?>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
					updateServo(-OutputPID);
					
					// Update speed
					updateSpeed(positionLineError);
					
					// Light signal for line detection status
					lightLine();
					
					if (CarState == LOST)
						lostTrack++;
					else 
						lostTrack = 0;
					
					if (lostTrack > 80)
						offTheTrack = 1;
					
				}
			
			break;
					
			// IDLE MODE
			default:	
				if (TFC_Ticker[0] >= 10) {
					
					if (transition)
						resetAll();
					
					TFC_Ticker[0] = 0;
					t >= 4 ? t=0 : t++;
					TFC_SetBatteryLED_Level(t);
				}	
			break;
		}
    }
    
}
 
