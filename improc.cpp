/*
 *	improc.cpp
 *
 *	Image processing and position detection
 *
 *	Contains image processing functionality to improve edge detection and reduce noise/error from the camera,
 *  then processes data to detect car position in relation to the center of the race track.
 *	Author: Hubert
 */

 #include "settings.h"												// Car settings


// Kernel array calculation for gaussian blur
#define KERNEL_SIZE		9											// Kernel size has to be ODD!!!
#define SIGMA			2.0f										// f is added to make sure no promotion to double happens
#define SIDEBAND		(KERNEL_SIZE - 1) /2						// 1/(2*PI*SIGMA^2) - this is always constant
#define SCALING			5											// Scale smoothened data back to orig values (1/kernelArray[center] = scaling factor

// Static variables used for pos detection
static float 			kernelArray[KERNEL_SIZE]= 	{0};			// Array to store generated kernel values
static uint16_t 		negEdgesCnt(0);								// Negative edges found
static uint16_t 		posEdgesCnt(0);								// Positive edges found
static float 			thresholdNegEdge(0);						// Threshold neg edge
static float 			thresholdPosEdge(0);						// Threshold pos edge
static float 			LinePositionBuffer[MAX_EDGES]={0};				// Line buffer
static uint8_t			LineCnt(0);

extern uint8_t			posEdges[MAX_EDGES];
extern uint8_t			negEdges[MAX_EDGES];

//extern float 			CurrentLinePosition;
//extern float 			OldLinePosition;
//extern TrackState_t 	CarState;

/** Compute gaussian blur kernel
 *	kernel size has to be ODD!
 */
	uint8_t i;
void computeKernel() {

	for(i = 0; i <= SIDEBAND; i++)
		kernelArray[i + SIDEBAND] = CONST_MULT*(1.0f/SIGMA)*pow(EXP,(-pow((i),2.0f))/(2.0f*pow(SIGMA,2.0f)));

	for(i = 0; i < SIDEBAND; i++)
		kernelArray[i] = kernelArray[KERNEL_SIZE - 1 - i];
}

/** Prints camera data (uint16_t)
 *  in form of Python plotted code.
 * 
 *	@param pointer to first row of data array 
 *  @param first pixel number
 *  @param last pixes number
*/
void printCamera(uint16_t * data, uint16_t start, uint16_t end) {
	
	pc.printf("\r\n");
	pc.printf("L:");
		
	// Camera values to Python via COM
	for(uint16_t i=start;i<end+1;i++) {
		if(i==end)
			pc.printf("%u\r\n", data[i]);
		 else
			pc.printf("%u,", data[i]);
	 }

}

/** Prints camera data (uint16_t)
 *  in form of Python plotted code.
 * 
 *	@param pointer to first row of data array 
 *  @param first pixel number
 *  @param last pixes number
*/
void printCamera(volatile uint16_t * data, uint16_t start, uint16_t end) {
	
	pc.printf("\r\n");
	pc.printf("L:");
		
	// Camera values to Python via COM
	for(uint16_t i=start;i<end+1;i++) {
		if(i==end)
			pc.printf("%u\r\n", data[i]);
		 else
			pc.printf("%u,", data[i]);
	 }

}

/** Prints camera data (int16_t)
 *  in form of Python plotted code.
 * 
 *	@param pointer to first row of data array 
 *  @param first pixel number
 *  @param last pixes number
*/
void printCameraInt(int16_t * data, uint16_t start, int16_t end) {
	
	pc.printf("\r\n");
	pc.printf("L:");
		
	// Camera values to Python via COM
	for(uint16_t i=start;i<end+1;i++) {
		if(i==end)
			pc.printf("%d\r\n", data[i]);
		 else
			pc.printf("%d,", data[i]);
	 }

}

/** Applies gaussian filter on the raw camera image
 *
 *  @param rawData pointer to raw camera data array
 *  @param dataOut pointer to the output array
 */
void gaussianBlur(volatile uint16_t* cameraRaw, uint16_t* dataOut) {

	uint32_t i, j, start, end, off;

	for (i=0; i < CAM_DATA_LEN; i++) {

		// Reset previous value
		dataOut[i] = 0;

		// Start index
		if (i >= SIDEBAND)
			start = i - SIDEBAND;
		else
			start = 0;

		// End index
		if (i <= CAM_DATA_LEN - 1 - SIDEBAND)
			end =  i + SIDEBAND;
		else
			end = CAM_DATA_LEN - 1;

		// Offset
		if(i <= CAM_DATA_LEN - 1 - SIDEBAND)
			off = KERNEL_SIZE - (end - start + 1);
		else
			off = 0;

		// Iterate from start to end neighbouring pixels to find gaussian blur value for the i pixel of the raw camera image,
		// calculate the value and divide by scaling factor used in kernel generation process to maintain proportion
		for (j = start; j <= end; j++)
			dataOut[i] += cameraRaw[j] * kernelArray[j - start + off];

		dataOut[i] = dataOut[i] * SCALING;
	}
}

/** Applies gradient filter on camera image
 *
 *  @param rawData pointer to raw camera data array
 *	@param lenght lenght of the camera data (128 by default)
 *  @param dataOut pointer to the output array
 */
void gradientFilter(const uint16_t* blurred , int16_t* dataOut) {

	float 		upperGV				(MAX_AIN);						// Upper neighbour
	float 		lowerGV				(MIN_AIN);						// Lower neightbour
	uint32_t	i;													// Iterator

	for (i=0; i < CAM_DATA_LEN ; i++)
	{
		// First and end pixels cannot provide gradient (1 neighbour only)
		if (i != 0 || i != (CAM_DATA_LEN  - 1)) {
			upperGV = (float) blurred[i+1];
			lowerGV = (float) blurred[i-1];
			dataOut[i] =  (upperGV - lowerGV) / 2;
		} 			
	}
	
	dataOut[0] =  dataOut[CAM_DATA_LEN  - 1] = 0;
	
}

/** Calculate derivative thresholds on gradient image
 *
 *  @param gradientArray const pointer to gradient filtered camera data
 *  @param dataOut pointer to the output array
 */
void findThresholds(const int16_t* gradientArray) {

	float		avgGradient			(0);							// Average gradient value
	float		minGradient			(MAX_AIN);						// Store min gradient of a scan
	float		maxGradient			(MIN_AIN);						// Store max gradient of a scan
	uint8_t 	i;													// Iterator

		// Override calculated value with fixed threshold
	if (CALCULATE_THRESHOLDS) {	
		if (PRINT_EDGES_VIS) pc.printf("[THRESHOLDS]: Neg:\t %3.2f \tPos:\t %3.2f \r\n", thresholdNegEdge,  thresholdPosEdge);


		for (i=0; i < CAM_DATA_LEN ; i++) {

			// Check with global min/max
			if (gradientArray[i] < minGradient)
				minGradient = gradientArray[i];

			if (gradientArray[i] > maxGradient)
				maxGradient = gradientArray[i];

			// Add to the average
			avgGradient += gradientArray[i];

		}

		// Calculate average gradient value and adjust thresholds
		avgGradient = (float) avgGradient / CAM_DATA_LEN;
		thresholdNegEdge = (float) 0.5*(minGradient - avgGradient);
		thresholdPosEdge = (float) 0.5*(maxGradient - avgGradient);
		
	} else {
		thresholdPosEdge = FIXED_GRADIENT_THRESHOLD; 
		thresholdNegEdge = -FIXED_GRADIENT_THRESHOLD;
	}
}

/** Applies Non-maximum supression on camera gradient image
 *	to find peak gradient values.
 *
 *  @param gradientArray const pointer to gradient filtered camera data
 *  @param dataOut pointer to the output array
 */
void nonMaxSupression(const int16_t* gradientArray, int16_t* outputArray) {
	uint32_t i;

	// Iterate all pixels to find peak values
	for (i=0; i < CAM_DATA_LEN; i++) {

		// First and last 2 pixels do not contain any information, used only for calculation
		if ( i < 2  || i > CAM_DATA_LEN-3) {
					outputArray[i] = 0;
        }
		// Middle pixels
		else {
			// Value is greater or equal than neighbours or leq 
			if ((gradientArray[i] >= gradientArray[i-1] && gradientArray[i] >= gradientArray[i+1]) 
				|| (gradientArray[i] <= gradientArray[i-1] && gradientArray[i] <= gradientArray[i+1])) {
					
				outputArray[i] = gradientArray[i];
			
				// In case of duplicate peak, take most outer neg edge and most outer pos edge (largest line)
				if (outputArray[i-1] == outputArray[i] && outputArray[i] < 0 )
					outputArray[i] = 0;
				else if (outputArray[i-1] == outputArray[i] && outputArray[i] > 0 )
					outputArray[i-1] = 0;
				
			} else
				outputArray[i] = 0;
		}
	}
	
	
	
}


/** Detect transition edges
 *	and store them in array-like struct
 *
 *  @param edgesArray pointer to data array with detected edges
 *  @param detectedArray pointer to the output struct that stores all edges
 */
void detectEdges(const int16_t* edgesArray, uint8_t * posEdges, uint8_t * negEdges) {
	
	// Reset counters
	posEdgesCnt = negEdgesCnt = 0;
	
	
	if (PRINT_EDGES_VIS) pc.printf("\r\n\r\n--");
	
	// Remove non relevant edges caused by shadows etc. 
	for ( uint8_t i = 2; i < CAM_DATA_LEN -3; i++) {	
		
		// BLACK-WHITE
		if (edgesArray[i] >= (int16_t) thresholdPosEdge && posEdgesCnt < MAX_EDGES) {	
			posEdges[posEdgesCnt] = i;
			posEdgesCnt++;
			//  Debug print
			if (PRINT_EDGES_VIS) pc.printf("P", edgesArray[i]);
			//if (PRINT_EDGES_DET) pc.printf("[POSITIVE EDGE]:\tPos:\t%u \tVal:\t%d" , i, edgesArray[i]);
		}

		// WHITE-BLACK
		else if (edgesArray[i] <= (int16_t) thresholdNegEdge && negEdgesCnt < MAX_EDGES) {	
			negEdges[negEdgesCnt] = i;
			negEdgesCnt++;
			//  Debug print
			if (PRINT_EDGES_VIS) pc.printf("N", edgesArray[i]);
			//	if (PRINT_EDGES_DET) pc.printf("[NEGATIVE EDGE]:\tPos:\t%u \tVal:\t%d" , i, edgesArray[i]);
		}
		 else {
			if (PRINT_EDGES_VIS) pc.printf("-");
		 }
			 
	}
	if (PRINT_EDGES_VIS) pc.printf("--\r\n");
	// Reveal count of each
	if (PRINT_EDGES_DET) pc.printf( "[EDGES]: Neg. edges:\t%u \tPos. edges:\t%u \r\n", negEdgesCnt, posEdgesCnt);	
}

/** Process edges and find lines
 *  Not supporting multiline siatuation 
 *  Not supporting both edges of track situation
*/
void process_edges(TrackState_t * CarState, float * CurrentLinePosition, float * OldLinePosition, uint8_t * posEdges, uint8_t * negEdges, float * OldPosError) {
	
	*OldLinePosition = *CurrentLinePosition;
	LineCnt = 0;
	
	
	if (PRINT_LINE_VIS) pc.printf("-- %u   %u \r\n", posEdgesCnt , negEdgesCnt);
	
	// Case 1: Single line detected (background correction included)
	if ( posEdgesCnt > 0 && negEdgesCnt > 0 ) {
			
		//Pair every N-P configuration and check proper line width
		for (uint8_t  i = 0; i < negEdgesCnt; i++) {	
			for (uint8_t j = 0; j < posEdgesCnt; j++) {
				if (PRINT_LINE_VIS) pc.printf("-xx- %d  \r\n", posEdges[j] - negEdges[i]);
				if ( (posEdges[j] - negEdges[i]) >= MIN_WIDTH && (posEdges[j] - negEdges[i]) <= MAX_WIDTH ) {
					// Calculate new line position, break (multiline support for both edges needed later
					LinePositionBuffer[LineCnt] = (float) (posEdges[j] + negEdges[i]) / 2;
					LineCnt++;
					*CarState = LINE;
					if (PRINT_LINE_VIS) pc.printf("Line found:\tneg:\t%u \tpos:\t%u \tcnt:\t%u \r\n", i , j, LineCnt);
				}
			}
		}
		
		// Select the most inner line 
		for (uint8_t k = 0; k < LineCnt; k++) {
			if (k==0) {
				*CurrentLinePosition = LinePositionBuffer[k];	
			} else {
				if ( abs(*OldLinePosition - LinePositionBuffer[k]) < abs(*OldLinePosition - LinePositionBuffer[k-1])) {
					*CurrentLinePosition = LinePositionBuffer[k];
			//		pc.printf("Line replaced");
				}
				
			}
		}
	}
	
	// Case 2: Possibly line on the left side
	else if (posEdgesCnt == 1 && negEdgesCnt == 0 && posEdges[posEdgesCnt-1] > MAX_WIDTH/2 && OldPosError > 0 ) {
		*CurrentLinePosition = ( posEdges[posEdgesCnt-1] - (MAX_WIDTH / 2) );
		//pc.printf("left");
		*CarState = LINE_LEFT;
	}
	
	// Case 3: Possibly line on the right side 
	else if (posEdgesCnt == 0 && negEdgesCnt == 1 && (negEdges[negEdgesCnt-1] < (CAM_DATA_LEN - MAX_WIDTH/2)) && OldPosError < 0) {
		*CurrentLinePosition = negEdges[negEdgesCnt-1] + (MAX_WIDTH / 2);
		//pc.printf("right");
		*CarState = LINE_RIGHT; 
	}
	
	// Case 5 No line detected
	else {
		//pc.printf("lost");
		*CarState = LOST;
	}
	
	// Debug line
	if (PRINT_LINE_VIS && *CarState != LOST ) {
		pc.printf("\r\n");
		for (int i = 0; i < CAM_DATA_LEN; i++) {
			if (i == int(*CurrentLinePosition)) 
				pc.printf("X");
			else
				pc.printf("-");
		}
		pc.printf("--\r\n");
	}
	
	if (PRINT_LINE_VIS) pc.printf("-- %2.2f\r\n", *CurrentLinePosition);
}

/** Consolidates imageprocessing into one function
 *  and provides edge detection data
 */
void imageprocessing(int16_t* cameraSnapshotPeaks) {
	
	// Processing stage holder variables
	uint16_t	cameraSnapshot[CAM_DATA_LEN];							// Snaphshot of camera data
	int16_t	 	cameraSnapshotGft[CAM_DATA_LEN]; 						// Snapshot after gradient filter n=1
	
	// Apply gaussian blur
	gaussianBlur(&TFC_LineScanImage0[0], &cameraSnapshot[0]);
	
	// Gradient (n=1)
	gradientFilter(&cameraSnapshot[0] , &cameraSnapshotGft[0]);
	
	// Filter out non-peaks
	nonMaxSupression(&cameraSnapshotGft[0], &cameraSnapshotPeaks[0]);
	
	// Print camera debug
	switch (PRINT_SNAPSHOT) {
		
		case SNAP_RAW_STREAM:
			printCamera(&TFC_LineScanImage0[0], 0, 127);
		break;
		
		case SNAP_GAUSSIAN_FLT:
			printCamera(&cameraSnapshot[0], 0, 127);
		break;
		
		case SNAP_GRADIENT_FLT:
			printCameraInt(&cameraSnapshotGft[0], 0, 127);
		break;
		
		case SNAP_NON_MAX_SUP:
			printCameraInt(&cameraSnapshotPeaks[0], 0, 127);
		break;
	}
	
	
	
	// Find min/max thresholds
	findThresholds(&cameraSnapshotGft[0]);
}

