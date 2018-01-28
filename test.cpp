///*
// * LineDetectionDouble.c
// *
// *  Created on: Oct 29, 2014
// *  Author: Miroslav Dobrev
// */

//#include "LineDetectionDouble.h"

//extern float speedL;		// Used for Cross Section Detection
//extern float speedR;

//static struct detectedTransitions_s lineTransitions = { .numberOfTransitions = 0 };
//static struct detectedTransitions_s lineTransitionsStop = { .numberOfTransitions = 0 };
//static struct detectedLines_s detectedLines = { .numberOfLines = 0 };
//static struct detectedTracks_s detectedTracks = { .numberOfTracks = 0 };
//static struct detectedDoubleLines_s detectedDoubleLines = { .numberOfLines = 0 };

//static uint32_t trackWidthAverage = TRACK_WIDTH_MEAN;	// Used for track width probability evaluation

//// Variables used for cross section detection
//	static int8_t lineCenterPrevious;
//	static int8_t edgePreviousValid;
//	static float lineCenterTemp;
//	extern int32_t newExposure;		// Gives the time between the two frames
///////////////////////////////////////////////

//void findLine(volatile uint16_t* lineScanImage, volatile uint16_t* lineScanImage1, carState_s* carState)
//{
//	/////////////////////////////////////////////////////////////
//	static int16_t imageDifferential[128];	    // TODO: Remove static in case of not enough RAM
//	static int16_t imageDifferentialTwo[128];	//
//	
//	lineTransitions.numberOfTransitions = 0;
//	lineTransitionsStop.numberOfTransitions = 0;
//	detectedLines.numberOfLines = 0;
//	
//	detectedTracks.numberOfTracks = 0;
//	struct detectedTrack_s bestTrack = { .certainty = 0 };
//	static struct detectedTrack_s previousTrack;
//	
//	detectedDoubleLines.numberOfLines = 0;
//	detectedDoubleLines.maxDerivativeLeft = 0;
//	detectedDoubleLines.maxDerivativeRight = 0;
//	
//	struct detectedDoubleLine_s bestLeft = { .relativePositionCertainty = 0, .certainty = 0 };
//	struct detectedDoubleLine_s bestRight = { .relativePositionCertainty = 0, .certainty = 0};
//	static struct detectedDoubleLine_s previousLeft = { .position = 64 + TRACK_WIDTH_MEAN/2};
//	static struct detectedDoubleLine_s previousRight = { .position = 64 - TRACK_WIDTH_MEAN/2 };
//	/////////////////////////////////////////////////////////////

//	////////////////////////////////////////////////////////////////////////////////////////////////
//	edgeDetection(lineScanImage, imageDifferential, 128);
//	#ifdef LINE_DISTANCE_ENABLE
//	edgeDetection(lineScanImage1, imageDifferentialTwo, 128);
//	carState->lineDistance = findLineDistance(imageDifferentialTwo, DISTANCE_DERIVATIVE_THRESHOLD, DISTANCE_MAX);
//	#endif
//	findTransitions(&lineTransitions, imageDifferential, DERIVATIVE_THRESHOLD);
//	findTransitions(&lineTransitionsStop, imageDifferential, DERIVATIVE_THRESHOLD_STOP);
//	transitionsToLines(&detectedLines, &lineTransitionsStop);
//	transitionsToTracks(&detectedTracks, &lineTransitions);
//	transitionsToDoubleLines(&detectedDoubleLines, &lineTransitions);
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	
//	if(cross_section_enable == 1)
//	{
//		lineCenterPrevious = carState->lineCenter;
//		if (carState->edge == LEFT_EDGE || carState->edge == RIGHT_EDGE)
//		{
//			edgePreviousValid = carState->edge;
//		}
//	}
//		
//	if(carState->carMode == NORMAL_MODE
//			&& carState->lineDistance > STRAIGHT_DISTANCE_THRESHOLD
//			&& abs(carState->lineCenter) <= STRAIGHT_ERROR_THRESHOLD
//			&& carState->lineDetectionState == LINE_FOUND)
//	{
//		carState->carMode = STRAIGHT_MODE;
//	}	
//	else if(carState->carMode == STRAIGHT_MODE
//				&& (carState->lineDistance <=STRAIGHT_DISTANCE_THRESHOLD
//					|| abs(carState->lineCenter) > STRAIGHT_CRITICAL_ERROR))
//	{
//		carState->carMode = NORMAL_MODE;
//	}

//	////////////////////////////////////////////////////////////////////////////////////////////////
//	if (detectedTracks.numberOfTracks == 0 && detectedDoubleLines.numberOfLines == 0)
//	{
//		if (carState->lineDetectionState != STOPLINE_DETECTED)
//		{
//			if(TFC_Ticker[3] < MAX_LOST_LINE_DURATION)
//			{
//				carState->lineDetectionState = LINE_TEMPORARILY_LOST;
//				carState->detectedType = NONE;
//				carState->edge = NO_EDGE;
//				
//				if(cross_section_enable == 1)
//				{
//					if(carState->crossSection == YES)
//					{
//						carState->crossSectionPosition = MIDDLE;
//						lineCenterTemp = lineCenterTemp*CROSS_LINE_ERROR_REDUCTION;
//						carState->lineCenter = (int8_t)(lineCenterTemp);
//					}
//				}
//			}
//			else if(TFC_Ticker[3] >= MAX_LOST_LINE_DURATION)
//			{
//				carState->lineDetectionState = LINE_LOST;
//				carState->detectedType = NONE;
//				carState->edge = NO_EDGE;
//			}
//		}
//		return;
//	}
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	//Populate lines certainty values
//	weightTracks(&previousTrack, &detectedTracks);
//	weightDoubleLines(&previousLeft, &detectedDoubleLines, LEFT);
//	weightDoubleLines(&previousRight, &detectedDoubleLines, RIGHT);
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	//Select track with highest certainty
//	for (uint8_t k = 0; k < detectedTracks.numberOfTracks; k++)
//	{
//		if (detectedTracks.track[k].certainty > bestTrack.certainty)
//		{
//			bestTrack = detectedTracks.track[k];
//		}
//	}
//	// TERMINAL_PRINTF("Found line at %i with width %i,\t width certainty %i,\t relative position difference %i and certainty %i,\t total certainty %i\n", bestLine.center, bestLine.width, (int16_t)(1000.0f*bestLine.widthCertainty), previousLine.center - bestLine.center, (int16_t)(1000.0f*bestLine.relativePositionCertainty), (int16_t)(1000.0f*bestLine.certainty));

//	//Select doubleLines with highest certainty
//	for (uint8_t k = 0; k < detectedDoubleLines.numberOfLines; k++)
//	{
//		if (detectedDoubleLines.line[k].lineType == LEFT && detectedDoubleLines.line[k].certainty > bestLeft.certainty)
//		{
//			bestLeft = detectedDoubleLines.line[k];
//		}
//		else if (detectedDoubleLines.line[k].lineType == RIGHT && detectedDoubleLines.line[k].certainty > bestRight.certainty)
//		{
//			bestRight = detectedDoubleLines.line[k];
//		}
//	}
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	
//	if (bestLeft.certainty > MIN_CERTAINTY)
//		previousLeft = bestLeft;
//	if (bestRight.certainty > MIN_CERTAINTY)
//		previousRight = bestRight;
//	
//	////////////////////////////////////////////////////////////////////////////////////////////////
//#ifdef DOUBLE_LINE_DETECTION
//	if (bestTrack.certainty > MIN_CERTAINTY)
//	{
//		previousTrack = bestTrack;
//		
//		carState->lineCenter = bestTrack.center;
//		
//		if (carState->lineDetectionState != STOPLINE_DETECTED)
//		{
//			carState->lineDetectionState = LINE_FOUND;
//			carState->detectedType = DOUBLE_EDGE;
//			carState->edge = BOTH_EDGE;
//		}
//		
//		if(cross_section_enable == 1)
//		{
//			if(carState->crossSection == YES)
//			{
//				carState->crossSection = NO;
//				carState->crossSectionPosition = NO_CROSS;
//			}
//		}
//		
//		if (bestTrack.width >= TRACK_WIDTH_MIN && bestTrack.width <= TRACK_WIDTH_MAX)
//		{
//			trackWidthAverage = bestTrack.width;
//		}	
//			
//		TFC_Ticker[3] = 0;
//		return;
//	}
//	else
//	{
//		bestTrack.widthCertainty = 0;
//		//Select line with highest certainty based on just width
//		for (uint8_t k = 0; k < detectedTracks.numberOfTracks; k++)
//		{
//			if (bestTrack.widthCertainty < detectedTracks.track[k].widthCertainty)
//			{
//				bestTrack = detectedTracks.track[k];
//			}
//		}
//		if (bestTrack.widthCertainty > MIN_CERTAINTY)
//		{
//			previousTrack = bestTrack;
//			TFC_Ticker[3] = 0;
//			
//			carState->lineCenter = bestTrack.center;
//			
//			if (carState->lineDetectionState != STOPLINE_DETECTED)
//			{
//				carState->lineDetectionState = LINE_FOUND;
//				carState->detectedType = DOUBLE_EDGE;
//				carState->edge = BOTH_EDGE;
//			}
//			
//			if(cross_section_enable == 1)
//			{
//				if(carState->crossSection == YES)
//				{
//					carState->crossSection = NO;
//					carState->crossSectionPosition = NO_CROSS;
//				}
//			}
//			
//			return;
//		}
//	} 
//#endif	
//	
//	if (bestLeft.certainty > MIN_CERTAINTY || bestRight.certainty > MIN_CERTAINTY)
//	{	
//		if (bestLeft.certainty >= bestRight.certainty)
//		{
//			carState->lineCenter = (bestLeft.position - trackWidthAverage/2 - 64) - CAMERA_CENTER_OFFSET;
//			carState->edge = LEFT_EDGE;
//		} 
//		else if (bestRight.certainty > bestLeft.certainty)
//		{
//			carState->lineCenter = (bestRight.position + trackWidthAverage/2 - 64) - CAMERA_CENTER_OFFSET;
//			carState->edge = RIGHT_EDGE;
//		}
//		
//		if(cross_section_enable == 1)
//		{
//			if(carState->crossSection == NO)
//			{	
//				float carSpeed = (speedL + speedR)/2;
//				float crossDerivative = ((float)lineCenterPrevious - (float)carState->lineCenter)/(((float)newExposure/5000.0f)*(carSpeed/8.0f));
//				
//				if(((crossDerivative > CROSS_DERIVATIVE_THRESHOLD && carState->edge == RIGHT_EDGE) || (crossDerivative < -CROSS_DERIVATIVE_THRESHOLD && carState->edge == LEFT_EDGE))
//					&& carSpeed > CROSS_MIN_SPEED
//					&& carState->lineDistance >= CROSS_MINIMUM_DISTANCE
//					/* Might Require lineCenter threshold as well */)
//				{
//					lineCenterTemp = lineCenterTemp*CROSS_LINE_ERROR_REDUCTION;
//					carState->lineCenter = (int8_t)(lineCenterTemp);
//					carState->crossSection = YES;
//					carState->crossSectionPosition = BEGINNING;
//					lineCenterTemp = (float)lineCenterPrevious;
//				}
//			}
//			else
//			{
//				if(carState->crossSectionPosition == BEGINNING)
//				{
//					if(carState->edge == edgePreviousValid)
//					{
//						lineCenterTemp = lineCenterTemp*CROSS_LINE_ERROR_REDUCTION;
//						carState->lineCenter = (int8_t)(lineCenterTemp);
//					}
//					else
//					{
//						carState->crossSection = NO;
//						carState->crossSectionPosition = NO_CROSS;						
//					}
//				}
//				else
//				{
//					carState->crossSection = NO;
//					carState->crossSectionPosition = NO_CROSS;
//				}
//			}
//		}
//		
//		if (carState->lineDetectionState != STOPLINE_DETECTED)
//		{
//			carState->lineDetectionState = LINE_FOUND;
//			carState->detectedType = SINGLE_EDGE;
//		}
//		
//		TFC_Ticker[3] = 0;
//		return;
//	}
//	else
//	{
//		bestLeft.derivativeCertainty = 0.0f;
//		bestRight.derivativeCertainty = 0.0f;
//		
//		for (uint8_t k = 0; k < detectedDoubleLines.numberOfLines; k++)
//		{
//			if (detectedDoubleLines.line[k].lineType == LEFT && detectedDoubleLines.line[k].derivativeCertainty >= bestLeft.derivativeCertainty)
//			{
//				bestLeft = detectedDoubleLines.line[k];
//			}
//			else if (detectedDoubleLines.line[k].lineType == RIGHT && detectedDoubleLines.line[k].derivativeCertainty >= bestRight.derivativeCertainty)
//			{
//				bestRight = detectedDoubleLines.line[k];
//			}
//		}
//		
//		if (bestLeft.derivativeCertainty > MIN_CERTAINTY || bestRight.derivativeCertainty > MIN_CERTAINTY)
//		{		
//			if (bestLeft.derivativeCertainty >= bestRight.derivativeCertainty)
//			{
//				carState->lineCenter = (bestLeft.position + trackWidthAverage/2 - 64) - CAMERA_CENTER_OFFSET;
//				carState->edge = LEFT_EDGE;
//			}
//			else if (bestRight.derivativeCertainty > bestLeft.derivativeCertainty)
//			{
//				carState->lineCenter = (bestRight.position - trackWidthAverage/2 - 64) - CAMERA_CENTER_OFFSET;
//				carState->edge = RIGHT_EDGE;
//			}
//			
//			if(cross_section_enable == 1)
//			{
//				if(carState->crossSection == YES && carState->crossSectionPosition == MIDDLE)
//				{
//					carState->crossSection = NO;
//					carState->crossSectionPosition = NO_CROSS;
//				}
//				else if(carState->crossSection == YES && carState->crossSectionPosition == BEGINNING)
//				{
//					lineCenterTemp = lineCenterTemp*CROSS_LINE_ERROR_REDUCTION;
//					carState->lineCenter = (int8_t)(lineCenterTemp);
//				}
//			}
//			
//			if (bestLeft.derivativeCertainty > MIN_CERTAINTY)
//				previousLeft = bestLeft;
//			if (bestRight.derivativeCertainty > MIN_CERTAINTY)
//				previousRight = bestRight;
//			
//			if (carState->lineDetectionState != STOPLINE_DETECTED)
//			{
//				carState->lineDetectionState = LINE_FOUND;
//				carState->detectedType = SINGLE_EDGE;
//			}
//			
//			TFC_Ticker[3] = 0;
//			return;
//		}
//	} 
//	////////////////////////////////////////////////////////////////////////////////////////////////

//	////////////////////////////////////////////////////////////////////////////////////////////////
//	if (TFC_Ticker[3] < MAX_LOST_LINE_DURATION)
//	{
//		if (carState->lineDetectionState != STOPLINE_DETECTED)
//		{
//			carState->lineDetectionState = LINE_TEMPORARILY_LOST;
//			carState->detectedType = NONE;
//			carState->edge = NO_EDGE;			
//		}
//		
//		if(cross_section_enable == 1)
//		{
//			if(carState->crossSection == YES && TFC_Ticker[3] >= CROSS_MID_TIME_THRESHOLD)
//			{
//				carState->crossSectionPosition = MIDDLE;
//				carState->lineCenter = (int8_t)((float)lineCenterPrevious*CROSS_LINE_ERROR_REDUCTION);
//			}
//		}
//	
//		return;
//	}

//	// No line found
//	if (carState->lineDetectionState != STOPLINE_DETECTED && TFC_Ticker[3] >= MAX_LOST_LINE_DURATION)
//	{
//		carState->lineDetectionState = LINE_LOST;
//		carState->detectedType = NONE;
//		carState->edge = NO_EDGE;
//	}
//	////////////////////////////////////////////////////////////////////////////////////////////////
//	
//	return;
//}

//int8_t findTransitions(struct detectedTransitions_s* detectedTransitions, int16_t* derivative, uint16_t derivativeThreshold)
//{
//	//Find and log positions of all transitions and their directions
//	for (uint8_t k = 0; k < 128 && detectedTransitions->numberOfTransitions < MAX_NUMBER_OF_TRANSITIONS; k++)
//	{
//		if (derivative[k] >= derivativeThreshold)
//		{
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].position = k;
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].direction = blackToWhite;
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].derivative = derivative[k];
//			//TERMINAL_PRINTF("end at %i, ", detectedTransitions->transition[detectedTransitions->numberOfTransitions].position);
//			detectedTransitions->numberOfTransitions++;
//		}
//		else if (derivative[k] <= -derivativeThreshold)
//		{
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].position = k;
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].direction = whiteToBlack;
//			detectedTransitions->transition[detectedTransitions->numberOfTransitions].derivative = (-derivative[k]);
//			//TERMINAL_PRINTF("start at %i, ", detectedTransitions->transition[detectedTransitions->numberOfTransitions].position);
//			detectedTransitions->numberOfTransitions++;
//		}
//	}
//	//TERMINAL_PRINTF("/n");
//	return 0;
//}

//#ifdef LINE_DISTANCE_ENABLE
//uint8_t findLineDistance(int16_t* derivative, uint16_t derivativeThreshold, uint8_t maxDistance)
//{
//	if(maxDistance > 120)
//		maxDistance = 120;
//	
//	for (uint8_t k = 5; k < maxDistance; k++)
//	{
//		if (derivative[k] <= -derivativeThreshold)	// Checks only for white to black transitions
//		{
//			return k;
//		}
//	}
//	
//	return STRAIGHT_DISTANCE_DEFAULT;	// Returns this if no distance edge in the interval [5 : MAX) detected.
//}
//#endif

//int8_t transitionsToLines(struct detectedLines_s* detectedLines, struct detectedTransitions_s* detectedTransitions)
//{
//	//Start from first white to black transition and look for pairs, store starts and ends
//	for (uint8_t k = 0; k < detectedTransitions->numberOfTransitions && detectedLines->numberOfLines < MAX_NUMBER_OF_LINES; k++)
//	{
//		if (detectedTransitions->transition[k].direction == whiteToBlack)
//		{
//			detectedLines->line[detectedLines->numberOfLines].start = detectedTransitions->transition[k].position;
//			k++;
//			if (detectedTransitions->transition[k].direction == blackToWhite && k < detectedTransitions->numberOfTransitions)
//			{
//				detectedLines->line[detectedLines->numberOfLines].end = detectedTransitions->transition[k].position;
//				detectedLines->line[detectedLines->numberOfLines].width = detectedLines->line[detectedLines->numberOfLines].end
//						- detectedLines->line[detectedLines->numberOfLines].start;
//				detectedLines->line[detectedLines->numberOfLines].center = detectedLines->line[detectedLines->numberOfLines].start
//						+ ((detectedLines->line[detectedLines->numberOfLines].width) / 2) - 64;
//				//Successfully found a line
//				//TERMINAL_PRINTF("Start: %i, end: %i ", detectedLines->line[detectedLines->numberOfLines].start, detectedLines->line[detectedLines->numberOfLines].end);
//				detectedLines->numberOfLines++;
//			}
//			else
//			{
//				//Found a second line start
//				k--;
//			}
//		}
//	}
//	//TERMINAL_PRINTF("\n");
//	return 0;
//}

//void transitionsToTracks(struct detectedTracks_s* detectedTracks, struct detectedTransitions_s* detectedTransitions)
//{
//	//Start from first black to white transition and look for pairs, store starts and ends
//	for (uint8_t k = 0; k < detectedTransitions->numberOfTransitions && detectedTracks->numberOfTracks < MAX_NUMBER_OF_TRACKS; k++)
//	{
//		if (detectedTransitions->transition[k].direction == blackToWhite)
//		{
//			detectedTracks->track[detectedTracks->numberOfTracks].start = detectedTransitions->transition[k].position;

//			for (uint8_t j = k+1; j < detectedTransitions->numberOfTransitions && detectedTracks->numberOfTracks < MAX_NUMBER_OF_TRACKS; j++)
//			{
//				if (detectedTransitions->transition[j].direction == whiteToBlack)
//				{
//					detectedTracks->track[detectedTracks->numberOfTracks].end = detectedTransitions->transition[j].position;
//					detectedTracks->track[detectedTracks->numberOfTracks].width = detectedTracks->track[detectedTracks->numberOfTracks].end
//							- detectedTracks->track[detectedTracks->numberOfTracks].start;
//					detectedTracks->track[detectedTracks->numberOfTracks].center = detectedTracks->track[detectedTracks->numberOfTracks].start
//							+ ((detectedTracks->track[detectedTracks->numberOfTracks].width) / 2) - 64 - CAMERA_CENTER_OFFSET;

//					detectedTracks->numberOfTracks++;
//				}
//			}
//		}
//	}
////	TERMINAL_PRINTF("\n");
//}

//void transitionsToDoubleLines(struct detectedDoubleLines_s* detectedDoubleLines, struct detectedTransitions_s* detectedTransitions)
//{
//	for (uint8_t k = 0; k < detectedTransitions->numberOfTransitions && detectedDoubleLines->numberOfLines < MAX_NUMBER_OF_DOUBLE_LINES; k++)
//	{
//		if (detectedTransitions->transition[k].direction == blackToWhite)
//		{
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].position = detectedTransitions->transition[k].position;
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].lineType = RIGHT;
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative = detectedTransitions->transition[k].derivative;
//			
//			if (detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative > detectedDoubleLines->maxDerivativeLeft)
//			{
//				detectedDoubleLines->maxDerivativeLeft = detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative;
//			}
//			
//			detectedDoubleLines->numberOfLines++;	
//		}
//		else if (detectedTransitions->transition[k].direction == whiteToBlack)
//		{
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].position = detectedTransitions->transition[k].position;
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].lineType = LEFT;
//			detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative = detectedTransitions->transition[k].derivative;
//			
//			if (detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative > detectedDoubleLines->maxDerivativeRight)
//			{
//				detectedDoubleLines->maxDerivativeRight = detectedDoubleLines->line[detectedDoubleLines->numberOfLines].derivative;
//			}
//			
//			detectedDoubleLines->numberOfLines++;		
//		}
//	}
//}

//void weightTracks(struct detectedTrack_s* previousTrack, struct detectedTracks_s* detectedTracks)
//{
//	for (uint8_t k = 0; k < detectedTracks->numberOfTracks; k++)
//	{
//		detectedTracks->track[k].widthCertainty = getProbability(detectedTracks->track[k].width, TRACK_WIDTH_SD, trackWidthAverage);

//		detectedTracks->track[k].relativePositionCertainty = getProbability(previousTrack->center - detectedTracks->track[k].center, TRACK_CENTER_DIFFERENCE_SD,
//				TRACK_CENTER_DIFFERENCE_MEAN);

//		detectedTracks->track[k].certainty = detectedTracks->track[k].widthCertainty * detectedTracks->track[k].relativePositionCertainty;
//	}
//}

//void weightDoubleLines(struct detectedDoubleLine_s* previousLine, struct detectedDoubleLines_s* detectedDoubleLines, doubleLineType_t lineType)
//{
//	if (lineType == LEFT)
//	{
//		for (uint8_t k = 0; k < detectedDoubleLines->numberOfLines; k++)
//		{
//			if (detectedDoubleLines->line[k].lineType == LEFT)
//			{
//				detectedDoubleLines->line[k].relativePositionCertainty = getProbability(previousLine->position - detectedDoubleLines->line[k].position, DOUBLE_LINE_POSITION_DIFFERENCE_SD,
//						DOUBLE_LINE_POSITION_DIFFERENCE_MEAN);
//				
//				detectedDoubleLines->line[k].derivativeCertainty = detectedDoubleLines->line[k].derivative / detectedDoubleLines->maxDerivativeLeft;
//				
//				detectedDoubleLines->line[k].certainty = detectedDoubleLines->line[k].relativePositionCertainty * detectedDoubleLines->line[k].derivativeCertainty;
//			}
//		}
//	}
//	else if (lineType == RIGHT)
//	{
//		for (uint8_t k = 0; k < detectedDoubleLines->numberOfLines; k++)
//		{
//			if (detectedDoubleLines->line[k].lineType == RIGHT)
//			{
//				detectedDoubleLines->line[k].relativePositionCertainty = getProbability(previousLine->position - detectedDoubleLines->line[k].position, DOUBLE_LINE_POSITION_DIFFERENCE_SD,
//						DOUBLE_LINE_POSITION_DIFFERENCE_MEAN);
//				
//				detectedDoubleLines->line[k].derivativeCertainty = detectedDoubleLines->line[k].derivative / detectedDoubleLines->maxDerivativeRight;
//				
//				detectedDoubleLines->line[k].certainty = detectedDoubleLines->line[k].relativePositionCertainty * detectedDoubleLines->line[k].derivativeCertainty;
//			}
//		}
//	}
//}

//void findStop(carState_s* carState)		// Function modified for double line track
//{
//	if (detectedLines.numberOfLines < 2)
//		return;

//	struct stopLines_s stopLines =
//	{ .numberOfStopLines = 0 };

//	for (int8_t i = 0; i <= detectedLines.numberOfLines - 2; i++)
//	{
//		for (int8_t j = i + 1; j <= detectedLines.numberOfLines - 1; j++)
//		{
//			stopLines.stopLine[stopLines.numberOfStopLines].line[0] = detectedLines.line[i];
//			stopLines.stopLine[stopLines.numberOfStopLines].line[1] = detectedLines.line[j];
//			stopLines.numberOfStopLines++;
//		}
//	}

//	weightStopLines(&stopLines);
//	
//	// Stopline Serial Debug - May not be able to send data if hall sensors signal blocks the UART pin
//	/* for (uint8_t i = 0; i < stopLines.numberOfStopLines; i++)
//	{
//		TERMINAL_PRINTF("Found stop line %i with certainty %i, gap certainty %i, side width certainties of %i and %i.\t\t", i, (int16_t)(stopLines.stopLine[i].certainty*1000.0f), (int16_t)(stopLines.stopLine[i].gapWidthCertainty*1000.0f), (int16_t)(stopLines.stopLine[i].line[0].widthCertainty*1000.0f), (int16_t)(stopLines.stopLine[i].line[1].widthCertainty*1000.0f));
//	}
//	TERMINAL_PRINTF("\r\n");  */ 
//	
//	for (uint8_t i = 0; i < stopLines.numberOfStopLines; i++)
//	{
//		if (stopLines.stopLine[i].certainty > MIN_STOPLINE_CERTAINTY && TFC_Ticker[5] > STOPLINE_START_TIME)
//		{
//			carState->lineDetectionState = STOPLINE_DETECTED;
//		}
//	}

//	return;
//}

//void preloadProbabilityTables()
//{
//	getProbability(0, STOPLINE_SIDE_WIDTH_SD, STOPLINE_SIDE_WIDTH_MEAN);
//	getProbability(0, STOPLINE_GAP_WIDTH_SD, STOPLINE_GAP_WIDTH_MEAN);
//}

//int8_t weightStopLines(struct stopLines_s* stopLines)	// Function modified for double line track
//{
//	for (uint8_t i = 0; i < stopLines->numberOfStopLines; i++)
//	{
//		stopLines->stopLine[i].line[0].widthCertainty = getProbability(stopLines->stopLine[i].line[0].width, STOPLINE_SIDE_WIDTH_SD, (int8_t)(STOPLINE_SIDE_PERCENTAGE*((float)trackWidthAverage)+0.5));
//		stopLines->stopLine[i].line[1].widthCertainty = getProbability(stopLines->stopLine[i].line[1].width, STOPLINE_SIDE_WIDTH_SD, (int8_t)(STOPLINE_SIDE_PERCENTAGE*((float)trackWidthAverage)+0.5));
//		
//		stopLines->stopLine[i].gap = stopLines->stopLine[i].line[1].start - stopLines->stopLine[i].line[0].end;
//		stopLines->stopLine[i].gapWidthCertainty = getProbability(stopLines->stopLine[i].gap, STOPLINE_GAP_WIDTH_SD, (int8_t)(STOPLINE_GAP_PERCENTAGE*((float)trackWidthAverage)+0.5));

//		stopLines->stopLine[i].certainty = stopLines->stopLine[i].line[0].widthCertainty * stopLines->stopLine[i].line[1].widthCertainty
//				 * stopLines->stopLine[i].gapWidthCertainty;
//	}
//	return 0;
//}

