/*
 *	position.cpp
 *
 *	Contains functions reponsible for position detection
 *
 */




//bool checkWidthNeg() {
//	uint16_t line(0);
//	if (posEdgeIndex[posEdges-1] > 0 )
//		line = posEdgeIndex[posEdges-1] - negEdgeIndex[negEdges-1];
//	else 
//		line = CAM_DATA_LEN - negEdgeIndex[negEdges-1];
//	
//	return (line >= MIN_WIDTH && line <= MAX_WIDTH);
//}

//inline bool checkWidthPos() {
//	return posEdgeIndex[posEdges-1] - negEdgeIndex[negEdges-1] >= MIN_WIDTH  && posEdgeIndex[posEdges-1] - negEdgeIndex[negEdges-1] <= MAX_WIDTH;
//};
//void processdata_v4() {
//	
//	// Overwrite old state
//	stateOldOld = stateOld;
//	stateOld = stateNow;
//	positionLineOld = positionLine;
//	
//		
//		// Line left
//		// Edge cannot jump directly to another edge, the camera processing is too fast
//		if ( posEdges > negEdges && checkWidthPos() && posEdges > 0) {
//		
//			// Take most inner index
//			positionLine = (float) posEdgeIndex[posEdges-1];
//			stateNow = LINE_LEFT;

//		}
//		
//		// Line right
//		else if ( negEdges > posEdges && checkWidthNeg() && negEdges  > 0) {
//			// Take most inner index
//			positionLine = (float) negEdgeIndex[negEdges-1] - CAM_DATA_LEN;
//			stateNow = LINE_RIGHT;		
//		}
//		
//		// We have one line but no idea which side, let's look at historical data to find out
//		else if (negEdges == posEdges && checkWidthPos() && negEdges  > 0) {
//			
//			// Last X scans gave out a positive error, it might be that there was interference
//			if ( positionLineErrorOld < 0 && stateOld == LINE_LEFT ) {
//				positionLine = posEdgeIndex[posEdges-1];
//				stateNow = LINE_LEFT;
//			}
//			// The car gets closer to the line or
//			// drives away from the line
//			// If we had negative error before, the car was going right,
//			// so we keep going same way as it might be on u turn
//			else if ( positionLineErrorOld > 0 && stateOld == LINE_RIGHT ) {
//				positionLine = negEdgeIndex[negEdges-1] - CAM_DATA_LEN;
//				stateNow = LINE_RIGHT;
//			}
//			else {
//				if (negEdgeIndex[negEdges-1] > CENTER_CAMERA) {
//					positionLine = (float) negEdgeIndex[negEdges-1] - CAM_DATA_LEN;
//					stateNow = LINE_RIGHT;
//				}
//				else if (posEdgeIndex[posEdges-1] <= CENTER_CAMERA) {
//					positionLine = posEdgeIndex[posEdges-1];
//					stateNow = LINE_LEFT;
//				}
//			}
//	}
//	// Case 5 : No line detected
//	else {
//			// It is impossible to jump straight from seeing the whole line to seeing nothing,
//			// therefore it is classified as interference of the camera
////			if (stateOld != LINE_LEFT_EDGE  && stateOld != LINE_RIGHT_EDGE ) {
//	//			positionLine = positionLineOld/2;
//				stateNow = NO_LINE;
//				noLineCnt++;
//			
//				if (noLineCnt > MAX_NO_LINE) {
//					offTheTrack = 1;
//					if (PRINT_PROCESSING) DEBUG_PRINT( "[SITUATION]: Out of track?? \tPos:\t%f\tCnt:\t%u \r\n", positionLine, noLineCnt );
//				
//				}
////			}
//	
//	}
//	
//	// Reset counter on new state detected
//	if (stateNow != NO_LINE) noLineCnt = 0;
//	
//	if (PRINT_PROCESSING) {
//		switch(stateNow) {
//			
//			case NO_LINE:
//				DEBUG_PRINT( "[SITUATION]: NO_LINE\tPos:\t%f\r\n", positionLine );
//			break;
//			
//			case LINE_LEFT:
//				DEBUG_PRINT( "[SITUATION]: LINE LEFT\tPos:\t%f\r\n", positionLine );
//			break;;

//			case LINE_LEFT_CLOSE:
//				DEBUG_PRINT( "[SITUATION]: LINE_LEFT_CLOSE\tPos:\t%f\r\n", positionLine );
//			break;
//			
//			case LINE_LEFT_EDGE:
//				DEBUG_PRINT( "[SITUATION]: LINE_LEFT_EDGE\tPos:\t%f\r\n", positionLine );
//			break;
//			
//			case LINE_RIGHT:
//				DEBUG_PRINT( "[SITUATION]: LINE_RIGHT\tPos:\t%f\r\n", positionLine );
//			break;
//			
//			case LINE_RIGHT_CLOSE:
//				DEBUG_PRINT( "[SITUATION]: LINE_RIGHT_CLOSE\tPos:\t%f\r\n", positionLine );
//			break;
//			
//			case LINE_RIGHT_EDGE:
//				DEBUG_PRINT( "[SITUATION]: LINE_RIGHT_EDGE\tPos:\t%f\r\n", positionLine );
//			break;
//		}
//		
//	}
//}

//void positionDetection() {
//	
//}


