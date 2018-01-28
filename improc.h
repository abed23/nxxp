/**
*	Keeps program settings
*
*	Author: Hubert
*	Date:	21/21/2017
*
* 	void findThresholds(const int16_t* gradientArray, int16_t* outputArray);
*/

void computeKernel() ;
void printCamera(volatile uint16_t * data, uint16_t start, uint16_t end);
void gaussianBlur(volatile uint16_t* cameraRaw, uint16_t* dataOut);
void gradientFilter(const uint16_t* blurred , int16_t* dataOut);
void findThresholds(const int16_t* gradientArray);
void nonMaxSupression(const int16_t* gradientArray, int16_t* outputArray);
void detectEdges(const int16_t* edgesArray, uint8_t * posEdges, uint8_t * negEdges);
void process_edges(TrackState_t * CarState, float * CurrentLinePosition, float * OldLinePosition,  uint8_t * posEdges, uint8_t * negEdge, float * OldPosError);
void imageprocessing(int16_t* cameraSnapshotPeaks);

