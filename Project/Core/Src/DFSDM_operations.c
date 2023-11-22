/*
 * DFSDM_operations.c
 *
 *  Created on: Nov 21, 2023
 *      Author: kristianmyzeqari
 */

#include "DFSDM_operations.h"

int32_t maxVal = INT32_MIN;
int32_t minVal = INT32_MAX;

void HalfFullBufferOperations(int32_t* recBuf, uint32_t* playBuf, int BUFSIZE, int* flag){
	for(int i = 0; i < BUFSIZE/2; i++){

		recBuf[i] = recBuf[i] >> 8;

		if(recBuf[i] < minVal){
			minVal = recBuf[i];
		}
		if(recBuf[i] > maxVal){
			maxVal = recBuf[i];
		}
	}

	if(minVal < 0) minVal = -1 * (minVal);

	float temp = (float)((float)4095/((float)(maxVal)+(float)(minVal)));

	for(int j = 0; j < BUFSIZE/2; j++){
		recBuf[j] = recBuf[j] + minVal;
		playBuf[j] = temp * recBuf[j];
	}
	*flag = 0;
}

void FullBufferOperations(int32_t* recBuf, uint32_t* playBuf, int BUFSIZE, int* flag){
	for(int i = BUFSIZE/2; i < BUFSIZE; i++){

		recBuf[i] = recBuf[i] >> 8;

		if(recBuf[i] < minVal){
			minVal = recBuf[i];
		}
		if(recBuf[i] > maxVal){
			maxVal = recBuf[i];
		}
	}

	if(minVal < 0) minVal = -1 * minVal;

	float temp = (float)((float)4095/((float)maxVal+(float)minVal));

	for(int j = BUFSIZE/2; j < BUFSIZE; j++){
		recBuf[j] = recBuf[j] + minVal;
		playBuf[j] = temp * recBuf[j];
	}
	flag = 0;
}
