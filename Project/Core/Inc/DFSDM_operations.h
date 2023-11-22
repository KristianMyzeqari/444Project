/*
 * DFSDM_operations.h
 *
 *  Created on: Nov 21, 2023
 *      Author: kristianmyzeqari
 */

#ifndef INC_DFSDM_OPERATIONS_H_
#define INC_DFSDM_OPERATIONS_H_


#include <stdint.h>

void HalfFullBufferOperations(int32_t* recBuf, uint32_t* playBuf, int BUFSIZE, int *flag);
void FullBufferOperations(int32_t* recBuf, uint32_t* playBuf, int BUFSIZE, int *flag);

#endif /* INC_DFSDM_OPERATIONS_H_ */
