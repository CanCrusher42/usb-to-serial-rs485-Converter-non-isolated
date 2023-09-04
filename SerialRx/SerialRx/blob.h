#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifndef BLOBS_DEFINE_H
#define BLOBS_DEFINE_H


#define BLOBS_IN_LIST 10

typedef struct blob_struct {
	int8_t xLeft;
	int8_t xRight;
	int8_t yUpper;
	int8_t yLower;
	uint8_t numSamples;
} blobStruct_t;


void ClearBlobs();
void ClearBlobNumber(uint8_t i);
void ClearFinalLineData();
int8_t addPointToBlobList(int8_t x, int8_t y);
void MergeSecondBlobIntoFirst(uint8_t firstBlob, uint8_t secondBlob);
int  CreateBlobsFromFinalLineData(uint16_t xPerColumn, uint16_t yPerRow);


uint16_t GetBlobCount();
float    GetAngleToBlob(int8_t blob);
float    GetRealAngleToBlob(int8_t blob);
uint32_t GetBlobSize(uint8_t blob);
uint16_t GetDistanceToBlobCenter(uint8_t blob);
uint16_t GetRealDistanceToBlobCenter(uint8_t blob);
void     GetBlobCenter(uint8_t blob, int16_t* x, int16_t* y);
uint8_t  GetLargestBlob();
void     GetLargestBlobData(float* angle, uint16_t* distance);

void PrintBlobList();



#endif
