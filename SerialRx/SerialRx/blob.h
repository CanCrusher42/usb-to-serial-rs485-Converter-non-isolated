#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifndef BLOBS_DEFINE_H
#define BLOBS_DEFINE_H


#define BLOBS_IN_LIST 10

typedef struct blob_detail_struct {
    int16_t   minX; 
    int16_t   maxX;
	int16_t   minY;
	int16_t   maxY;

}blobDetailStruct_t;

typedef struct blob_struct {
	uint8_t xLeft;
	uint8_t xRight;
	uint8_t yUpper;
	uint8_t yLower;
	uint8_t numSamples;
} blobStruct_t;

#define DEGREES_PER_RADIAN 57.2958F

void ClearBlobs();
void ClearBlobNumber(uint8_t i);
void ClearFinalLineData();
int8_t addPointToBlobList(uint8_t x, uint8_t y);
void MergeSecondBlobIntoFirst(uint8_t firstBlob, uint8_t secondBlob);
int  CreateBlobsFromFinalLineData(uint16_t xPerColumnA, uint16_t yPerRowA);
int8_t addDetailedPointToBlobList(int8_t x, int8_t y, int16_t realX, int16_t realY);
void MergeDetailedSecondBlobIntoFirst(uint8_t firstBlob, uint8_t secondBlob);

uint16_t GetBlobCount();
uint8_t  GetBlobHits(uint8_t blob);
float    GetAngleToBlob(uint8_t blob);
float    GetRealAngleToBlob(uint8_t blob);
uint16_t GetBlobSize(uint8_t blob);
uint16_t GetRealBlobSize(uint8_t blob);
uint16_t GetDistanceToBlobCenter(uint8_t blob);
uint16_t GetRealDistanceToBlobCenter(uint8_t blob);
void     GetBlobCenter(uint8_t blob, int16_t* x, int16_t* y);
uint8_t  GetLargestBlob();
void     GetLargestBlobData(float* angle, uint16_t* distance);

void PrintBlobList();



#endif
