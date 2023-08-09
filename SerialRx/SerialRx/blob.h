#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifndef BLOBS_DEFINE_H
#define BLOBS_DEFINE_H


#define BLOBS_IN_LIST 10

typedef struct blob_struct {
	int16_t xLeft;
	int16_t xRight;
	int16_t yUpper;
	int16_t yLower;
	uint16_t numSamples;
} blobStruct_t;


void ClearBlobs();
void ClearBlobNumber(uint16_t i);
void ClearFinalLineData();
int16_t addPointToBlobList(int16_t x, int16_t y);
void MergeSecondBlobIntoFirst(uint16_t firstBlob, uint16_t secondBlob);
uint16_t GetBlobCount();

//} __attribute__((packed)) blobStruct_t;


#endif
