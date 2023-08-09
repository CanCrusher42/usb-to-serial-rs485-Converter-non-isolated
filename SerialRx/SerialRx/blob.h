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
uint16_t GetBlobCount();

//} __attribute__((packed)) blobStruct_t;


#endif
