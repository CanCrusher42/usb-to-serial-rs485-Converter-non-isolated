#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifndef BLOBS_DEFINE_H
#define BLOBS_DEFINE_H


#define BLOBS_IN_LIST 5

typedef struct blob_struct {
	uint16_t xStart;
	uint16_t xEnd;
	uint16_t yStart;
	uint16_t yEnd;
	uint16_t numSamples;

} blobStruct_t;

//} __attribute__((packed)) blobStruct_t;


#endif
