#pragma once

#ifndef LP_DEFINES_h
#define LP_DEFINES_h
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

// How does this differ from 
//struct sample_struct {
	//uint8_t quality;
	//uint16_t angle;
	//uint16_t distance;
//};


// How many samples do we want to look at per degree.  1 = 0,1,2,3,4,5 degrees.  2=0,0.5, 1.0, 1.5,. 2.0, .....
#define SAMPLES_PER_DEGREE  1           



#endif