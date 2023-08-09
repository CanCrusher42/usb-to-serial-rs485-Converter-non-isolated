#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"


extern rplidar_response_measurement_node_t finalLineData[1 * 180];


void clea

int setupTest1()
{
	for (i = 0; i < 180; i++)
		memset(&finalLineData[i], 0, sizeof(rplidar_response_measurement_node_t));
}
bool testHorzBasic1()
{
	return false;
}

bool testVertBasic1()
{
	return false;
}

bool testFilterBasic1()
{
	return false;
}
test

