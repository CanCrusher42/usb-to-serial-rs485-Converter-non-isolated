#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "lp_defines.h"
#include "inc/rplidar_cmd.h"

extern rplidar_response_measurement_node_t finalLineData[SAMPLES_PER_DEGREE * 180];

void DisplayLineDistance( int numSamples, uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	char displayLine[365];
	uint16_t width = (endAngle - startAngle) * SAMPLES_PER_DEGREE;

	if (width > 360)
		width = 360;
	char barChar = '#';

	// get tallest bar
	uint16_t highest = 0;

	// ERROR THIS SHOULD CHECK BE CHECKING BETWEEN THE TARGETS
	for (uint16_t check = startAngle; check < endAngle; check++)
	{
		if (finalLineData[check].distance_q2 > highest  )
		
			highest = finalLineData[check].distance_q2;
	}

	int scale = highest / maxHeight;
	for (uint16_t rows = maxHeight; rows >= 1; rows--)
	{
		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (finalLineData[cols].distance_q2 >= (rows * scale))
				displayLine[cols - startAngle] = barChar;
			else
				displayLine[cols - startAngle] = ' ';

//			if ((rows == 1) && (sampleLine[cols].quality < 3))
//				displayLine[cols - startAngle] = '?';
		}

		// dispay left hand scale
		displayLine[width] = 0;
		printf("%05d|", rows * scale);
		printf("%s\n", displayLine);
	}

	printf("%s", "      ");
	for (int loc = startAngle; loc < endAngle; loc++)
		printf("%c", '-');

	printf("\n");


	// Display bottom scale
	for (int loc = 2; loc >= 0; loc--)
	{
		unsigned int power = 0;
		if (loc == 2) power = 100;
		if (loc == 1) power = 10;

		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (cols >= power)
				if (power == 100)
				{
					displayLine[cols] = '0' + ((cols / power));
				}
				else if (power == 10)
				{
					displayLine[cols] = '0' + ((cols / power) % power);
				}
				else
				{
					displayLine[cols] = '0' + (cols % 10);
				}
			else
				displayLine[cols] = '0';
		}
		displayLine[width] = 0;
		printf("      %s\n", displayLine);

	}


}
