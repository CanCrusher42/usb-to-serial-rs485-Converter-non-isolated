#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lp_defines.h"
#include "inc/rplidar_cmd.h"
extern int maxValue ;
extern rplidar_response_measurement_node_t finalLineData[SAMPLES_PER_DEGREE * 180];

void DisplayLineDistance(  uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
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

	printf("max value = %d/n", maxValue);

}



void DisplayLineAngle(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
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
		if (finalLineData[check].angle_q6_checkbit > highest)

			highest = finalLineData[check].angle_q6_checkbit;
	}

	int scale = highest / maxHeight;
	for (uint16_t rows = maxHeight; rows >= 1; rows--)
	{
		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (finalLineData[cols].angle_q6_checkbit >= (rows * scale))
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


void DisplayLineToRoom(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	// Middle Angle = startAngle+90;
	// Get Max Neg and Pos X
	// Get Max Neg and Pos Y
	int xValue[180] = { 0 };
	int yValue[180] = { 0 };
	int minX = 1000, maxX = -1000;
	int maxY = -1000;

	uint16_t angleOffset = startAngle;
	int yPerRow, xPerColumn;

	// Get Scale Values
	for (uint16_t angle = startAngle; angle < (startAngle + 180); angle++)
	{
		float ang = (angle - angleOffset);
		ang *= 0.0174533;
		xValue[angle - startAngle] = round(-1.0 * cos(ang) * finalLineData[angle].distance_q2);
		yValue[angle - startAngle] = round(       sin(ang) * finalLineData[angle].distance_q2);

		if (xValue[angle - startAngle] < minX)
			minX = xValue[angle - startAngle];
		if (xValue[angle - startAngle] > maxX)
			maxX = xValue[angle - startAngle];
		if (yValue[angle - startAngle] > maxY)
			maxY = yValue[angle - startAngle];
	}
	yPerRow = maxY / maxHeight;
	xPerColumn = (max(abs(minX), abs(maxX)) * 2) / 180;
	yPerRow = maxY / maxHeight;

	int rowUpper, rowLower;

	int colStart = xPerColumn * (-90);
	char displayLine[200];

	// Now paint from the top row (Farthest away from the sensor first)

	for (int row = maxHeight; row > 0; row--)
	{
		rowUpper = row * yPerRow;
		rowLower = (row - 1) * yPerRow;
		for (int col = 0; col < 180; col++)
		{
			displayLine[col] = ' ';
			if ((yValue[col] > rowLower) && (yValue[col] <= rowUpper))
			{
				int xDiv = (xValue[col] - colStart) / xPerColumn;
				displayLine[col] = '*';

			}
		}
		if (row < 3)
		{
			displayLine[89] = '#';
			displayLine[90] = '#';
			displayLine[91] = '#';
		}
		displayLine[180] = 0;
		printf("%s\n", displayLine);
	}

	memset(displayLine, '-', 179);
	displayLine[180] = 0;
	printf("%s\n", displayLine);


}

