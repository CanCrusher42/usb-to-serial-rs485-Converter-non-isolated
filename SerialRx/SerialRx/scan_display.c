#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "inc/lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"
#include "blob.h"

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


void printBottomScale(int startAngle, int endAngle, int pads)
{
	char displayLine[200] = { 0 };
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
		displayLine[endAngle] = 0;
		for (int i = 0; i < pads; i++) printf(" ");
		printf("      %s\n", displayLine);

	}
}
//                        -8000        8000       200
/*
v = 12,345;
v /10,000 = 1     1    % 10   = 1
v /  1000 = 12,   12   % 10   = 2
v /   100 = 123   123  % 10   = 3
v /    10 = 1234  1234 % 10   = 4
v /     1 = 12345 12345% 10   = 5
*/
void printBottomXScale(int startX, int endX, int xPerIndex, int pads )
{
	char displayLine[200] = { 0 };
	int sizeX = 180;
	// Display bottom scale
	for (int loc = 5; loc >= 0; loc--)
	{
		unsigned int power = 1;
		if (loc == 5) power = 100000;
		if (loc == 4) power = 10000;
		if (loc == 3) power = 1000;
		if (loc == 2) power = 100;
		if (loc == 1) power = 10;
          
		for (uint16_t cols = 0; cols < 180; cols++)
		{
			int value = (startX + cols * xPerIndex);
			int aValue = abs(value);

			int t = aValue / power;
			t = t % 10;
			if ((aValue/power) > 0)
				displayLine[cols] = '0' + t;
			else 
				displayLine[cols] = ' ';
		}
		displayLine[180] = 0;
		for (int i = 0; i < pads; i++) printf(" ");
		printf("      %s\n", displayLine);

	}
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


void ConvertDisplayLineToRoom(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	// Middle Angle = startAngle+90;
	// Get Max Neg and Pos X
	// Get Max Neg and Pos Y
	int xValue[182] = { 0 };
	int yValue[182] = { 0 };
	int minX = 1000, maxX = -1000;
	int maxY = -1000;
	int maxYcol = 0;
	uint16_t angleOffset = startAngle;
	int yPerRow, xPerColumn;
	char displayLine[200];
	float ang;
	// Get Scale Values
	for (uint16_t angle = startAngle; angle < (startAngle + 180); angle++)
	{
		if ((finalLineData[angle].angle_q6_checkbit >> 6) != 0)
		{
			ang = (float)(finalLineData[angle].angle_q6_checkbit >> 6);
			ang =  ang * (float)0.0174533;
			xValue[angle - startAngle] = (int)trunc(-1.0 * cos(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
			yValue[angle - startAngle] = (int)trunc(       sin(ang) * (float)(finalLineData[angle].distance_q2 >> 2));
		}
		else
		{
			xValue[angle - startAngle] = 0;
			yValue[angle - startAngle] = 0;
		}
		if (xValue[angle - startAngle] < minX)
			minX = xValue[angle - startAngle];
		if (xValue[angle - startAngle] > maxX)
			maxX = xValue[angle - startAngle];
		if (yValue[angle - startAngle] > maxY)
		{
			maxY = yValue[angle - startAngle];
			maxYcol = angle - startAngle;
		}
		
	}

	printf("maxY = %d @ %d\n", maxY, maxYcol);
	yPerRow = maxY / maxHeight;

	int absBounds = max(abs(minX), abs(maxX));
	// Need to force round up.
	xPerColumn = ((absBounds * 2)+179) / 180; // 180 columns

//	int rowUpper, rowLower;
	int divX, divY;
	int colStart = xPerColumn * (-90);
	

	for (int angle = 0; angle < 180; angle++)
	{
		divX = (xValue[angle]-colStart) / xPerColumn;
		divY = (yValue[angle] / yPerRow);
		printf("%d %d (%3.2f) %d  %d %d - mod %d %d\n", angle, finalLineData[angle].angle_q6_checkbit, (1.0*finalLineData[angle].angle_q6_checkbit)/64.0, finalLineData[angle].distance_q2,  xValue[angle], yValue[angle], divX, divY);
	}

	// Now paint from the top row (Farthest away from the sensor first)
	ClearBlobs();
	for (int row = maxHeight; row > 0; row--)
	{
		memset(displayLine, ' ', 195);
		for (uint16_t col = 0; col < 180; col++)
		{
			divX = (xValue[col] - colStart) / xPerColumn;
			divY = (yValue[col] / yPerRow);
			if (divY == row)
			{
				if ((divX < 0) || (divX > 180))
					printf("ERROR %d\n\n\n\n", divX);
				else
				{
					if (finalLineData[col].sync_quality != 0)
					{
						displayLine[divX] = '@';
						addPointToBlobList(divX, divY);


					}
					else
						displayLine[divX] = '.';
				}
			}
		}
		if (row < 3)
		{
			displayLine[89] = '#';
			displayLine[90] = '#';
			displayLine[91] = '#';
		}
		displayLine[180] = 0;
		printf("%3d %05d|", row, row * yPerRow);
		printf("        %s\n", displayLine);


	}

	printBottomXScale(-1 * absBounds, absBounds, 2 * absBounds / 180, 5);

}

