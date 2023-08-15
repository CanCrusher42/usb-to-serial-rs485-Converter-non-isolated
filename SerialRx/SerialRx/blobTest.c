#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lp_defines.h"
#include "inc/rplidar_cmd.h"
#include "string.h"
#include "blob.h"


extern rplidar_response_measurement_node_t finalLineData[1 * 180];




int SetupTestAreas()
{
	ClearBlobs();
	ClearFinalLineData();


	return 0;

}

void SetupTest1()
{
	SetupTestAreas();
	int location = addPointToBlobList(-10, 0);
	location = addPointToBlobList(-10, 10);
	location = addPointToBlobList(-10, 20);
	location = addPointToBlobList(20, 20);
}

bool testAddPointToBlankList()
{
	SetupTestAreas();
	int location = addPointToBlobList(-10, 0);
	if (location <  0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
			return false;
	}

	location = addPointToBlobList(-10, 10);
	if (location <  0)
	{
		printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location);
		return false;
	}

	location = addPointToBlobList(-10, 20);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location);
		return false;
	}

}
bool testHorzVertBasic1()
{
	SetupTest1();
	
	// X tests
	int location = addPointToBlobList(-11, 0);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(-10, 1);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location);
		return false;
	}

	// Y tests
	location = addPointToBlobList(-10, 9);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(-10, 11);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 3.  %i\n", __FUNCTION__, location);
		return false;
	}

	return true;
}

bool testCornerBasic1()
{
//	int location = addPointToBlobList(-10, 0);
//	location = addPointToBlobList(-10, 10);
//	location = addPointToBlobList(-10, 20);
//	location = addPointToBlobList(20, 20);

	SetupTest1();

	int location = addPointToBlobList(-9, 1);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(-11, 11);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(-11, 9);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(21, 21);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}

	return true;
}


// X_X --> XxX

bool testTriangle1()
{
	SetupTestAreas();
	if (GetBlobCount() != 0)
	{
		printf("ERROR = Blobcount != 0\n");
		return false;
	}
	int location = addPointToBlobList(-1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(+1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList( 0, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 after combine, not %d\n", __FUNCTION__, GetBlobCount());

	return true;
}

// X_X 
// _X_ 
bool testTriangle2()
{
	SetupTestAreas();
	if (GetBlobCount() != 0)
	{
		printf("ERROR = Blobcount != 0\n");
		return false;
	}
	int location = addPointToBlobList(-1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(+1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(0, 3);
	if (location < 0) { printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(0, 1);
	if (location < 0) { printf("Error %s Failed to add new point at location 3  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 again after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(4, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 4  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2 again random add, not %d\n", __FUNCTION__, GetBlobCount());

	return true;
}


// X_X 
// ___
// X_X
// THen add the one in the middle
bool testTriangle3()
{
	SetupTestAreas();
	if (GetBlobCount() != 0)
	{
		printf("ERROR = Blobcount != 0\n");
		return false;
	}
	int location = addPointToBlobList(-1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(+1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(1, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 3)
		printf("ERROR: %s Blob Count should equal 3 after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(-1, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 4.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 4)
		printf("ERROR: %s Blob Count should equal 4 after combine, not %d\n", __FUNCTION__, GetBlobCount());



	location = addPointToBlobList(0, 3);
	if (location < 0) { printf("Error %s Failed to add new point at location 5  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 again after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(7, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 4  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2 again random add, not %d\n", __FUNCTION__, GetBlobCount());

	return true;
}


bool testAngle45()
{
	ClearBlobs();
	int location = addPointToBlobList(-10, 10);
	int16_t x=0, y=0, angleResult;
	bool result = false;
	uint8_t blob = 0;
	GetBlobCenter(blob, &x, &y);
	if ((x != -10) || (y != 10))
	{
		printf("ERROR: %s Blob center should be -10, 10, not %d  %d \n", __FUNCTION__, x,y);
		return false;
	}

	float angle = GetAngleToBlob(blob);
	if ((angle < -0.786) || (angle > -0.785))
	{
		printf("ERROR: %s Angle to blob sbould be -0.785, not %2.5f  \n", __FUNCTION__, angle);
		return false;
	}
	printf("Angle = %2.2f\n", angle);

	location = addPointToBlobList(-11, 10);
	location = addPointToBlobList(-12, 10);
	GetBlobCenter(blob, &x, &y);
	if ((x != -11) || (y != 10))
	{
		printf("ERROR: %s Blob center should be -11, 10, not %d  %d \n", __FUNCTION__, x, y);
		return false;
	}

	angle = GetAngleToBlob(blob);
	if ((angle < -0.738) || (angle > -0.736))
	{
		printf("ERROR: %s Angle to blob sbould be -0.737, not %2.5f  \n", __FUNCTION__, angle);
		return false;
	}

}

int RunBlobTests()
{
	bool result = true;
	result &= testAddPointToBlankList();
	result &= testHorzVertBasic1();
	result &= testCornerBasic1();
	result &= testTriangle1();
	result &= testTriangle2();
	result &= testTriangle3();
	result &= testAngle45();

	if (result != true)
	{
		printf("\n\n\n---------------FAIL FAIL FAIL ------------------------\n");
	}

}