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

extern blobStruct_t blobList[BLOBS_IN_LIST];
extern blobDetailStruct_t blobDetails[BLOBS_IN_LIST];


int SetupTestAreas()
{
	ClearBlobs();
	ClearFinalLineData();


	return 0;

}

void SetupTest1()
{
	SetupTestAreas();
	int location = addPointToBlobList(80, 1);
	location = addPointToBlobList(80, 10);
	location = addPointToBlobList(80, 20);
	location = addPointToBlobList(100, 20);
}

bool testAddPointToBlankList()
{
	SetupTestAreas();
	int location = addPointToBlobList(0, 0);
	if (location <  0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
			return false;
	}

	location = addPointToBlobList(10, 10);
	if (location <  0)
	{
		printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location);
		return false;
	}

	location = addPointToBlobList(10, 20);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location);
		return false;
	}

	return true;
}

bool testHorzVertBasic1()
{
	SetupTest1();
	
	// X tests
	int location = addPointToBlobList(81, 0);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(80, 1);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location);
		return false;
	}

	// Y tests
	location = addPointToBlobList(80, 9);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(80, 11);
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

	int location = addPointToBlobList(79, 1);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(79, 11);
	if (location < 0)
	{
		printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location);
		return false;
	}
	location = addPointToBlobList(79, 9);
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
	int location = addPointToBlobList(3, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(+1, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList( 2, 2);
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
	int location = addPointToBlobList(0, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(2, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(1, 3);
	if (location < 0) { printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(0, 1);
	if (location < 0) { printf("Error %s Failed to add new point at location 3  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1 again after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(5, 5);
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
	int location = addPointToBlobList(0, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 0.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 1)
		printf("ERROR: %s Blob Count should equal 1, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(2, 2);
	if (location < 0) { printf("Error %s Failed to add new point at location 1.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 2)
		printf("ERROR: %s Blob Count should equal 2, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(2, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 2.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 3)
		printf("ERROR: %s Blob Count should equal 3 after combine, not %d\n", __FUNCTION__, GetBlobCount());

	location = addPointToBlobList(0, 4);
	if (location < 0) { printf("Error %s Failed to add new point at location 4.  %i\n", __FUNCTION__, location); return false; }
	if (GetBlobCount() != 4)
		printf("ERROR: %s Blob Count should equal 4 after combine, not %d\n", __FUNCTION__, GetBlobCount());



	location = addPointToBlobList(1, 3);
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
	int location = addPointToBlobList(80, 10);
	int16_t x=0, y=0;
	bool result = false;
	uint8_t blob = 0;
	GetBlobCenter(blob, &x, &y);
	if ((x != 80) || (y != 10))
	{
		printf("ERROR: %s Blob center should be 80, 10, not %d  %d \n", __FUNCTION__, x,y);
		return false;
	}

	float angle = GetRealAngleToBlob(blob);
	if (fabs(0.908-angle)>0.01)
	{
		printf("ERROR: %s Angle to blob sbould be -0.908, not %2.5f  \n", __FUNCTION__, angle);
		return false;
	}
//	printf("Angle = %2.2f\n", angle);

	location = addPointToBlobList(79, 10);
	location = addPointToBlobList(78, 10);
	GetBlobCenter(blob, &x, &y);
	if ((x != 79) || (y != 10))
	{
		printf("ERROR: %s Blob center should be 79, 10, not %d  %d \n", __FUNCTION__, x, y);
		return false;
	}

	angle = GetAngleToBlob(blob);
	if ((angle < -0.738) || (angle > -0.736))
	{
		printf("ERROR: %s Angle to blob sbould be -0.737, not %2.5f  \n", __FUNCTION__, angle);
		return false;
	}

	return true;
}



bool testGetLargestBlob()
{
	ClearBlobs();
	int location = addPointToBlobList(10, 10);
	
	location = addPointToBlobList(20, 10);
	location = addPointToBlobList(21, 10);

	location = addPointToBlobList(2, 10);
	location = addPointToBlobList(3, 9);

	if (GetBlobCount() != 3)
	{
		printf("ERROR: %s Number of blobs should be 3 and not %d  \n", __FUNCTION__, GetBlobCount());
		return false;
	}

	if (GetLargestBlob() != 2)
	{
		printf("ERROR: %s GetLargetBlob should be 2 and not %d  \n", __FUNCTION__, GetLargestBlob());
		return false;

	}

	return true;
}

bool testGetDistanceToBlobCenter()
{
	ClearBlobs();
	int location = addPointToBlobList(80, 10);

	uint16_t distance = GetDistanceToBlobCenter(0);
	if (distance != 14)
	{
		printf("ERROR: %s GetDistanceToBlobCenter() should be 14 and not %d  \n", __FUNCTION__, distance);
		return false;

	}
	return true;
}

bool testPrintBlobs()
{
	ClearBlobs();
	int location = addPointToBlobList(80, 10);

	location = addPointToBlobList(85, 10);
	location = addPointToBlobList(85, 10);

	location = addPointToBlobList(102, 10);
	location = addPointToBlobList(103, 9);
	PrintBlobList();
	return true;
}


bool testAliasBlobs1()
{
	ClearBlobs();

	int count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(82, 5);
	count = addPointToBlobList(82, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(85, 5);



	
/*	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(96, 4);
	addPointToBlobList(97, 4);
	addPointToBlobList(97, 4);
	addPointToBlobList(97, 4); */
//	PrintBlobList();

	uint16_t distance = GetDistanceToBlobCenter(0);
	int16_t count1 = GetBlobCount();
	uint8_t hits = GetBlobHits(0);
	if (count1 != 1)
	{
		printf("ERROR %s Expected Blob count of 1, got %d\n", __FUNCTION__, count1);
		return false;
	}
	if (hits != 18)
	{
		printf("ERROR %s Expected Blob hits of 18, got %d\n", __FUNCTION__, hits);
		return false;
	}

	return true;
}

bool testAliasBlobs2()
{
	ClearBlobs();

	int count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(97, 4);
	count = addPointToBlobList(97, 4);
	count = addPointToBlobList(97, 4);
//	PrintBlobList();
	uint16_t distance = GetDistanceToBlobCenter(0);
	int16_t count1 = GetBlobCount();
	uint8_t hits = GetBlobHits(0);
	if (count1 != 1)
	{
		printf("ERROR %s Expected Blob count of 1, got %d\n", __FUNCTION__, count1);
		return false;
	}
	if (hits != 11)
	{
		printf("ERROR %s Expected Blob hits of 11, got %d\n", __FUNCTION__, hits);
		return false;
	}

	return true;

}

bool testAliasBlobs3()
{
	ClearBlobs();

	int count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(96, 4);
	count = addPointToBlobList(97, 4);
	count = addPointToBlobList(97, 4);
	count = addPointToBlobList(97, 4);

    count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(85, 6);
	count = addPointToBlobList(82, 5);
	count = addPointToBlobList(82, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(83, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(84, 5);
	count = addPointToBlobList(85, 5);


//	PrintBlobList();
	uint16_t distance = GetDistanceToBlobCenter(0);
	int16_t count1 = GetBlobCount();
	uint8_t hits = GetBlobHits(0);
	if (count1 != 2)
	{
		printf("ERROR %s Expected Blob count of 2, got %d\n", __FUNCTION__, count1);
		return false;
	}
	if (hits != 11)
	{
		printf("ERROR %s Expected Blob hits of 11, got %d\n", __FUNCTION__, hits);
		return false;
	}

	return true;

}


bool testDetailedBlobs1()
{
	ClearBlobs();
	bool result = true;
	int8_t	value = addDetailedPointToBlobList(85, 6, -455, 604);
	value = addDetailedPointToBlobList(85, 6, -442, 609);
	value = addDetailedPointToBlobList(85, 6, -431, 616);
	value = addDetailedPointToBlobList(85, 6, -421, 624);
	value = addDetailedPointToBlobList(85, 6, -410, 632);
	value = addDetailedPointToBlobList(85, 6, -443, 608);


/*	value = addDetailedPointToBlobList(82, 5, -640, 518);
	value = addDetailedPointToBlobList(82, 5, -630, 529);
	value = addDetailedPointToBlobList(83, 5, -612, 532);
	value = addDetailedPointToBlobList(83, 5, -598, 539);
	value = addDetailedPointToBlobList(83, 5, -582, 542);
	value = addDetailedPointToBlobList(83, 5, -569, 550);
	value = addDetailedPointToBlobList(83, 5, -555, 555);
	value = addDetailedPointToBlobList(84, 5, -542, 561);
	value = addDetailedPointToBlobList(84, 5, -529, 568);
	value = addDetailedPointToBlobList(84, 5, -515, 572);
	value = addDetailedPointToBlobList(84, 5, -503, 579);
	value = addDetailedPointToBlobList(84, 5, -489, 583);
	value = addDetailedPointToBlobList(84, 5, -478, 590);
	value = addDetailedPointToBlobList(85, 5, -467, 598);
	value = addDetailedPointToBlobList(96, 4, 490, 490);
	value = addDetailedPointToBlobList(96, 4, 497, 480);
	value = addDetailedPointToBlobList(96, 4, 504, 470);
	value = addDetailedPointToBlobList(96, 4, 511, 460);
	value = addDetailedPointToBlobList(96, 4, 517, 450);
	value = addDetailedPointToBlobList(96, 4, 524, 440);
	value = addDetailedPointToBlobList(96, 4, 531, 430);
	value = addDetailedPointToBlobList(96, 4, 538, 421);
	value = addDetailedPointToBlobList(97, 4, 547, 412);
	value = addDetailedPointToBlobList(97, 4, 584, 409);
	value = addDetailedPointToBlobList(97, 4, 595, 401); */
	if (blobDetails[0].minX != -455)
	{
		printf("ERROR %s MinX Not eqaul to -455.  Its %i\n", __FUNCTION__, blobDetails[0].minX);
		result = false;
	}
	if (blobDetails[0].maxX != -410)
	{
		printf("ERROR %s MaxX Not eqaul to -410.  Its %i\n", __FUNCTION__, blobDetails[0].maxX);
		result = false;
	}
	if (blobDetails[0].minY != 604)
	{
		printf("ERROR %s MinY Not eqaul to 604.  Its %i\n", __FUNCTION__, blobDetails[0].minY);
		result = false;
	}
	if (blobDetails[0].maxY != 632)
	{
		printf("ERROR %s MaxY Not eqaul to 632.  Its %i\n", __FUNCTION__, blobDetails[0].maxY);
		result = false;
	}

	PrintBlobList();
	return result;
}


bool testDetailedBlobs2()
{
	ClearBlobs();
	bool result = true;
	int8_t	value = addDetailedPointToBlobList(85, 6, -455, 604);
	value = addDetailedPointToBlobList(85, 6, -442, 609);
	value = addDetailedPointToBlobList(85, 6, -431, 616);
	value = addDetailedPointToBlobList(85, 6, -421, 624);
	value = addDetailedPointToBlobList(85, 6, -410, 632);
	value = addDetailedPointToBlobList(85, 6, -443, 608); 


  		value = addDetailedPointToBlobList(85, 5, -467, 598); 
		value = addDetailedPointToBlobList(82, 5, -640, 518);
		value = addDetailedPointToBlobList(82, 5, -630, 529);
		value = addDetailedPointToBlobList(83, 5, -612, 532);
		value = addDetailedPointToBlobList(83, 5, -598, 539);
		value = addDetailedPointToBlobList(83, 5, -582, 542);
		value = addDetailedPointToBlobList(83, 5, -569, 550);
		value = addDetailedPointToBlobList(83, 5, -555, 555);
		value = addDetailedPointToBlobList(84, 5, -542, 561);
		value = addDetailedPointToBlobList(84, 5, -529, 568);
		value = addDetailedPointToBlobList(84, 5, -515, 572);
		value = addDetailedPointToBlobList(84, 5, -503, 579);
		value = addDetailedPointToBlobList(84, 5, -489, 583);
		value = addDetailedPointToBlobList(84, 5, -478, 590);
		
		value = addDetailedPointToBlobList(96, 4, 490, 490);
		value = addDetailedPointToBlobList(96, 4, 497, 480);
		value = addDetailedPointToBlobList(96, 4, 504, 470);
		value = addDetailedPointToBlobList(96, 4, 511, 460);
		value = addDetailedPointToBlobList(96, 4, 517, 450);
		value = addDetailedPointToBlobList(96, 4, 524, 440);
		value = addDetailedPointToBlobList(96, 4, 531, 430);
		value = addDetailedPointToBlobList(96, 4, 538, 421);
		value = addDetailedPointToBlobList(97, 4, 547, 412);
		value = addDetailedPointToBlobList(97, 4, 584, 409);
		value = addDetailedPointToBlobList(97, 4, 595, 401); 


	if (blobDetails[0].minX != -640)
	{
		printf("ERROR %s MinX Not eqaul to -640.  Its %i\n", __FUNCTION__, blobDetails[0].minX);
		result = false;
	}
	if (blobDetails[0].maxX != -410)
	{
		printf("ERROR %s MaxX Not eqaul to -410.  Its %i\n", __FUNCTION__, blobDetails[0].maxX);
		result = false;
	}
	if (blobDetails[0].minY != 518)
	{
		printf("ERROR %s MinY Not eqaul to 518.  Its %i\n", __FUNCTION__, blobDetails[0].minY);
		result = false;
	}
	if (blobDetails[0].maxY != 632)
	{
		printf("ERROR %s MaxY Not eqaul to 632.  Its %i\n", __FUNCTION__, blobDetails[0].maxY);
		result = false;
	}
	uint8_t hits = GetBlobHits(0);
	if (hits != 20)
	{
		printf("ERROR %s Expected Blob hits of 20, got %d\n", __FUNCTION__, hits);
		return false;
	}

	PrintBlobList();
	return result;
}


int RunBlobTests()
{
	bool result = true;
	rb_begin();
	result &= testAliasBlobs1();
	result &= testAliasBlobs2();
	result &= testAliasBlobs3();
	result &= testDetailedBlobs1();
	result &= testDetailedBlobs2();

	result &= testAddPointToBlankList();
	result &= testHorzVertBasic1();
	result &= testCornerBasic1();
	result &= testTriangle1();
	result &= testTriangle2();
	result &= testTriangle3();
	result &= testAngle45();
	result &= testGetLargestBlob();
	result &= testGetDistanceToBlobCenter();
	result &= testPrintBlobs();



	if (result != true)
	{
		printf("\n\n\n---------------FAIL FAIL FAIL ------------------------\n");
	}
	return result;
}