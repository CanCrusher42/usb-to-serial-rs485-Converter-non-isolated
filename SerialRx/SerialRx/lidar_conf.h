#define LIDAR_MAX_X_RADIUS     6000 
#define LIDAR_MAX_Y_RADIUS     2500
#define LIDAR_BLOB_ROWS        20U
#define LIDAR_BLOB_COLUMNS     180U 
#define LIDAR_BLOB_ROW_RESOLUTION    (LIDAR_MAX_Y_RADIUS/LIDAR_BLOB_ROWS)    
#define LIDAR_BLOB_COLUMN_RESOLUTION (2*LIDAR_MAX_X_RADIUS/LIDAR_BLOB_COLUMNS)  



struct S_LidarDefines {
    int16_t LidarAbsMaxY;           // Maximum y that will not be clipped
    int16_t LidarMaxX;           // Maximum X that will not be clipped (Neg and Pos) 
    int16_t LidarMinX;           // Maximum X that will not be clipped (Neg and Pos) 
    int16_t LidarFenceAbsMaxY;      // Maximum Y to be included in Blob Y Calculations
    int16_t LidarFenceMinX;         // Minimum X to be included in Blob X Calculations
    int16_t LidarFenceMaxX;         // Maximum X to be included in Blob X Calculations
    int16_t BlobNumRows;            // Number of rows in the blob.    
    int16_t BlobNumColumns;         // 180, used for 1 degree per column
    
    int16_t BlobXPerColumn;         // CEIL/FLOOR of (2*LidarFenceMaxX)/BlobNumColumns 
    int16_t BlobYPerRow;            // CEIL/FLOOR of (LidarFenceMaxY/BlobNumColumns)

} ;

typedef struct S_LidarDefines LidarDefinesStruct;


