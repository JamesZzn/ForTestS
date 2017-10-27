#if OPENCV
#include <opencv.hpp>
//#include <opencv2\core\internal.hpp>
#else
#include <eol_opencv_adapter.h>
#endif

#include "utility\common\reuse.h"
#include "utility\common\bev_data_type.h"
#include "eol_data_type.h"
#include "eol_config.h"

#include <time.h>
#include <fstream>
using namespace std;
using std::ifstream;

#ifndef __CVCALIBINIT3_H__
#define __CVCALIBINIT3_H__
// Defines
#define MAX_CONTOUR_APPROX  7
#define reference_dist 60
#define reference_offset 120

//Ming #define VIS 1    
#define DEBUG_FILE_PATH "E:/work/EOL_product\\temp"


// Definition Contour Struct
typedef struct CvContourEx
{
    CV_CONTOUR_FIELDS()
    int counter;
}
CvContourEx;


typedef enum PatternSide
{
	SIDE_LEFT = 0,
	SIDE_RIGHT = 1,
	SIDE_BOTH = 2
}PatternSide;


// Definition Corner Structure
typedef struct CvCBCorner
{
	CvPoint2D32f pt;					// X and y coordinates
	int row;							// Row and column of the corner 
	int column;							// in the found pattern
	bool needsNeighbor;					// Does the corner require a neighbor?
	int count;							// number of corner neighbors
	int connected_way;                    // connected way(2:0-2, 3:1-3, -1:non connected)
	float max_edge_len_ratio;                 // edge_len1/edge_len2
	float min_edge_len_ratio;                 // edge_len1/edge_len2
	struct CvCBCorner* neighbors[4];	// pointer to all corner neighbors
}
CvCBCorner;


// Definition Quadrangle Struct
// This structure stores information about the chessboard quadrange
typedef struct CvCBQuad
{
	int count;							// Number of quad neihbors
	int group_idx;						// Quad group ID
	float min_edge_len;						// Smallest side length^2
	float max_edge_len;                       //  Largest side length^2
	CvCBCorner *corners[4];				// Coordinates of quad corners
	struct CvCBQuad *neighbors[4];		// Pointers of quad neighbors
	bool labeled;						// Has this corner been labeled?
}
CvCBQuad;

typedef struct
{
	IplImage *img_in;                               //输入图像指针
	//int elem_size;                                //根据板子模式分配的点空间大小
	//CvPoint2D32f* image_points_buf;               //点暂存区
	//Rect2f rect_mask_roi[4];                      //掩膜位置
	Point2f reference_point[4];                     //参考点
	Point2f boundary_point[4][4];                   //存放每块pattern的4个边界角点
	Chessboard_Pattern pattern_corner[2];           //输出的4*4,4*3,4*2 pattern, modified by Yanshuo noopencv version is not array
	Chessboard_Pattern pattern_Single_corner[2];    //输出的两个1*1 pattern
	Chessboard_Corner_Config corner_config;         //输入的检测配置参数
	PatternSide pattern_side;                       //当前棋盘格应在图像左侧时为2，右侧时为1，两侧均可时为0
	int32_t station_type;

	int corner_count;
	int valid_num;

	void init();
	void deinit();
	int cvFindChessboardCorners3(const IplImage* img);          //4*4,4*3,4*2棋盘检测函数
	int cvFindChessboardCorners2(const IplImage* img);		 //1*1棋盘检测

	int icvGenerateQuads( CvCBQuad **quads, CvCBCorner **corners,
								 CvMemStorage *storage, IplImage *image, int flags, int dilation,
								 bool firstRun );                              //棋盘中产生多边形

	void mrFindQuadNeighbors2( CvCBQuad *quads, int quad_count, int dilation); //寻找相邻的多边形

	int mrAugmentBestRun( CvCBQuad *new_quads, int new_quad_count, int new_dilation,      //查找遗漏的多边形群
								 CvCBQuad **old_quads, int old_quad_count, int old_dilation );

	int icvFindConnectedQuads( CvCBQuad *quads, int quad_count, CvCBQuad **quad_group,
									  int group_idx,
									  CvMemStorage* storage, int dilation );            //查找多边形群

	void mrLabelQuadGroup( CvCBQuad **quad_group, int count, CvSize pattern_size, 
								  bool firstRun );                                     //标记多边形群

	void mrCopyQuadGroup( CvCBQuad **temp_quad_group, CvCBQuad **out_quad_group, 
								 int count );

	int icvCleanFoundConnectedQuads( int quad_count, CvCBQuad **quads, 
											CvSize pattern_size );                    //滤除不符合pattern尺寸的多边形群

	int mrWriteCorners( CvCBQuad **output_quads, int count, CvSize pattern_size,
							   int min_number_of_corners );                          //输出最后的corner群位置
	void PointLine_dist(Point2f point_in,Point2f line_start,Point2f line_end,float &dist );

	double triangleArea(Point2f a, Point2f b, Point2f c);                              //计算三点abc组成的三角形面积

	bool pInQuadrangle(Point2f a, Point2f b, Point2f c, Point2f d, Point2f p);           //判断点p是否在四边形abcd内，若在内部，返回1，否则，返回0

	void removeInvalidQuads(CvCBQuad** quad_group, CvSize board_size, int& count);        //滤除位置不正确的多边形
}chessBoard_corner_detector;

typedef struct
{
	CvCBQuad rect[2];
}CvCBQuad2;

bool SaveRawFile1(const char* sfilename, char *pBuffer, int iSize);
#endif