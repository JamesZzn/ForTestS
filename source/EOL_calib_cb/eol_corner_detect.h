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
	IplImage *img_in;                               //����ͼ��ָ��
	//int elem_size;                                //���ݰ���ģʽ����ĵ�ռ��С
	//CvPoint2D32f* image_points_buf;               //���ݴ���
	//Rect2f rect_mask_roi[4];                      //��Ĥλ��
	Point2f reference_point[4];                     //�ο���
	Point2f boundary_point[4][4];                   //���ÿ��pattern��4���߽�ǵ�
	Chessboard_Pattern pattern_corner[2];           //�����4*4,4*3,4*2 pattern, modified by Yanshuo noopencv version is not array
	Chessboard_Pattern pattern_Single_corner[2];    //���������1*1 pattern
	Chessboard_Corner_Config corner_config;         //����ļ�����ò���
	PatternSide pattern_side;                       //��ǰ���̸�Ӧ��ͼ�����ʱΪ2���Ҳ�ʱΪ1���������ʱΪ0
	int32_t station_type;

	int corner_count;
	int valid_num;

	void init();
	void deinit();
	int cvFindChessboardCorners3(const IplImage* img);          //4*4,4*3,4*2���̼�⺯��
	int cvFindChessboardCorners2(const IplImage* img);		 //1*1���̼��

	int icvGenerateQuads( CvCBQuad **quads, CvCBCorner **corners,
								 CvMemStorage *storage, IplImage *image, int flags, int dilation,
								 bool firstRun );                              //�����в��������

	void mrFindQuadNeighbors2( CvCBQuad *quads, int quad_count, int dilation); //Ѱ�����ڵĶ����

	int mrAugmentBestRun( CvCBQuad *new_quads, int new_quad_count, int new_dilation,      //������©�Ķ����Ⱥ
								 CvCBQuad **old_quads, int old_quad_count, int old_dilation );

	int icvFindConnectedQuads( CvCBQuad *quads, int quad_count, CvCBQuad **quad_group,
									  int group_idx,
									  CvMemStorage* storage, int dilation );            //���Ҷ����Ⱥ

	void mrLabelQuadGroup( CvCBQuad **quad_group, int count, CvSize pattern_size, 
								  bool firstRun );                                     //��Ƕ����Ⱥ

	void mrCopyQuadGroup( CvCBQuad **temp_quad_group, CvCBQuad **out_quad_group, 
								 int count );

	int icvCleanFoundConnectedQuads( int quad_count, CvCBQuad **quads, 
											CvSize pattern_size );                    //�˳�������pattern�ߴ�Ķ����Ⱥ

	int mrWriteCorners( CvCBQuad **output_quads, int count, CvSize pattern_size,
							   int min_number_of_corners );                          //�������cornerȺλ��
	void PointLine_dist(Point2f point_in,Point2f line_start,Point2f line_end,float &dist );

	double triangleArea(Point2f a, Point2f b, Point2f c);                              //��������abc��ɵ����������

	bool pInQuadrangle(Point2f a, Point2f b, Point2f c, Point2f d, Point2f p);           //�жϵ�p�Ƿ����ı���abcd�ڣ������ڲ�������1�����򣬷���0

	void removeInvalidQuads(CvCBQuad** quad_group, CvSize board_size, int& count);        //�˳�λ�ò���ȷ�Ķ����
}chessBoard_corner_detector;

typedef struct
{
	CvCBQuad rect[2];
}CvCBQuad2;

bool SaveRawFile1(const char* sfilename, char *pBuffer, int iSize);
#endif