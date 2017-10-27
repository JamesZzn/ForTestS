#ifndef _EOL_DATA_TYPE_H_
#define _EOL_DATA_TYPE_H_
#include "utility\common\reuse.h"
#include "eol_opencv_adapter.h"

#undef  EOL_MAX_NUM_LAYERS_
#define EOL_MAX_NUM_LAYERS_ ( 2 )

#define EOL_MAX_CORNER_NUM      (64) 

#define CAMERA_NUM (4)
#define CB_MAX_NUM (4)
#define CB_MAX_ROW (4)
#define CB_MAX_COL (15)

#define EOL_CORNER_NUM                         (8)
#define EOL_MAX_CC_ID_NUM                      (2048*2)
#define EOL_MAX_ROI_NUM                        (8)
#define EOL_MAX_CANDIDATEDREGION_NUM           (1000*2)
#define EOL_MAX_MAT_DIM                        (50)

#define EOL_DOUGLAS_ID_MAX                     (25)

#define EOL_W_extension (16)
#define EOL_out_maxpoint_num (16*2)


#define EOL_RETURN_ERROR  return MEM_MALLOC_FAIL;

#define EOL_SAFE_FREE(x)   ((x) = NULL)
#define EOL_MEM_ALLIGN (32)

#ifndef _TAG_SOURCE_IMAGE_TYPE_
#define _TAG_SOURCE_IMAGE_TYPE_
typedef enum EOL_Source_Image_Type_Tag
{
	TYPE_YUV420,
	TYPE_YUV422,
	TYPE_NV12,
	TYPE_BGR,
	TYPE_RGBA
}EOL_Source_Image_Type_T;
#endif

#ifndef _TAG_EOL_RESULT_
#define _TAG_EOL_RESULT_
typedef struct eol_result_
{
	CvPoint eol_corner_point[CAMERA_NUM][8];
	float64_t result_error[CAMERA_NUM];
}eol_result;
#endif

#ifndef _EOL_STR_WORLD_MODEL_
#define _EOL_STR_WORLD_MODEL_
typedef struct{
	float64_t _dist_axis ;
	float64_t _veh_len ;
	float64_t _veh_width ;
	float64_t _rear_axis_2_bumper ;
	float64_t _front_axis_2_head ;
} EOL_str_world_model ;
#endif

#ifndef _TAG_CHESSBOARD_CORNER_CONFIG
#define _TAG_CHESSBOARD_CORNER_CONFIG
typedef struct _tag_Chessboard_Corner_Config
{
	CvSize board_size;				 //板子尺寸模式4*4,4*3,4*2
	int32_t min_number_of_corners;   //最小corner数目，一般是板子模式长宽积
	int32_t correct_number_of_corners;
	int32_t correct_number_of_quads;
	//int32_t rect_mask_num;           //掩膜个数
	//int32_t reference_pointNum;      //参考点数目
	int32_t camid;                   //相机id
	float32_t width_ratio;           //1*1模式棋盘滤波位置参数
	float32_t height_ratio;          //1*1模式棋盘滤波位置参数
	float32_t RectMinAreaRatio;		 //1*1模式棋盘滤波面积参数
	int32_t reflection;              //是否使能处理反光操作
	int32_t gloabal_min_dilate;      //最小膨胀指数，0是保持原图，负数是腐蚀操作
}Chessboard_Corner_Config; 
#endif

#ifndef _STR_PATTERN_CONFIG
#define _STR_PATTERN_CONFIG
typedef struct _tag_EOL_pattern_config
{
	float32_t top_left_corner_x;
	float32_t top_left_corner_y;

	float32_t inner_quad_height;
	float32_t inner_quad_width;

	float32_t pattern_height;
	float32_t pattern_width;

	float32_t left_boundary_to_axis;
}EOL_pattern_config;	
#endif

#ifndef _TAG_CHESSBOARD_PATTERN_
#define _TAG_CHESSBOARD_PATTERN_
typedef struct _tag_Chessboard_Pattern
{
	EOL_pattern_config pattern_config;
	Chessboard_Corner_Config chessboard_config; // to be adjusted to outer struct
	Point2f corner_point[CB_MAX_ROW][CB_MAX_COL];
	Point2f ground_corner_point[CB_MAX_ROW][CB_MAX_COL];
	int32_t flag[CB_MAX_ROW][CB_MAX_COL];
	int32_t connected_type[CB_MAX_ROW][CB_MAX_COL];
	float max_edge_len_ratio[CB_MAX_ROW][CB_MAX_COL];
	float min_edge_len_ratio[CB_MAX_ROW][CB_MAX_COL];
	int32_t selected_corner_x;
	int32_t selected_corner_y;
	int32_t selected_corner_connected_type;
	int32_t rows;
	int32_t cols;
	int32_t valid_point_num;
}Chessboard_Pattern;
#endif

#ifndef _TAG_CBPATTERN_GROUP_
#define _TAG_CBPATTERN_GROUP_
typedef struct _tag_CbPattern_group
{
	int32_t valid_num; // valid pattern
	Chessboard_Pattern pattern[CB_MAX_NUM]; 	
	Point2f refer_point[CB_MAX_NUM]; // will be removed in the next version
	Point2f boundary_point[4][4]; // 4 boundary corner points of each patten
}CbPattern_group;
#endif

#ifndef _TAG_EOL_PARAM_
#define _TAG_EOL_PARAM
typedef struct _tag_EOL_Param
{
	int32_t station_type;
	float32_t station_height;
	float32_t station_width;
	float32_t board_2_veh_center_axis;
	CbPattern_group cb_pattern_group[CAMERA_NUM];
}EOL_Param, *P_EOL_Param;
#endif

#ifndef _TAG_STATION_TYPE_
#define _TAG_STATION_TYPE_
typedef enum _tag_Station_Type
{
	STATION_TYPE_OFILM = 0,
	STATION_TYPE_GAC,
	STATION_TYPE_JAC,
	STATION_TYPE_GEELY,
	STATION_TYPE_RESERVED
}Station_Type;
#endif


#ifndef _TAG_OPTIMIZE_TYPE_
#define _TAG_OPTIMIZE_TYPE_
typedef enum _tag_Optimize_Type
{
	OPTIMIZE_TYPE_NONE = 0,
	OPTIMIZE_TYPE_FOV,
	OPTIMIZE_TYPE_CENTER,
	OPTIMIZE_TYPE_BOTH
}Optimize_Type;
#endif

#endif