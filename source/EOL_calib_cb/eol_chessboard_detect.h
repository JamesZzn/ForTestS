#include "eol_corner_detect.h"
#include "eol_utility.h"

static void EOL_config_detecter(chessBoard_corner_detector* p_corner_detecter, 
								CbPattern_group* p_cb_pattern_group,
								P_EOL_Param p_Eol_param, 
								int32_t* p_pattern_index,
								int32_t camid,
								int32_t pattern_id);

static void EOL_copy_corner_and_flag(Point2f p_corner_dst[CB_MAX_ROW][CB_MAX_COL],
									 Point2f p_corner_src[CB_MAX_ROW][CB_MAX_COL],
									 int32_t p_flag_dst[CB_MAX_ROW][CB_MAX_COL],
									 int32_t p_flag_src[CB_MAX_ROW][CB_MAX_COL]);

static void EOL_copy_corner_to_structure(chessBoard_corner_detector corner_detector,
										 Chessboard_Pattern* pattern,
										 P_EOL_Param p_Eol_param,
										 int32_t pattern_index[CB_MAX_NUM],
										 Point2f refer_point[CB_MAX_NUM],
										 Point2f boundary_point[4][4],
										 int32_t cam_id,
										 int32_t& pattern_id,
										 int32_t pattern_corner_num,
										 int32_t ret_corner_detect);

int32_t EOL_chessboard_detect(IplImage* sProcessImg_in[CAMERA_NUM], P_EOL_Param p_Eol_param);

bool SaveRawFile(const char* sfilename, char *pBuffer, int iSize);