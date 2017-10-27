#ifndef EOL_DEBUG_H_
#define EOL_DEBUG_H_
#include "eol_pose_estimator.h"
#include "eol_image_process.h"

// for drawing BEV result
typedef struct CarParam_Tag
{
	CvPoint	carFrontLeft;
	CvPoint	carFrontRight;
	CvPoint	carRearLeft;
	CvPoint	carRearRight;
	CvPoint	boundaryLeftFront;
	CvPoint	boundaryFrontLeft;
	CvPoint	boundaryRightFront;
	CvPoint	boundaryFrontRight;
	CvPoint	boundaryLeftRear;
	CvPoint	boundaryRearLeft;
	CvPoint	boundaryRightRear;
	CvPoint	boundaryRearRight;
	int carWidth;
	int carHeight;
}CarParam;

void mark_corners(IplImage* img,
				  float64_t avm_pose[6],
				  float64_t camera_center[2],
				  float64_t camera_fov[2],
				  Smc_Cal_T* pSMC, 
				  EOL_str_cam_intrin* bev_cam,
				  EOL_str_world_model* world_model,
				  const Chessboard_Pattern* const pattern, 
				  int pattern_num);

int32_t EOL_draw_corners(IplImage* rgb_img, 
						 EOL_Buffer_Mng_T* calib_data_buff, 
						 Smc_Cal_T* const pSMC, 
						 EOL_str_avm_intrin* avm_cam_bev, 
						 EOL_str_world_model* world_model, 
						 const Chessboard_Pattern* const pattern, 
						 int pattern_num, 
						 int camid);

void fill_bev_image(IplImage* rgb_img[CAMERA_NUM], 
					uchar* yuv420_img,
					EOL_str_cam_intrin camera_model[4],
					Smc_Cal_T* pSMC, 
					EOL_str_world_model* world_model,
					CarParam* car_param, 
					float64_t R[][9],
					float64_t pose_rt[][6],
					float64_t slope[][2],
					float64_t mm_per_pixel,
					int32_t width,
					int32_t height);

int32_t EOL_draw_world_coordinate(P_EOL_Param p_Eol_param);

int32_t EOL_save_images(const IplImage* rgb_img, FILE* p_file, uchar* p_result);

int32_t EOL_draw_stitch_result(IplImage* rgb_img[4], 
							   EOL_Buffer_Mng_T* calib_data_buff, 
							   Smc_Cal_T* pSMC, 
							   EOL_str_avm_intrin* avm_cam_bev,
							   EOL_str_world_model* world_model);

#endif