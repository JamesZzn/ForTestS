#include "eol_debug.h"
#include "eol_config.h"

extern uchar* yuv420_img;

char FILE_SAVE_PATH[200] = "E:/project/EOL_product\\temp";
int32_t frame_count = 0;

#define DRAW_DETECTED_CORNERS
//#define DRAW_REPROJECTED_CORNERS

/*
	Function Name:      Mark_corners
	Function Function : Draw the detected corners on source image
	Input             : 
	                    rgb_img    : the rgb image
						pattern    : pattern information
						pattern_num: valid pattern
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/02/07       Create
						YanShuo        2017/02/23       Each row is drawn with a type of color 
*/
void mark_corners(IplImage* img, 
	              float64_t avm_pose[6], 
				  float64_t camera_center[2],
				  float64_t camera_fov[2],
				  Smc_Cal_T* pSMC, 
				  EOL_str_cam_intrin* bev_cam,
				  EOL_str_world_model* world_model,
				  const Chessboard_Pattern* const pattern, 
				  int32_t pattern_num)
{
	float64_t pxyz[3], tvec[3], ray[3], R[9], uv[2];
	EOL_str_cam_intrin camera_model;

	Cvt_Angles_To_Rotation_DB(R, avm_pose); // R is cam2world
	Inverse_RT_InPlace_DB(R, &avm_pose[3]); // update R = R' and t = -R'T // world2cam
	Cvt_Rotation_To_Angles_DB(R, avm_pose); // update r in order x, y and z

	memcpy(tvec, &avm_pose[3], sizeof(float64_t) * 3);	
	memcpy(&camera_model, bev_cam, sizeof(EOL_str_cam_intrin));
	camera_model.cu = camera_center[0];
	camera_model.cv = camera_center[1];
	
	Cam_InitIntrinsic64(bev_cam,
		bev_cam->w,
		bev_cam->h ,
		bev_cam->cu ,
		bev_cam->cv,
		bev_cam->skew_c , 
		bev_cam->skew_d , 
		bev_cam->skew_e ,
		bev_cam->lut,
		camera_fov[0], 
		camera_fov[1], 
		true) ;

	uchar* pData = (uchar*)img->imageData;
	/*draw circle on bgr image*/
	for(int32_t num = 0; num < pattern_num; num++)
	{
		for(int32_t i = 0; i < pattern[num].rows; i++)
		{
			for(int32_t j = 0;j < pattern[num].cols; j++)
			{
				if(pattern[num].flag[i][j] > 0)
				{
#ifdef DRAW_DETECTED_CORNERS
					// draw corner points
					CvPoint pt = cvPoint((int32_t)pattern[num].corner_point[i][j].x, (int32_t)pattern[num].corner_point[i][j].y);
					for(int32_t m = -5; m <= 5; m++)
					{
						for(int32_t n = -5; n <= 5; n++)
						{
							float64_t radius = sqrt((float64_t)m*m + n*n);
							if(radius > 2.99 && radius < 5.01)
							{
								int cur_x, cur_y;
								cur_y = pt.y + m;
								cur_x = pt.x + n;
								if(cur_y >= 0 && cur_y <= img->height - 1 && cur_x >= 0 && cur_x <= img->width - 1)
								{
									if(1 == img->nChannels)
									{
										pData[cur_y*img->widthStep + cur_x*img->nChannels] = 255 - 20 * i;
									}
									else
									{
										int color[4][3] = {0, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 0};
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 0] = color[i][0];
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 1] = color[i][1];
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 2] = color[i][2];
									}
								}
							}
						}
					}
#endif
#ifdef DRAW_REPROJECTED_CORNERS
					// draw re-projected points

					pxyz[0] = pattern[num].ground_corner_point[i][j].x + pSMC->bev_calib_param.bev_eol_param.board_veh_flboard2fwheel
						+ world_model->_dist_axis -pSMC->bev_calib_param.bev_eol_param.station_height * 0.5;
					pxyz[1] = pattern[num].ground_corner_point[i][j].y;
					pxyz[2] = 0;

					ray[0] = (R[0] * pxyz[0] + R[1] * pxyz[1] + tvec[0]);
					ray[1] = (R[3] * pxyz[0] + R[4] * pxyz[1] + tvec[1]);
					ray[2] = (R[6] * pxyz[0] + R[7] * pxyz[1] + tvec[2]);

					Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model);
					
					// draw cross on world coordinate reprojected point
					for (int32_t m = -5; m <= 5; m++)
					{
						for (int32_t n = -5; n <= 5; n++)
						{
							if (m >= -1 && m <= 1 || n >= -1 && n <= 1)
							{
								int32_t cur_x, cur_y;
								cur_y = uv[1] + m;
								cur_x = uv[0] + n;
								if (cur_y >= 0 && cur_y <= img->height - 1 && cur_x >= 0 && cur_x <= img->width - 1)
								{
									if (1 == img->nChannels)
									{
										pData[cur_y*img->widthStep + cur_x*img->nChannels] = 227 + 20 * i;
									}
									else
									{
										int32_t color[4][3] = { 0, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 0 };
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 0] = color[i][0];
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 1] = color[i][1];
										pData[cur_y*img->widthStep + cur_x*img->nChannels + 2] = color[i][2];
									}
								}
							}
						}
					}
#endif		
				}
			}
		}
	}
}

/*
    Function Name     : EOL_draw_corners
	Function Function : Draw circle onto src image's corners
	Input:
		rgb_img:        Src image
		calib_data_buff:The optimized camera extrinsic params and camera center(if optimized)
		pSMC          : The SMC information
		avm_cam_bev   : Camera intrinsic params
		world_model   : the transformation between vehicle coordinate system and world coordinate system
		pattern:        Detected pattern array
		pattern_num:    The valid pattern num in current camera
		camid:          The camera id
	Return:             Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/01/09       Create
 */
int32_t EOL_draw_corners(IplImage* rgb_img, 
	                     EOL_Buffer_Mng_T* calib_data_buff, 
						 Smc_Cal_T* const pSMC, 
						 EOL_str_avm_intrin* avm_cam_bev,
						 EOL_str_world_model* world_model,
						 const Chessboard_Pattern* const pattern, 
						 int32_t pattern_num, 
						 int32_t camid)
{
	char str[256];
	int32_t height, width, half_height, half_width, pitch;
	FILE* p_file = NULL;
	EOL_str_cam_intrin* bev_cam = NULL;
	float64_t avm_pose[6], camera_center[2], camera_fov[2];

	switch (camid)
	{
	case 0:
		bev_cam = (EOL_str_cam_intrin*)avm_cam_bev->_cam_front;
		break;
	case 1:
		bev_cam = (EOL_str_cam_intrin*)avm_cam_bev->_cam_rear;
		break;
	case 2:
		bev_cam = (EOL_str_cam_intrin*)avm_cam_bev->_cam_left;
		break;
	case 3:
		bev_cam = (EOL_str_cam_intrin*)avm_cam_bev->_cam_right;
		break;
	}

	camera_center[0] = bev_cam->cu;
	camera_center[1] = bev_cam->cv;

	camera_fov[0] = pSMC->camera_param[camid].cam_int.cam_int_fu_or_hfov_at_cu;
	camera_fov[1] = pSMC->camera_param[camid].cam_int.cam_int_fv_or_vfov_at_cv;

	height = rgb_img->height;
	width = rgb_img->width;
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;
	
	sprintf(str, "%s/corner_result_frame%d_cam_%d_%dx%d.I420", FILE_SAVE_PATH, frame_count, camid, width, height);
		
	memcpy(avm_pose, (int8_t*)calib_data_buff->address + sizeof(float64_t) * 6 * camid, sizeof(float64_t) * 6);

	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:		
		break;
	case OPTIMIZE_TYPE_CENTER:
		memcpy(camera_center, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * camid), sizeof(float64_t) * 2);
		break;
	case OPTIMIZE_TYPE_FOV:
		memcpy(camera_fov, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * camid), sizeof(float64_t) * 2);
		break;	
	case OPTIMIZE_TYPE_BOTH:
		memcpy(camera_center, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * camid), sizeof(float64_t) * 2);
		memcpy(camera_fov, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 8 + 2 * camid), sizeof(float64_t) * 2);
		break;
	default:
		break;
	}

	mark_corners(rgb_img, avm_pose, camera_center, camera_fov, pSMC, bev_cam, world_model, pattern, pattern_num);

	uchar* yuv420_img = (uchar*)malloc(sizeof(uchar) * pitch * 3 >> 1);
	if (NULL == yuv420_img)
	{
		return MEM_MALLOC_FAIL;
	}
	memset(yuv420_img, 0, sizeof(uchar) * pitch * 3 >> 1);

	bgr_to_yuv420(rgb_img, yuv420_img);

	//save file	
	p_file = fopen(str, "wb");
	if(NULL == p_file)
	{
		return OPEN_FILE_ERROR;
	}
	fwrite(yuv420_img, sizeof(uchar) * pitch * 3 >> 1, 1, p_file);
	fclose(p_file);

	return 0;
}

/*
    Function Name     : set_car_info
	Function Function : set car_param structure by assigned params
	Input:
		dstImgSize :    bev image size
		horiTrans :     trans in horizontal direction
		vertTrans:      trans in vertical direction
		carSize:        car's real size
		car_param:      the car param structure to be output
		slope:          the boundary's slope in x and y direction respectively
	Return:             Error code
	Note              : In this function, the coordinate system direction is the same with 
	                    image coordinate system, i corresponding to y, j corresponding to x
						original point is the up-left point of the image
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/04/07       Create
 */
void set_car_info(CvSize dstImgSize, 
				  int32_t horiTrans, 
				  int32_t vertTrans, 
				  CvSize carSize, 
				  CarParam* car_param, 
				  float64_t slope[8][2])
{
	car_param->boundaryFrontLeft.x = horiTrans;
	car_param->boundaryFrontLeft.y = 0;
	car_param->boundaryFrontRight.x = dstImgSize.width - horiTrans - 1;
	car_param->boundaryFrontRight.y = 0;
	car_param->boundaryLeftFront.x = 0;
	car_param->boundaryLeftFront.y = vertTrans;
	car_param->boundaryRightFront.x = dstImgSize.width - 1;
	car_param->boundaryRightFront.y = vertTrans;
	car_param->boundaryRearLeft.x = horiTrans;
	car_param->boundaryRearLeft.y = dstImgSize.height - 1;
	car_param->boundaryRearRight.x = dstImgSize.width - horiTrans - 1;
	car_param->boundaryRearRight.y = dstImgSize.height - 1;
	car_param->boundaryLeftRear.x = 0;
	car_param->boundaryLeftRear.y = dstImgSize.height - vertTrans - 1;
	car_param->boundaryRightRear.x = dstImgSize.width - 1;
	car_param->boundaryRightRear.y = dstImgSize.height - vertTrans - 1;

	carSize.width -= carSize.width % 4; // make sure carWidth is 4's times
	carSize.height -= carSize.height % 2; // make sure carHeight is 2's times
	car_param->carWidth = carSize.width;
	car_param->carHeight = carSize.height;

	car_param->carFrontLeft.x = dstImgSize.width / 2 - carSize.width / 2;
	if(car_param->carFrontLeft.x % 2) 
		car_param->carFrontLeft.x++;
	car_param->carFrontLeft.y = dstImgSize.height / 2 - carSize.height / 2;

	car_param->carRearRight.x = car_param->carFrontLeft.x + carSize.width - 1;
	car_param->carRearRight.y = car_param->carFrontLeft.y + carSize.height - 1;

	car_param->carFrontRight.x = car_param->carRearRight.x;
	car_param->carFrontRight.y = car_param->carFrontLeft.y;

	car_param->carRearLeft.x = car_param->carFrontLeft.x;
	car_param->carRearLeft.y = car_param->carRearRight.y;

	// slope[i][0] 表示以x为自变量直线的斜率； slope[i][1]表示以y为自变量直线的斜率
	slope[0][0] = (float64_t)(car_param->boundaryLeftFront.y - car_param->carFrontLeft.y) / 
		                      (car_param->boundaryLeftFront.x - car_param->carFrontLeft.x);
	slope[0][1] = (float64_t)(car_param->boundaryLeftFront.x - car_param->carFrontLeft.x) /
		                      (car_param->boundaryLeftFront.y - car_param->carFrontLeft.y);
	slope[1][0] = (float64_t)(car_param->boundaryFrontLeft.y - car_param->carFrontLeft.y) / 
			                  (car_param->boundaryFrontLeft.x - car_param->carFrontLeft.x);
	slope[1][1] = (float64_t)(car_param->boundaryFrontLeft.x - car_param->carFrontLeft.x) / 
			                  (car_param->boundaryFrontLeft.y - car_param->carFrontLeft.y);

	slope[2][0] = (float64_t)(car_param->boundaryRightFront.y - car_param->carFrontRight.y) / 
		                      (car_param->boundaryRightFront.x - car_param->carFrontRight.x);
	slope[2][1] = (float64_t)(car_param->boundaryRightFront.x - car_param->carFrontRight.x) / 
		                      (car_param->boundaryRightFront.y - car_param->carFrontRight.y);
	slope[3][0] = (float64_t)(car_param->boundaryFrontRight.y - car_param->carFrontRight.y) / 
		                      (car_param->boundaryFrontRight.x - car_param->carFrontRight.x);
	slope[3][1] = (float64_t)(car_param->boundaryFrontRight.x - car_param->carFrontRight.x) / 
		                      (car_param->boundaryFrontRight.y - car_param->carFrontRight.y);

	slope[4][0] = (float64_t)(car_param->boundaryLeftRear.y - car_param->carRearLeft.y) / 
		                      (car_param->boundaryLeftRear.x - car_param->carRearLeft.x);
	slope[4][1] = (float64_t)(car_param->boundaryLeftRear.x - car_param->carRearLeft.x) / 
		                      (car_param->boundaryLeftRear.y - car_param->carRearLeft.y);
	slope[5][0] = (float64_t)(car_param->boundaryRearLeft.y - car_param->carRearLeft.y) / 
		                      (car_param->boundaryRearLeft.x - car_param->carRearLeft.x);
	slope[5][1] = (float64_t)(car_param->boundaryRearLeft.x - car_param->carRearLeft.x) / 
		                      (car_param->boundaryRearLeft.y - car_param->carRearLeft.y);

	slope[6][0] = (float64_t)(car_param->boundaryRightRear.y - car_param->carRearRight.y) / 
		                      (car_param->boundaryRightRear.x - car_param->carRearRight.x);
	slope[6][1] = (float64_t)(car_param->boundaryRightRear.x - car_param->carRearRight.x) / 
		                      (car_param->boundaryRightRear.y - car_param->carRearRight.y);
	slope[7][0] = (float64_t)(car_param->boundaryRearRight.y - car_param->carRearRight.y) / 
		                      (car_param->boundaryRearRight.x - car_param->carRearRight.x);
	slope[7][1] = (float64_t)(car_param->boundaryRearRight.x - car_param->carRearRight.x) / 
		                      (car_param->boundaryRearRight.y - car_param->carRearRight.y);
}

/*
    Function Name     : fill_bev_image
	Function Function : 
	Input:
	    rgb_img       : source image array
		yuv420_img    : the generated yuv420 format result image
		camera_model  : the camera model
		world_model   : the transformation between vehicle coordinate system and world coordinate system
		car_param     : car region information
		R             : rotation matrix
		pose_rt       : transition matrix
		slope:          the boundary's slope in x and y direction respectively
		width         : result image width
		height        : result image height
	Return:             Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/05/15       Create
 */
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
					int32_t height)
{
	int32_t half_width, pitch;
	float64_t pxyz[3], ray[3], uv[2];

	half_width = width >> 1;
	pitch = height * width;
	for(int32_t i = 0; i <= height - 1; i++)
	{
		for(int32_t j = 0; j <= width - 1; j++)
		{
			int32_t rgb[3], yuv[3], rgb1[3], rgb2[3];
			float64_t interp_result[3];
			float64_t lenFrontLeft = 1000, lenFrontRight = 1000, lenRearLeft = 1000, lenRearRight = 1000;
			float64_t fusionWeight1, fusionWeight2;
			if(j < car_param->carFrontLeft.x) // 若在左侧融合区域，计算融合长度
			{
				float64_t a = slope[0][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y;
				float64_t b = MAX(slope[1][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y, 0);
				lenFrontLeft = a - b;

				float64_t c = MIN(slope[5][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y, height - 1);
				float64_t d = slope[4][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y;
				lenRearLeft = c - d;
			}
			else if(j > car_param->carFrontRight.x) // 若在右侧融合区域，计算融合长度
			{
				float64_t a = slope[2][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y;
				float64_t b = MAX(slope[3][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y, 0);
				lenFrontRight = a - b;

				float64_t c = MIN(slope[7][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y, height - 1);
				float64_t d = (slope[6][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y);
				lenRearRight = c - d;
			}

			// get point's coordinate in vehicle coordinate system
			pxyz[0] = (- i + height * 0.5) * mm_per_pixel + pSMC->bev_calib_param.bev_eol_param.board_veh_flboard2fwheel
				+ world_model->_dist_axis - pSMC->bev_calib_param.bev_eol_param.station_height * 0.5;
			
			pxyz[1] = (j - width * 0.5) * mm_per_pixel;
			pxyz[2] = 0;

			// 处理小车区域

			if(i >= car_param->carFrontLeft.y && i <= car_param->carRearLeft.y && j >= car_param->carFrontLeft.x && j <= car_param->carFrontRight.x)
			{
				rgb[0] = 128;
				rgb[1] = 128;
				rgb[2] = 128;
			}
			// 处理前侧区域
			else if(i < car_param->carFrontLeft.y && j >= slope[1][1] * (i - car_param->carFrontLeft.y) + car_param->carFrontLeft.x 
				&& j <= slope[3][1] * (i - car_param->carFrontRight.y) + car_param->carFrontRight.x)
			{
				ray[0] = (R[0][0] * pxyz[0] + R[0][1] * pxyz[1] + pose_rt[0][3]);
				ray[1] = (R[0][3] * pxyz[0] + R[0][4] * pxyz[1] + pose_rt[0][4]);
				ray[2] = (R[0][6] * pxyz[0] + R[0][7] * pxyz[1] + pose_rt[0][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[0]);
				bilinear_interpolation(rgb_img[0], rgb, uv[0], uv[1]);
			}
			// 处理后侧区域
			else if (i > car_param->carRearLeft.y && j >= slope[5][1] * (i - car_param->carRearLeft.y) + car_param->carRearLeft.x 
				&& j <= slope[7][1] * (i - car_param->carRearRight.y) + car_param->carRearRight.x)
			{
				ray[0] = (R[1][0] * pxyz[0] + R[1][1] * pxyz[1] + pose_rt[1][3]);
				ray[1] = (R[1][3] * pxyz[0] + R[1][4] * pxyz[1] + pose_rt[1][4]);
				ray[2] = (R[1][6] * pxyz[0] + R[1][7] * pxyz[1] + pose_rt[1][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[1]);
				bilinear_interpolation(rgb_img[1], rgb, uv[0], uv[1]);
			}
			// 处理左侧区域
			else if(j < car_param->carFrontLeft.x && i >= slope[0][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y
				&& i <= slope[4][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y)
			{
				ray[0] = (R[2][0] * pxyz[0] + R[2][1] * pxyz[1] + pose_rt[2][3]);
				ray[1] = (R[2][3] * pxyz[0] + R[2][4] * pxyz[1] + pose_rt[2][4]);
				ray[2] = (R[2][6] * pxyz[0] + R[2][7] * pxyz[1] + pose_rt[2][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[2]);
				bilinear_interpolation(rgb_img[2], rgb, uv[0], uv[1]);
			}
			// 处理右侧区域
			else if(j > car_param->carFrontRight.x && i >= slope[2][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y
				&& i <= slope[6][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y)
			{
				ray[0] = (R[3][0] * pxyz[0] + R[3][1] * pxyz[1] + pose_rt[3][3]);
				ray[1] = (R[3][3] * pxyz[0] + R[3][4] * pxyz[1] + pose_rt[3][4]);
				ray[2] = (R[3][6] * pxyz[0] + R[3][7] * pxyz[1] + pose_rt[3][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[3]);
				bilinear_interpolation(rgb_img[3], rgb, uv[0], uv[1]);
			}
			// 处理左上融合区域
			else if(j <= car_param->carFrontLeft.x && i >= slope[1][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y &&
				i >= 0 && i < slope[0][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y)
			{
				fusionWeight1 = (slope[0][0] * (j - car_param->carFrontLeft.x) + car_param->carFrontLeft.y - i) / lenFrontLeft;
				ray[0] = (R[0][0] * pxyz[0] + R[0][1] * pxyz[1] + pose_rt[0][3]);
				ray[1] = (R[0][3] * pxyz[0] + R[0][4] * pxyz[1] + pose_rt[0][4]);
				ray[2] = (R[0][6] * pxyz[0] + R[0][7] * pxyz[1] + pose_rt[0][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[0]);
				bilinear_interpolation(rgb_img[0], rgb1, uv[0], uv[1]);

				fusionWeight2 = 1 - fusionWeight1;
				ray[0] = (R[2][0] * pxyz[0] + R[2][1] * pxyz[1] + pose_rt[2][3]);
				ray[1] = (R[2][3] * pxyz[0] + R[2][4] * pxyz[1] + pose_rt[2][4]);
				ray[2] = (R[2][6] * pxyz[0] + R[2][7] * pxyz[1] + pose_rt[2][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[2]);
				bilinear_interpolation(rgb_img[2], rgb2, uv[0], uv[1]);

				for(int32_t m = 0; m < 3; m++)	
				{
					rgb[m] = fusionWeight1 * rgb1[m] + fusionWeight2 * rgb2[m];
				}

			}
			// 处理右上融合区域
			else if(j >= car_param->carFrontRight.x && i >= slope[3][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y &&
				i >= 0 && i <= slope[2][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y)
			{
				fusionWeight1 = (slope[2][0] * (j - car_param->carFrontRight.x) + car_param->carFrontRight.y - i) / lenFrontRight;
				ray[0] = (R[0][0] * pxyz[0] + R[0][1] * pxyz[1] + pose_rt[0][3]);
				ray[1] = (R[0][3] * pxyz[0] + R[0][4] * pxyz[1] + pose_rt[0][4]);
				ray[2] = (R[0][6] * pxyz[0] + R[0][7] * pxyz[1] + pose_rt[0][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[0]);
				bilinear_interpolation(rgb_img[0], rgb1, uv[0], uv[1]);

				fusionWeight2 = 1 - fusionWeight1;
				ray[0] = (R[3][0] * pxyz[0] + R[3][1] * pxyz[1] + pose_rt[3][3]);
				ray[1] = (R[3][3] * pxyz[0] + R[3][4] * pxyz[1] + pose_rt[3][4]);
				ray[2] = (R[3][6] * pxyz[0] + R[3][7] * pxyz[1] + pose_rt[3][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[3]);
				bilinear_interpolation(rgb_img[3], rgb2, uv[0], uv[1]);

				for(int32_t m = 0; m < 3; m++)	
				{
					rgb[m] = fusionWeight1 * rgb1[m] + fusionWeight2 * rgb2[m];
				}
			}
			// 处理左下融合区域
			else if(j <= car_param->carFrontLeft.x && i >= slope[4][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y	
				&& i <= height - 1 && i < slope[5][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y)
			{
				fusionWeight1 = (i - (slope[4][0] * (j - car_param->carRearLeft.x) + car_param->carRearLeft.y)) / lenRearLeft;
				ray[0] = (R[1][0] * pxyz[0] + R[1][1] * pxyz[1] + pose_rt[1][3]);
				ray[1] = (R[1][3] * pxyz[0] + R[1][4] * pxyz[1] + pose_rt[1][4]);
				ray[2] = (R[1][6] * pxyz[0] + R[1][7] * pxyz[1] + pose_rt[1][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[1]);
				bilinear_interpolation(rgb_img[1], rgb1, uv[0], uv[1]);

				fusionWeight2 = 1 - fusionWeight1;
				ray[0] = (R[2][0] * pxyz[0] + R[2][1] * pxyz[1] + pose_rt[2][3]);
				ray[1] = (R[2][3] * pxyz[0] + R[2][4] * pxyz[1] + pose_rt[2][4]);
				ray[2] = (R[2][6] * pxyz[0] + R[2][7] * pxyz[1] + pose_rt[2][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[2]);
				bilinear_interpolation(rgb_img[2], rgb2, uv[0], uv[1]);

				for(int32_t m = 0; m < 3; m++)
				{
					rgb[m] = fusionWeight1 * rgb1[m] + fusionWeight2 * rgb2[m];
				}
			}
			// 处理右下融合区域
			else if(j >= car_param->carRearRight.x && i >= slope[6][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y 
				&& i <= height - 1 && i <= slope[7][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y)
			{
				fusionWeight1 = (i - (slope[6][0] * (j - car_param->carRearRight.x) + car_param->carRearRight.y)) / lenRearRight;
				ray[0] = (R[1][0] * pxyz[0] + R[1][1] * pxyz[1] + pose_rt[1][3]);
				ray[1] = (R[1][3] * pxyz[0] + R[1][4] * pxyz[1] + pose_rt[1][4]);
				ray[2] = (R[1][6] * pxyz[0] + R[1][7] * pxyz[1] + pose_rt[1][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[1]);
				bilinear_interpolation(rgb_img[1], rgb1, uv[0], uv[1]);

				fusionWeight2 = 1 - fusionWeight1;
				ray[0] = (R[3][0] * pxyz[0] + R[3][1] * pxyz[1] + pose_rt[3][3]);
				ray[1] = (R[3][3] * pxyz[0] + R[3][4] * pxyz[1] + pose_rt[3][4]);
				ray[2] = (R[3][6] * pxyz[0] + R[3][7] * pxyz[1] + pose_rt[3][5]);
				Cam_MapCamRay2ImagePoint64(uv, ray, &camera_model[3]);
				bilinear_interpolation(rgb_img[3], rgb2, uv[0], uv[1]);

				for(int32_t m = 0; m < 3; m++)
				{
					rgb[m] = fusionWeight1 * rgb1[m] + fusionWeight2 * rgb2[m];
				}
			}

			// convert rgb to yuv and write into image
			yuv[0] = (int32_t)( 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]);
			yuv[0] = yuv[0] < 0 ? 0 : (yuv[0] > 255 ? 255 : yuv[0]);
			yuv420_img[i * width + j] = yuv[0];
			if (!(i & 0x1) && (!(j & 0x1)))
			{
				yuv[1] = (int32_t)(-0.169 * rgb[0] - 0.331 * rgb[1] + 0.500 * rgb[2] + 128);
				yuv[2] = (int32_t)( 0.500 * rgb[0] - 0.418 * rgb[1] - 0.082 * rgb[2] + 128);
				yuv[1] = yuv[1] < 0 ? 0 : (yuv[1] > 255 ? 255 : yuv[1]);
				yuv[2] = yuv[2] < 0 ? 0 : (yuv[2] > 255 ? 255 : yuv[2]);
				yuv420_img[pitch + (i >> 1) * half_width + (j >> 1)] = (uchar)yuv[1];
				yuv420_img[(pitch * 5 >> 2) + (i >> 1) * half_width + (j >> 1)] = (uchar)yuv[2];
			}
		}		
	}
}

/*
    Function Name     : generate_bev_image
	Function Function : generate desired bev image
	Input:
	    rgb_img       : src image array
		calib_data_buff:The optimized camera extrinsic params and camera center(if optimized)
		pSMC          : The SMC information
		avm_cam_bev   : Camera intrinsic params
		world_model   : the transformation between vehicle coordinate system and world coordinate system
		dstImgSize :    bev image size
		yuv420_img :    the bev image to be generated
		car_param:      the car param structure to be output
		slope:          the boundary's slope in x and y direction respectively
		mm_per_pixel  : how much real distance per pixel represented
	Return:             Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/04/07       Create
 */
void generate_bev_image( IplImage* rgb_img[CAMERA_NUM], 
						 EOL_Buffer_Mng_T* calib_data_buff, 
						 Smc_Cal_T* pSMC, 
						 EOL_str_avm_intrin* avm_cam_bev,
						 EOL_str_world_model* world_model,
						 CvSize dstImgSize, 
						 uchar* yuv420_img, 
						 CarParam* car_param, 
						 float64_t slope[8][2],
						 float64_t mm_per_pixel)
{
	int32_t height, width, half_height, half_width, pitch;
	float64_t pose_rt[CAMERA_NUM][6];
	float64_t R[CAMERA_NUM][9], pxyz[3], ray[3], uv[2];

	height = dstImgSize.height;
	width = dstImgSize.width;
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;

	EOL_str_cam_intrin camera_model[4];
	memcpy(&camera_model[0], avm_cam_bev->_cam_front, sizeof(EOL_str_cam_intrin));
	memcpy(&camera_model[1], avm_cam_bev->_cam_rear, sizeof(EOL_str_cam_intrin));
	memcpy(&camera_model[2], avm_cam_bev->_cam_left, sizeof(EOL_str_cam_intrin));
	memcpy(&camera_model[3], avm_cam_bev->_cam_right, sizeof(EOL_str_cam_intrin));

	// get each camera's inner param and rt from world to cam
	for(int32_t i = 0; i < CAMERA_NUM; i++)
	{
		float64_t camera_fov[2];
		memcpy(&pose_rt[i], (int8_t*)calib_data_buff->address + sizeof(float64_t) * (6 * i), sizeof(float64_t) * 6);


		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			memcpy(&camera_model[i].cu, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i), sizeof(float64_t));
			memcpy(&camera_model[i].cv, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i + 1), sizeof(float64_t));
			break;
		case OPTIMIZE_TYPE_FOV:
			memcpy(&camera_fov[0], (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i), sizeof(float64_t));
			memcpy(&camera_fov[1], (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i + 1), sizeof(float64_t));
			Cam_InitIntrinsic64(&camera_model[i],
				camera_model[i].w,
				camera_model[i].h ,
				camera_model[i].cu ,
				camera_model[i].cv,
				camera_model[i].skew_c , 
				camera_model[i].skew_d , 
				camera_model[i].skew_e ,
				camera_model[i].lut,
				camera_fov[0], 
				camera_fov[1], 
				true) ;
			break;
		case OPTIMIZE_TYPE_BOTH:
			memcpy(&camera_model[i].cu, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i), sizeof(float64_t));
			memcpy(&camera_model[i].cv, (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 6 + 2 * i + 1), sizeof(float64_t));
			memcpy(&camera_fov[0], (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 8 + 2 * i), sizeof(float64_t));
			memcpy(&camera_fov[1], (int8_t*)calib_data_buff->address + sizeof(float64_t) * (CAMERA_NUM * 8 + 2 * i + 1), sizeof(float64_t));
			Cam_InitIntrinsic64(&camera_model[i],
				camera_model[i].w,
				camera_model[i].h ,
				camera_model[i].cu ,
				camera_model[i].cv,
				camera_model[i].skew_c , 
				camera_model[i].skew_d , 
				camera_model[i].skew_e ,
				camera_model[i].lut,
				camera_fov[0], 
				camera_fov[1], 
				true) ;
			break;
		default:
			break;
		}

		Cvt_Angles_To_Rotation_DB(R[i], pose_rt[i]); // R is cam2world
		Inverse_RT_InPlace_DB(R[i], &pose_rt[i][3]); // update R = R' and t = -R'T // world2cam
		Cvt_Rotation_To_Angles_DB(R[i], pose_rt[i]); // update r in order x, y and z
	}

	fill_bev_image(rgb_img, 
		yuv420_img,
		camera_model,
		pSMC,
		world_model,
		car_param, 
		R,
		pose_rt,
		slope,
		mm_per_pixel,
		dstImgSize.width,
		dstImgSize.height);	
}

/*
    Function Name     : EOL_draw_stitch_result
	Function Function : Draw stitch result
	Input:
		rgb_img:            Source images
		calib_data_buff:    buffer storage camera extrinsic params and camera center(if center is updated)
		pSMC:               The SMC
		avm_cam_bev:        Camera intrinsic param
		world_model:        vehicle param

	Return:             Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/04/07       Create
 */
int32_t EOL_draw_stitch_result(IplImage* rgb_img[CAMERA_NUM], 
	                     EOL_Buffer_Mng_T* calib_data_buff, 
						 Smc_Cal_T* pSMC, 
						 EOL_str_avm_intrin* avm_cam_bev,
						 EOL_str_world_model* world_model)
{
	char str[256];
	int32_t height, width, half_height, half_width, pitch;
	FILE* p_file = NULL;
	EOL_str_cam_intrin* bev_cam = NULL;
	float64_t avm_pose[6], mm_per_pixel;
	CvSize car_size, bev_image_size;
	CarParam car_param;
	int32_t hori_trans = 72, vert_trans = 200;
	//int32_t hori_trans = 1, vert_trans = 1;
	float64_t slope[8][2];

	height = 800, width = 600;	
	bev_image_size.height = height;
	bev_image_size.width = width;
	
	mm_per_pixel = (float64_t)(pSMC->bev_stitch_param.view_param[0].view_range + 
		pSMC->bev_stitch_param.view_param[1].view_range + 
		pSMC->veh_param.veh_length) / height; 
	
	car_size.height = pSMC->veh_param.veh_length / mm_per_pixel + 0.5;
	car_size.width = pSMC->veh_param.veh_width / mm_per_pixel + 0.5;

	car_size.height += car_size.height & 0x1;
	car_size.width += car_size.width & 0x1;

	// malloc result memory
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;

	sprintf(str, 
			"%s/stitch_result_frame%d_%dx%d.I420", 
			FILE_SAVE_PATH, frame_count, 
			width, 
			height);

	uchar* yuv420_img = (uchar*)malloc(sizeof(uchar) * pitch * 3 >> 1);
	if (NULL == yuv420_img)
	{
		return MEM_MALLOC_FAIL;
	}
	memset(yuv420_img, 128, sizeof(uchar) * pitch * 3 >> 1);

	// get car's position in bev image
	set_car_info(bev_image_size, 
				 hori_trans, 
				 vert_trans, 
				 car_size, 
				 &car_param, 
				 slope);
	
	generate_bev_image(rgb_img, 
					   calib_data_buff, 
					   pSMC, 
					   avm_cam_bev, 
					   world_model, 
					   bev_image_size, 
					   yuv420_img, 
					   &car_param, 
					   slope, 
					   mm_per_pixel);

	//save file	
	p_file = fopen(str, "wb");
	if(NULL == p_file)
	{
		return OPEN_FILE_ERROR;
	}
	fwrite(yuv420_img, sizeof(uchar) * pitch * 3 >> 1, 1, p_file);
	fclose(p_file);

	return 0;
}

/*
Function Name     : EOL_save_images
Function Function : Draw circle onto src image's corners
Input:
rgb_img:            source rgb image
p_file:             output yuv420 image file
p_result:           yuv420 image
Return:             Error code
Note              :
Revision History  : Author         Data             Changes
                    Wangluyao      2017/03/31       Create
*/
int32_t EOL_save_images(const IplImage* rgb_img, FILE* p_file, uchar* p_result)
{	
	char str[256];
	int32_t height, width, half_height, half_width, pitch;

	height = rgb_img->height;
	width = rgb_img->width;
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;

	bgr_to_yuv420(rgb_img, p_result);

	fwrite(p_result, sizeof(uchar) * pitch * 3 >> 1, 1, p_file);

	return 0;
}

/*
	Function Name     : EOL_draw_world_coordinate
	Function Function : Draw world coordinates' distribution on an image
	Input:
		p_Eol_param :   p_Eol_param
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/01/09       Create
 */
int32_t EOL_draw_world_coordinate(P_EOL_Param p_Eol_param)
{
	char str[256];
	int32_t height, width, half_height, half_width, pitch;
	int32_t station_width, station_height;
	FILE* p_file = NULL;

	height = 800;
	width = 600;
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;
	
	station_width = (int32_t)p_Eol_param->station_width *0.1;
	station_height = (int32_t)p_Eol_param->station_height *0.1;
	

	sprintf(str, 
			"%s/world_coordinate_result_frame%d_%dx%d.I420", 
			FILE_SAVE_PATH, 
			frame_count, 
			width, 
			height);
	uchar* yuv420_img = (uchar*)malloc(sizeof(uchar) * pitch * 3 >> 1);
	if (NULL == yuv420_img)
	{
		return MEM_MALLOC_FAIL;
	}
	memset(yuv420_img, 128, sizeof(uchar) * pitch * 3 >> 1);

	/* draw calibration station */
	for(int32_t i = 0; i < height; i++)
	{
		for(int32_t j = 0; j < width; j++)
		{			
			if( (i + (station_height >> 1) >= half_height - 1 && i + (station_height >> 1) <= half_height + 1 ||
				 i - (station_height >> 1) >= half_height - 1 && i - (station_height >> 1) <= half_height + 1) &&
				 j >= half_width - (station_width >> 1) && j <= half_width + (station_width >> 1) ||
				(j + (station_width >> 1) >= half_width - 1 && j + (station_width >> 1) <= half_width + 1 ||
				 j - (station_width >> 1) >= half_width - 1 && j - (station_width >> 1) <= half_width + 1) &&
				 i >= half_height - (station_height >> 1) && i <= half_height + (station_height >> 1)
			  )
			{
				int32_t b = 255, g = 0, r = 0;
				int32_t Y, U, V, y, u, v;
				Y = (int32_t)( 0.299 * r + 0.587 * g + 0.114 * b);
				U = (int32_t)(-0.169 * r - 0.331 * g + 0.500 * b + 128);
				V = (int32_t)( 0.500 * r - 0.418 * g - 0.082 * b + 128);

				Y = Y < 0 ? 0 : (Y > 255 ? 255 : Y);
				U = U < 0 ? 0 : (U > 255 ? 255 : U);
				V = V < 0 ? 0 : (V > 255 ? 255 : V);

				y = (uchar)Y;
				u = (uchar)U;
				v = (uchar)V;

				yuv420_img[i * width + j] = y;
				yuv420_img[pitch + (i >> 1) * half_width + (j >> 1)] = u;
				yuv420_img[(pitch * 5 >> 2) + (i >> 1) * half_width + (j >> 1)] = v;
			}
		}
	}

	
	/* draw world coordinate */
	int32_t b = 0, g = 0, r = 0;
	for(int32_t camid = 0; camid < CAMERA_NUM; camid++)
	{
		CbPattern_group pattern_group = p_Eol_param->cb_pattern_group[camid];
		switch(camid)
		{
		case 0:
		case 1:
			b = g = 0;
			r = 255;
			break;
		case 2:
		case 3:
			b = r = 0;
			g = 255;
			break;
		}
		
		for(int32_t pattern_id = 0; pattern_id < pattern_group.valid_num; pattern_id++)
		{			
			for(int32_t i = 0; i < pattern_group.pattern[pattern_id].rows; i++)
			{
				for(int32_t j = 0; j < pattern_group.pattern[pattern_id].cols; j++)
				{
					CvPoint pt;
					pt.x = (int32_t)(half_width + pattern_group.pattern[pattern_id].ground_corner_point[i][j].y * 0.1);
					pt.y = (int32_t)(half_height - pattern_group.pattern[pattern_id].ground_corner_point[i][j].x * 0.1);
					if(pattern_group.pattern[pattern_id].flag[i][j])
					{						
						for(int32_t m = -5; m < 5; m++)
						{
							for(int32_t n = -5; n < 5; n++)
							{
								float64_t radius = sqrt((float64_t)m*m + n*n);
								if(radius > 3 && radius < 5 && camid < 2 ||
								   radius < 3 && camid >= 2)
								{
									int cur_x, cur_y;
									cur_y = pt.y + m;
									cur_x = pt.x + n;
									if(cur_y >= 0 && cur_y <= height - 1 && cur_x >= 0 && cur_x <= width - 1)
									{										
										int32_t Y, U, V, y, u, v;
										Y = (int32_t)( 0.299 * r + 0.587 * g + 0.114 * b);
										U = (int32_t)(-0.169 * r - 0.331 * g + 0.500 * b + 128);
										V = (int32_t)( 0.500 * r - 0.418 * g - 0.082 * b + 128);

										Y = Y < 0 ? 0 : (Y > 255 ? 255 : Y);
										U = U < 0 ? 0 : (U > 255 ? 255 : U);
										V = V < 0 ? 0 : (V > 255 ? 255 : V);

										y = (uchar)Y;
										u = (uchar)U;
										v = (uchar)V;

										yuv420_img[cur_y * width + cur_x] = y;
										yuv420_img[pitch + (cur_y >> 1) * half_width + (cur_x >> 1)] = u;
										yuv420_img[(pitch * 5 >> 2) + (cur_y >> 1) * half_width + (cur_x >> 1)] = v;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	//save file	
	p_file = fopen(str, "wb");
	if(NULL == p_file)
	{
		return OPEN_FILE_ERROR;
	}
	fwrite(yuv420_img, sizeof(uchar) * pitch * 3 >> 1, 1, p_file);
	fclose(p_file);

	return  EOL_SUCCESS;
}

