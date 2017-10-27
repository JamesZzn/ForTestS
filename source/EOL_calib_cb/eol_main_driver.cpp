#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "eol_pose_estimator.h"
#include "eol_image_process.h"
#include "eol_calc_ground_coordinate.h"
#include "eol_chessboard_detect.h"
#include "eol_debug.h"
#include "eol_config.h"


#ifndef CAM_MODEL_MACRO
#define CAM_MODEL_MACRO
typedef Cam_Model_Intrinsic str_cam_intrin ;
typedef Cam_Model_Extrinsic str_cam_extrin ;
typedef Cam_Model str_cam ;
#endif

extern IplImage *EOL_bev_data_c_raw[CAMERA_NUM];
extern IplImage *EOL_bev_data_c[CAMERA_NUM];
extern IplImage *EOL_bev_data_y[CAMERA_NUM];
IplImage* thresh_img, *norm_img, *temp_img, *temp_img_copy;


FILE*  fp_log = NULL;
extern int32_t frame_count;
extern char FILE_SAVE_PATH[20];
extern EOL_avm_param_adjuster EOL_g_param_adjuster;
static float64_t uv2_bev[EOL_MAX_NUM_LAYERS_][CAMERA_NUM][EOL_out_maxpoint_num*2];
static float64_t EOL_All_Mem[1000];
static EOL_Buffer_Mng_T   EOL_calib_data_buff;
chessBoard_corner_detector corner_detector[CAMERA_NUM];


int32_t start_frame_id = START_FRAME_ID;
int32_t total_count = VALID_FRAME_COUNT; // for framework batch process

// pattern configuration for world coordinate calculation
// init_x, init_y, quad_height, quad_width, pattern_height, pattern_width, left_boundary_to_axis
static EOL_pattern_config pattern_config_ofilm[4] = 
{
	{600, 600, 400, 400, 1600, 1600, 0}, // pattern22_config
	{300, 400, 300, 300, 1600, 900 , 0}, // pattern42_config
	{300, 400, 300, 300, 1600, 1200, 0}, // pattern43_config
	{450, 450, 300, 300, 1600, 1500, 0}  // pattern44_config
};

static EOL_pattern_config pattern_config_gac[4] = 
{
	{400, 400, 200, 200, 800,  800,  0}, // pattern11_config
	{250, 400, 200, 200, 1200, 700,  0}, // pattern42_config
	{250, 400, 200, 200, 1200, 1050, 0}, // pattern43_config
	{300, 400, 200, 200, 1200, 1200, 0}  // pattern44_config
};

static EOL_pattern_config pattern_config_jac[7] =
{
	{ 300, 300, 600, 600, 1700, 1700, 0 }, // front_cam_left_pattern
	{ 200, 600, 600, 600, 1400, 1400, 0 }, // front_cam_center_pattern	
	{ 200, 300, 600, 600, 1700, 1700, 0 }, // front_cam_right_pattern
	{ 300, 300, 600, 600, 2300, 1700, 0 }, // rear_cam_left_pattern
	{ 200, 600, 600, 600, 2000, 1600, 0 }, // rear_cam_center_pattern	
	{ 200, 300, 600, 600, 2300, 1700, 0 }, // rear_cam_right_pattern
	{ 300, 300, 600, 600, 1700, 9000, 0 }, // left&right_pattern
};

/*board_size(width, height), min_number_of_corners, rect_mask_num, reference_pointNum, camid, 
  width_ratio, height_ratio, RectMinAreaRatio, reflection, gloabal_min_dilate;*/
static Chessboard_Corner_Config chessboard_config_ofilm[4] = 
{
	{ CvSize(2,2), 3, 4, 4, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)0.5, 1, 1}, // chessboard22_config
	{ CvSize(2,4), 5, 8, 8, 0, (float32_t)0.5, (float32_t)0.5, (float32_t)0.5, 1, 1}, // chessboard42_config
	{ CvSize(3,4), 6, 12, 10, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)0.5, 1, 1}, // chessboard43_config
	{ CvSize(4,4), 7, 16, 13, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)0.5, 1, 1}  // chessboard44_config
};

static Chessboard_Corner_Config chessboard_config_gac[4] = 
{
	{ CvSize(1,1), 1, 1, 2, 3, (float32_t)0.4, (float32_t)0.4, (float32_t)2.0, 1, 1}, // chessboard11_config
	{ CvSize(2,4), 5, 8, 8, 2, (float32_t)0.5, (float32_t)0.5, (float32_t)0.5, 1, 1}, // chessboard42_config
	{ CvSize(3,4), 6, 12, 10, 2, (float32_t)0.5, (float32_t)0.5, (float32_t)0.5, 1, 1}, // chessboard43_config
	{ CvSize(4,4), 7, 16, 13, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)0.5, 1, 1}  // chessboard44_config
};

static Chessboard_Corner_Config chessboard_config_jac[4] = 
{
	{ CvSize(2,2), 3, 4, 5, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)2.0, 1, 1}, // chessboard11_config
	{ CvSize(3,3), 5, 6, 6, 0, (float32_t)0.5, (float32_t)0.5, (float32_t)0.5, 1, 1}, // chessboard42_config
	{ CvSize(3,4), 6, 9, 8, 0, (float32_t)0.5, (float32_t)0.5, (float32_t)0.5, 1, 1}, // chessboard43_config
	{ CvSize(15,3), 17, 39, 28, 0, (float32_t)0.4, (float32_t)0.4, (float32_t)0.5, 1, 1}  // chessboard44_config
};

/*
	Function Name     : save_rt
	Function Function : save optimized r and t
	Input:
		filename      : the filename to be written
		rt_front      : r and t of front camera
		rt_rear       : r and t of rear camera
		rt_left       : r and t of left camera
		rt_right      : r and t of right camera
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/04/19       Create
 */
int32_t save_rt(const char *filename, 
			    EOL_Buffer_Mng_T   EOL_calib_data_buff)
{

	FILE *fp = fopen(filename, "w");
	if(!fp) return -1;

	float64_t pose[CAMERA_NUM][6];

	for(int32_t i = 0; i < CAMERA_NUM; i++)
	{
		for(int32_t j = 0; j < 6; j++)
		{
			pose[i][j] = ((float64_t*)(EOL_calib_data_buff.address))[i * 6 + j];
		}
	}	

	fprintf(fp, "%f %f %f\n", pose[0][0], pose[0][1], pose[0][2]);
	fprintf(fp, "%f %f %f\n", pose[0][3], pose[0][4], pose[0][5]);
	fprintf(fp, "%f %f %f\n", pose[1][0], pose[1][1], pose[1][2]);
	fprintf(fp, "%f %f %f\n", pose[1][3], pose[1][4], pose[1][5]);
	fprintf(fp, "%f %f %f\n", pose[2][0], pose[2][1], pose[2][2]);
	fprintf(fp, "%f %f %f\n", pose[2][3], pose[2][4], pose[2][5]);
	fprintf(fp, "%f %f %f\n", pose[3][0], pose[3][1], pose[3][2]);
	fprintf(fp, "%f %f %f\n", pose[3][3], pose[3][4], pose[3][5]);
	fclose(fp);
	return 0;
}

/*
	Function Name:      EOL_Smc_Cfg
	Function Function : Get camera param and vehicle model for SMC
	Input             : 
	    smc :           SMC structure
	    bev_cams :      Camera params
		world_model :   Vehicle model
		simg_width :    Image width
		simg_height :   Image height
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    
*/
void EOL_Smc_Cfg(Smc_Cal_T* const smc, 
				 EOL_str_cam *bev_cams, 
				 EOL_str_world_model *world_model, 
				 int32_t simg_width, 
				 int32_t simg_height )
{
	//consistent with design
    int32_t camid = 0;
	for(camid=0 ; camid < CAMERA_NUM ; camid++)
	{
		Cam_InitIntrinsic64(&bev_cams[camid].cam_int ,
			smc->camera_param[camid].cam_int.cam_int_w,
			smc->camera_param[camid].cam_int.cam_int_h ,
			smc->camera_param[camid].cam_int.cam_int_cu ,
			smc->camera_param[camid].cam_int.cam_int_cv,
			smc->camera_param[camid].cam_int.cam_int_skew_c , 
			smc->camera_param[camid].cam_int.cam_int_skew_d , 
			smc->camera_param[camid].cam_int.cam_int_skew_e ,
			smc->lut_table[ smc->camera_param[camid].cam_int.cam_int_lut],
			smc->camera_param[camid].cam_int.cam_int_fu_or_hfov_at_cu, 
			smc->camera_param[camid].cam_int.cam_int_fv_or_vfov_at_cv , 
			smc->camera_param[camid].cam_int.cam_int_use_fov ) ;
	}

	//consistent with design
	world_model->_dist_axis = smc->veh_param.veh_axis_length;
	world_model->_veh_len = smc->veh_param.veh_length;
	world_model->_veh_width = smc->veh_param.veh_width;
	world_model->_rear_axis_2_bumper = smc->veh_param.veh_rwheel2tail;
	world_model->_front_axis_2_head =  smc->veh_param.veh_fwheel2head;
}

/*
	Function Name:      EOL_config_pattern_params
	Function Function : Config pattern's params
	Input             : 
	   p_Eol_Param    : contains all info used in calibration
	   pEol_smc_param : station information stored in SMC
		        camid : camera id
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    
*/
void EOL_config_pattern_params(P_EOL_Param p_Eol_param,
							   Bev_Calib_Param_Bev_EOL_Param_T* pEol_smc_param,
							   int32_t camid)
{
	CbPattern_group* p_cb_pattern_group = p_Eol_param->cb_pattern_group;
	switch (p_Eol_param->station_type)
	{
	case STATION_TYPE_OFILM:
		if(0 == camid || 1 == camid)
		{
			p_cb_pattern_group[camid].valid_num = 3;

			p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_ofilm[0];
			p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_ofilm[2];
			p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_ofilm[0];

			// Get the config for world coordinate calculation
			p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_ofilm[0];
			p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_ofilm[2];
			p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_ofilm[0];
		}
		else if(2 == camid)
		{
			p_cb_pattern_group[camid].valid_num = pEol_smc_param->lr_board_num;

			if(3 == p_cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_ofilm[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_ofilm[0];
			}
			else if(4 == p_cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[3].chessboard_config = chessboard_config_ofilm[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[3].pattern_config = pattern_config_ofilm[0];
			}			
		}
		else if(3 == camid)
		{
			p_cb_pattern_group[camid].valid_num = pEol_smc_param->lr_board_num;

			if(3 == p_cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_ofilm[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_ofilm[0];
			}
			else if(4 == p_cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[3].chessboard_config = chessboard_config_ofilm[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_ofilm[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_ofilm[2];
				p_cb_pattern_group[camid].pattern[3].pattern_config = pattern_config_ofilm[0];
			}
		}
		break;
	case STATION_TYPE_GAC:
		if(0 == camid || 1 == camid)
		{
			p_cb_pattern_group[camid].valid_num = 3;

			p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_gac[0];
			p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_gac[3];
			p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_gac[0];

			p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_gac[0];
			p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_gac[3];
			p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_gac[0];
		}
		else if(2 == camid)
		{
			p_cb_pattern_group[camid].valid_num = pEol_smc_param->lr_board_num;

			if (3 == p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_gac[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_gac[0];
			}
			else if (4 == p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_gac[1];
				p_cb_pattern_group[camid].pattern[3].chessboard_config = chessboard_config_gac[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_gac[1];
				p_cb_pattern_group[camid].pattern[3].pattern_config = pattern_config_gac[0];
			}			
		}
		else if(3 == camid)
		{
			p_cb_pattern_group[camid].valid_num = pEol_smc_param->lr_board_num;

			if (3 == p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_gac[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_gac[2];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_gac[0];
			}
			else if (4 == p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				p_cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_gac[1];
				p_cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_gac[2];
				p_cb_pattern_group[camid].pattern[3].chessboard_config = chessboard_config_gac[0];

				p_cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_gac[0];
				p_cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_gac[1];
				p_cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_gac[2];
				p_cb_pattern_group[camid].pattern[3].pattern_config = pattern_config_gac[0];
			}
		}
		break;
	case STATION_TYPE_JAC:
		p_Eol_param->cb_pattern_group[camid].valid_num = 3;
		if (0 == camid)
		{
			p_Eol_param->cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_jac[1];
			p_Eol_param->cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_jac[0];
			p_Eol_param->cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_jac[1];

			p_Eol_param->cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_jac[0];
			p_Eol_param->cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_jac[1];
			p_Eol_param->cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_jac[2];
		}
		else if (1 == camid)
		{
			p_Eol_param->cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_jac[2];
			p_Eol_param->cb_pattern_group[camid].pattern[1].chessboard_config = chessboard_config_jac[1];
			p_Eol_param->cb_pattern_group[camid].pattern[2].chessboard_config = chessboard_config_jac[2];

			p_Eol_param->cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_jac[3];
			p_Eol_param->cb_pattern_group[camid].pattern[1].pattern_config = pattern_config_jac[4];
			p_Eol_param->cb_pattern_group[camid].pattern[2].pattern_config = pattern_config_jac[5];
		}
		else if (2 == camid || 3 == camid)
		{
			p_cb_pattern_group[camid].valid_num = pEol_smc_param->lr_board_num;
			p_Eol_param->cb_pattern_group[camid].pattern[0].chessboard_config = chessboard_config_jac[3];
			p_Eol_param->cb_pattern_group[camid].pattern[0].pattern_config = pattern_config_jac[6];
		}
		break;
	case STATION_TYPE_GEELY:
		break;
	case STATION_TYPE_RESERVED:
		break;
	default:
		break;
	}

}


/*
	Function Name:      EOL_config_pattern_params
	Function Function : Config pattern's params
	Input             : 
	p_cb_pattern_group: config information of a camera
	   pEol_smc_param : station information stored in SMC
		        camid : camera id
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    
*/
void EOL_calc_pattern_trans(CbPattern_group* p_cb_pattern_group,
							Bev_Calib_Param_Bev_EOL_Param_T* pEol_smc_param,
							int32_t camid)
{
	float64_t half_station_height, half_station_width, board_2_veh_center_axis;
	half_station_height = pEol_smc_param->station_height * 0.5;
	half_station_width = pEol_smc_param->station_width * 0.5;
	board_2_veh_center_axis = pEol_smc_param->board_veh_flboard2centralaxis;

	/* left_boundary_to_axis storage the distance from current pattern's left boundary 
	 to the station's center axis */
	switch(camid)
	{
	case 0:
		p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
			- ( p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
			+ board_2_veh_center_axis);
		p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
			- board_2_veh_center_axis + pEol_smc_param->board_flboard2fcboard;
		p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
			pEol_smc_param->station_width - board_2_veh_center_axis;
		break;
	case 1:
		p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
			- (p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
			+ pEol_smc_param->station_width - board_2_veh_center_axis);
		p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
			board_2_veh_center_axis - (pEol_smc_param->board_flboard2fcboard 
			+ p_cb_pattern_group->pattern[1].pattern_config.pattern_width);
		p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
			board_2_veh_center_axis;
		break;
	case 2:
		if(3 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
				+ half_station_height);
			p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[1].pattern_config.pattern_width 
				+ pEol_smc_param->board_flboard2lucboard) + half_station_height;
			p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
				half_station_height;
		}
		else if(4 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
				+ half_station_height);
			p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[1].pattern_config.pattern_width 
				+ pEol_smc_param->board_flboard2llcboard) + half_station_height;
			p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[2].pattern_config.pattern_width 
				+ pEol_smc_param->board_flboard2lucboard) + half_station_height;
			p_cb_pattern_group->pattern[3].pattern_config.left_boundary_to_axis = 
				half_station_height;
		}
		else if (1 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis =
				-(p_cb_pattern_group->pattern[0].pattern_config.pattern_width * 0.5 + 300);
		}
		break;
	case 3:
		if(3 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
				+ half_station_height);
			p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
				- half_station_height + pEol_smc_param->board_flboard2lucboard;
			p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
				half_station_height;
		}
		else if(4 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis = 
				- (p_cb_pattern_group->pattern[0].pattern_config.pattern_width 
				+ half_station_height);
			p_cb_pattern_group->pattern[1].pattern_config.left_boundary_to_axis = 
				- half_station_height + pEol_smc_param->board_flboard2lucboard;
			p_cb_pattern_group->pattern[2].pattern_config.left_boundary_to_axis = 
				- half_station_height + pEol_smc_param->board_flboard2llcboard;
			p_cb_pattern_group->pattern[3].pattern_config.left_boundary_to_axis = 
				half_station_height;
		}
		else if (1 == p_cb_pattern_group->valid_num)
		{
			p_cb_pattern_group->pattern[0].pattern_config.left_boundary_to_axis =
				-(p_cb_pattern_group->pattern[0].pattern_config.pattern_width * 0.5 - 300);
		}
		break;
	default:
		break;
	}	
}

/*
	Function Name:      EOL_init
	Function Function : Malloc ppEolHandle memory and init chessboard config
	Input             : 
	    ppEolHandle :   P_EOL_Param structure, invisible out of this module
	    pSMC :          The SMC information
	Return            : error code described in eol_landmark_detector.h
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/1/18        Create
*/
int EOL_init(void** ppEolHandle, Smc_Cal_T* const pSMC)
{
	EOL_Calib_Diag_Status_Tag ret = EOL_SUCCESS;
	P_EOL_Param p_Eol_param = NULL; 
	float32_t half_station_height, half_station_width;
	p_Eol_param = (P_EOL_Param)malloc(sizeof(EOL_Param));
	if (!p_Eol_param)
	{
		printf("Malloc p_Eol_Param memory error.\n");
		ret = MEM_MALLOC_FAIL;
	}
	memset(p_Eol_param, 0, sizeof(EOL_Param));

	// Malloc global variable's memory
	thresh_img = cvCreateImage(cvSize(pSMC->camera_param[0].cam_int.cam_int_w, 
		pSMC->camera_param[0].cam_int.cam_int_h),	IPL_DEPTH_8U, 1);
	norm_img = cvCreateImage(cvSize(pSMC->camera_param[0].cam_int.cam_int_w, 
		pSMC->camera_param[0].cam_int.cam_int_h),	IPL_DEPTH_8U, 1);
	temp_img = cvCreateImage(cvSize(pSMC->camera_param[0].cam_int.cam_int_w, 
		pSMC->camera_param[0].cam_int.cam_int_h),	IPL_DEPTH_8U, 1);
	temp_img_copy = cvCreateImage(cvSize(pSMC->camera_param[0].cam_int.cam_int_w, 
		pSMC->camera_param[0].cam_int.cam_int_h),	IPL_DEPTH_8U, 1);
	if(!thresh_img || !norm_img || !temp_img || !temp_img_copy)
	{
		ret = MEM_MALLOC_FAIL;
	}

	/*for calc world coordinate convenience, this function transfer bev_eol_cb_param into left_boundary_to_axis*/
	Bev_Calib_Param_Bev_EOL_Param_T* pEol_smc_param = &pSMC->bev_calib_param.bev_eol_param;

	p_Eol_param->station_type = (Station_Type)pEol_smc_param->station_type;
	p_Eol_param->station_height = pEol_smc_param->station_height;
	p_Eol_param->station_width = pEol_smc_param->station_width;
	p_Eol_param->board_2_veh_center_axis = pEol_smc_param->board_veh_flboard2centralaxis;

	half_station_height = p_Eol_param->station_height * (float32_t)0.5;
	half_station_width = p_Eol_param->station_width * (float32_t)0.5;

	for (int32_t camid = 0; camid < CAMERA_NUM; camid++)
	{
		EOL_config_pattern_params(p_Eol_param, pEol_smc_param, camid);

		EOL_calc_pattern_trans(&p_Eol_param->cb_pattern_group[camid], pEol_smc_param, camid);
		for (int pattern_id = 0; pattern_id < p_Eol_param->cb_pattern_group[camid].valid_num; pattern_id++)
		{
			p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].rows = 
				p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].chessboard_config.board_size.height;
			p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].cols = 
				p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].chessboard_config.board_size.width;
		}
	}
	(*ppEolHandle) = p_Eol_param;

	return ret;
}

/*
	Function Name:      EOL_process
	Function Function : detect corners, set ground world coordinate and optimize camera parameters
	Input             : 
	    sProcessImg_in :The four camera's image with chessboard patterns
	    pSMC :          The SMC information
	    ppEolHandle :   P_EOL_Param structure, invisible out of this module
	Return            : Error code described in eol_landmark_detector.h
	Note              : The Function includes three important parts:
	                        1. chessboard detection
	                        2. calc world coordinate by smc input
	                        3. optimize camera extrinsic params r t and camera intrinsic param xc, yc(if necessary)
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/1/18        Create
*/
int EOL_process(void* pImageHandle[CAMERA_NUM],Smc_Cal_T* const pSMC, void** ppEolHandle)
{
	frame_count++; // this variable is designed for sequence result save
	IplImage* sProcessImg_in[CAMERA_NUM];

	for(int i = 0; i < CAMERA_NUM; i++)
	{
		sProcessImg_in[i] = (IplImage*)pImageHandle[i];
	}
	P_EOL_Param p_Eol_param = (P_EOL_Param)(*ppEolHandle);

	EOL_str_cam bev_cams[CAMERA_NUM];

	bool pattern_result[CAMERA_NUM][CB_MAX_NUM]={0};
	EOL_str_avm_intrin avm_intrin_bev ;
	EOL_str_world_model world_model ;
	EOL_str_avm_cam avm_cam_bev;
	uint16_t g_height, g_width;
	EOL_Calib_Diag_Status_T ret;
	
	//step.1: extract 

	if(NULL==sProcessImg_in)
	{
		return LOAD_INIT_IMG_FAIL;
	}
	if(NULL==pSMC)
	{
		return LOAD_SMC_FAIL;
	}

	// the optimizer's result is saved in this structure
	EOL_calib_data_buff.address = EOL_All_Mem;
	EOL_calib_data_buff.size = sizeof(float64_t)*1000;
	EOL_calib_data_buff.used_size = 0;

	g_height = pSMC->camera_param[0].cam_int.cam_int_h;
	g_width = pSMC->camera_param[0].cam_int.cam_int_w;
	if(g_height!=sProcessImg_in[0]->height||g_width!=sProcessImg_in[0]->width)
	{
		printf("EOL:smc data and image not compatible!\n");
		return SMC_IMG_NOT_COMPATIBLE;
	}

	//if(3 == pSMC->bev_calib_param.bev_eol_param.lr_board_num && 
	//   800 > pSMC->bev_calib_param.bev_eol_param.board_flboard2lucboard)
	//{
	//	return SMC_CONFIG_ERROR;
	//}
	//else if( 4 == pSMC->bev_calib_param.bev_eol_param.lr_board_num && 
	//	    (800 > pSMC->bev_calib_param.bev_eol_param.board_flboard2lucboard ||
	//		 pSMC->bev_calib_param.bev_eol_param.station_height - 800 < pSMC->bev_calib_param.bev_eol_param.board_flboard2llcboard
	//		 )
	//	   )
	//{
	//	return SMC_CONFIG_ERROR;
	//}

	//time_t  start_time = clock();
	ret = (EOL_Calib_Diag_Status_T)EOL_chessboard_detect(sProcessImg_in, p_Eol_param);
	//double time0_1 = (double) (clock() - start_time);
	//printf("%f", time0_1);

	/* pattern_result means if each chessboard is correctly detected */
	Chessboard_Pattern* pattern = p_Eol_param->cb_pattern_group[0].pattern;

	// calculate corner point's world coordinate 
	EOL_calculate_ground_coordinate(p_Eol_param);

	// get smc config
	EOL_Smc_Cfg(pSMC, &bev_cams[0], &world_model, g_width, g_height);

    //bev_cams就是输出的smc参数
	avm_intrin_bev._cam_front = &bev_cams[0].cam_int ;
	avm_intrin_bev._cam_rear = &bev_cams[1].cam_int ;
	avm_intrin_bev._cam_left = &bev_cams[2].cam_int ;
	avm_intrin_bev._cam_right = &bev_cams[3].cam_int ;

#ifdef REMOVE_OUTLIERS
	/* if a corner's re-projection error is bigger than 4 times of the average re-projection error of current camera, remove it*/
	ret = (EOL_Calib_Diag_Status_T)EOL_remove_outliers(pSMC, 
		&avm_intrin_bev, 
		p_Eol_param, 
		&world_model);
	CHECK_ERROR(ret);
#endif

#ifdef SOLVE_INIT_RT
	/* for solving initial r and t, not finished yet */
	ret = (EOL_Calib_Diag_Status_T)EOL_calculate_init_rt(pSMC, 
		&avm_intrin_bev, 
		p_Eol_param, 
		&world_model);
	CHECK_ERROR(ret);
#endif

	ret = EOL_pose_estimator(&EOL_calib_data_buff, //等价于cam_pose
		                     pSMC,
							 &avm_intrin_bev,
							 p_Eol_param,
							 &world_model );
	//CHECK_ERROR(ret);

	save_rt("E:/pose.txt", EOL_calib_data_buff);

#ifdef SAVE_CORNER_IMAGE
	EOL_draw_world_coordinate(p_Eol_param);
	EOL_draw_stitch_result(sProcessImg_in, 
		&EOL_calib_data_buff, 
		pSMC, 
		&avm_intrin_bev,
		&world_model);
	for (int camid = 0; camid < CAMERA_NUM; camid++)
	{
		EOL_draw_corners(sProcessImg_in[camid], 
			             &EOL_calib_data_buff, 
						 pSMC, 
						 &avm_intrin_bev, 
						 &world_model,
						 p_Eol_param->cb_pattern_group[camid].pattern, 
						 p_Eol_param->cb_pattern_group[camid].valid_num, 
						 camid);
	}
	
#endif

#ifdef SAVE_LOG
	if(frame_count < 10) fprintf(fp_log, "frame number:  %d  ", frame_count);
	else fprintf(fp_log, "frame number: %d  ", frame_count);
	fprintf(fp_log, "RMS:");
	float64_t ave_err = 0;
	for(int i = 0; i < CAMERA_NUM; i++)
	{
		ave_err += EOL_g_param_adjuster.rmse[i] * 0.25;
		fprintf(fp_log, "%5.2f  ", EOL_g_param_adjuster.rmse[i]);
	}
	fprintf(fp_log, "%5.2f  ", ave_err);
	for(int i = 0; i < CAMERA_NUM; i++)
	{
		fprintf(fp_log, "   ");
		for(int j = 0; j < p_Eol_param->cb_pattern_group[i].valid_num; j++)
		{
			fprintf(fp_log, "%d ", p_Eol_param->cb_pattern_group[i].pattern[j].valid_point_num);
		}
	}

	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		break;
	case OPTIMIZE_TYPE_CENTER:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			float64_t cam_center_trans[2];
			cam_center_trans[0] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2] - 352;
			cam_center_trans[1] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2 + 1] - 240;
			fprintf(fp_log, "    %4.2f   %4.2f ", cam_center_trans[0], cam_center_trans[1]);	
		}			
		break;
	case OPTIMIZE_TYPE_FOV:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			float64_t cam_fov_trans[2];

			cam_fov_trans[0] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2] / PI * 360;
			cam_fov_trans[1] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2 + 1] / PI * 360;
			fprintf(fp_log, "    %4.2f   %4.2f ", cam_fov_trans[0], cam_fov_trans[1]);	
		}
		break;
	case OPTIMIZE_TYPE_BOTH:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			float64_t cam_center_trans[2];
			cam_center_trans[0] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2] - 352;
			cam_center_trans[1] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 6 + i * 2 + 1] - 240;
			fprintf(fp_log, "    %4.2f   %4.2f ", cam_center_trans[0], cam_center_trans[1]);
		}
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			float64_t cam_fov_trans[2];
			cam_fov_trans[0] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 8 + i * 2] / PI * 360;
			cam_fov_trans[1] = ((float64_t*)(EOL_calib_data_buff.address))[CAMERA_NUM * 8 + i * 2 + 1] / PI * 360;
			fprintf(fp_log, "    %4.2f   %4.2f ", cam_fov_trans[0], cam_fov_trans[1]);	
		}
	default:
		break;
	}

	fprintf(fp_log, "\n");
#endif

	return ret;
}

/*
	Function Name:      EOL_deinit
	Function Function : release memory malloced in EOL_init
	Input             : 
	    ppEolHandle :   P_EOL_Param structure, invisible out of this module
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/1/18        Create
*/
void EOL_deinit(void** ppEolHandle)
{
	if((*ppEolHandle))
	{
		free(*ppEolHandle);
		*ppEolHandle = NULL;
	}
	if(thresh_img)
	{
		cvReleaseImage(&thresh_img);
	}
	if(norm_img)
	{
		cvReleaseImage(&norm_img);
	}
	if(temp_img)
	{
		cvReleaseImage(&temp_img);
	}
	if(temp_img_copy)
	{
		cvReleaseImage(&temp_img_copy);
	}
}

/*
	Function Name:      EOL_get_result
	Function Function : Get the optimized center for other module
	Input             : 
	    cam_pose :      For updating camera extrinsic params

		result :        For updating camera corner params
		cam_center :    For updating camera center
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/1/18        Create
*/

void EOL_get_result(str_avm_pose_t *cam_pose, float64_t *cam_center, float64_t *cam_fov, eol_result *result)
{
	double *calib_data_pointer=(double*)EOL_calib_data_buff.address;
	// fetch r and t
	for(int i=0;i<6;i++)
	{
		cam_pose->_pose_front[i]=(float64_t)calib_data_pointer[i];
		cam_pose->_pose_rear[i] = (float64_t)calib_data_pointer[6 + i];
		cam_pose->_pose_left[i] = (float64_t)calib_data_pointer[12 + i];
		cam_pose->_pose_right[i] = (float64_t)calib_data_pointer[18 + i];
	}

	
	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:

		break;
	case OPTIMIZE_TYPE_CENTER:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			cam_center[i * 2] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i];
			cam_center[i * 2 + 1] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i + 1];
		}		
		break;
	case OPTIMIZE_TYPE_FOV:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			cam_fov[i * 2] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i];
			cam_fov[i * 2 + 1] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i + 1];
		}
		break;
	case OPTIMIZE_TYPE_BOTH:
		for(int32_t i = 0; i < CAMERA_NUM; i++)
		{
			cam_center[i * 2] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i];
			cam_center[i * 2 + 1] = (float64_t)calib_data_pointer[CAMERA_NUM * 6 + 2 * i + 1];
			cam_fov[i * 2] = (float64_t)calib_data_pointer[CAMERA_NUM * 8 + 2 * i];
			cam_fov[i * 2 + 1] = (float64_t)calib_data_pointer[CAMERA_NUM * 8 + 2 * i + 1];
		}
		break;
	default:
		break;
	}

	for(int i=0;i<CAMERA_NUM;i++)
	{
		for(int j=0;j<8;j++)
		{
			result->eol_corner_point[i][j].x=(int)uv2_bev[0][i][2*j];
			result->eol_corner_point[i][j].y=(int)uv2_bev[0][i][2*j+1];
		}
	}
}

/*
	Function Name     : save_pose
	Function Function : save optimized r and t
	Input:
		filename      : the filename to be written
		rt_front      : r and t of front camera
		rt_rear       : r and t of rear camera
		rt_left       : r and t of left camera
		rt_right      : r and t of right camera
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/04/19       Create
 */
int32_t save_rt_pose(const char *filename, 
					 float32_t *rt_front, 
					 float32_t *rt_right, 
					 float32_t *rt_rear, 
					 float32_t *rt_left)
{
	if(!filename || !rt_front || !rt_left || !rt_right || !rt_rear) 
	{
		return -1;
	}

	FILE *fp = fopen(filename, "w");
	if(!fp) return -1;

	fprintf(fp, "%f %f %f\n", rt_front[0], rt_front[1], rt_front[2]);
	fprintf(fp, "%f %f %f\n", rt_front[3], rt_front[4], rt_front[5]);
	fprintf(fp, "%f %f %f\n", rt_right[0], rt_right[1], rt_right[2]);
	fprintf(fp, "%f %f %f\n", rt_right[3], rt_right[4], rt_right[5]);
	fprintf(fp, "%f %f %f\n", rt_rear[0], rt_rear[1], rt_rear[2]);
	fprintf(fp, "%f %f %f\n", rt_rear[3], rt_rear[4], rt_rear[5]);
	fprintf(fp, "%f %f %f\n", rt_left[0], rt_left[1], rt_left[2]);
	fprintf(fp, "%f %f %f\n", rt_left[3], rt_left[4], rt_left[5]);
	fclose(fp);
	return 0;
}


/*
	Function Name:      main
	Function Function : if source file is in yuv420, yuv422, and bgr format, use this function
	Input             : 
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    2016/8/5         Create
	                    YanShuo        2017/2/23        Add support different image format
						YanShuo        2017/3/8         Add support for NV12 format
*/

int main()
{
	int ret = 0;
	FILE *fp, *fp_avm, *fp_src[CAMERA_NUM];
	int32_t camid, i = 0, j = 0, raw_memory_size, src_memory_size;
	Smc_Cal_T avm_mc;
	IplImage *bgr_img[CAMERA_NUM];
	uchar* raw_img[CAMERA_NUM], *src_img[CAMERA_NUM];
	char str[256], subfix_name[20];
	void* pEolHandle = NULL;

#ifdef CONVERT_SAMPLE
	//Code for splitting raw data
	char path_name1[200] = "E:/project/EOL_product/sampel_data/GAC/splited_image";
	char video_file_name[200] = "E:/project/EOL_product/sampel_data/GAC/source_image/TestCase2/image_0.yuv";
	split_raw_data(video_file_name, path_name1,0, 1, 480, 736);
	/*int start_frame = 838;
	int frame_num = 23;
	char str1[256], str2[256];
	char path_name1[200] = "E:/temp/Group3_6";
	char video_file_name[200] = "E:/temp/3";

	for (int i = 0; i < 4; i++)
	{
		switch(i)
		{
		case 0:
			sprintf(str1, "%s/Front.I420", video_file_name);
			sprintf(str2, "%s/Front.nv12", path_name1);
			split_raw_data2(str1, str2, start_frame, frame_num, 720, 1280);
			break;
		case 1:
			sprintf(str1, "%s/Rear.I420", video_file_name);
			sprintf(str2, "%s/Rear.nv12", path_name1);
			split_raw_data2(str1, str2, start_frame, frame_num, 720, 1280);
			break;
		case 2:
			sprintf(str1, "%s/Left.I420", video_file_name);
			sprintf(str2, "%s/Left.nv12", path_name1);
			split_raw_data2(str1, str2, start_frame, frame_num, 720, 1280);
			break;
		case 3:
			sprintf(str1, "%s/Right.I420", video_file_name);
			sprintf(str2, "%s/Right.nv12", path_name1);
			split_raw_data2(str1, str2, start_frame, frame_num, 720, 1280);
			break;
		}
	}*/
	return 0;
#endif

#ifdef SAVE_LOG
	// open log file
	char log_name[40];
	sprintf(log_name, "%s/log.txt", FILE_SAVE_PATH);
	fp_log = fopen(log_name, "w");
	if(!fp_log)
	{
		printf("Open file error!\n");
		return OPEN_FILE_ERROR;
	}
#endif

	// load smc

	//if ((fp_avm = fopen("D:/project/Code/COMM/Data/Config/SMC.bin","rb")) == NULL)
	//if ((fp_avm = fopen("../../../../COMM/Data/Config/AVM_SMC(ENVISION_4_1_2_HD_LVDS_2017_5_12).bin","rb")) == NULL)
	//if ((fp_avm = fopen("E:/work/EOL_product/sampel_data/HeFei_JAC/AVM_SMC(C53F_4_1_2_HD_CVBS_2017_5_5).bin", "rb")) == NULL)
	//if ((fp_avm = fopen("E:/project/EOL_product/sampel_data/GAC/smc/AVM_SMC(CHB041_4_1_2_HD_CVBS_2017_9_12).bin", "rb")) == NULL)
	if ((fp_avm = fopen("E:/project/EOL_product/sampel_data/GAC/smc/AVM_SMC(DX3_4_2_0_SD_CVBS_2017_10_11).bin", "rb")) == NULL)
	{
		printf("read ASM error!\n");
		return -1;
	}
	fread(&avm_mc,sizeof(Smc_Cal_T),1,fp_avm);
	fclose(fp_avm);

	// create image
	switch (CUR_IMAGE_TYPE)
	{
	case TYPE_YUV420:
		raw_memory_size = sizeof(uchar) * RAW_HEIGHT * RAW_WIDTH * 3 >> 1;
		src_memory_size = sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH * 3 >> 1;
		//sprintf(subfix_name, "yuv420");yp
		sprintf(subfix_name, "yuv");
		break;
	case TYPE_YUV422:
		raw_memory_size = sizeof(uchar) * RAW_HEIGHT * RAW_WIDTH * 2;
		src_memory_size = sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH * 2;
		sprintf(subfix_name, "yuv422");
		break;
	case TYPE_NV12:
		raw_memory_size = sizeof(uchar) * RAW_HEIGHT * RAW_WIDTH * 3 >> 1;
		src_memory_size = sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH * 3 >> 1;
		sprintf(subfix_name, "nv12");
		break;
	case TYPE_BGR:
		raw_memory_size = sizeof(uchar) * RAW_HEIGHT * RAW_WIDTH * 3;
		src_memory_size = sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH * 3;
		sprintf(subfix_name, "bgr");
		break;
	case TYPE_RGBA:
		raw_memory_size = sizeof(uchar) * RAW_HEIGHT * RAW_WIDTH * 4;
		src_memory_size = sizeof(uchar) * IMAGE_HEIGHT * IMAGE_WIDTH * 4;
		sprintf(subfix_name, "rgba");
		break;
	default:
		break;
	}

	for(int i = 0; i < CAMERA_NUM; i++)
	{
		bgr_img[i] = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
		raw_img[i] = (uchar*)malloc(raw_memory_size);
		src_img[i] = (uchar*)malloc(src_memory_size);
		if(!src_img[i] || !bgr_img[i] || !raw_img[i])
		{
			printf("malloc image memory error.\n");
			return MEM_MALLOC_FAIL;
		}
		memset(raw_img[i], 0, raw_memory_size);
		memset(src_img[i], 0, src_memory_size);
	}

	avm_mc.bev_calib_param.bev_eol_param.board_veh_flboard2centralaxis = 1000;
	//char path_name[256] = "E:/图片和视频/ZhuZhou/Group1";
	//char path_name[256] = "E:/work/EOL_product/sampel_data/HeFei_JAC/Group5";
    //char path_name[256] = "E:/work/EOL_product/sampel_data/JAC/JAC楼下车库/souce_data";
	//char path_name[256] = "E:/work/EOL_product/sampel_data/JAC/合肥采集图像/01";
	//char path_name[256] = "E:/work/EOL_product/sampel_data/GAC/source_data/1417/Test02";
	char path_name[256] = "E:/project/EOL_product/sampel_data/GAC/source_data/Test01";
	//char path_name[256] = "E:/Samples/EOL_Envision_HD_Dr_Ma/VideoFragment/Group1_1";
	for(camid = 0; camid < CAMERA_NUM; camid++)
	{
		char file_name[256];
		switch (camid)
		{
		case 0:
			sprintf(file_name, "%s/Front.%s", path_name, subfix_name);			
			break;
		case 1:
			sprintf(file_name, "%s/Rear.%s", path_name, subfix_name);
			break;
		case 2:
			sprintf(file_name, "%s/Left.%s", path_name, subfix_name);
			break;
		case 3:
			sprintf(file_name, "%s/Right.%s", path_name, subfix_name);
			break;
		default:
			break;
		}
		fp_src[camid] = fopen(file_name, "rb");
		if(!fp_src[camid])
		{
			printf("Open file error!\n");
			return OPEN_FILE_ERROR;
		}
	}

	// jump to assigned frame
	int32_t frame_size = 0;
	if(CUR_IMAGE_TYPE == TYPE_YUV420 || CUR_IMAGE_TYPE == TYPE_NV12)
		frame_size = RAW_HEIGHT * RAW_WIDTH * 3 >> 1;
	else if(CUR_IMAGE_TYPE == TYPE_YUV422)
		frame_size = RAW_HEIGHT * RAW_WIDTH * 2;
	else if(CUR_IMAGE_TYPE == TYPE_RGBA)
		frame_size = RAW_HEIGHT * RAW_WIDTH * 4;
	for(int i = 0; i < CAMERA_NUM; i++)
	{		
		fseek(fp_src[i], frame_size * START_FRAME_ID, 0);
	}
	frame_count = START_FRAME_ID;
	// run frame loop
	while( load_image(fp_src[0], raw_img[0], RAW_HEIGHT, RAW_WIDTH, CUR_IMAGE_TYPE)
		&& load_image(fp_src[1], raw_img[1], RAW_HEIGHT, RAW_WIDTH, CUR_IMAGE_TYPE)
		&& load_image(fp_src[2], raw_img[2], RAW_HEIGHT, RAW_WIDTH, CUR_IMAGE_TYPE)
		&& load_image(fp_src[3], raw_img[3], RAW_HEIGHT, RAW_WIDTH, CUR_IMAGE_TYPE)
		&& frame_count < START_FRAME_ID + VALID_FRAME_COUNT)
	{		
		printf("Processing frame %d...\n", frame_count);
		for(i = 0; i < CAMERA_NUM; i++)
		{			
			switch(CUR_IMAGE_TYPE)
			{
			case TYPE_YUV420:
				yuv420_cut(src_img[i], raw_img[i], RAW_HEIGHT, RAW_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH);
				yuv420_to_bgr(bgr_img[i], src_img[i]);
				break;
			case TYPE_YUV422:
				yuv422_cut(src_img[i], raw_img[i], RAW_HEIGHT, RAW_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH);
				yuv422_to_bgr(bgr_img[i], src_img[i]);
				break;
			case TYPE_NV12:
				nv12_cut(src_img[i], raw_img[i], RAW_HEIGHT, RAW_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH);
				nv12_to_bgr(bgr_img[i], src_img[i]);
				break;
			case TYPE_BGR:
				raw_to_bgr(bgr_img[i], src_img[i]);
				break;
			case TYPE_RGBA:
				rgba_cut(src_img[i], raw_img[i], RAW_HEIGHT, RAW_WIDTH, IMAGE_HEIGHT, IMAGE_WIDTH);
				rgba_to_bgr(bgr_img[i], src_img[i]);
				break;
			}
			if(0 == i || 1 == i)
			{
				//flip_image(bgr_img[i], true);
			}
		}
		ret = EOL_init(&pEolHandle, &avm_mc);
		if(EOL_SUCCESS != EOL_process((void**)bgr_img, &avm_mc, &pEolHandle))
		{
			printf("BEV_calib error\n");
#ifdef SAVE_LOG
			fprintf(fp_log, "Frame %d calibration error.  \n", frame_count);
#endif
		}
		EOL_deinit(&pEolHandle);
	}


	for(int i = 0; i < CAMERA_NUM; i++)
	{
		if(bgr_img[i])
		{
			cvReleaseImage(&bgr_img[i]);
		}
		if(src_img[i])
		{
			free(src_img[i]);
		}
		if(raw_img[i])
		{
			free(raw_img[i]);
		}
		fclose(fp_src[i]);
	}

#ifdef SAVE_LOG
	fclose(fp_log);
#endif

	return 0 ;
}
