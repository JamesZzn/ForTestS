#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "eol_pose_estimator.h"
#include "utility\math\levmarqt_minimizer.h"
#include "eol_utility.h"

extern IplImage *EOL_bev_data_c_raw[CAMERA_NUM];
extern IplImage *EOL_bev_data_c[CAMERA_NUM];
extern IplImage *EOL_bev_data_y[CAMERA_NUM];

#define MEM_FREE(a) if(a) { free(a); }
#define RMSE_THRESHOLD 4.0
#define TRANS_THRESHOLD 1000
#define CENTER_THRESHOLD 25
#define FOV_THRESHOLD ((20) / (180.0) * (PI)) 

EOL_avm_param_adjuster EOL_g_param_adjuster;

/*
	Function Name:      EOL_avm_ext_params_model
	Function Function : The error function used in LM algorithm
	Input             : 
	   params :         The params need to be optimized
	   num_errs :       Valid sample point's num
	   data :           NULL, not used
	   evec :           to store the re-projection error of each point sample
	   inform :         return lm status
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                    JiangRuyi      unknown          Create
						YanShuo        2017/02/12       Add camera center into optimizer
*/
static void  EOL_avm_ext_params_model(float64_t *params, 
									  int32_t num_errs, 
									  const void *data,
									  float64_t *evec, 
									  int32_t *inform ) 
{
	int32_t camid, i, j, k, np;

	float64_t  uv[2], ray[3] , rmat[9] , tvec[3] ;
	float64_t pxyz[3] ;
	float64_t sum_err[CAMERA_NUM];
	float64_t camera_center[2], camera_fov[2];

	EOL_str_cam_intrin *bev_cam ;
//	const EOL_v3_t *uv_icors , * xyz_wcors ;

	memset(evec, 0, sizeof(float64_t)*num_errs);

	np = 0 ;

	for(camid=0; camid < CAMERA_NUM; camid++ )
	{
		sum_err[camid] = 0;
		bev_cam = EOL_g_param_adjuster._bev_cam[camid] ;
		//EOL_rmat_from_rvec((float64_t*)rmat , &params[camid*6] ) ;
		
		Cvt_Angles_To_Rotation_DB(rmat, &params[camid*6]);
		memcpy(tvec, &params[6 * camid + 3], sizeof(float64_t) * 3);
		
	
		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			memcpy(camera_center, &params[CAMERA_NUM*6 + 2*camid] , sizeof(float64_t)*2 );
			break;
		case OPTIMIZE_TYPE_FOV:
			memcpy(camera_fov, &params[CAMERA_NUM*6 + 2*camid] , sizeof(float64_t)*2 );
			break;
		case OPTIMIZE_TYPE_BOTH:
			memcpy(camera_center, &params[CAMERA_NUM*6 + 2*camid] , sizeof(float64_t)*2 );
			memcpy(camera_fov, &params[CAMERA_NUM*8 + 2*camid] , sizeof(float64_t)*2 );
			break;
		default:
			break;
		}

#ifdef RE_PROJECTION_ERROR
		float64_t R[9], angle[3], t[3];

		Cvt_Angles_To_Rotation_DB(R, &params[camid * 6]); // convert Euler angle to rotation matrix

		t[0] = params[camid * 6 + 3];
		t[1] = params[camid * 6 + 4];
		t[2] = params[camid * 6 + 5];

		Inverse_RT_InPlace_DB(R, t); // update R = R' and t = -R'T
		Cvt_Rotation_To_Angles_DB(R, angle); // update angle in order x, y and z
#endif

		//float64_t chessboard_area[4] = { 0, 0, 0, 0 }, max_area = 0;
		//for (i = 0; i < CAMERA_NUM; i++)
		//{
		//	const Chessboard_Pattern *p = EOL_g_param_adjuster.pCB[camid][i];
		//	float64_t min_row = 1280, min_col = 1280, max_row = 0, max_col = 0;
		//	if (p)
		//	{
		//		for (j = 0; j < p->rows; j++)
		//		{
		//			for (k = 0; k < p->cols; k++)
		//			{
		//				if (p->flag[j][k] > 0)
		//				{
		//					if (p->corner_point[j][k].x > max_col) max_col = p->corner_point[j][k].x;
		//					if (p->corner_point[j][k].x < min_col) min_col = p->corner_point[j][k].x;
		//					if (p->corner_point[j][k].y > max_row) max_row = p->corner_point[j][k].y;
		//					if (p->corner_point[j][k].y < min_row) min_row = p->corner_point[j][k].y;
		//				}
		//			}
		//		}
		//		chessboard_area[i] = (float64_t)(max_row - min_row + 1) * (max_col - min_col + 1) / (p->rows * p->cols);
		//		max_area = chessboard_area[i] > max_area ? chessboard_area[i] : max_area;
		//	}
		//}

		for(i = 0; i < CB_MAX_NUM; i++)
		{
			const Chessboard_Pattern *p = EOL_g_param_adjuster.pCB[camid][i];
			if (p)
			{
				//float weight = 1.0 / chessboard_area[i] * max_area;
				float64_t weight = 1.0;
				if (p->cols == 2 && p->rows == 2) weight = 2.0;
				float64_t weight_chessboard = 1.0;
				if (p->cols == 2 && p->rows == 2) weight_chessboard = 1.5;
				float64_t weight_ground = 0.9;
				float64_t weight_image = 1 - weight_ground;
				
				for(j = 0; j < p->rows; j++)
				{
					for(k = 0; k < p->cols; k++)
					{
						if(p->flag[j][k]>0)
						{
							pxyz[0] = p->ground_corner_point[j][k].x + EOL_g_param_adjuster.offset_x;
							pxyz[1] = p->ground_corner_point[j][k].y + EOL_g_param_adjuster.offset_y;
							pxyz[2] = 0;							


							switch (CURR_OPTIMIZE_TYPE)
							{
							case OPTIMIZE_TYPE_NONE:
								break;
							case OPTIMIZE_TYPE_CENTER:
								bev_cam->cu = camera_center[0];
								bev_cam->cv = camera_center[1];
								break;
							case OPTIMIZE_TYPE_FOV:
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
								break;
							case OPTIMIZE_TYPE_BOTH:
								bev_cam->cu = camera_center[0];
								bev_cam->cv = camera_center[1];

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
								break;
							default:
								break;
							}

							ray[0] = (rmat[0] * pxyz[0] + rmat[1] * pxyz[1] + tvec[0]);
							ray[1] = (rmat[3] * pxyz[0] + rmat[4] * pxyz[1] + tvec[1]); 
							ray[2] = (rmat[6] * pxyz[0] + rmat[7] * pxyz[1] + tvec[2]);

							Cam_MapCamRay2ImagePoint64(uv , ray , bev_cam ) ;

							float64_t dx = fabs(uv[0] - p->corner_point[j][k].x);
							float64_t dy = fabs(uv[1] - p->corner_point[j][k].y);
							evec[np * 2] = weight_image * weight_image * weight * weight * dx * dx;
							evec[np * 2 + 1] = weight_image * weight_image * weight * weight * dy * dy;
							//printf(" %7.2f  ", sqrt(evec[np*2]+evec[np*2+1]));	
							sum_err[camid] += sqrt(evec[np * 2] + evec[np * 2 + 1]) / weight;

#ifdef RE_PROJECTION_ERROR
							float64_t ptGnd[2], ptImg[2], incline_ray[3], coef;
							ptImg[0] = p->corner_point[j][k].x;
							ptImg[1] = p->corner_point[j][k].y;

							Cam_MapImagePoint2CamRay64(incline_ray, ptImg, bev_cam); // image point to incline ray

							// calculate the ray in world coordinate system
							ray[0] = R[0] * incline_ray[0] + R[1] * incline_ray[1] + R[2] * incline_ray[2];
							ray[1] = R[3] * incline_ray[0] + R[4] * incline_ray[1] + R[5] * incline_ray[2];
							ray[2] = R[6] * incline_ray[0] + R[7] * incline_ray[1] + R[8] * incline_ray[2];

							coef = t[2] / (-ray[2]);

							ptGnd[0] = ray[0] * coef + t[0];
							ptGnd[1] = ray[1] * coef + t[1];

							float64_t delat_x = 0.1 * fabs(ptGnd[0] - pxyz[0]);
							float64_t delat_y = 0.1 * fabs(ptGnd[1] - pxyz[1]);
							evec[np * 2] += weight_ground * weight_ground * weight_chessboard * weight_chessboard * delat_x * delat_x;
							evec[np * 2 + 1] += weight_ground * weight_ground * weight_chessboard * weight_chessboard * delat_y * delat_y;
							sum_err[camid] += sqrt(evec[np * 2] + evec[np * 2 + 1]) / weight_chessboard;
#endif	
							np++;
						}
					}
				}
			}
		}
		EOL_g_param_adjuster.rmse[camid] = (float32_t)sum_err[camid]/EOL_g_param_adjuster.valid_pts_num[camid];
		//printf("\n");
		//printf("rmse[%d]: %7.2f  ", camid, EOL_g_param_adjuster.rmse[camid]);		
	}
	//printf("\n");
}

/*
	Function Name:      EOL_delete_outliers
	Function Function : Delete the outliers points of the current array
	Input             : 
	   fvec :           The array need to be selected
	   num_pts :        The length of value_array
	   p_Eol_param :    p_Eol_param
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/22       Create
						YanShuo        2017/05/11       Remove dynamic memory malloc
*/
static int32_t EOL_delete_outliers(float64_t* fvec, int32_t num_pts, 
								   const P_EOL_Param p_Eol_param)
{
	int32_t ret = EOL_SUCCESS;
	int32_t i = 0, np = 0, max_group_num = 2;
	/*reproj_error_array is the array whose outliers need to be removed*/

	float64_t reproj_error_array[EOL_MAX_CORNER_NUM * CAMERA_NUM];
	float64_t value_array[EOL_MAX_CORNER_NUM];
	int32_t label_array[EOL_MAX_CORNER_NUM];

	for(i = 0; i < num_pts; i++)
	{
		reproj_error_array[i] = sqrt(fvec[i * 2] + fvec[i * 2 + 1]);
	}

	int32_t cur_index = 0;
	for(int32_t camid = 0; camid < CAMERA_NUM; camid++)
	{
		// for each camera, call k-means algorithm
		int32_t valid_corner_count = 0;
		float64_t sum_error = 0, median_value;		

		//memcpy(value_array, &reproj_error_array[cur_index], sizeof(float64_t) * EOL_g_param_adjuster.valid_pts_num[camid]);
		//memset(label_array, 0, sizeof(int32_t) * EOL_MAX_CORNER_NUM);
		for(int32_t i = 0; i < EOL_g_param_adjuster.valid_pts_num[camid]; i++)
		{
			value_array[i] = reproj_error_array[cur_index + i];
			label_array[i] = 0;
		}

		ret = find_median(value_array, EOL_g_param_adjuster.valid_pts_num[camid], median_value);
		
		//memcpy(value_array, &reproj_error_array[cur_index], sizeof(float64_t) * EOL_g_param_adjuster.valid_pts_num[camid]);

		for(int32_t i = 0; i < EOL_g_param_adjuster.valid_pts_num[camid]; i++)
		{
			value_array[i] = reproj_error_array[cur_index + i];		
		}

		for(int i = 0; i < EOL_g_param_adjuster.valid_pts_num[camid]; i++)
		{
			if(value_array[i] > 4 * median_value)
			{
				label_array[i] = 1;
			}
		}

		/* label outliers */
		for(int i = 0; i < p_Eol_param->cb_pattern_group[camid].valid_num; i++)
		{
			for(int m = 0; m < 4; m++)
			{
				for(int n = 0; n < 4; n++)
				{
					if( p_Eol_param->cb_pattern_group[camid].pattern[i].flag[m][n] > 0)
					{
						if(label_array[valid_corner_count] > 0)
						{
							p_Eol_param->cb_pattern_group[camid].pattern[i].flag[m][n] = 0;
							p_Eol_param->cb_pattern_group[camid].pattern[i].corner_point[m][n].x = 0;
							p_Eol_param->cb_pattern_group[camid].pattern[i].corner_point[m][n].y = 0;
							p_Eol_param->cb_pattern_group[camid].pattern[i].valid_point_num--;
						}
						valid_corner_count++;
					}
				}
			}
		}
		
		cur_index += EOL_g_param_adjuster.valid_pts_num[camid];

	}
	return ret;
}

/*
	Function Name:      EOL_remove_outliers
	Function Function : Cal re-projection error and remove outliers
	Input             : 
	   pSmc :           pSmc
	   avm_cam_bev :    camera intrinsic param
	   p_Eol_param :    p_Eol_param
	   world_model:     describe the relationship between vehicle coordinate system and world coordinate system
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/22       Create
*/
int32_t EOL_remove_outliers(Smc_Cal_T* const           pSmc,
							EOL_str_avm_intrin* const  avm_cam_bev ,
							P_EOL_Param const          p_Eol_param,
							EOL_str_world_model* const world_model)
{
	int ret = EOL_SUCCESS;
	float64_t pose_t[CAMERA_NUM][6], camera_center[CAMERA_NUM][2], camera_fov[CAMERA_NUM][2];
	int32_t num_pts , n_p, num_params, camid;
	float64_t* fvec;
	EOL_str_cam_intrin *bev_cam[CAMERA_NUM];

	float64_t* params = NULL;
	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		num_params = CAMERA_NUM * 6;
		break;
	case OPTIMIZE_TYPE_CENTER:
	case OPTIMIZE_TYPE_FOV:
		num_params = CAMERA_NUM * 8;
		break;
	case OPTIMIZE_TYPE_BOTH:
		num_params = CAMERA_NUM * 10;
		break;
	default:
		break;
	}

	params = (float64_t*)malloc(sizeof(float64_t) * num_params);
	memset(params, 0, sizeof(float64_t) * num_params);

	num_pts = 0;
	n_p = 0;

	bev_cam[0] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_front ;
	bev_cam[1] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_rear ;
	bev_cam[2] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_left ;
	bev_cam[3] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_right ;
	for(int i = 0; i < CAMERA_NUM; i++)
	{
		pose_t[i][0] = pSmc->camera_param[i].cam_ext.rx;
		pose_t[i][1] = pSmc->camera_param[i].cam_ext.ry;
		pose_t[i][2] = pSmc->camera_param[i].cam_ext.rz;
		pose_t[i][3] = pSmc->camera_param[i].cam_ext.tx;
		pose_t[i][4] = pSmc->camera_param[i].cam_ext.ty;
		pose_t[i][5] = pSmc->camera_param[i].cam_ext.tz;
		// when camera_center need to be optimized, do this initialization
		camera_center[i][0] = bev_cam[i]->cu;
		camera_center[i][1] = bev_cam[i]->cv;
		camera_fov[i][0] = bev_cam[i]->fu;
		camera_fov[i][1] = bev_cam[i]->fv;
	}

	for(camid = 0; camid < CAMERA_NUM ; camid++) 
	{
		EOL_g_param_adjuster._bev_cam[camid] = bev_cam[camid];

		float64_t R[9];
		Cvt_Angles_To_Rotation_DB(R, pose_t[camid]); // convert Eular angle to rotation matrix

		params[6*camid+3] = pose_t[camid][3];
		params[6*camid+4]= pose_t[camid][4];
		params[6*camid+5] = pose_t[camid][5];
		Inverse_RT_InPlace_DB(R, &params[6*camid+3]); // update t
		Cvt_Rotation_To_Angles_DB(R, &params[6*camid]); // update r in order x, y and z

		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_center[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_center[camid][1];
			break;
		case OPTIMIZE_TYPE_FOV:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_fov[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_fov[camid][1];
			break;
		case OPTIMIZE_TYPE_BOTH:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_center[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_center[camid][1];
			params[CAMERA_NUM * 8 + 2*camid + 0] = camera_fov[camid][0];
			params[CAMERA_NUM * 8 + 2*camid + 1] = camera_fov[camid][1];
			break;
		default:
			break;
		}
	}

	/* Config corner information as sample points of LM*/
	for (camid = 0; camid < CAMERA_NUM; camid++)
	{
		n_p = 0;
		for (int pattern_id = 0; pattern_id < CB_MAX_NUM; pattern_id++)
		{
			if(pattern_id < p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				n_p += get_valid_corner_num(&p_Eol_param->cb_pattern_group[camid].pattern[pattern_id]);
				EOL_g_param_adjuster.pCB[camid][pattern_id] = &p_Eol_param->cb_pattern_group[camid].pattern[pattern_id];
			}
			else
			{
				EOL_g_param_adjuster.pCB[camid][pattern_id] = NULL;
			}

		}
		EOL_g_param_adjuster.valid_pts_num[camid] = n_p;
		if(n_p <= 3)
		{
			ret = INSUFFICIENT_CORNER_NUM;
			return ret;
		}

		num_pts += n_p;
	}

	EOL_g_param_adjuster._world_model = world_model ;
	EOL_g_param_adjuster.site_x = (float32_t)p_Eol_param->station_height;
	EOL_g_param_adjuster.site_y = (float32_t)p_Eol_param->station_width;
	EOL_g_param_adjuster.offset_x = (float32_t)(world_model->_veh_len / 2 - world_model->_rear_axis_2_bumper);
	EOL_g_param_adjuster.offset_y = 0;

	float64_t ffvec[EOL_MAX_CORNER_NUM * 2];

	fvec = (float64_t*)malloc(sizeof(float64_t) * num_pts * 2);
	if(!fvec)
	{
		return MEM_MALLOC_FAIL;
	}
	memset(fvec, 0, sizeof(float64_t) * num_pts * 2);

	EOL_avm_ext_params_model(params, num_params, NULL, fvec, &ret);

	ret = EOL_delete_outliers(fvec, num_pts, p_Eol_param);

	if(fvec)
		free(fvec);
	if(params)
		free(params);
	return ret;
}

/*
	Function Name:      EOL_calculate_homography
	Function Function : calculate homography matrix by the input point pairs
	Input             : 
	   homography :     the homography matrix to be output
	   ground_points :  world coordinates
	   calib_points :   corner coordinate on planer
	  valid_points_num: valid points num
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/05/15       Create
*/
static int32_t EOL_calculate_homography(float64_t homography[9], 
								 float64_t* ground_points,
								 float64_t* calib_points,
								 int32_t valid_points_num)
{
	int32_t ret = EOL_SUCCESS;
	float64_t A[EOL_MAX_CORNER_NUM*3][9], w[EOL_MAX_CORNER_NUM*3], v[9][9];

	/* Because of the cross product of 2 collinear vector = 0, we have
	*    [u]       [ [r11  r12  t1]   [X] ]        [0]
	*    [v]   X   [ [r21  r22  t2] * [Y] ]   =    [0]
	*    [w]       [ [r31  r32  t3]   [1] ]        [0]
	*
	* Expand this equation, we can obtain three equations, that is:
	*    v * [r31X + R32Y + t3] - w * [r21X + R22Y + t2]   =   0
	*    w * [r11X + R12Y + t1] - u * [r21X + R22Y + t3]   =   0
	*    u * [r21X + R22Y + t2] - v * [r11X + R12Y + t1]   =   0
	*
	* Expand this equation a step more, we can obtain:
	*   0 * r11 +  0 * r12 +  0 * t1 - wX * r21 - wY * r22 - wZ * t2 + vX * r31 + vY * r32 + vZ * t3 = 0
	*  wX * r11 + wY * r12 + wZ * t1 -  0 * r21 -  0 * r22 -  0 * t2 - uX * r31 - uY * r32 - uZ * t3 = 0
	* -vX * r11 - vY * r12 - vZ * t1 + uX * r21 + uY * r22 + uZ * t2 +  0 * r31 +  0 * r32 +  0 * t3 = 0
	*
	* Let H = [r11, r12, t1, r21, r22, t2, r31, r32, t3], 
	*
	* We obtain M*H' = 0, where M is defined as
	*     [   0,   0,   0, -wX, -wY, -wZ,  vX,  vY,  vZ ] 
	* M = [  wX,  wY,  wZ,   0,   0,   0, -uX, -uY, -uZ ]
	*     [ -vX, -vY, -vZ,  uX,  uY,  uZ,   0,   0,   0 ]
	*
	* Let A = M, x = H', we got Ax = 0; x is the parameters to be solved.
	* We use SVD decomposition to solve this function.
	*/
	for(int i = 0; i < valid_points_num; i++)
	{
		A[i * 3][0] = 0;
		A[i * 3][1] = 0;
		A[i * 3][2] = 0;
		A[i * 3][3] = - calib_points[i * 3 + 2] * ground_points[i * 3 + 0];
		A[i * 3][4] = - calib_points[i * 3 + 2] * ground_points[i * 3 + 1];
		A[i * 3][5] = - calib_points[i * 3 + 2] * ground_points[i * 3 + 2];
		A[i * 3][6] = calib_points[i * 3 + 1] * ground_points[i * 3 + 0];
		A[i * 3][7] = calib_points[i * 3 + 1] * ground_points[i * 3 + 1];
		A[i * 3][8] = calib_points[i * 3 + 1] * ground_points[i * 3 + 2];

		A[i * 3 + 1][0] = calib_points[i * 3 + 2] * ground_points[i * 3 + 0];
		A[i * 3 + 1][1] = calib_points[i * 3 + 2] * ground_points[i * 3 + 1];
		A[i * 3 + 1][2] = calib_points[i * 3 + 2] * ground_points[i * 3 + 2];
		A[i * 3 + 1][3] = 0;
		A[i * 3 + 1][4] = 0;
		A[i * 3 + 1][5] = 0;
		A[i * 3 + 1][6] = - calib_points[i * 3 + 0] * ground_points[i * 3 + 0];
		A[i * 3 + 1][7] = - calib_points[i * 3 + 0] * ground_points[i * 3 + 1];
		A[i * 3 + 1][8] = - calib_points[i * 3 + 0] * ground_points[i * 3 + 2];

		A[i * 3 + 2][0] = - calib_points[i * 3 + 1] * ground_points[i * 3 + 0];
		A[i * 3 + 2][1] = - calib_points[i * 3 + 1] * ground_points[i * 3 + 1];
		A[i * 3 + 2][2] = - calib_points[i * 3 + 1] * ground_points[i * 3 + 2];
		A[i * 3 + 2][3] = calib_points[i * 3 + 0] * ground_points[i * 3 + 0];
		A[i * 3 + 2][4] = calib_points[i * 3 + 0] * ground_points[i * 3 + 1];
		A[i * 3 + 2][5] = calib_points[i * 3 + 0] * ground_points[i * 3 + 2];
		A[i * 3 + 2][6] = 0;
		A[i * 3 + 2][7] = 0;
		A[i * 3 + 2][8] = 0;
	}

	/* The minimum eigen value's corresponding eigen vector is the solution */
	ret = svd(A, valid_points_num * 3, 9, w, v);
	CHECK_ERROR(ret);

	/* Find the minimum eigen value's index */
	int32_t min_index = 0;
	for(int i = 0; i < 9; i++)
	{
		if(w[i] < w[min_index])
		{
			min_index = i;
		}
	}
	/* the corresponding column vector is the solution */
	for(int i = 8; i >= 0; i--)
	{
		homography[i] = v[i][min_index];
	}

	/* homography normalization */
	float64_t scale1 = sqrt(homography[0]*homography[0] + homography[3]*homography[3] + homography[6]*homography[6]);
	float64_t scale2 = sqrt(homography[1]*homography[1] + homography[4]*homography[4] + homography[7]*homography[7]);
	float64_t scale3 = sqrt(0.5 * (scale1*scale1 + scale2*scale2));

	for(int i = 0; i < 3; i++)
	{
		homography[i * 3 + 0] /= scale1;
		homography[i * 3 + 1] /= scale2;
		homography[i * 3 + 2] /= scale3;
	}

	// Any corner's z coordinate in camera coordinate system should be positive 
	int32_t symbol_coef = ( homography[6] * ground_points[0] 
	+ homography[7] * ground_points[1]
	+ homography[8] ) > 0 ? 1 : -1;
	for(int i = 0; i < 9; i++)
	{
		homography[i] *= symbol_coef;
	}
	return ret;
}

/*
	Function Name:      EOL_calculate_init_rt
	Function Function : use ground coordinate and the detected corner coordinate to calculate 
	                    the initial value of camera extrinsic param
	Input             : 
	   pSmc :           pSmc
	   avm_cam_bev :    camera intrinsic param
	   p_Eol_param :    p_Eol_param
	   world_model:     describe the relationship between vehicle coordinate system and world coordinate system
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/23       Create
*/
int32_t EOL_calculate_init_rt(Smc_Cal_T* const           pSmc,
						      EOL_str_avm_intrin*        avm_cam_bev ,
						      P_EOL_Param                p_Eol_param,
						      EOL_str_world_model* const world_model)
{
	int ret = EOL_SUCCESS;
	float64_t pose_t[CAMERA_NUM][6];
	int32_t num_pts, n_p, num_params, camid, pattern_id, row, col;
	//float64_t ground_points[EOL_MAX_CORNER_NUM*3]; // x, y, z = 0
	//float64_t corner_points[EOL_MAX_CORNER_NUM*3]; // x, y
	//float64_t calib_points[EOL_MAX_CORNER_NUM*3];  // x, y, z
	float64_t* ground_points = NULL;
	ground_points = (float64_t*)malloc(sizeof(float64_t) * 45 * 3);
	float64_t* corner_points = NULL;
	corner_points = (float64_t*)malloc(sizeof(float64_t) * 45 * 3);
	float64_t*calib_points = NULL;
	calib_points = (float64_t*)malloc(sizeof(float64_t) * 45 * 3);
	float64_t ray[3];
	int32_t valid_points_num[CAMERA_NUM];

	EOL_str_cam_intrin *bev_cam[CAMERA_NUM];

	bev_cam[0] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_front ;
	bev_cam[1] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_rear ;
	bev_cam[2] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_left ;
	bev_cam[3] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_right ;

	// get ground world coordinate and corner point coordinate
	for(camid = 0; camid < CAMERA_NUM; camid++)
	{	
		float64_t homography[9];
		int32_t valid_pattern_num = p_Eol_param->cb_pattern_group[camid].valid_num;

		valid_points_num[camid] = 0;
		for(pattern_id = 0; pattern_id < valid_pattern_num; pattern_id++)
		{
			for(row = 0; row < p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].rows; row++)
			{
				for(col = 0; col < p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].cols; col++)
				{
					if(p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].flag[row][col])
					{
						ground_points[valid_points_num[camid] * 3] = 
							p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].ground_corner_point[row][col].x;
						ground_points[valid_points_num[camid] * 3 + 1] = 
							p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].ground_corner_point[row][col].y;
						ground_points[valid_points_num[camid] * 3 + 2] = 1;

						/* world coordinate system to vehicle coordinate system*/
						ground_points[valid_points_num[camid] * 3] += (float32_t)(world_model->_veh_len / 2 - world_model->_rear_axis_2_bumper);


						corner_points[valid_points_num[camid] * 2] = 
							p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].corner_point[row][col].x;
						corner_points[valid_points_num[camid] * 2 + 1] = 
							p_Eol_param->cb_pattern_group[camid].pattern[pattern_id].corner_point[row][col].y;

						/*corner point to ray[3]*/
						Cam_MapImagePoint2CamRay64(ray, &corner_points[valid_points_num[camid] * 2], bev_cam[0]);						
						
						/* default focal len is 1.0 */
						calib_points[valid_points_num[camid] * 3 + 0] = ray[0] / ray[2]; 
						calib_points[valid_points_num[camid] * 3 + 2] = 1.0;
						calib_points[valid_points_num[camid] * 3 + 1] = ray[1] / ray[2];

						valid_points_num[camid]++;
					} // end of if 
				} // end of loop col
			} // end of loop row 
		} // end of loop pattern_id
	
		EOL_calculate_homography(homography, ground_points, calib_points, valid_points_num[camid]);
		
		/* Because the ground coordinate is set as 0, so the 3rd column is not used, 
		 * thus the homography of world->cam is defined as:
         * for x and z axis, anti-clock rotation is positive direction,
		 * for y axis, clock rotation is positive direction. 
		 *
		 * [ cos(z)*cos(y), -cos(x)*sin(z)+sin(x)*sin(y)*cos(z),  sin(z)*sin(x)+cos(x)*sin(y)*cos(z), tx]
		 * [ sin(z)*cos(y),  cos(x)*cos(z)+sin(x)*sin(y)*sin(z), -sin(x)*cos(z)+cos(x)*sin(y)*sin(z), ty]
		 * [      - sin(y),                       sin(x)*cos(y),                       cos(x)*cos(y), tz]
		 */
		float64_t angle[6];
		homography_to_angle(homography, angle);

		/* The transformation from vehicle coordinate system to camera coordinate system */
		pose_t[camid][0] = angle[0]; /* The rotation of x axis, anti-clockwise direction is positive direction */
		pose_t[camid][1] = angle[1]; /* The rotation of y axis, clockwise direction is positive direction */
		pose_t[camid][2] = angle[2]; /* The rotation of z axis, anti-clockwise direction is positive direction */
		pose_t[camid][3] = homography[2]; /* The world coordinate system's origin point's x coordinate in camera coordinate system */
		pose_t[camid][4] = homography[5]; /* The world coordinate system's origin point's y coordinate in camera coordinate system */
		pose_t[camid][5] = homography[8]; /* The world coordinate system's origin point's z coordinate in camera coordinate system */

		/* Change the r, t of world->cam into the r, t of cam->world */
		float64_t R[9];
		Cvt_Angles_To_Rotation_DB(R, pose_t[camid]); // convert Euler angle to rotation matrix
		Inverse_RT_InPlace_DB(R, &pose_t[camid][3]); // update R = R' and t = -R'T
		Cvt_Rotation_To_Angles_DB(R, pose_t[camid]); // update r in order x, y and z

		pSmc->camera_param[camid].cam_ext.rx = pose_t[camid][0];
		pSmc->camera_param[camid].cam_ext.ry = pose_t[camid][1];
		pSmc->camera_param[camid].cam_ext.rz = pose_t[camid][2];
		pSmc->camera_param[camid].cam_ext.tx = pose_t[camid][3];
		pSmc->camera_param[camid].cam_ext.ty = pose_t[camid][4];
		pSmc->camera_param[camid].cam_ext.tz = pose_t[camid][5];
	} // end of loop camid
	if (ground_points)
	{
		free(ground_points);
	}
	if (corner_points)
	{
		free(corner_points);
	}
	if (calib_points)
	{
		free(calib_points);
	}
	return ret;
}

/*
	Function Name:      EOL_absolute_estimator_driver
	Function Function : use ground coordinate and the detected corner coordinate to calculate 
	                    the initial value of camera extrinsic param
	Input             : 
	   avm_pose :       camera extrinsic param
	   camera_center :  camera center coordinate
	   camera_fov :     camera field of view
	   bev_cam :        camera intrinsic param
	   p_Eol_param :    p_Eol_param
	   world_model:     describe the relationship between vehicle coordinate system and world coordinate system
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    JiangRuyi      unknown          Create
	                   	YanShuo        2017/02/14       Add camera center into optimizer
*/
static EOL_Calib_Diag_Status_T EOL_absolute_estimator_driver( 
														float64_t avm_pose[CAMERA_NUM][6] ,
														float64_t camera_center[CAMERA_NUM][2],
														float64_t camera_fov[CAMERA_NUM][2],
														EOL_str_cam_intrin *bev_cam[CAMERA_NUM] , 
														const P_EOL_Param p_Eol_param,
														const EOL_str_world_model*  world_model  ) 
{

	/*Bit of dirty here...*/
//	float64_t rmse0[4] , rmse1[4] , rmat_t[9] , tmat_t[3] , avm_rt[4][12] ;
	int32_t loop_count = 0;
	int32_t camid;

	int32_t num_pts , n_p, num_params;

	float64_t* params = NULL;
	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		num_params = CAMERA_NUM * 6;
		break;
	case OPTIMIZE_TYPE_CENTER:
	case OPTIMIZE_TYPE_FOV:
		num_params = CAMERA_NUM * 8;
		break;
	case OPTIMIZE_TYPE_BOTH:
		num_params = CAMERA_NUM * 10;
		break;
	default:
		break;
	}

	params = (float64_t*)malloc(sizeof(float64_t) * num_params);
	memset(params, 0, sizeof(float64_t) * num_params);


	levmarqt_config_struct levmarqt_config = levmarqt_config_double ;
	levmarqt_status_struct levmarqt_status ;
	lM_OPTIMIZOR_STATUS_T ret;

	

	/*estimate extrinsic*/
	//fprintf(stdout , "\n [absolute_estimator_driver] : Refine extrinsic param : \n") ;
	
	/* previous r and t is the transformation from camera coordinate system
	   to world coordinate system, after this for loop, r and t becomes the 
	   transformation from world coordinate system to camera coordinate
	   system */
	for(camid =0; camid < CAMERA_NUM; camid++) 
	{
		EOL_g_param_adjuster._bev_cam[camid] = bev_cam[camid];

		float64_t R[9];
		Cvt_Angles_To_Rotation_DB(R, avm_pose[camid]); // convert Euler angle to rotation matrix

		params[6*camid+3] = avm_pose[camid][3];
		params[6*camid+4]= avm_pose[camid][4];
		params[6*camid+5] = avm_pose[camid][5];
		Inverse_RT_InPlace_DB(R, &params[6*camid+3]); // update R = R' and t = -R'T
		Cvt_Rotation_To_Angles_DB(R, &params[6*camid]); // update r in order x, y and z

		memcpy(avm_pose[camid], &params[6*camid], sizeof(float64_t) * 3);
		

		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_center[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_center[camid][1];
			break;
		case OPTIMIZE_TYPE_FOV:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_fov[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_fov[camid][1];
			break;
		case OPTIMIZE_TYPE_BOTH:
			params[CAMERA_NUM * 6 + 2*camid + 0] = camera_center[camid][0];
			params[CAMERA_NUM * 6 + 2*camid + 1] = camera_center[camid][1];
			params[CAMERA_NUM * 8 + 2*camid + 0] = camera_fov[camid][0];
			params[CAMERA_NUM * 8 + 2*camid + 1] = camera_fov[camid][1];
			break;
		default:
			break;
		}
	}

	num_pts = 0;
	n_p = 0;

	/* Config corner information as sample points of LM*/
	for (camid = 0; camid < CAMERA_NUM; camid++)
	{
		n_p = 0;
		for (int pattern_id = 0; pattern_id < CB_MAX_NUM; pattern_id++)
		{
			if(pattern_id < p_Eol_param->cb_pattern_group[camid].valid_num)
			{
				n_p += get_valid_corner_num(&p_Eol_param->cb_pattern_group[camid].pattern[pattern_id]);
				EOL_g_param_adjuster.pCB[camid][pattern_id] = &p_Eol_param->cb_pattern_group[camid].pattern[pattern_id];
			}
			else
			{
				EOL_g_param_adjuster.pCB[camid][pattern_id] = NULL;
			}
			
		}
		EOL_g_param_adjuster.valid_pts_num[camid] = n_p;
		if(n_p <= 3)
		{
			return (EOL_Calib_Diag_Status_T)(FRONT_IMG_CORNER_NUM_WRONG + camid);
		}

		num_pts += n_p;
	}
		
	if(!levmarqt_minimizer_init(num_params, 2*num_pts))
		printf("LM error in memory allocaltion\n");

	float64_t optimized_params[CAMERA_NUM*10], optimized_rmse[CAMERA_NUM];
	
	float64_t *fvec = (float64_t *)malloc(2 * num_pts * sizeof(float64_t));
	memset(fvec, 0, 2 * num_pts * sizeof(float64_t));
	EOL_avm_ext_params_model(params, 2 * num_pts, NULL, fvec, &(levmarqt_status.userbreak));
	for (int i = 0; i < CAMERA_NUM; i++)
	{
		optimized_rmse[i] = EOL_g_param_adjuster.rmse[i]; // the rooted mean square of initial params
	}
	if (fvec) free(fvec);

	while (loop_count < 4)
	{
		ret = levmarqt_minimizer(
			num_params,
			params,
			2 * num_pts,
			NULL,
			EOL_avm_ext_params_model,
			&levmarqt_config,
			&levmarqt_status);

		for (int i = 0; i < CAMERA_NUM; i++)
		{
			if (EOL_g_param_adjuster.rmse[i] < optimized_rmse[i])
			{
				optimized_rmse[i] = EOL_g_param_adjuster.rmse[i];
				memcpy(&optimized_params[i * 6], &params[6 * i], sizeof(float64_t) * 6);

				switch (CURR_OPTIMIZE_TYPE)
				{
				case OPTIMIZE_TYPE_NONE:
					break;
				case OPTIMIZE_TYPE_CENTER:
				case OPTIMIZE_TYPE_FOV:
					memcpy(&optimized_params[CAMERA_NUM * 6 + i * 2], &params[CAMERA_NUM * 6 + 2 * i], sizeof(float64_t) * 2);
					break;
				case OPTIMIZE_TYPE_BOTH:
					memcpy(&optimized_params[CAMERA_NUM * 6 + i * 2], &params[CAMERA_NUM * 6 + 2 * i], sizeof(float64_t) * 2);
					memcpy(&optimized_params[CAMERA_NUM * 8 + i * 2], &params[CAMERA_NUM * 8 + 2 * i], sizeof(float64_t) * 2);
					break;
				default:
					break;
				}
			}
		}

		if (LM_SUCCESS == ret ||
			optimized_rmse[0] * 2 < RMSE_THRESHOLD &&
			optimized_rmse[1] * 2 < RMSE_THRESHOLD &&
			optimized_rmse[2] * 2 < RMSE_THRESHOLD &&
			optimized_rmse[3] * 2 < RMSE_THRESHOLD)
		{
			break;
		}
		else if (LM_PARAMETERS_NUM_INVAD == ret ||
				 LM_PARAMETERS_NUM_SMALLER_THAN_POINT_NUM == ret ||
				 LM_PARAMETERS_NEGATIVE_TOLERANCE == ret ||
				 LM_PARAMETERS_NONPOSITIVE_FUNCTION_EVALUATION_LIMIT == ret ||
				 LM_PARAMETERS_NONPOSITIVE_STEPBOUND == ret ||
				 LM_PARAMETERS_VARIABLE_NONLOGICAL == ret ||
				 LM_MEM_MALLOC_FAIL == ret ||
				 LM_FOUND_ZERO == ret)
		{
			printf("Config error\n");
			break;
		}
		else if (LM_DEGENERATE == ret ||
				 LM_CALL_LIMIT == ret ||
				 LM_FAILED_F == ret ||
				 LM_FAILED_P == ret ||
				 LM_FAILED_G == ret ||				 
				 LM_CONVERGED_F == ret ||
				 LM_CONVERGED_P == ret ||
				 LM_CONVERGED_BOTH == ret ||
				 LM_PARAMETERS_ERROR == ret ||
				 LM_STOP == ret) // in this case do this iteratively till convergence or max iteration time is arrivied at
		{
			for (int i = 0; i < CAMERA_NUM; i++)
			{
				if (EOL_g_param_adjuster.rmse[i] > RMSE_THRESHOLD)
				{
					switch (CURR_OPTIMIZE_TYPE)
					{
					case OPTIMIZE_TYPE_NONE:
						break;
					case OPTIMIZE_TYPE_CENTER:
						params[CAMERA_NUM * 6 + i * 2] = camera_center[i][0] + 5 * ((loop_count & 0x1) * 2 - 1);
						params[CAMERA_NUM * 6 + i * 2 + 1] = camera_center[i][1] + 5 * ((loop_count &0x2) - 1);
						break;
					case OPTIMIZE_TYPE_FOV:
						params[CAMERA_NUM * 6 + i * 2] = camera_fov[i][0] + 2 * PI / 180.0 * ((loop_count & 0x1) * 2 - 1);
						params[CAMERA_NUM * 6 + i * 2 + 1] = camera_fov[i][1] + 2 * PI / 180.0 * ((loop_count &0x2) - 1);
						break;
					case OPTIMIZE_TYPE_BOTH:
						params[CAMERA_NUM * 6 + i * 2] = camera_center[i][0] + 5 * ((loop_count & 0x1) * 2 - 1);
						params[CAMERA_NUM * 6 + i * 2 + 1] = camera_center[i][1] + 5 * ((loop_count &0x2) - 1);
						params[CAMERA_NUM * 8 + i * 2] = camera_fov[i][0] + 2 * PI / 180.0 * ((loop_count & 0x1) * 2 - 1);
						params[CAMERA_NUM * 8 + i * 2 + 1] = camera_fov[i][1] + 2 * PI / 180.0 * ((loop_count &0x2) - 1);
						break;
					default:
						break;
					}
					params[i * 6 + 0] = avm_pose[i][0] + PI * 1.0 / 180 * ((loop_count & 0x1) * 2 - 1);
					params[i * 6 + 1] = avm_pose[i][1] + PI * 1.0 / 180 * ((loop_count & 0x2) - 1);
					params[i * 6 + 2] = avm_pose[i][2] + PI * 1.0 / 180 * ((loop_count & 0x1) * 2 - 1);
					params[i * 6 + 3] = avm_pose[i][3] + 2 * ((loop_count & 0x2) - 1);
					params[i * 6 + 4] = avm_pose[i][4] + 2 * ((loop_count & 0x1) * 2 - 1);
					params[i * 6 + 5] = avm_pose[i][5] + 2 * ((loop_count & 0x2) - 1);
				}
			}
		}
		loop_count++;
	}
	levmarqt_minimizer_release();

	for(camid =0 ; camid<CAMERA_NUM ; camid++) 
	{
		float64_t R[9];
		EOL_g_param_adjuster.rmse[camid] = optimized_rmse[camid];
		Cvt_Angles_To_Rotation_DB(R, &optimized_params[6 * camid]); // convert Euler angle to rotation matrix
		Inverse_RT_InPlace_DB(R, &optimized_params[6 * camid + 3]); // update t
		Cvt_Rotation_To_Angles_DB(R, &optimized_params[6 * camid]); // update r in order x, y and z
		for(int i = 0; i < 6; i++)
		{
			avm_pose[camid][i] = optimized_params[6 * camid + i];
		}


		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			camera_center[camid][0] = params[CAMERA_NUM * 6 + 2*camid + 0];
			camera_center[camid][1] = params[CAMERA_NUM * 6 + 2*camid + 1];
			break;
		case OPTIMIZE_TYPE_FOV:
			camera_fov[camid][0] = params[CAMERA_NUM * 6 + 2*camid + 0];
			camera_fov[camid][1] = params[CAMERA_NUM * 6 + 2*camid + 1];
			break;
		case OPTIMIZE_TYPE_BOTH:
			camera_center[camid][0] = params[CAMERA_NUM * 6 + 2*camid + 0];
			camera_center[camid][1] = params[CAMERA_NUM * 6 + 2*camid + 1];
			camera_fov[camid][0] = params[CAMERA_NUM * 8 + 2*camid + 0];
			camera_fov[camid][1] = params[CAMERA_NUM * 8 + 2*camid + 1];
			break;
		default:
			break;
		}

		//char msg[1024];
		//sprintf(msg, "RMSE = %.2f (pix)", EOL_g_param_adjuster.rmse[camid]);
		//cv::putText(cv::cvarrToMat(EOL_bev_data_c_raw[camid]), msg, cv::Point(10, 100), 1, 2.2, CV_RGB(255,0,0), 2);
		//sprintf(msg, "Position(mm) : (%.2f,  %.2f, %.2f)",avm_pose[camid][3], avm_pose[camid][4], avm_pose[camid][5]);
		//cv::putText(cv::cvarrToMat(EOL_bev_data_c_raw[camid]), msg, cv::Point(10, 150), 1, 2.2, CV_RGB(255,0,0), 2);
		//sprintf(msg, "Posture(rad) : (%.2f,  %.2f, %.2f)",avm_pose[camid][0], avm_pose[camid][1], avm_pose[camid][2]);
		//cv::putText(cv::cvarrToMat(EOL_bev_data_c_raw[camid]), msg, cv::Point(10, 200), 1, 2.2, CV_RGB(255,0,0), 2);
	}

	/*export*/
	//for(camid=0 ; camid<4 ; camid++) 
	//{
	//	for(int k = 0; k< 6; k++)
	//	{
	//		avm_pose[camid][k] = params[6*camid+k];
	//	}
	//}
	if(params)	free(params);
  	return EOL_SUCCESS ;
}

/*
	Function Name:      EOL_pose_estimator
	Function Function : use ground coordinate and the detected corner coordinate to calculate 
	                    the initial value of camera extrinsic param
	Input             : 
	   calib_data_buff :Buffer for saving optimized result
	   pSmc :           pSmc
	   avm_cam_bev :    camera intrinsic param
	   p_Eol_param :    p_Eol_param
	   world_model:     describe the relationship between vehicle coordinate system and world coordinate system
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    JiangRuyi      unknown          Create
	                   	YanShuo        2017/02/14       Add camera center into optimizer
*/
EOL_Calib_Diag_Status_T EOL_pose_estimator(EOL_Buffer_Mng_T* calib_data_buff,
										   Smc_Cal_T* pSmc,
										   EOL_str_avm_intrin* avm_cam_bev,
										   const P_EOL_Param p_Eol_param,
										   EOL_str_world_model*  world_model )
{
	int32_t layid=0 ,  camid=0 , boxid=0 , icid=0 , wcid = 0 ;
	int32_t np=0 , nb=0 , nl=0 ;

//	float32_t *uv2_bev[EOL_MAX_NUM_LAYERS_][4]  ;
	float32_t *puv2_src  ;
	
	// camera_center is needed only when camera center need to be optimized
	float64_t pose_t[CAMERA_NUM][6], camera_center[CAMERA_NUM][2], camera_fov[CAMERA_NUM][2];

	EOL_str_cam_intrin *bev_cam[CAMERA_NUM] ,  *pin_cam[CAMERA_NUM];

	bev_cam[0] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_front ;
	bev_cam[1] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_rear ;
	bev_cam[2] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_left ;
	bev_cam[3] = (EOL_str_cam_intrin*)avm_cam_bev->_cam_right ;

	for(int i = 0; i< CAMERA_NUM; i++)
	{
		pose_t[i][0] = pSmc->camera_param[i].cam_ext.rx;
		pose_t[i][1] = pSmc->camera_param[i].cam_ext.ry;
		pose_t[i][2] = pSmc->camera_param[i].cam_ext.rz;
		pose_t[i][3] = pSmc->camera_param[i].cam_ext.tx;
		pose_t[i][4] = pSmc->camera_param[i].cam_ext.ty;
		pose_t[i][5] = pSmc->camera_param[i].cam_ext.tz;
		// when camera_center need to be optimized, do this initialization
		camera_center[i][0] = bev_cam[i]->cu;
		camera_center[i][1] = bev_cam[i]->cv;
		
		camera_fov[i][0] = pSmc->camera_param[i].cam_int.cam_int_fu_or_hfov_at_cu;
		camera_fov[i][1] = pSmc->camera_param[i].cam_int.cam_int_fv_or_vfov_at_cv;
	}

	EOL_g_param_adjuster._world_model = world_model ;
	EOL_g_param_adjuster.site_x = (float32_t)p_Eol_param->station_height;
	EOL_g_param_adjuster.site_y = (float32_t)p_Eol_param->station_width;
	EOL_g_param_adjuster.offset_x = pSmc->bev_calib_param.bev_eol_param.board_veh_flboard2fwheel
		+ world_model->_dist_axis - pSmc->bev_calib_param.bev_eol_param.station_height * 0.5;
	EOL_g_param_adjuster.offset_y = 0;

	EOL_Calib_Diag_Status_T ret = EOL_absolute_estimator_driver(
		pose_t ,
		camera_center,
		camera_fov,
		bev_cam , 
		p_Eol_param,
		world_model ) ;
	CHECK_ERROR(ret);

	//该指针开始后占有的内存，即标定结果占用的内存4X9 RT float64_t OR 4X6 float64_t HOMO
	//calib_data_buff->size 可用内存  calib_data_buff->used_size
	
	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		calib_data_buff->used_size  = CAMERA_NUM*6*sizeof(float64_t);
		break;
	case OPTIMIZE_TYPE_CENTER:
	case OPTIMIZE_TYPE_FOV:
		calib_data_buff->used_size  = CAMERA_NUM*8*sizeof(float64_t);
		break;
	case OPTIMIZE_TYPE_BOTH:
		calib_data_buff->used_size  = CAMERA_NUM*10*sizeof(float64_t);
		break;
	default:
		break;
	}

	if ( calib_data_buff->size < calib_data_buff->used_size )
	{
		return CALIB_DATA_BUFFER_SIZE_ERROR;
	}
	calib_data_buff->size = calib_data_buff->size - calib_data_buff->used_size;

	for(int32_t camid = 0; camid < CAMERA_NUM; camid++)
	{
		memcpy((int8_t*)calib_data_buff->address + sizeof(float64_t) * 6 * camid, pose_t[camid] , sizeof(float64_t) * 6 );

		switch (CURR_OPTIMIZE_TYPE)
		{
		case OPTIMIZE_TYPE_NONE:
			break;
		case OPTIMIZE_TYPE_CENTER:
			memcpy((int8_t*)calib_data_buff->address + sizeof(float64_t) * (6 * CAMERA_NUM + camid * 2), camera_center[camid], sizeof(float64_t) * 2 );
			break;
		case OPTIMIZE_TYPE_FOV:
			memcpy((int8_t*)calib_data_buff->address + sizeof(float64_t) * (6 * CAMERA_NUM + camid * 2), camera_fov[camid], sizeof(float64_t) * 2 );
			break;
		case OPTIMIZE_TYPE_BOTH:
			memcpy((int8_t*)calib_data_buff->address + sizeof(float64_t) * (6 * CAMERA_NUM + camid * 2), camera_center[camid], sizeof(float64_t) * 2 );
			memcpy((int8_t*)calib_data_buff->address + sizeof(float64_t) * (8 * CAMERA_NUM + camid * 2), camera_fov[camid], sizeof(float64_t) * 2 );
			break;
		default:
			break;
		}
	}

	//char space_char = ' ';
	//FILE* fp_rt = fopen("E:/temp/rt.txt", "wt");
	//for(int i = 0; i < 4; i++)
	//{
	//	for(int j = 0; j < 6; j++)
	//	{
	//		fprintf(fp_rt, "%f ", pose_t[i][j]);
	//		fprintf(fp_rt, " ");
	//	}
	//	fprintf(fp_rt, "\n");
	//}
	//fclose(fp_rt);

	// transformation success judgement
	//if (abs(pose_t[0][3] - pSmc->camera_param[0].cam_ext.tx) > TRANS_THRESHOLD ||
	//	abs(pose_t[0][4] - pSmc->camera_param[0].cam_ext.ty) > TRANS_THRESHOLD ||
	//	abs(pose_t[0][5] - pSmc->camera_param[0].cam_ext.tz) > TRANS_THRESHOLD)
	//{
	//	ret = LM_FRONT_TRANS_OVERSIZE;
	//}
	//else if (abs(pose_t[1][3] - pSmc->camera_param[1].cam_ext.tx) > TRANS_THRESHOLD ||
	//	abs(pose_t[1][4] - pSmc->camera_param[1].cam_ext.ty) > TRANS_THRESHOLD ||
	//	abs(pose_t[1][5] - pSmc->camera_param[1].cam_ext.tz) > TRANS_THRESHOLD)
	//{
	//	ret = LM_REAR_TRANS_OVERSIZE;
	//}
	//else if (abs(pose_t[2][3] - pSmc->camera_param[2].cam_ext.tx) > TRANS_THRESHOLD ||
	//	abs(pose_t[2][4] - pSmc->camera_param[2].cam_ext.ty) > TRANS_THRESHOLD ||
	//	abs(pose_t[2][5] - pSmc->camera_param[2].cam_ext.tz) > TRANS_THRESHOLD)
	//{
	//	ret = LM_LEFT_TRANS_OVERSIZE;
	//}
	//else if (abs(pose_t[3][3] - pSmc->camera_param[3].cam_ext.tx) > TRANS_THRESHOLD ||
	//	abs(pose_t[3][4] - pSmc->camera_param[3].cam_ext.ty) > TRANS_THRESHOLD ||
	//	abs(pose_t[3][5] - pSmc->camera_param[3].cam_ext.tz) > TRANS_THRESHOLD)
	//{
	//	ret = LM_RIGHT_TRANS_OVERSIZE;
	//}


	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		break;
	case OPTIMIZE_TYPE_CENTER:
		if( abs(camera_center[0][0] - bev_cam[0]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[0][1] - bev_cam[0]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_FRONT_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[1][0] - bev_cam[1]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[1][1] - bev_cam[1]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_REAR_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[2][0] - bev_cam[2]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[2][1] - bev_cam[2]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_LEFT_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[3][0] - bev_cam[3]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[3][1] - bev_cam[3]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_RIGHT_CENTER_OVERSIZE;
		}
	case OPTIMIZE_TYPE_FOV:
		if( abs(camera_fov[0][0] - pSmc->camera_param[0].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[0][1] - pSmc->camera_param[0].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_FRONT_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[1][0] - pSmc->camera_param[1].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[1][1] - pSmc->camera_param[1].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_REAR_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[2][0] - pSmc->camera_param[2].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[2][1] - pSmc->camera_param[2].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_LEFT_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[3][0] - pSmc->camera_param[3].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[3][1] - pSmc->camera_param[3].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_RIGHT_FOV_OVERSIZE;
		}
		break;
	case OPTIMIZE_TYPE_BOTH:
		if( abs(camera_center[0][0] - bev_cam[0]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[0][1] - bev_cam[0]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_FRONT_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[1][0] - bev_cam[1]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[1][1] - bev_cam[1]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_REAR_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[2][0] - bev_cam[2]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[2][1] - bev_cam[2]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_LEFT_CENTER_OVERSIZE;
		}
		else if(abs(camera_center[3][0] - bev_cam[3]->cu) > CENTER_THRESHOLD ||
			abs(camera_center[3][1] - bev_cam[3]->cv) > CENTER_THRESHOLD)
		{
			ret = LM_RIGHT_CENTER_OVERSIZE;
		}

		if( abs(camera_fov[0][0] - pSmc->camera_param[0].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[0][1] - pSmc->camera_param[0].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_FRONT_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[1][0] - pSmc->camera_param[1].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[1][1] - pSmc->camera_param[1].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_REAR_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[2][0] - pSmc->camera_param[2].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[2][1] - pSmc->camera_param[2].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_LEFT_FOV_OVERSIZE;
		}
		else if(abs(camera_fov[3][0] - pSmc->camera_param[3].cam_int.cam_int_fu_or_hfov_at_cu) > FOV_THRESHOLD ||
			abs(camera_fov[3][1] - pSmc->camera_param[3].cam_int.cam_int_fv_or_vfov_at_cv) > FOV_THRESHOLD)
		{
			ret = LM_RIGHT_FOV_OVERSIZE;
		}
		break;
	default:
		break;
	}


//
	if(EOL_g_param_adjuster.rmse[0] > RMSE_THRESHOLD)
		ret =  LM_FRONT_ERROR_OVERSIZE;
	else if(EOL_g_param_adjuster.rmse[1] > RMSE_THRESHOLD)
		ret =  LM_REAR_ERROR_OVERSIZE;
	else if(EOL_g_param_adjuster.rmse[2] > RMSE_THRESHOLD)
		ret =  LM_LEFT_ERROR_OVERSIZE;
	else if(EOL_g_param_adjuster.rmse[3] > RMSE_THRESHOLD)
		ret =  LM_RIGHT_ERROR_OVERSIZE;

	/*clean up
	*/
	for(camid=0 ; camid<CAMERA_NUM ; camid++ ) 
	{
		bev_cam[camid] = 0 ;
		pin_cam[camid] = 0 ;
	}

	puv2_src = 0 ;

	return ret ;
}
