#ifndef __POSE_ESTIMATOR_H__
#define __POSE_ESTIMATOR_H__
#include "utility\intrinsic_modeller\camera_model64.h"
#include "utility\common\reuse.h"
#include "eol_data_type.h"
#include "eol_error.h"
#include "utility/common//smc.h"
#include "utility\math\math_common.h"
#include "eol_config.h"

#define EOL_CAMERA_CALIB_VIEW_POINT_CNT_MAX (10)

#undef EOL_ID_CAM_2_BD
#define EOL_ID_CAM_2_BD(camid_ , kb_) ( older_box[camid_][kb_] )

#undef EOL_ID_IMG_2_WLD
#define EOL_ID_IMG_2_WLD(camid_ , bdid_ , icoordid_ ) ( cam_board_icd_wcd[camid_][bdid_][icoordid_] ) 

#ifndef CAM_MODEL_MACRO
#define CAM_MODEL_MACRO
	typedef Cam_Model_Intrinsic64 EOL_str_cam_intrin ;
	typedef Cam_Model_Extrinsic64 EOL_str_cam_extrin ;
	typedef Cam_Model64 EOL_str_cam ;
#endif

typedef struct
{
	int32_t size ;
	int32_t used_size;
	void *address;
	//float64_t *address;
}EOL_Buffer_Mng_T;

#ifndef _STR_AVM_INTRIN_
#define _STR_AVM_INTRIN_
	typedef struct {
		void *_cam_front ;
		void *_cam_rear ;
		void *_cam_left ; 
		void *_cam_right ;

	} EOL_str_avm_intrin ;
#endif

typedef struct
{
	Point2f corner_point[42];
	int point_flag[42];
	int count;
}laneMarkInfo;

#ifndef _STR_AVM_POSE_
#define _STR_AVM_POSE_
	typedef struct{
		float64_t *_pose_front ; 
		float64_t *_pose_rear ;
		float64_t *_pose_left ;
		float64_t *_pose_right ;

	} EOL_str_avm_pose ; 
#endif

#ifndef _STR_AVM_CAM_
#define _STR_AVM_CAM_
	typedef struct {
		EOL_str_cam *_cam_model_front ;
		EOL_str_cam *_cam_model_rear ; 
		EOL_str_cam *_cam_model_left ;
		EOL_str_cam *_cam_model_right ;

	} EOL_str_avm_cam ;
#endif

typedef struct Eol_Pose_Estimator_Tag
{
	float64_t *params ;

	float64_t *fvec;
	float64_t *diag;
	float64_t *fjac;
	float64_t *qtf;
	float64_t *wa1;
	float64_t *wa2;
	float64_t *wa3;
	float64_t *wf;
	int32_t *ipvt;

}Eol_Pose_Estimator_T;

typedef struct
{	
	EOL_str_cam_intrin *_bev_cam[CAMERA_NUM] ; 
	const Chessboard_Pattern *pCB[CAMERA_NUM][CB_MAX_NUM];
	const EOL_str_world_model *_world_model ;
	float offset_x;
	float offset_y;
	float site_x;
	float site_y;
	float rmse[CAMERA_NUM];
	int   valid_pts_num[CAMERA_NUM];
} EOL_avm_param_adjuster ;

int32_t EOL_remove_outliers(Smc_Cal_T* const           pSmc,
							EOL_str_avm_intrin* const  avm_cam_bev ,
							P_EOL_Param const          p_Eol_param,
							EOL_str_world_model* const world_model);

int32_t EOL_calculate_init_rt(Smc_Cal_T* const				pSmc,
						      EOL_str_avm_intrin*			avm_cam_bev ,
						      P_EOL_Param				    p_Eol_param,
						      EOL_str_world_model* const    world_model);

EOL_Calib_Diag_Status_T EOL_pose_estimator(EOL_Buffer_Mng_T*    calib_data_buff,
						                   Smc_Cal_T*           pSmc,
						                   EOL_str_avm_intrin*  avm_cam_bev,
 						                   const P_EOL_Param    p_Eol_param,
						                   EOL_str_world_model* world_model );



#endif
