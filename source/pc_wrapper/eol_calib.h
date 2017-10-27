//#pragma once
#ifndef _EOL_CALIB_
#define _EOL_CALIB_

#include "stdafx.h"
#include <iostream> 

#include "FrameworkBase\FrameworkBase.h"
#include "utility\common\commondef.h"
#include "utility\common\bev_data_type.h"
#include "utility\gpu3d\BevLUT_OPT.h"


#if OPENCV
#include "opencv\cv.hpp"  
#include "opencv2\core\core.hpp"  
#include "opencv2\highgui\highgui.hpp"  
#include "opencv2\imgproc\imgproc.hpp"  
#include "opencv\highgui.h"

#else
#include "eol_opencv_adapter.h"
#include "eol_image_process.h"
#endif


#define CAMERA_CALIB_VIEW_POINT_CNT_MAX  32
/*Module Interface*/
class EOL_calib : public IBaseModule
{
public:
	EOL_calib(void);
	int Init();
	int LoadSetting();
	int RegistData();
	int ConnectData();
    int StartProcess();
	int StopProcess();
    int Release();
	int32_t UpdateBG();
	int32_t UpdateFG();

	static void MsgProc();
public:
	DATA_HANDLE data_def_video;
	DATA_HANDLE data_def_display;
	DATA_HANDLE data_def_new_frame;
	DATA_HANDLE data_def_vehicle;
	DATA_HANDLE data_def_calib;
	DATA_HANDLE data_def_eol;
	DATA_HANDLE data_def_cam_center; // add by Yanshuo
	DATA_HANDLE data_def_cam_fov; // add by Yanshuo
	DATA_HANDLE data_def_eol_point;
	COMMON_VEHICLE_DATA v_data;
	str_avm_pose_t calib_data;
	bool stop; 
	static int32_t process();
	CvRect display_src_roi; 
	IplImage *gac_img;
	void* pEolHandle;
	Smc_Cal_T avm_smc;
private:
	
};

EOL_calib obj;


M_API M_HANDLE WINAPI GetModule()
{
	return &obj;
}

//
//typedef struct
//{
//	uint32_t		 nPoint_Cnt;
//	Calib_Point_T pPoint[CAMERA_CALIB_VIEW_POINT_CNT_MAX];
//}
//EOL_Calib_Point_List_T;

#endif


