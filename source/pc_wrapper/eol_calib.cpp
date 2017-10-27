#include "eol_calib.h"
#include "../EOL_calib_cb/eol_data_type.h"
#include "../EOL_calib_cb/eol_debug.h"
#include "../../include/eol_calib_cb_interface.h"

IplImage *EOL_bev_data_c_raw[CAMERA_NUM] = {NULL,NULL,NULL,NULL};
IplImage *EOL_bev_data_c[CAMERA_NUM] = {NULL,NULL,NULL,NULL};
IplImage *EOL_bev_data_y[CAMERA_NUM] = {NULL,NULL,NULL,NULL};

#define CONVERT_VIDEO

#ifdef CONVERT_VIDEO
uchar* yuv420_img = NULL;
//save file	
FILE* p_file[CAMERA_NUM] = { NULL, NULL, NULL, NULL};
#endif

extern char FILE_SAVE_PATH[20];
extern int total_count;
extern int start_frame_id;
extern FILE*  fp_log;
extern int frame_count;
#define SAVE_LOG

bool calibrated = false;
int32_t EOL_calib::process()
{
	// Do Calib EOL
	int32_t ret;
	IplImage *pData1 = NULL;
	IplImage *pData = NULL;
	// if(calibrated)return EOL_SUCCESS;


	while (1)
	{
		FW_LockData_Read(obj.data_def_video, (void **)&pData);
		if (!pData)
		{
			continue;
		}

		
		if (frame_count < start_frame_id)
		{
			frame_count++;
			FW_UnlockData_R(obj.data_def_video);
			return EOL_SUCCESS;
		}


		for (int i = 0; i < CAMERA_NUM; i++)
		{

			if (EOL_bev_data_c_raw[i] == NULL)
			{
				EOL_bev_data_c_raw[i] = cvCreateImage(CvSize(pData->width, pData->height / 4), pData->depth, pData->nChannels);
			}

			memcpy(EOL_bev_data_c_raw[i]->imageData, pData->imageData + i * EOL_bev_data_c_raw[i]->imageSize, EOL_bev_data_c_raw[i]->imageSize);

			if (EOL_bev_data_y[i] == NULL)
			{
				EOL_bev_data_y[i] = cvCreateImage(CvSize(EOL_bev_data_c_raw[0]->width, EOL_bev_data_c_raw[0]->height), 8, 1);
			}
			bgr_to_gray(EOL_bev_data_c_raw[i], EOL_bev_data_y[i]);

#ifdef CONVERT_VIDEO
			EOL_save_images(EOL_bev_data_c_raw[i], p_file[i], yuv420_img);
#endif
		}

#ifdef CONVERT_VIDEO
		if (frame_count > 1000)
		{
			for (int i = 0; i < CAMERA_NUM; i++)
			{
				fclose(p_file[i]);
			}
		}
		frame_count++;
		FW_UnlockData_R(obj.data_def_video);
		ret = EOL_SUCCESS;
		break;
#endif
		obj.calib_data._flag_rt = FALSE;
		
		str_avm_pose_t cam_pose;
		eol_result corner_result;


		// EOL_init should be called, Yanshuo
		EOL_init(&(obj.pEolHandle), &obj.avm_smc);
		ret = EOL_process((void**)EOL_bev_data_y, &obj.avm_smc, &obj.pEolHandle);
		EOL_deinit(&obj.pEolHandle);
#ifdef SAVE_LOG
		if (EOL_SUCCESS != ret)
		{
			fprintf(fp_log, "Frame %d calibration error.  \n", frame_count);
		}		

		if (frame_count == start_frame_id + total_count)
		{
			fclose(fp_log);
		}
#endif

		if(EOL_SUCCESS == ret)
		{
			float64_t cam_center[8], cam_fov[8];
			EOL_get_result(&cam_pose, cam_center, cam_fov, &corner_result);

			cam_pose._flag_rt = 1;
			FW_WriteData(FW_GetDataHandle("CALIB_DATA"),&cam_pose);

			float64_t *p_center = NULL, *p_fov = NULL;
			DATA_HANDLE t1, t2;

			switch (CURR_OPTIMIZE_TYPE)
			{
			case OPTIMIZE_TYPE_NONE:
				break;
			case OPTIMIZE_TYPE_CENTER:
				t1 = FW_LockData_Write(FW_GetDataHandle("CALIB_EOL_CAM_CENTER"), (void**)&p_center);
				memcpy(p_center, cam_center, 8 * sizeof(float64_t));
				FW_UnlockData(t1);
				break;
			case OPTIMIZE_TYPE_FOV:
				t1 = FW_LockData_Write(FW_GetDataHandle("CALIB_EOL_CAM_FOV"), (void**)&p_fov);
				memcpy(p_fov, cam_fov, 8 * sizeof(float64_t));
				FW_UnlockData(t1);
				break;
				break;
			case OPTIMIZE_TYPE_BOTH:
				t1 = FW_LockData_Write(FW_GetDataHandle("CALIB_EOL_CAM_CENTER"), (void**)&p_center);
				memcpy(p_center, cam_center, 8 * sizeof(float64_t));
				FW_UnlockData(t1);

				t2 = FW_LockData_Write(FW_GetDataHandle("CALIB_EOL_CAM_FOV"), (void**)&p_fov);
				memcpy(p_fov, cam_fov, 8 * sizeof(float64_t));
				FW_UnlockData(t2);
				break;
			default:
				break;
			}

			if ( obj.calib_data._flag_rt == 1 )
			{
				// Update
				memcpy(&obj.calib_data,&cam_pose,sizeof(str_avm_pose_t));

				std::string path = FW_GetDataPath();
				path += "Calib\\pose.txt";

				save_rt_pose( (const char*)(path.c_str()),
					obj.calib_data._pose_front, 
					obj.calib_data._pose_right, 
					obj.calib_data._pose_rear, 
					obj.calib_data._pose_left );
			
			}
			calibrated = true;
			FW_EnableData(obj.data_def_calib);
		}
		//FW_EnableData(obj.data_def_video);
		FW_UnlockData_R(obj.data_def_video);
		break;
	
	}
	//Sleep(6000);
	return ret;
}

EOL_calib::EOL_calib(void)
{
	return;
}

int EOL_calib::Init()
{
	FW_ReadData(FW_GetDataHandle("MC_DATA"), &obj.avm_smc); // New Data

	display_src_roi = cvRect(avm_smc.bev_stitch_param.bev_stitch_img_width, 0,
		avm_smc.camera_param[0].cam_int.cam_int_w - avm_smc.bev_stitch_param.bev_stitch_img_width,
		avm_smc.camera_param[0].cam_int.cam_int_h);
	
	/*
	const char *data_path = FW_GetDataPath();
	char gac_bmp_path[1024];
	sprintf(gac_bmp_path, "%s\\resource\\gac.bmp", data_path);
	gac_img = cvLoadImage(gac_bmp_path);
	*/

#ifdef SAVE_LOG
	// open log file
	char log_name[40];
	sprintf(log_name, "%s/log.txt", FILE_SAVE_PATH);
	fp_log = fopen(log_name, "w");
	if (!fp_log)
	{
		printf("Open file error!\n");
		return OPEN_FILE_ERROR;
	}
#endif

#ifdef CONVERT_VIDEO
	int pitch = 1280 * 720;
	yuv420_img = (uchar*)malloc(sizeof(uchar) * pitch * 3 >> 1);
	if (NULL == yuv420_img)
	{
		return MEM_MALLOC_FAIL;
	}
	memset(yuv420_img, 0, sizeof(uchar) * pitch * 3 >> 1);

	char str[200];
	for (int i = 0; i < CAMERA_NUM; i++)
	{
		switch (i)
		{
		case 0:
			sprintf(str, "%s/Front.I420", FILE_SAVE_PATH);			
			break;
		case 1:
			sprintf(str, "%s/Rear.I420", FILE_SAVE_PATH);
			break;
		case 2:
			sprintf(str, "%s/Left.I420", FILE_SAVE_PATH);
			break;
		case 3:
			sprintf(str, "%s/Right.I420", FILE_SAVE_PATH);
			break;
		}
		p_file[i] = fopen(str, "wb");
		if (NULL == p_file[i])
		{
			return OPEN_FILE_ERROR;
		}
	}
#endif

	calibrated = true;
	return M_R_SUCESS;
}

int EOL_calib::LoadSetting()
{
	int ret = M_R_FAILED;
	
	while(1)
	{
		ret = M_R_SUCESS;
		break;	
	}
	return ret; 
}

int EOL_calib::RegistData()
{
	data_def_eol = FW_AddData("CALIB_EOL",1,sizeof(int));
	data_def_eol_point = FW_AddData("CALIB_EOL_POINT",1,sizeof(eol_result));
	//data_def_cam_center = FW_AddData("CALIB_EOL_CAM_CENTER", 1, sizeof(float64_t) * 8);
	//data_def_cam_fov = FW_AddData("CALIB_EOL_CAM_FOV", 1, sizeof(float64_t) * 8);

	switch (CURR_OPTIMIZE_TYPE)
	{
	case OPTIMIZE_TYPE_NONE:
		break;
	case OPTIMIZE_TYPE_CENTER:
		data_def_cam_center = FW_AddData("CALIB_EOL_CAM_CENTER", 1, sizeof(float64_t) * 8);
		break;
	case OPTIMIZE_TYPE_FOV:
		data_def_cam_fov = FW_AddData("CALIB_EOL_CAM_FOV", 1, sizeof(float64_t) * 8);
		break;
	case OPTIMIZE_TYPE_BOTH:
		data_def_cam_center = FW_AddData("CALIB_EOL_CAM_CENTER", 1, sizeof(float64_t) * 8);
		data_def_cam_fov = FW_AddData("CALIB_EOL_CAM_FOV", 1, sizeof(float64_t) * 8);
		break;
	default:
		break;
	}

	return M_R_SUCESS;
}

int EOL_calib::ConnectData()
{
	//FW_PlugInData("Process_Task", process, 1, data_def_eol,data_def_eol_point);

	data_def_video= FW_GetDataHandle("Video_Source");	
	data_def_display= FW_GetDataHandle("Display_Source");
	data_def_vehicle = FW_GetDataHandle( "Vehicle_Data_Source"); // New Data
	data_def_new_frame = FW_GetDataHandle( "NEW_FRAME"); // Get Data access

	FW_PlugInData("Process_Task", process, 1, data_def_video);
	//FW_PlugInData( "Sys_Msg_Proc", MsgProc, 1,FW_GetDataHandle( "Sys_Msg_Proc") );

	return M_R_SUCESS;
}

int EOL_calib::StartProcess()
{
 	return M_R_SUCESS;
}

int EOL_calib::StopProcess()
{
    //关闭视频文件  
	stop = true; 
	return M_R_SUCESS;
}

int EOL_calib::Release()
{
	for ( int i = 0; i < 4; i ++ )
	{
		if( EOL_bev_data_c_raw[i] ) 
			cvReleaseImage(&EOL_bev_data_c_raw[i]);
		if( EOL_bev_data_c[i] ) 
			cvReleaseImage(&EOL_bev_data_c[i]);
		if( EOL_bev_data_y[i]  ) 
			cvReleaseImage(&EOL_bev_data_y[i]);
	}
	return M_R_SUCESS;
}

void EOL_calib::MsgProc()
{
	void *pData = NULL;
	STR_MSG_BUFFER  strData;
	memset(&strData,0,sizeof(STR_MSG_BUFFER));
	FW_ReceiveMsg(FW_GetDataHandle( "Sys_Msg_Proc"),&strData,sizeof(STR_MSG_BUFFER));

	switch (strData.uwMsgId)
	{
	case GLOBL_MSG_STOP:
		//obj.StopProcess();
		break;
	case GLOBL_MSG_PLAY:
		//obj.StartProcess();
		break;
	case GLOBL_MSG_OUTPUTXML:
		//OutPutXML(&strData);
		break;
	case  GLOBL_MSG_INPUTXML:
		//InputXML(&strData);
		break;
	case GLOBL_MSG_DISPLYWNDCHANGE:
		break;
	case  GLOBL_MSG_SHOW_MODE_SWITCH:
		break;
	case  GLOBL_MSG_SHOW_LOADING:
		break;
	case  GLOBL_MSG_APP_EXIT:
		break;
	case Qt_Key_1:
		calibrated = false;
		obj.process();
		break;
	default:
		break;
	}
}

int32_t EOL_calib::UpdateFG()
{

	//if ( obj.stop == true)	return M_R_SUCESS; 


	//IplImage *pData2 = NULL;
	//FW_LockData_Write( obj.data_def_display, (void **)&pData2);


	//CvRect rois[4];
	//rois[0] = cvRect(display_src_roi.x, display_src_roi.y, display_src_roi.width / 2, display_src_roi.height / 2);
	//rois[1] = cvRect(rois[0].x + rois[0].width, rois[0].y, rois[0].width, rois[0].height);
	//rois[2] = cvRect(rois[0].x, rois[0].y + rois[0].height, rois[0].width, rois[0].height);
	//rois[3] = cvRect(rois[0].x + rois[0].width, rois[0].y + rois[0].height, rois[0].width, rois[0].height);


	//// CvRect roi_text = cvRect(0, 480, display_src_roi.x, display_src_roi.height-480);
	//if ( pData2)
	//{
	//	for(int i = 0; i < 4; i++)
	//	{
	//		if (EOL_bev_data_c_raw[i] != NULL)
	//		{
	//			image_copy_roi(EOL_bev_data_c_raw[i], pData2, rois[i]);
	//		}				
	//	}

	//	//if(gac_img != NULL)
	//	//{
	//	//	cvSetImageROI(pData2, roi_text);
	//	//	cvResize(obj.gac_img, pData2);
	//	//	cvResetImageROI(pData2);
	//	//}


	//	//if(calibrated)
	//	//{
	//	//	cv::putText(cv::cvarrToMat(pData2), "Calibrated", cv::Point(20, 50), 2, 1, CV_RGB(255, 0, 0));
	//	//}
	//	//else
	//	//{
	//	//	cv::putText(cv::cvarrToMat(pData2), "Calibrating", cv::Point(10, 50), 2, 1, CV_RGB(255, 0, 0));
	//	//}
	//}


	//FW_UnlockData(obj.data_def_display);
	return FW_RESULT_SUCESS;
}

int32_t EOL_calib::UpdateBG()
{
	return FW_RESULT_SUCESS;
}