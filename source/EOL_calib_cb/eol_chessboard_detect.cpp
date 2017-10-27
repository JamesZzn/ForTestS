#include "eol_chessboard_detect.h"

extern chessBoard_corner_detector corner_detector[CAMERA_NUM];
extern IplImage* thresh_img, *norm_img, *temp_img, *temp_img_copy;

/*
	Function Name:      EOL_config_detecter
	Function Function : The entrance of chessboard corner detect function
	Input             : 
	    sProcessImg_in :The four camera's image with chessboard patterns
	    p_Eol_param :   P_EOL_Param structure
	Return            : Error code described in eol_landmark_detector.h
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/10        Create
*/
static void EOL_config_detecter(chessBoard_corner_detector* p_corner_detecter, 
						 CbPattern_group* p_cb_pattern_group,
						 P_EOL_Param p_Eol_param, 
						 int32_t* p_pattern_index,
						 int32_t camid,
						 int32_t pattern_id)
{
	int32_t valid_pattern_num = p_cb_pattern_group->valid_num;

	corner_detector[camid].img_in = thresh_img;
	/* configuring the current chessboard before corner detect*/			
	corner_detector[camid].corner_config = p_cb_pattern_group->pattern[p_pattern_index[pattern_id]].chessboard_config;
	corner_detector[camid].corner_config.camid = camid;
	corner_detector[camid].valid_num = valid_pattern_num;
	corner_detector[camid].station_type = p_Eol_param->station_type;
	if (corner_detector[camid].station_type == STATION_TYPE_JAC)
	{
		if (camid == 1 && pattern_id == 0)
		{
			corner_detector[camid].corner_config.correct_number_of_corners = 7;
			corner_detector[camid].corner_config.correct_number_of_quads = 8;
		}
		if (camid == 0 && pattern_id == 0)
		{
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 0;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 0 && pattern_id == 1)
		{
			/*corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 0;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;*/
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 1;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 0 && pattern_id == 2)
		{
			/*corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 0;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;*/
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 1;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 1 && pattern_id == 0)
		{
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 1;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 1 && pattern_id == 1)
		{
			/*corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 0;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;*/
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 1 && pattern_id == 2)
		{
			/*corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 0;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;*/
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 1;
		}
		if (camid == 2)
		{
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 3;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 8;
		}
		if (camid == 3)
		{
			corner_detector[camid].pattern_corner[0].selected_corner_connected_type = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_x = 2;
			corner_detector[camid].pattern_corner[0].selected_corner_y = 6;
		}
	}
	/* get the reference point and rect mask roi config if it is available*/
	//corner_detector[camid].corner_config.rect_mask_num = p_cb_pattern_group->reference_point_num;
	//corner_detector[camid].corner_config.reference_pointNum = p_cb_pattern_group->reference_point_num;
	if (valid_pattern_num > 1)
	{
		if (pattern_id == valid_pattern_num - 2)
			corner_detector[camid].pattern_side = SIDE_LEFT;
		else if (pattern_id == valid_pattern_num - 1)
			corner_detector[camid].pattern_side = SIDE_RIGHT;
		else
			corner_detector[camid].pattern_side = SIDE_BOTH;
	}
	else
	{
		corner_detector[camid].pattern_side = SIDE_BOTH;
	}
	
	// get reference point and mask
	for (int i = 0; i < CB_MAX_NUM; i++)
	{
		corner_detector[camid].reference_point[i] = p_cb_pattern_group->refer_point[i];	
		//corner_detector[camid].rect_mask_roi[i] = p_cb_pattern_group->roi_mask[i];
		for (int j = 0; j < 4; j++)
		{
			corner_detector[camid].boundary_point[i][j] = p_cb_pattern_group->boundary_point[i][j];
		}
	}
}


/*
	Function Name:      EOL_copy_corner_and_flag
	Function Function : Copy corner point's and flags
	Input             : 
	     p_corner_dst : the address to accept corner point array
	     p_corner_src : the source of corner point array
		   p_flag_dst : the address to accept flag array
		   p_flag_src : the source of flag array
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/5/15        Create
*/
static void EOL_copy_corner_and_flag(Point2f p_corner_dst[CB_MAX_ROW][CB_MAX_COL],
									 Point2f p_corner_src[CB_MAX_ROW][CB_MAX_COL],
									 int32_t p_flag_dst[CB_MAX_ROW][CB_MAX_COL],
									 int32_t p_flag_src[CB_MAX_ROW][CB_MAX_COL])
{
	memcpy(p_corner_dst, p_corner_src,sizeof(Point2f) * CB_MAX_ROW * CB_MAX_COL);
	memcpy(p_flag_dst, p_flag_src,sizeof(int32_t) * CB_MAX_ROW * CB_MAX_COL);
}

/*
	Function Name:      EOL_copy_corner_to_structure
	Function Function : Copy the detected corner structure to correct p_Eol_param
	Input             : 
	  corner_detector : The structure returned by corner detect structure
	          pattern : the pointer to pattern array of the current camera id
		  p_Eol_param : p_EOL_param
		pattern_index : the pattern's index in descending order
		  refer_point : the center of current pattern, it's used to decides pattern's 
		                position in p_Eol_param structure
	   boundary_point : the four boundary of pattern, it is used to mask pattern to avoid
	                    re-detected of the pattern
		   pattern_id : the current pattern's id in the current camera
   pattern_corner_num : the valid corner's num of the current
    ret_corner_detect : the return value of corner detect function
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/5/15        Create
*/
static void EOL_copy_corner_to_structure(chessBoard_corner_detector corner_detector,
										 Chessboard_Pattern* pattern,
										 P_EOL_Param p_Eol_param,
										 int32_t pattern_index[CB_MAX_NUM],
										 Point2f refer_point[CB_MAX_NUM],
										 Point2f boundary_point[4][4],
										 int32_t cam_id,
										 int32_t& pattern_id,
										 int32_t pattern_corner_num,
										 int32_t ret_corner_detect)
{
	if(0 < ret_corner_detect)
	{
		/*1*1 pattern case*/
		if(1 == pattern_corner_num)
		{
			// copy detected corners
			EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id]].corner_point,
				corner_detector.pattern_Single_corner[0].corner_point,
				pattern[pattern_index[pattern_id]].flag,
				corner_detector.pattern_Single_corner[0].flag);

			EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id + 1]].corner_point,
				corner_detector.pattern_Single_corner[1].corner_point,
				pattern[pattern_index[pattern_id + 1]].flag,
				corner_detector.pattern_Single_corner[1].flag);

			// check effectiveness
			pattern_checking(&pattern[pattern_index[pattern_id]], p_Eol_param->station_type, cam_id, pattern_id);
			pattern_checking(&pattern[pattern_index[pattern_id + 1]], p_Eol_param->station_type, cam_id, pattern_id);

			pattern_id += 1;
		}

		/*n*n pattern case*/
		else
		{
			Chessboard_Pattern pattern_temp; 

			memcpy(&pattern_temp, &pattern[pattern_index[pattern_id]], sizeof(Chessboard_Pattern));
			EOL_copy_corner_and_flag(pattern_temp.corner_point,
				corner_detector.pattern_corner[0].corner_point,
				pattern_temp.flag,
				corner_detector.pattern_corner[0].flag);
			pattern_checking(&pattern_temp, p_Eol_param->station_type, cam_id, pattern_id);

			// calculate the current reference point
			Point2f reference_point;
			refenrece_point(&corner_detector.pattern_corner[0],&reference_point);

			/*need exchange*/
			if( pattern_id > 0 &&
				corner_detector.corner_config.board_size.height * corner_detector.corner_config.board_size.width ==
				( pattern[pattern_index[pattern_id - 1]].chessboard_config.board_size.height 
				* pattern[pattern_index[pattern_id - 1]].chessboard_config.board_size.width) &&
				reference_point.x < refer_point[pattern_index[pattern_id - 1]].x)
			{
				// exchange this pattern with the previous pattern
				EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id]].corner_point,
					pattern[pattern_index[pattern_id - 1]].corner_point,
					pattern[pattern_index[pattern_id]].flag,
					pattern[pattern_index[pattern_id - 1]].flag);

				pattern[pattern_index[pattern_id]].valid_point_num = 
					pattern[pattern_index[pattern_id - 1]].valid_point_num;


				EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id - 1]].corner_point,
					pattern_temp.corner_point,
					pattern[pattern_index[pattern_id - 1]].flag,
					pattern_temp.flag);

				pattern[pattern_index[pattern_id - 1]].valid_point_num = pattern_temp.valid_point_num;
			}
			else
			{
				EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id]].corner_point,
					pattern_temp.corner_point,
					pattern[pattern_index[pattern_id]].flag,
					pattern_temp.flag);

				pattern[pattern_index[pattern_id]].valid_point_num = pattern_temp.valid_point_num;
			}

			// if the detected chessboard corner num > 4
			if (4 <= corner_detector.pattern_corner[0].rows * corner_detector.pattern_corner[0].cols)
			{
				refenrece_point(&corner_detector.pattern_corner[0],&refer_point[pattern_index[pattern_id]]);
				if (STATION_TYPE_JAC == p_Eol_param->station_type)
				{
					calc_boundary_point_jac(&corner_detector.pattern_corner[0], boundary_point[pattern_index[pattern_id]], cam_id, pattern_id);
				}
				else
				{
					calc_boundary_point(&corner_detector.pattern_corner[0], boundary_point[pattern_index[pattern_id]]);
				}				
			}
		}
	}
	else
	{
		if( pattern_id > 0 &&
			corner_detector.corner_config.board_size.height * corner_detector.corner_config.board_size.width ==
			( pattern[pattern_index[pattern_id - 1]].chessboard_config.board_size.height 
			*pattern[pattern_index[pattern_id - 1]].chessboard_config.board_size.width) &&
			refer_point[pattern_index[pattern_id - 1]].x > (thresh_img->width >> 1) )
		{
			EOL_copy_corner_and_flag(pattern[pattern_index[pattern_id]].corner_point,
				pattern[pattern_index[pattern_id - 1]].corner_point,
				pattern[pattern_index[pattern_id]].flag,
				pattern[pattern_index[pattern_id - 1]].flag);

			pattern[pattern_index[pattern_id]].valid_point_num = pattern[pattern_index[pattern_id - 1]].valid_point_num;

			for(int m = 0; m < CB_MAX_ROW; m++)
			{
				for(int n = 0; n < CB_MAX_COL; n++)
				{
					pattern[pattern_index[pattern_id - 1]].corner_point[m][n].x = 0;
					pattern[pattern_index[pattern_id - 1]].corner_point[m][n].y = 0;
				}
			}
			memset(pattern[pattern_index[pattern_id - 1]].flag, 
				0, sizeof(int32_t) * CB_MAX_ROW * CB_MAX_COL);
			pattern[pattern_index[pattern_id - 1]].valid_point_num = 0;
		}
	}
	
}

/*
	Function Name:      EOL_Process
	Function Function : The entrance of chessboard corner detect function
	Input             : 
	    sProcessImg_in :The four camera's image with chessboard patterns
	    p_Eol_param :   P_EOL_Param structure
	Return            : Error code described in eol_landmark_detector.h
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/10        Create
*/
int32_t EOL_chessboard_detect(IplImage* sProcessImg_in[CAMERA_NUM], 
							  P_EOL_Param p_Eol_param)
{
	int32_t ret = 0;
	int32_t camid;
	int32_t block_size;
	/* For each camera, detect Chessboard corners */
	for (camid=0 ; camid < CAMERA_NUM; camid++ )
	{
		memset(&corner_detector[camid],0,sizeof(chessBoard_corner_detector));
		if( sProcessImg_in[camid]->nChannels != 1 )
		{
			cvCvtColor( sProcessImg_in[camid], norm_img, 0);//Rgb2Gray
		}
		else
		{
			cvCopy(sProcessImg_in[camid], norm_img);
		}

		block_size = cvRound(MIN(sProcessImg_in[camid]->width,sProcessImg_in[camid]->height)*0.2)|1;
		cvAdaptiveThreshold( norm_img, thresh_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, block_size, 0 );
		
		char str1[250];
		sprintf(str1, "d://TestThreshold_%d.raw",camid);
		SaveRawFile(str1, thresh_img->imageData, sProcessImg_in[camid]->width*sProcessImg_in[camid]->height);
		
		/* the corner num of each pattern */
		int32_t pattern_corner_num[CB_MAX_NUM] = { EOL_MAX_CORNER_NUM, 
			                                       EOL_MAX_CORNER_NUM, 
												   EOL_MAX_CORNER_NUM, 
												   EOL_MAX_CORNER_NUM };
		/* the index order of pattern_corner_num in descending order */
		int32_t pattern_index[CB_MAX_NUM];

		CbPattern_group* p_cb_pattern_group = &p_Eol_param->cb_pattern_group[camid];
		int32_t valid_pattern_num = p_cb_pattern_group->valid_num;

		/* Calculate each pattern's corner num in the current camera */
		for(int32_t patternId = 0; patternId < valid_pattern_num; patternId++)
		{
			CvSize board_size = p_cb_pattern_group->pattern[patternId].chessboard_config.board_size;
			pattern_corner_num[patternId] = board_size.width * board_size.height;
		}

		/* reorder the patterns in descending order of pattern_corner_num and return it's subscript index */
		if (STATION_TYPE_JAC == p_Eol_param->station_type)
		{
			get_order(pattern_corner_num, pattern_index, valid_pattern_num, true);
		}
		else
		{
			get_order(pattern_corner_num, pattern_index, valid_pattern_num, false);
		}	
		

		for(int32_t pattern_id = 0; pattern_id < valid_pattern_num; pattern_id++)
		{

			int32_t ret_corner_detect;

			EOL_config_detecter(corner_detector, p_cb_pattern_group, p_Eol_param, pattern_index, camid, pattern_id);

			corner_detector[camid].init();

			if (1 == pattern_corner_num[pattern_id])
			{
				if (STATION_TYPE_GAC == p_Eol_param->station_type)
				{

				}
				else
				{
					ret_corner_detect = corner_detector[camid].cvFindChessboardCorners2(thresh_img);
				}
				
			}
			else
			{
				ret_corner_detect = corner_detector[camid].cvFindChessboardCorners3(thresh_img);
			}

			EOL_copy_corner_to_structure(corner_detector[camid],
				p_cb_pattern_group->pattern,
				p_Eol_param,
				pattern_index,
				p_cb_pattern_group->refer_point,
				p_cb_pattern_group->boundary_point,
				camid,
				pattern_id,
				pattern_corner_num[pattern_id],
				ret_corner_detect);
			
			corner_detector[camid].deinit();
		} /* end for(int patternId = 0; patternId < valid_pattern_num; patternId++) */
	} /* end for(camid=0 ; camid<4 ; camid++ ) */
	return ret;
}
bool SaveRawFile(const char* sfilename, char *pBuffer, int iSize)
{
	FILE* p_file = fopen(sfilename, "wb");
	if (NULL == p_file)
	{
		return OPEN_FILE_ERROR;
	}
	fwrite(pBuffer, sizeof(uchar) * iSize, 1, p_file);
	fclose(p_file);
	return 0;
}