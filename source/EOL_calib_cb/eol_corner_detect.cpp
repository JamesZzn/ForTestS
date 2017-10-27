#include<algorithm>
#include "eol_corner_detect.h"
extern IplImage* thresh_img, *norm_img, *temp_img, *temp_img_copy;


void chessBoard_corner_detector::init()
{

	/*elem_size = corner_config.board_size.width*corner_config.board_size.height*sizeof(CvPoint2D32f);
	image_points_buf = (CvPoint2D32f*)cvAlloc( elem_size );*/
	corner_count = 0;
	corner_config.gloabal_min_dilate = 1;
	corner_config.reflection=0;
}

void chessBoard_corner_detector::deinit()
{

	/*cvFree(&image_points_buf);
	image_points_buf = NULL;*/
	memset(&pattern_corner,0,2*sizeof(Chessboard_Pattern));
	memset(&pattern_Single_corner,0,2*sizeof(Chessboard_Pattern));
	memset(reference_point,0,sizeof(Point2f)*2);
}
/*
	Function Name:      cvFindChessboardCorners3
	Function Function : detect corners corners of non 1x1 pattern
	Input             : 
	    img :           The input binary image
	Return            : error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    unknown          Create
					  WangLuyao      2017/4/17        Modify

*/
int chessBoard_corner_detector::cvFindChessboardCorners3( const IplImage* img )
{

	// Initialize variables
	//clock_t startTime0 = clock();

	int flags					=  1;	
	int max_count				=  0;
	int max_dilation_run_ID		= -1;
    const int min_dilations		=  corner_config.gloabal_min_dilate;
    const int max_dilations		=  6;
    int found					=  0;
    CvMemStorage* storage		=  0;
	
	CvCBQuad *quads				=  0;
	CvCBQuad **quad_group		=  0;
    CvCBCorner *corners			=  0;
	CvCBCorner **corner_group	=  0;
	CvCBQuad **output_quad_group = 0;
	
	int block_size = 0;

    int quad_count, group_idx, dilations;

    if( img->depth != IPL_DEPTH_8U || img->nChannels == 2 )
	{
		return -1;
	}
    if( corner_config.board_size.width < 2 || corner_config.board_size.height < 2 )
	{
		return -1;
	}
	if( corner_config.board_size.width > 127 || corner_config.board_size.height > 127 )
	{
		return -1;
	}



	int height = img->height;
    int width = img->width;
    storage = cvCreateMemStorage(0);
	cvCopy(img, temp_img);//temp_img:binary image

	for(int k=0;k<4;k++)
	{
		if (boundary_point[k][0].x == 0 && boundary_point[k][0].y == 0 
			&& boundary_point[k][1].x == 0 && boundary_point[k][1].y == 0
			&& boundary_point[k][2].x == 0 && boundary_point[k][2].y == 0
			&& boundary_point[k][3].x == 0 && boundary_point[k][3].y == 0)
		{
			continue;
		}

		for (int p=0;p<4;p++)
		{
			if (boundary_point[k][p].x < 0 || boundary_point[k][p].x >= temp_img->width 
				|| boundary_point[k][p].y < 0 ||boundary_point[k][p].y >= temp_img->height)
			{
				return -1;
			}
		}

		for(int i=MIN(boundary_point[k][0].y, boundary_point[k][1].y);i<MAX(boundary_point[k][2].y, boundary_point[k][3].y);i++)
		{
			for(int j=MIN(boundary_point[k][0].x, boundary_point[k][2].x);j<MAX(boundary_point[k][1].x, boundary_point[k][3].x);j++)
			{
				Point2f p(j,i);
				bool flag = pInQuadrangle(boundary_point[k][0], boundary_point[k][1], boundary_point[k][2], boundary_point[k][3], p);
				uchar* data = (uchar*)temp_img->imageData;
				if (flag == 1)
					data[i*temp_img->widthStep+j]=255;
			}
		}
	}
	char str1[250];
	sprintf(str1, "d://TestFirst.raw");
	SaveRawFile1(str1, temp_img->imageData, temp_img->width*temp_img->height);

	if(corner_config.reflection==1)
	{

		for(int i=0;i<temp_img->height;i++)
		{
			for(int j=0;j<temp_img->width;j++)
			{
				uchar* data = (uchar*)temp_img->imageData;
				uchar imgPixValue = data[i*temp_img->widthStep+j];
				imgPixValue = 255-imgPixValue;
				data[i*temp_img->widthStep+j] = imgPixValue;
			}
		}


		cvDilate(temp_img,temp_img,3,CV_SHAPE_CROSS,2);

		cvErode(temp_img,temp_img,3,CV_SHAPE_RECT,2);

		for(int i=0;i<temp_img->height;i++)
		{
			for(int j=0;j<temp_img->width;j++)
			{
				uchar* data = (uchar*)temp_img->imageData;
				uchar imgPixValue = data[i*temp_img->widthStep+j];
				imgPixValue = 255-imgPixValue;
				data[i*temp_img->widthStep+j] = imgPixValue;
			}
		}
	}

	if (temp_img->height > 480 || (corner_config.camid == 3 && station_type == STATION_TYPE_JAC))
	{
		cvErode( temp_img, temp_img, 3, CV_SHAPE_BAR_HORI_ALL, 1);
		cvDilate( temp_img, temp_img, 3, CV_SHAPE_BAR_HORI_ALL, 1);
	}
	cvCopy( temp_img, temp_img_copy);


    for( dilations = min_dilations; dilations <= max_dilations; dilations++ )
    {
		cvCopy( temp_img_copy, temp_img);

		if(temp_img->height <= 480) // standard definition
		{
			switch (station_type)
			{
			case STATION_TYPE_JAC:
				//if (dilations == 1)
				//	cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
				//if (dilations == 2)
				//	cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_LEFT, 1);
				//if (dilations == 3)
				//	cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_RIGHT, 1);
				//if (dilations == 4)
				//	cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_UP, 1);
				//if (dilations == 5)
				//	cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_DOWN, 1);
				//if (dilations == 6)
				//	cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_UP, 1);
				if (dilations == 1)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
				}
				if (dilations == 2)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_LEFT, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_RIGHT, 1);
				}
				if (dilations == 3)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_UP, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_DOWN, 1);
				}
				if (dilations == 4)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_HORI_ALL, 1);
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_VERT_ALL, 1);
				}
				if (dilations == 5)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_048, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_INCL_48, 1);
				}
				if (dilations == 6)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_246, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_INCL_46, 1);
				}
				break;
			case STATION_TYPE_OFILM:
			case STATION_TYPE_GAC:
				if (dilations == 1)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
				}
				if (dilations == 2)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 2);
				}
				if (dilations == 3)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_048, 1);
				}
				if (dilations == 4)
				{
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_INCL_246, 1);
				}
				if (dilations == 5)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_048, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_INCL_48, 1);
				}
				if (dilations == 6)
				{
					cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_246, 1);
					cvDilate(temp_img, temp_img, 2, CV_SHAPE_BAR_INCL_46, 1);
				}
				break;
			default:
				break;
			}
		}

		else // high definition
		{
			if (dilations == 1)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
			if (dilations == 2)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_HORI_ALL, 1);
			if (dilations == 3)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_VERT_ALL, 1);
			if (dilations == 4)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_048, 1);
			if (dilations == 5)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_246, 1);
			if (dilations == 6)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_RECT, 1);
		}
		sprintf(str1, "d://TestAfterDilate.raw");
		SaveRawFile1(str1, temp_img->imageData, temp_img->width*temp_img->height);

        quad_count = icvGenerateQuads( &quads, &corners, storage, temp_img, flags, dilations, true );

		sprintf(str1, "d://TestAfterGenerate.raw");
		SaveRawFile1(str1, temp_img->imageData, temp_img->width*temp_img->height);
#ifdef DEBUG_RESULT_SAVE
		char file_name[200];
		sprintf(file_name, "%s/detected_rects_%dx%d.GREY", DEBUG_FILE_PATH, temp_img->width, temp_img->height);
		FILE* fp_tmp1 = fopen(file_name, "wb");
		uchar* write_img = (uchar*)malloc(sizeof(uchar) * temp_img->height * temp_img->width);
		uchar* pData = (uchar*)temp_img->imageData;
		for(int i = 0; i < temp_img->height; i++)
		{
			for(int j = 0; j < temp_img->width; j++)
			{
				write_img[i * temp_img->width + j] = pData[i * temp_img->widthStep + j];
			}
		}
		for(int m = 0; m < quad_count; m++)
		{
			for(int n = 0; n < 4; n++)
			{
				for(int p = -1; p < 2; p++)
				{
					for(int q = -1; q < 2; q++)
					{
						int x, y;
						x = quads[m].corners[n]->pt.x + p;
						y = quads[m].corners[n]->pt.y + q;
						if(x > 0 && x < temp_img->width &&
							y > 0 && y < temp_img->height)
						{
							write_img[y * temp_img->width + x] = 128;
						}
					}
				}
			}
		}
		fwrite(write_img, sizeof(uchar) * temp_img->height * temp_img->width, 1, fp_tmp1);
		free(write_img);
		fclose(fp_tmp1);
#endif      
		if( quad_count <= 0 )
            continue; 

        mrFindQuadNeighbors2( quads, quad_count, dilations);
		
        quad_group = (CvCBQuad**)cvAlloc( sizeof(quad_group[0]) * quad_count);
        corner_group = (CvCBCorner**)cvAlloc( sizeof(corner_group[0]) * quad_count*4 );

	
        for( group_idx = 0; ; group_idx++ )
        {
            int count;
            count = icvFindConnectedQuads( quads, quad_count, quad_group, group_idx, storage, dilations );
	
            if( count == 0 )
                break;
			if (count > 28)
			    break;
        
			count = icvCleanFoundConnectedQuads( count, quad_group, corner_config.board_size );

			removeInvalidQuads(quad_group, corner_config.board_size, count);

			
			if( count >= max_count)
			{
				cvFree(&output_quad_group);
				max_count = count;
				max_dilation_run_ID = dilations;
				mrLabelQuadGroup( quad_group, max_count, corner_config.board_size, true );
				
				output_quad_group = (CvCBQuad**)cvAlloc( sizeof(output_quad_group[0]) * ((corner_config.board_size.height+2) * (corner_config.board_size.width+2)) );
				mrCopyQuadGroup( quad_group, output_quad_group, max_count );


				/*get founded corner num*/
				int inner_corner_num = 0;
				for (int i = 0; i < max_count; i++)
				{
					for(int j = 0; j < 4; j++)
					{
						if (output_quad_group[i]->neighbors[j])
						{
							inner_corner_num++;
						}
					}
				}
				inner_corner_num >>= 1;

				if (corner_config.correct_number_of_corners == inner_corner_num)
				{
					//goto PATTERN_FOUND;
					found = mrWriteCorners(output_quad_group, max_count, corner_config.board_size, corner_config.min_number_of_corners);
					goto EXIT;
				}
				
			}
        }

        cvFree( &quads );
        cvFree( &corners );
		cvFree(&quad_group);
		cvFree(&corner_group);
	}

//PATTERN_FOUND:
	if (&output_quad_group)
	{
		found = mrWriteCorners(output_quad_group, max_count, corner_config.board_size, corner_config.min_number_of_corners);

	}
	goto EXIT;

    for( dilations = max_dilations; dilations >= min_dilations; dilations-- )
    {
		cvCopy( temp_img_copy, temp_img);
        
		//IplConvKernel *kernel1 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CROSS,NULL);
		//IplConvKernel *kernel2 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);


		myDilateV4(temp_img, temp_img, dilations);
  //      if (dilations >= 1)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
		//if (dilations >= 2)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_RECT, 1);
		//if (dilations >= 3)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
		//if (dilations >= 4)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_RECT, 1);
		//if (dilations >= 5)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
		//if (dilations >= 6)
		//	cvDilate( temp_img, temp_img, 3, CV_SHAPE_RECT, 1);

    
        /*cvRectangle( temp_img, cvPoint(0,0), cvPoint(temp_img->cols-1,
                     temp_img->rows-1), CV_RGB(255,255,255), 3, 8);*/

        quad_count = icvGenerateQuads( &quads, &corners, storage, temp_img, flags, dilations, false );
        if( quad_count <= 0 )
            continue;
			
		int feedBack = -1;
		while ( feedBack == -1)
		{
			feedBack = mrAugmentBestRun( quads, quad_count, dilations, 
            							 output_quad_group, max_count, max_dilation_run_ID );
			
			if (feedBack == -1)
			{
				max_count = max_count + 1;
   				mrLabelQuadGroup( output_quad_group, max_count, corner_config.board_size, false );
				found = mrWriteCorners( output_quad_group, max_count, corner_config.board_size, corner_config.min_number_of_corners);
				if (found == -1 || found == 1)
					goto EXIT;
			}
		}
	}

    EXIT:

    cvReleaseMemStorage( &storage );
    cvFree( &quads );
    cvFree( &corners );
    cvFree( &quad_group );
    cvFree( &corner_group );
	cvFree( &output_quad_group );
	
	// -1  ->	Error or corner linking problem
	//  0  ->	Not enough corners were found
	//  1  ->	Enough corners were found
	//clock_t endTime0 = clock();
	//printf("cvFindChessboardCorners3 cost %d ms\n", endTime0 - startTime0);
    return found;
}

/*
	Function Name:      cvFindChessboardCorners2
	Function Function : detect corners corners of 1x1 pattern
	Input             : 
	    img :           The input binary image
	Return            : error code 
	                          1: exactly pattern is found; 
							  0: partly is found; 
	                         -1: not found.
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    unknown          Create
					  WangLuyao      2017/2/22        optimize call logic
*/
int chessBoard_corner_detector::cvFindChessboardCorners2( const IplImage* img )
{
	int flags					=  1;
	int max_count				=  0;
	int max_dilation_run_ID		= -1;
    const int min_dilations		=  corner_config.gloabal_min_dilate;
    const int max_dilations		=  2;
    int found					=  0;
    CvMemStorage* storage		=  0;
	
	CvCBQuad *quads				=  0;
	CvCBQuad **quad_group		=  0;
    CvCBCorner *corners			=  0;
	CvCBCorner **corner_group	=  0;
	CvCBQuad **output_quad_group = 0;
	
	int block_size = 0;
    int quad_count, group_idx, dilations;

    if( img->depth != IPL_DEPTH_8U || img->nChannels == 2 )
	{
		return -1;
	}
  
	if( corner_config.board_size.width > 127 || corner_config.board_size.height > 127 )
	{
		return -1;
	}
	
	//if( !image_points_buf )
	//{
	//	return -1;
	//}

    	// Create memory storage
	int height = img->height;
    int width = img->width;
    storage = cvCreateMemStorage(0);
	cvCopy(img, temp_img);
	//for(int k=0;k<corner_config.rect_mask_num;k++)
	//{
	//	if(rect_mask_roi[k].x<0||rect_mask_roi[k].x>=temp_img->width||rect_mask_roi[k].y<0||rect_mask_roi[k].y>=temp_img->height \
	//		||rect_mask_roi[k].x+rect_mask_roi[k].width<0||rect_mask_roi[k].x+rect_mask_roi[k].width>=temp_img->width
	//		||rect_mask_roi[k].y+rect_mask_roi[k].height<0||rect_mask_roi[k].y+rect_mask_roi[k].height>=temp_img->height)
	//	{
	//		return -1;
	//	}
	//	for(int i=(int)rect_mask_roi[k].y;i<(int)(rect_mask_roi[k].y+rect_mask_roi[k].height);i++ )
	//	for(int j=(int)rect_mask_roi[k].x;j<(int)(rect_mask_roi[k].x+rect_mask_roi[k].width);j++)
	//	{
	//		uchar* data = (uchar*)temp_img->imageData;
	//	    data[i*temp_img->widthStep+j]=255;
	//	}
	//}

	for(int k=0;k<4;k++)
	{

		if (boundary_point[k][0].x == 0 && boundary_point[k][0].y == 0 
			&& boundary_point[k][1].x == 0 && boundary_point[k][1].y == 0
			&& boundary_point[k][2].x == 0 && boundary_point[k][2].y == 0
			&& boundary_point[k][3].x == 0 && boundary_point[k][3].y == 0)
		{
			continue;
		}

		for (int p=0;p<4;p++)
		{
			if (boundary_point[k][p].x < 0 || boundary_point[k][p].x >= temp_img->width 
				|| boundary_point[k][p].y < 0 ||boundary_point[k][p].y >= temp_img->height)
			{
				return -1;
			}
		}

		for(int i=MIN(boundary_point[k][0].y, boundary_point[k][1].y);i<MAX(boundary_point[k][2].y, boundary_point[k][3].y);i++)
		{
			for(int j=MIN(boundary_point[k][0].x, boundary_point[k][2].x);j<MAX(boundary_point[k][1].x, boundary_point[k][3].x);j++)
			{
				Point2f p(j,i);
				bool flag = pInQuadrangle(boundary_point[k][0], boundary_point[k][1], boundary_point[k][2], boundary_point[k][3], p);
				uchar* data = (uchar*)temp_img->imageData;
				if (flag == 1)
					data[i*temp_img->widthStep+j]=255;
			}
		}
	}

	char str1[250];
	sprintf(str1, "d://TestFirst.raw");
	SaveRawFile1(str1, temp_img->imageData, temp_img->width*temp_img->height);

	if(corner_config.reflection==1)
	{
		//IplConvKernel *kernel_temp= cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CROSS,NULL);
		for(int i=0;i<temp_img->height;i++)
		for(int j=0;j<temp_img->width;j++)
		{
			uchar* data = (uchar*)temp_img->imageData;
			uchar imgPixValue = data[i*temp_img->widthStep+j];
			imgPixValue = 255-imgPixValue;
			data[i*temp_img->widthStep+j] = imgPixValue;
		}
		cvDilate(temp_img,temp_img,3,CV_SHAPE_CROSS,2);
		cvErode(temp_img,temp_img,3,CV_SHAPE_RECT,3);

		for(int i=0;i<temp_img->height;i++)
		for(int j=0;j<temp_img->width;j++)
		{
			uchar* data = (uchar*)temp_img->imageData;
			uchar imgPixValue = data[i*temp_img->widthStep+j];
			imgPixValue = 255-imgPixValue;
			data[i*temp_img->widthStep+j] = imgPixValue;
		}
	}



	cvCopy( temp_img, temp_img_copy);
	int two_group_founded = 0;

	vector<CvCBQuad2> corner_group_twoPoint_vector;
    for( dilations = min_dilations; dilations <= max_dilations; dilations++ )
    {
		if(two_group_founded==1)
			break;
		cvCopy( temp_img_copy, temp_img);
		

		//myDilateV4(temp_img, temp_img, dilations);
		if(temp_img->height <= 480) // standard definition
		{
			if (dilations == 1)
				cvDilate( temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
			if (dilations == 2)
				cvDilate( temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_LEFT, 1);
			if (dilations == 3)
				cvDilate( temp_img, temp_img, 2, CV_SHAPE_BAR_HORI_RIGHT, 1);
			if (dilations == 4)
				cvDilate( temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_UP, 1);
			if (dilations == 5)
				cvDilate( temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_DOWN, 1);
			if (dilations == 6)
				cvDilate( temp_img, temp_img, 2, CV_SHAPE_BAR_VERT_UP, 1);
		}

		else // high definition
		{
			if (dilations == 1)

				cvDilate(temp_img, temp_img, 3, CV_SHAPE_CROSS, 1);
			if (dilations == 2)

				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_HORI_ALL, 1);
			if (dilations == 3)

				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_VERT_ALL, 1);
			if (dilations == 4)

				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_048, 1);
			if (dilations == 5)

				cvDilate(temp_img, temp_img, 3, CV_SHAPE_BAR_INCL_246, 1);
			if (dilations == 6)
				cvDilate(temp_img, temp_img, 3, CV_SHAPE_RECT, 1);
		}

        /*cvRectangle( temp_img, cvPoint(0,0), cvPoint(temp_img->cols-1,
                     temp_img->rows-1), CV_RGB(255,255,255), 3, 8);*/

        quad_count = icvGenerateQuads( &quads, &corners, storage, temp_img, flags, dilations, true );
        if( quad_count <= 0 )
            continue;

		mrFindQuadNeighbors2( quads, quad_count, dilations);
		
        quad_group = (CvCBQuad**)cvAlloc( sizeof(quad_group[0]) * quad_count);
        corner_group = (CvCBCorner**)cvAlloc( sizeof(corner_group[0]) * quad_count*4 );

		CvCBQuad **corner_group_twoPoint = (CvCBQuad**)cvAlloc( sizeof(quad_group[0]) * quad_count*4 );
	
        for( group_idx = 0; ; group_idx++ )
        {
            int count;
            count = icvFindConnectedQuads( quads, quad_count, quad_group, group_idx, storage, dilations );
			if(count ==2)
			{
				quad_group[0]->group_idx = 255;
				quad_group[1]->group_idx = 255;
				CvCBQuad2 quad2;
				quad2.rect[0] = *quad_group[0];
				quad2.rect[1] = *quad_group[1];
				corner_group_twoPoint_vector.push_back(quad2);
				
			}
            if( count == 0 )
                break;
        }
		
		Point2f twoRectPoint1[255],twoRectPoint2[255];
		int twoRectMode_num=0;
		if(corner_config.camid>=2&&dilations < 0)
		{
			continue;
		}
		if(corner_group_twoPoint_vector.size()>1)
		{
			for(vector<CvCBQuad2>::iterator it=corner_group_twoPoint_vector.begin();it<corner_group_twoPoint_vector.end();it++)
				for(vector<CvCBQuad2>::iterator it1=it+1;it1<corner_group_twoPoint_vector.end();it1++)
			{
				Point2f p1,p2;
				for(int i=0;i<4;i++)
				{
					if(it->rect[0].neighbors[i])
					{
						p1.x=it->rect[0].corners[i]->pt.x;
						p1.y=it->rect[0].corners[i]->pt.y;
					}
					if(it1->rect[0].neighbors[i])
					{
						p2.x=it1->rect[0].corners[i]->pt.x;
						p2.y=it1->rect[0].corners[i]->pt.y;
					}
				}
				if(valid_num == 3)
				{
					if(reference_point[1].x==0&&reference_point[1].y==0)
					{
						if(ABS(p1.y-p2.y)<50&&ABS((p1.y+p2.y)/2)>img_in->height*corner_config.height_ratio&&ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio)
						{
							if(p1.x<p2.x)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
							}
							else
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
							}
							twoRectMode_num++;
						}
					}
					else
					{
						if(ABS(p1.y-p2.y)<50&&ABS((p1.y+p2.y)/2- reference_point[1].y)<reference_dist&&ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio)
						{
							if(p1.x<p2.x&&p1.x<reference_point[1].x-reference_offset&&p2.x>reference_point[1].x+reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
								twoRectMode_num++;
							}
							else if(p1.x>=p2.x&&p1.x>reference_point[1].x+reference_offset&&p2.x<reference_point[1].x-reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
								twoRectMode_num++;
							}
							
						}
					}
				}

				if(valid_num == 4)
				{
					if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x==0&&reference_point[2].y==0)
					{
						if(ABS(p1.y-p2.y)<50&&ABS((p1.y+p2.y)/2)>img_in->height*corner_config.height_ratio&&ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio)
						{
							if(p1.x<p2.x)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
							}
							else
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
							}
							twoRectMode_num++;
						}
					}
					if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x!=0&&reference_point[2].y!=0)
					{
						if(ABS((p1.y+p2.y)/2- reference_point[2].y)<reference_dist&&ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio)
						{
							if(p1.x<p2.x&&p1.x<reference_point[2].x-reference_offset&&p2.x>reference_point[2].x+reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
								twoRectMode_num++;
							}
							else if(p1.x>=p2.x&&p1.x>reference_point[2].x+reference_offset&&p2.x<reference_point[2].x-reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
								twoRectMode_num++;
							}
							
						}
					}

					if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x==0&&reference_point[2].y==0)
					{
						if(ABS((p1.y+p2.y)/2- reference_point[1].y)<reference_dist&&ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio)
						{
							if(p1.x<p2.x&&p1.x<reference_point[1].x-reference_offset&&p2.x>reference_point[1].x+reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
								twoRectMode_num++;
							}
							else if(p1.x>=p2.x&&p1.x>reference_point[1].x+reference_offset&&p2.x<reference_point[1].x-reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
								twoRectMode_num++;
							}
							
						}
					}

					if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x!=0&&reference_point[2].y!=0)
					{
						float dist2,dist1;
						PointLine_dist(p1,reference_point[1],reference_point[2],dist1 );
						PointLine_dist(p2,reference_point[1],reference_point[2],dist2 );
						if(ABS(p1.x-p2.x)>img_in->width*corner_config.width_ratio&&dist1<reference_dist&&dist2<reference_dist)
						{
							if(p1.x<p2.x&&p1.x<reference_point[1].x-reference_offset&&p2.x>reference_point[2].x+reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p1;
								twoRectPoint2[twoRectMode_num] = p2;
								twoRectMode_num++;
							}
							else if(p1.x>=p2.x&&p1.x>reference_point[2].x+reference_offset&&p2.x<reference_point[1].x-reference_offset)
							{
								twoRectPoint1[twoRectMode_num] = p2;
								twoRectPoint2[twoRectMode_num] = p1;
								twoRectMode_num++;
							}
							
						}
					}
				}
			}
			float symmetry_max=-ABS(ABS(twoRectPoint1[0].x-img_in->width/2)-ABS(twoRectPoint2[0].x-img_in->width/2));
			for(int i =0;i<twoRectMode_num;i++)
			{
				float symmetry = -ABS(ABS(twoRectPoint1[i].x-img_in->width/2)-ABS(twoRectPoint2[i].x-img_in->width/2));
				if(symmetry>=symmetry_max)
				{
					pattern_Single_corner[0].corner_point[0][0] = twoRectPoint1[i];
					pattern_Single_corner[0].flag[0][0] = 1;
					pattern_Single_corner[0].rows = 1;
					pattern_Single_corner[0].cols =1;
					pattern_Single_corner[1].corner_point[0][0] = twoRectPoint2[i];
					pattern_Single_corner[1].flag[0][0] = 1;
					pattern_Single_corner[1].rows = 1;
					pattern_Single_corner[1].cols =1;
				}
				two_group_founded=1;
			}

			if(twoRectMode_num==0)
			{
				for(vector<CvCBQuad2>::iterator it=corner_group_twoPoint_vector.begin();it<corner_group_twoPoint_vector.end();it++)
				{
					Point2f p;
					for(int i=0;i<4;i++)
					{
						if(it->rect[0].neighbors[i])
						{
							p.x = it->rect[0].corners[i]->pt.x;
							p.y = it->rect[0].corners[i]->pt.y;
						}
					}
					if(valid_num == 3)
					{
						if(reference_point[1].x==0&&reference_point[1].y==0)
						{
							if(p.y>img_in->height*corner_config.height_ratio)
							{
								if(p.x>img_in->width/6&&p.x<img_in->width/2)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*5/6&&p.x>img_in->width/2)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
									
							}
						}
						else
						{
							if(ABS(p.y-reference_point[1].y)<reference_dist)
							{
								if(p.x>img_in->width/6&&p.x<reference_point[1].x-reference_offset)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*5/6&&p.x>reference_point[1].x+reference_offset)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
							}
						}
					}

					if(valid_num == 4)
					{
						if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x==0&&reference_point[2].y==0)
						{
							if(p.y>img_in->height*corner_config.height_ratio)
							{
								if(p.x>img_in->width/6&&p.x<img_in->width/2)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*5/6&&p.x>img_in->width/2)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
									
							}
						}
						if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x==0&&reference_point[2].y==0)
						{
							if(ABS(p.y-reference_point[1].y)<reference_dist)
							{
								if(p.x>img_in->width/6&&p.x<reference_point[1].x-reference_offset)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*5/6&&p.x>reference_point[1].x+reference_offset)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
							}
						}

						if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x!=0&&reference_point[2].y!=0)
						{
							if(ABS(p.y-reference_point[2].y)<reference_dist)
							{
								if(p.x>img_in->width/6&&p.x<reference_point[2].x-reference_offset)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*5/6&&p.x>reference_point[2].x+reference_offset)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
							}
						}
						if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x!=0&&reference_point[2].y!=0)
						{
							float dist;
							PointLine_dist(p,reference_point[1],reference_point[2],dist );
							if(dist<reference_dist)
							{
								if(p.x>img_in->width/8&&p.x<reference_point[1].x-reference_offset)
								{
									pattern_Single_corner[0].corner_point[0][0] = p;
									pattern_Single_corner[0].flag[0][0] = 1;
									pattern_Single_corner[0].rows = 1;
									pattern_Single_corner[0].cols =1;
								}
								else if(p.x<img_in->width*7/8&&p.x>reference_point[2].x+reference_offset)
								{
									pattern_Single_corner[1].corner_point[0][0] = p;
									pattern_Single_corner[1].flag[0][0] = 1;
									pattern_Single_corner[1].rows = 1;
									pattern_Single_corner[1].cols =1;
								}
							}
						}
					}
				}
			}
		}

		else if(corner_group_twoPoint_vector.size()==1)
		{
			Point2f p;
			for(int i=0;i<4;i++)
			{
				if(corner_group_twoPoint_vector.at(0).rect[0].neighbors[i])
				{
					p.x = corner_group_twoPoint_vector.at(0).rect[0].corners[i]->pt.x;
					p.y = corner_group_twoPoint_vector.at(0).rect[0].corners[i]->pt.y;
				}
			}
			if(valid_num == 3)
			{
				if(reference_point[1].x==0&&reference_point[1].y==0)
				{
					if(p.y>img_in->height*corner_config.height_ratio)
					{
						if(p.x>img_in->width/6&&p.x<img_in->width/2)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>img_in->width/2)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
									
					}
				}
				else
				{
					if(ABS(p.y-reference_point[1].y)<reference_dist)
					{
						if(p.x>img_in->width/6&&p.x<reference_point[1].x-reference_offset)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>reference_point[1].x+reference_offset)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
					}
				}
			}

			if(valid_num == 4)
			{
				if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x==0&&reference_point[2].y==0)
				{
					if(p.y>img_in->height*corner_config.height_ratio)
					{
						if(p.x>img_in->width/6&&p.x<img_in->width/2)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>img_in->width/2)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
									
					}
				}
				if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x==0&&reference_point[2].y==0)
				{
					if(ABS(p.y-reference_point[1].y)<reference_dist)
					{
						if(p.x>img_in->width/6&&p.x<reference_point[1].x-reference_offset)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>reference_point[1].x+reference_offset)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
					}
				}

				if(reference_point[1].x==0&&reference_point[1].y==0&&reference_point[2].x!=0&&reference_point[2].y!=0)
				{
					if(ABS(p.y-reference_point[2].y)<reference_dist)
					{
						if(p.x>img_in->width/6&&p.x<reference_point[2].x-reference_offset)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>reference_point[2].x+reference_offset)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
					}
				}
				if(reference_point[1].x!=0&&reference_point[1].y!=0&&reference_point[2].x!=0&&reference_point[2].y!=0)
				{
					float dist;
					PointLine_dist(p,reference_point[1],reference_point[2],dist );
					if(dist<reference_dist)
					{
						if(p.x>img_in->width/6&&p.x<reference_point[1].x-reference_offset)
						{
							pattern_Single_corner[0].corner_point[0][0] = p;
							pattern_Single_corner[0].flag[0][0] = 1;
							pattern_Single_corner[0].rows = 1;
							pattern_Single_corner[0].cols =1;
						}
						else if(p.x<img_in->width*5/6&&p.x>reference_point[2].x+reference_offset)
						{
							pattern_Single_corner[1].corner_point[0][0] = p;
							pattern_Single_corner[1].flag[0][0] = 1;
							pattern_Single_corner[1].rows = 1;
							pattern_Single_corner[1].cols =1;
						}
					}
				}
			}
		}
        cvFree( &quads );
        cvFree( &corners );
		cvFree(&corner_group_twoPoint);
    }
	//found = mrWriteCorners( output_quad_group, max_count, corner_config.board_size, corner_config.min_number_of_corners);
	//	if (found == -1 || found == 1)
	//	goto	EXIT;

 //   EXIT:
 //   if( found == -1 )
	//{
	//	return -1;
	//}
	if((pattern_Single_corner[0].flag[0][0] == 1)||(pattern_Single_corner[1].flag[0][0] == 1))
		found = 1;
	else
		found = 0;
    cvReleaseMemStorage( &storage );
    cvFree( &quads );
    cvFree( &corners );
    cvFree( &quad_group );
    cvFree( &corner_group );
	cvFree( &output_quad_group );
	
	// -1  ->	Error or corner linking problem
	//  0  ->	Not enough corners were found
	//  1  ->	Enough corners were found
    return found;
}

/*
	Function Name:      PointLine_dist
	Function Function : calculate distance between point and line
	Input             : 
	    point_in :      The coordinates of point
	    line_start:     The start point of line
	    line_end:       The end point of line
	Return            : distance 
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    unknown          Create
*/
void chessBoard_corner_detector::PointLine_dist(Point2f point_in,Point2f line_start,Point2f line_end,float &dist )
{
	float a,b;
	a=line_end.x-line_start.x;
	b=line_end.y-line_start.y;
	dist=ABS(a*point_in.y-b*point_in.x+line_end.x*b-line_end.y*a)/sqrt(a*a+b*b);
}

/*
	Function Name:      triangleArea
	Function Function : calculate the area of a triangle
	Input             : 
	    a,b,c :         The vertexes of a triangle
	Return            : area 
	Note              : 
	Revision History  : Author         Data             Changes
	                    WangLuyao      unknown          Create
*/
double chessBoard_corner_detector::triangleArea(Point2f a, Point2f b, Point2f c) 
{
	double ab = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	double bc = sqrt((b.x-c.x)*(b.x-c.x) + (b.y-c.y)*(b.y-c.y));
	double ac = sqrt((a.x-c.x)*(a.x-c.x) + (a.y-c.y)*(a.y-c.y));
	double s = (ab + bc + ac) / 2;
	return sqrt(s*(s - ab)*(s - bc)*(s - ac));
}

/*
	Function Name:      pInQuadrangle
	Function Function : judge whether a point is inside the quadrangle
	Input             : 
	    a,b,c,d :        The vertexes of a quadrangle
	Return            : error code 
	                          1: inside the quadrangle; 
	                          0: outside the quadrangle.
	Note              : 
	Revision History  : Author         Data             Changes
	                    WangLuyao      unknown          Create
*/
bool chessBoard_corner_detector::pInQuadrangle(Point2f a, Point2f b, Point2f c, Point2f d, Point2f p)
{
	double dTriangle = triangleArea(a, b, p) + triangleArea(a, c, p)
		+ triangleArea(c, d, p) + triangleArea(b, d, p);
	double dQuadrangle = triangleArea(a, b, c) + triangleArea(c, b, d);
	if ((abs(dQuadrangle - dTriangle) < 0.01) && (dQuadrangle != 0))
		return 1;
	else
		return 0;
}

/*
	Function Name:      icvCleanFoundConnectedQuads
	Function Function : If we found too many connected quads, remove those which probably do not belong
	Input             : 
	    quad_count    : Detected quad num
	    quad_group    : Detected quad group
	    pattern_size  : the size of pattern required
	Return            : 
	    quand_count   : The valid quad count after outliers removing
	Note              : If we found too many connected quads, remove those which probably do not belong.
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/16        revise 2*2 and more than 2*2 pattern remove method.
*/
int chessBoard_corner_detector::icvCleanFoundConnectedQuads(int quad_count, CvCBQuad **quad_group, CvSize pattern_size)
{
    CvMemStorage *temp_storage = 0;
    CvPoint2D32f *centers = 0;

	CvPoint2D32f center;
	center.x = 0;
	center.y = 0;
    int i, j, k;


	int valid_quad_num = corner_config.correct_number_of_quads;
	int valid_corner_num = corner_config.correct_number_of_corners;

    // If we have more quadrangles than we should, try to eliminate duplicates
	// or ones which don't belong to the pattern rectangle. Else go to the end
	// of the function
	/*if (quad_count <= valid_quad_num)
        goto icvCleanFoundConnectedQuads_EXIT;*/

	// add by Yanshuo, remove the chessboard whom has only one neighbour for 2*2 chessboard.
	// in this case, if more than 4 quads are detected, remove the quads has only one neighbor
	CvCBQuad* q_remove = NULL, *temp = NULL, *q;
	if (valid_quad_num == 4)
	{
		// find the quad that only one neighbor is connected and remove it
		for (; quad_count > valid_quad_num; quad_count--)
		{
			int remove_id = 0;
			for(int i = 0; i < quad_count; i++)
			{
				q = quad_group[i];
				int count_neighbors = 0;
				for( j = 0; j < 4; j++ )
				{
					if( q->neighbors[j] )
					{
						count_neighbors++;
					}
				}
				if(count_neighbors == 1)
				{
					remove_id = i;
					q_remove = q;
					break;
				}
			}
			// remove any references to this quad as a neighbor
			if(q_remove)
			{
				for( i = 0; i < quad_count; i++ )
				{
					if(quad_group[i] != q_remove)
					{
						q = quad_group[i];
						for( j = 0; j < 4; j++ )
						{
							if( q->neighbors[j] == q_remove )
							{
								q->neighbors[j] = 0;
								q->count--;
								for( k = 0; k < 4; k++ )
								{
									if( q_remove->neighbors[k] == q )
									{
										q_remove->neighbors[k] = 0;
										q_remove->count--;
										break;
									}
								}
								break;
							}
						}
					}					
				}
			} // end if(q_remove)
			
			// put q_remove at the end of this quad group
			for(int i = remove_id + 1; i < quad_count; i++)
			{				 
				quad_group[i - 1] = quad_group[i] ;
			}
			quad_group[quad_count - 1] = q_remove;
		}
		// find 
	}
	// end of this add


	// add prune code for non 2*2 pattern, add by YanShuo on 2017/3/5
	// If we have more quadrangles than we desired, remove the quad who do not has 4 neighbors,
	// and none of it's neighbors has 4 neighbors.
	if (((valid_corner_num == 4 || valid_corner_num == 7) && station_type == STATION_TYPE_JAC) 
	  || (valid_corner_num > 4 && (station_type == STATION_TYPE_GAC || station_type == STATION_TYPE_OFILM)))
	{
		int loop = 0;
		int remove_quad_num = 0;
		int inner_corner_num = 0; // actually detected corner num
		for(int i = 0; i < quad_count; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				if(quad_group[i]->neighbors[j])
				{
					inner_corner_num = inner_corner_num + 1;
				}
			}
			//printf("%d %d %d %d %d ", quad_group[i], quad_group[i]->neighbors[0], quad_group[i]->neighbors[1], quad_group[i]->neighbors[2], quad_group[i]->neighbors[3]);
			//printf("\n");
		}
		inner_corner_num >>= 1;
		// find more than deserved corners, need to be pruned
		while(inner_corner_num > valid_corner_num && loop < 3)
		{
			loop++;
			for(int quad_id = 0; quad_id < quad_count; quad_id++)
			{
				int valid_quad_count = 0, remove_quad_id;
				// judge whether the current quad has 4 neighbors
				for(int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
				{
					if(quad_group[quad_id]->neighbors[neighbor_id] != 0)
					{
						valid_quad_count++;
					}
				}
				
				// else see whether one of it's neighbor has 4 neighbors
				if(valid_quad_count != 4)
				{

					for(int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
					{
						if(quad_group[quad_id]->neighbors[neighbor_id])
						{
							valid_quad_count = 0;
							CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
							for(int i = 0; i < 4; i++)
							{
								if(q->neighbors[i] != 0)
									valid_quad_count++;
							}
							if(valid_quad_count == 4)
							{
								break;
							}
						}
					}
				}
				// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
				if(valid_quad_count != 4)
				{
					remove_quad_id = quad_id;
					CvCBQuad* q_remove = quad_group[remove_quad_id];
					for(int i = 0; i < quad_count; i++ )
					{
						if(i != quad_id)
						{
							CvCBQuad* q = quad_group[i];
							for(int j = 0; j < 4; j++ )
							{
								if( q->neighbors[j] == q_remove )
								{
									q->neighbors[j] = NULL;
									q->count--;
									for(int k = 0; k < 4; k++ )
									{
										if( q_remove->neighbors[k] == q )
										{
											q_remove->neighbors[k] = NULL;
											q_remove->count--;
											break;
										}
									}
									break;
								}
							}
						}
					}

					quad_group[remove_quad_id] = quad_group[quad_count-1];
					quad_count--;
				}
			}
			int id = 0;
			for (int i = 0; i < quad_count; i++)
			{
				
				int num = 0;
				CvCBQuad* q = quad_group[i];
				for (int j = 0; j < 4; j++)
				{
					if (q->neighbors[j] == NULL)
					{
						num++;
					}	
				}
				if (num == 4)
				{
					id = i;
					break;
				}
			}
			quad_group[id] = quad_group[quad_count - 1];
			quad_count--;

			
			inner_corner_num = 0;
			for (int i = 0; i < quad_count; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (quad_group[i]->neighbors[j])
					{

						inner_corner_num++;
					}


				}
			}
			inner_corner_num >>= 1;
		}
	}
	if (station_type == STATION_TYPE_JAC)
	{
		if (pattern_side == SIDE_LEFT && valid_corner_num == 6)
		{
			int flag = 1;
			while (flag)
			{
				flag = 0;
				for (int quad_id = 0; quad_id < quad_count; quad_id++)
				{
					int valid_quad_count = 0, remove_quad_id;
					int neighborId;
					// judge whether the current quad has 4 neighbors
					for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
					{
						if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
						{
							valid_quad_count++;
							neighborId = neighbor_id;
						}
					}
				/*	if (valid_quad_count == 1 && neighborId == 2)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}*/
					int neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					/*	if (valid_quad_count == 1 && neighborId == 3)
						{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
						if (quad_group[quad_id]->neighbors[neighbor_id])
						{
						neighbor_label = 0;
						CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
						for (int i = 0; i < 4; i++)
						{
						if (q->neighbors[i] != 0)
						{
						neighbor_label |= (1) << i;
						}
						}
						if (neighbor_label == 10 || neighbor_label == 6 || neighbor_label == 3)
						{
						break;
						}
						}
						}
						}*/
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 10 || neighbor_label == 6 || neighbor_label == 3)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
				}
			}
		}

		if (pattern_side == SIDE_RIGHT && valid_corner_num == 6)
		{
			if ( quad_group[0]->corners[0]->pt.x < thresh_img->width*0.5)
			{
				quad_count = -1;
			}
			else
			{
				int flag = 1;
				while (flag)
				{
					flag = 0;
					for (int quad_id = 0; quad_id < quad_count; quad_id++)
					{
						int valid_quad_count = 0, remove_quad_id;
						int neighborId;
						// judge whether the current quad has 4 neighbors
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
							{
								valid_quad_count++;
								neighborId = neighbor_id;
							}
						}
						if (valid_quad_count == 1 && neighborId == 0)
						{
							flag = 1;
							remove_quad_id = quad_id;
							CvCBQuad* q_remove = quad_group[remove_quad_id];
							for (int i = 0; i < quad_count; i++)
							{
								if (i != quad_id)
								{
									CvCBQuad* q = quad_group[i];
									for (int j = 0; j < 4; j++)
									{
										if (q->neighbors[j] == q_remove)
										{
											q->neighbors[j] = NULL;
											q->count--;
											for (int k = 0; k < 4; k++)
											{
												if (q_remove->neighbors[k] == q)
												{
													q_remove->neighbors[k] = NULL;
													q_remove->count--;
													break;
												}
											}
											break;
										}
									}
								}
							}

							quad_group[remove_quad_id] = quad_group[quad_count - 1];
							quad_count--;
						}
						if (valid_quad_count == 1 && neighborId == 3)
						{
							flag = 1;
							remove_quad_id = quad_id;
							CvCBQuad* q_remove = quad_group[remove_quad_id];
							for (int i = 0; i < quad_count; i++)
							{
								if (i != quad_id)
								{
									CvCBQuad* q = quad_group[i];
									for (int j = 0; j < 4; j++)
									{
										if (q->neighbors[j] == q_remove)
										{
											q->neighbors[j] = NULL;
											q->count--;
											for (int k = 0; k < 4; k++)
											{
												if (q_remove->neighbors[k] == q)
												{
													q_remove->neighbors[k] = NULL;
													q_remove->count--;
													break;
												}
											}
											break;
										}
									}
								}
							}

							quad_group[remove_quad_id] = quad_group[quad_count - 1];
							quad_count--;
						}
						int neighbor_label = 0;
						// else see whether one of it's neighbor has 4 neighbors
						if (valid_quad_count == 1 && neighborId == 2)
						{
							for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
							{
								if (quad_group[quad_id]->neighbors[neighbor_id])
								{
									neighbor_label = 0;
									CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
									for (int i = 0; i < 4; i++)
									{
										if (q->neighbors[i] != 0)
										{
											neighbor_label |= (1) << i;
										}
									}
									if (neighbor_label == 5 || neighbor_label == 9 || neighbor_label == 3)
									{
										break;
									}
								}
							}
						}
						// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
						if (neighbor_label == 5 || neighbor_label == 9 || neighbor_label == 3)
						{
							flag = 1;
							remove_quad_id = quad_id;
							CvCBQuad* q_remove = quad_group[remove_quad_id];
							for (int i = 0; i < quad_count; i++)
							{
								if (i != quad_id)
								{
									CvCBQuad* q = quad_group[i];
									for (int j = 0; j < 4; j++)
									{
										if (q->neighbors[j] == q_remove)
										{
											q->neighbors[j] = NULL;
											q->count--;
											for (int k = 0; k < 4; k++)
											{
												if (q_remove->neighbors[k] == q)
												{
													q_remove->neighbors[k] = NULL;
													q_remove->count--;
													break;
												}
											}
											break;
										}
									}
								}
							}

							quad_group[remove_quad_id] = quad_group[quad_count - 1];
							quad_count--;
						}
					}
				}
			}
			
		}

		if (valid_corner_num == 9 && pattern_side == SIDE_LEFT)
		{
			int flag = 1;
			while (flag)
			{
				flag = 0;
				for (int quad_id = 0; quad_id < quad_count; quad_id++)
				{
					int valid_quad_count = 0, remove_quad_id;
					int neighborId;
					// judge whether the current quad has 4 neighbors
					for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
					{
						if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
						{
							valid_quad_count++;
							neighborId = neighbor_id;
						}
					}
					if (valid_quad_count == 1 && (neighborId == 2 || neighborId == 3))
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					int neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 1)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 12)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 12)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
				}
			}
		}

		if (valid_corner_num == 9 && pattern_side == SIDE_RIGHT)
		{
			if (quad_group[0]->corners[0]->pt.x > thresh_img->width*0.8 || quad_group[0]->corners[0]->pt.x<thresh_img->width*0.5)
			{
					quad_count=-1;				
			}
			else
			{
				int flag = 1;
				while (flag)
				{
					flag = 0;
					for (int quad_id = 0; quad_id < quad_count;quad_id++)
					{
						int valid_quad_count = 0, remove_quad_id;
					    int neighborId_1,neighborId_2;
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)//test how many neighbors the current quad has
						{
							if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
							{
								valid_quad_count++;
								if (valid_quad_count==1)
								{
									neighborId_1 = neighbor_id;
								}
								if (valid_quad_count==2)
								{
									neighborId_2 = neighbor_id;
								}
							}
						}
						int neighbor_label = 0;
						if (valid_quad_count==2&&neighborId_1==0&&neighborId_2==1)
						{
							for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
							{
								if (quad_group[quad_id]->neighbors[neighbor_id])
								{
									neighbor_label = 0;
									CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
									for (int i = 0; i < 4; i++)
									{
										if (q->neighbors[i] != 0)
										{
											neighbor_label |= (1) << i;
										}
									}
									if (neighbor_label == 6 || neighbor_label == 9)
									{
										continue;
									}
									else
									{
										flag = 1;
									/*	remove_quad_id = quad_id;*/
										CvCBQuad* q_remove = quad_group[quad_id]->neighbors[neighbor_id];
										 q_remove = NULL;
										 quad_count--;
										 /*		for (int i = 0; i < quad_count; i++)
												 {
												 if (i != quad_id)
												 {
												 CvCBQuad* q = quad_group[i];
												 for (int j = 0; j < 4; j++)
												 {
												 if (q->neighbors[j] == q_remove)
												 {
												 q->neighbors[j] = NULL;
												 q->count--;
												 for (int k = 0; k < 4; k++)
												 {
												 if (q_remove->neighbors[k] == q)
												 {
												 q_remove->neighbors[k] = NULL;
												 q_remove->count--;
												 break;
												 }
												 }
												 break;
												 }
												 }
												 }
												 }

												 quad_group[remove_quad_id] = quad_group[quad_count - 1];
												 quad_count--;*/
									}
								}
							}
						}

					}
				}
			}
			//else
			//{
			//	int flag = 1;
			//	while (flag)
			//	{
			//		flag = 0;
			//		for (int quad_id = 0; quad_id < quad_count; quad_id++)
			//		{
			//			int valid_quad_count = 0, remove_quad_id;
			//			int neighborId;
			//			// judge whether the current quad has 4 neighbors
			//			for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)//test how many neighbors the current quad has
			//			{
			//				if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
			//				{
			//					valid_quad_count++;
			//					neighborId = neighbor_id;
			//				}
			//			}
			//			//if (valid_quad_count == 1&&(neighborId==1))//if 1 position appeared ,delete
			//			//{


			//			//}
			//			if (valid_quad_count == 1 && (neighborId == 2 || neighborId == 3))//if 2or3 position appeared,continue
			//			{
			//				flag = 1;
			//				remove_quad_id = quad_id;
			//				CvCBQuad* q_remove = quad_group[remove_quad_id];
			//				for (int i = 0; i < quad_count; i++)
			//				{
			//					if (i != quad_id)
			//					{
			//						CvCBQuad* q = quad_group[i];
			//						for (int j = 0; j < 4; j++)
			//						{
			//							if (q->neighbors[j] == q_remove)
			//							{
			//								q->neighbors[j] = NULL;
			//								q->count--;
			//								for (int k = 0; k < 4; k++)
			//								{
			//									if (q_remove->neighbors[k] == q)
			//									{
			//										q_remove->neighbors[k] = NULL;
			//										q_remove->count--;
			//										break;
			//									}
			//								}
			//								break;
			//							}
			//						}
			//					}
			//				}

			//				quad_group[remove_quad_id] = quad_group[quad_count - 1];
			//				quad_count--;
			//			}
			//			int neighbor_label = 0;
			//			// else see whether one of it's neighbor has 4 neighbors
			//			if (valid_quad_count == 1 && neighborId == 0)
			//			{
			//				for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
			//				{
			//					if (quad_group[quad_id]->neighbors[neighbor_id])
			//					{
			//						neighbor_label = 0;
			//						CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
			//						for (int i = 0; i < 4; i++)
			//						{
			//							if (q->neighbors[i] != 0)
			//							{
			//								neighbor_label |= (1) << i;
			//							}
			//						}
			//						if (neighbor_label == 12)
			//						{
			//							break;
			//						}
			//					}
			//				}
			//			}
			//			// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
			//			if (neighbor_label == 12)
			//			{
			//				flag = 1;
			//				remove_quad_id = quad_id;
			//				CvCBQuad* q_remove = quad_group[remove_quad_id];
			//				for (int i = 0; i < quad_count; i++)
			//				{
			//					if (i != quad_id)
			//					{
			//						CvCBQuad* q = quad_group[i];
			//						for (int j = 0; j < 4; j++)
			//						{
			//							if (q->neighbors[j] == q_remove)
			//							{
			//								q->neighbors[j] = NULL;
			//								q->count--;
			//								for (int k = 0; k < 4; k++)
			//								{
			//									if (q_remove->neighbors[k] == q)
			//									{
			//										q_remove->neighbors[k] = NULL;
			//										q_remove->count--;
			//										break;
			//									}
			//								}
			//								break;
			//							}
			//						}
			//					}
			//				}

			//				quad_group[remove_quad_id] = quad_group[quad_count - 1];
			//				quad_count--;
			//			}
			//		}
			//	}
			//}
		}

		if (corner_config.camid == 2)
		{
			int flag = 1;
			while (flag)
			{
				flag = 0;
				for (int quad_id = 0; quad_id < quad_count; quad_id++)
				{
					int valid_quad_count = 0, remove_quad_id;
					int neighborId;
					int neighborId_1, neighborId_2;
					// judge whether the current quad has 4 neighbors
					for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
					{
						if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
						{
							valid_quad_count++;
							neighborId = neighbor_id;
						}
					}
					if (valid_quad_count==2)//outliers with two neighbors
					{
						valid_quad_count = 0;
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
							{
								valid_quad_count++;
								if (valid_quad_count==1)
								{
									neighborId_1 = neighbor_id;
								}
								if (valid_quad_count==2)
								{
									neighborId_2 = neighbor_id;
								}
							}
						}
						
					}
					if (valid_quad_count==2&&neighborId_1==2&&neighborId_2==3)//see if its  neighbor  has 4 neighbors
					{
						int neighbor_label2 = 0;
						if (quad_group[quad_id]->corners[0]->pt.x<thresh_img->width*0.5)
						{
							for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
							{ 
								neighbor_label2 = 0;
								if (quad_group[quad_id]->neighbors[neighbor_id])
								{
									
									CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];//neighbor's neighbor
									for (int i = 0; i < 4; i++)
									{
										if (q->neighbors[i] != 0)
										{
											neighbor_label2 |= (1) << i;
										}
									}
									if (neighbor_label2 == 15)
									{
										break;
									}
									
								}
							}
							if (neighbor_label2 !=15)
							{
								flag = 1;
								remove_quad_id = quad_id;
								CvCBQuad* q_remove = quad_group[remove_quad_id];
								for (int i = 0; i < quad_count; i++)
								{
									if (i != quad_id)
									{
										CvCBQuad* q = quad_group[i];
										for (int j = 0; j < 4; j++)
										{
											if (q->neighbors[j] == q_remove)
											{
												q->neighbors[j] = NULL;
												q->count--;
												for (int k = 0; k < 4; k++)
												{
													if (q_remove->neighbors[k] == q)
													{
														q_remove->neighbors[k] = NULL;
														q_remove->count--;
														break;
													}
												}
												break;
											}
										}
									}
								}

								quad_group[remove_quad_id] = quad_group[quad_count - 1];
								quad_count--;
							}
						}
					
					}
					int neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors 
					if (valid_quad_count == 1 && neighborId == 0)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 7 || neighbor_label == 5 || neighbor_label == 12)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 7 || neighbor_label == 5 || neighbor_label == 12)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 1)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 11 || neighbor_label == 14 || neighbor_label == 9 || neighbor_label == 12)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 11 || neighbor_label == 14 || neighbor_label == 9 || neighbor_label == 12)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 2)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 13 || neighbor_label == 5 || neighbor_label == 7 || neighbor_label == 9)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 13 || neighbor_label == 5 || neighbor_label == 7 || neighbor_label == 9)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 3)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 14 || neighbor_label == 3 || neighbor_label == 15 || neighbor_label == 10)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 14 || neighbor_label == 3 || neighbor_label == 15 || neighbor_label == 10)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
				}
			}

		}

		if (corner_config.camid == 3)
		{
			int flag = 1;
			while (flag)
			{
				flag = 0;
				for (int quad_id = 0; quad_id < quad_count; quad_id++)
				{
					int valid_quad_count = 0, remove_quad_id;
					int neighborId;
					// judge whether the current quad has 4 neighbors
					for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
					{
						if (quad_group[quad_id]->neighbors[neighbor_id] != 0)
						{
							valid_quad_count++;
							neighborId = neighbor_id;
						}
					}
					int neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 0)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 7 || neighbor_label == 5 || neighbor_label == 13)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 7 || neighbor_label == 5 || neighbor_label == 13)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 1)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 11 || neighbor_label == 10 || neighbor_label == 12)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 11 || neighbor_label == 10 || neighbor_label == 12)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 2)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 13 || neighbor_label == 5 || neighbor_label == 9)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 13 || neighbor_label == 5 || neighbor_label == 9)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
					neighbor_label = 0;
					// else see whether one of it's neighbor has 4 neighbors
					if (valid_quad_count == 1 && neighborId == 3)
					{
						for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)
						{
							if (quad_group[quad_id]->neighbors[neighbor_id])
							{
								neighbor_label = 0;
								CvCBQuad* q = quad_group[quad_id]->neighbors[neighbor_id];
								for (int i = 0; i < 4; i++)
								{
									if (q->neighbors[i] != 0)
									{
										neighbor_label |= (1) << i;
									}
								}
								if (neighbor_label == 14 || neighbor_label == 10 || neighbor_label == 11)
								{
									break;
								}
							}
						}
					}
					// if none of it's neighbor don't has 4 inner corners, that means this quad is redundant
					if (neighbor_label == 14 || neighbor_label == 10 || neighbor_label == 11)
					{
						flag = 1;
						remove_quad_id = quad_id;
						CvCBQuad* q_remove = quad_group[remove_quad_id];
						for (int i = 0; i < quad_count; i++)
						{
							if (i != quad_id)
							{
								CvCBQuad* q = quad_group[i];
								for (int j = 0; j < 4; j++)
								{
									if (q->neighbors[j] == q_remove)
									{
										q->neighbors[j] = NULL;
										q->count--;
										for (int k = 0; k < 4; k++)
										{
											if (q_remove->neighbors[k] == q)
											{
												q_remove->neighbors[k] = NULL;
												q_remove->count--;
												break;
											}
										}
										break;
									}
								}
							}
						}

						quad_group[remove_quad_id] = quad_group[quad_count - 1];
						quad_count--;
					}
				}
			}
		}
	}
	// end of prune code for non 2*2 pattern
	if (quad_count <= valid_quad_num)
		goto icvCleanFoundConnectedQuads_EXIT;
    // Create an array of quadrangle centers
    centers = (CvPoint2D32f *)cvAlloc( sizeof(centers[0])*quad_count );
    temp_storage = cvCreateMemStorage(0);
	//printf("\n");
    for( i = 0; i < quad_count; i++ )
    {
		CvPoint2D32f ci;
		ci.x = 0;
		ci.y = 0;
        CvCBQuad* q = quad_group[i];

		// output the pattern's neighbors for 2*2 pattern
		//printf("quad id: %d  neighbors:%d %d %d %d %d\n", i, q, q->neighbors[0], q->neighbors[1], q->neighbors[2], q->neighbors[3]);
        for( j = 0; j < 4; j++ )
        {
            CvPoint2D32f pt = q->corners[j]->pt;
            ci.x += pt.x;
            ci.y += pt.y;
        }

        ci.x *= 0.25f;
        ci.y *= 0.25f;
	

		// Centers(i), is the geometric center of quad(i)
		// Center, is the center of all found quads
        centers[i] = ci;
        center.x += ci.x;
        center.y += ci.y;
    }
    center.x /= quad_count;
    center.y /= quad_count;

	//if(quad_group[i])
	//{
	//	CvCBQuad* q = quad_group[i];
	//	printf("quad id: %d  neighbors: %d %d %d %d\n", i, q->neighbors[0], q->neighbors[1], q->neighbors[2], q->neighbors[3]);
	//}

    // If we have more quadrangles than we should, we try to eliminate bad
	// ones based on minimizing the bounding box. We iteratively remove the
	// point which reduces the size of the bounding box of the blobs the most
    // (since we want the rectangle to be as small as possible) remove the
	// quadrange that causes the biggest reduction in pattern size until we
	// have the correct number
	for (; quad_count > valid_quad_num; quad_count--)
    {
        double min_box_area = DBL_MAX;
        int skip, min_box_area_index = -1;
        CvCBQuad *q0, *q;


        // For each point, calculate box area without that point
        for( skip = 0; skip < quad_count; skip++ )
        {
            // get bounding rectangle
            CvPoint2D32f temp = centers[skip]; 
            centers[skip] = center;
			CvMemStorage* storage1 = cvCreateMemStorage(0);
			CvSeq* pointSeq = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage1 );
			for (i=0; i<quad_count; ++i)
				cvSeqPush( pointSeq,&centers[i]);
            CvSeq *hull = cvConvexHull2( pointSeq, temp_storage, CV_CLOCKWISE, 1 );
            centers[skip] = temp;
            double hull_area = fabs(cvContourArea(hull, CV_WHOLE_SEQ));


            // remember smallest box area
            if( hull_area < min_box_area )
            {
                min_box_area = hull_area;
                min_box_area_index = skip;
            }
            cvClearMemStorage( temp_storage );
        }

        q0 = quad_group[min_box_area_index];


        // remove any references to this quad as a neighbor
        for( i = 0; i < quad_count; i++ )
        {
            q = quad_group[i];
            for( j = 0; j < 4; j++ )
            {
                if( q->neighbors[j] == q0 )
                {
                    q->neighbors[j] = 0;
                    q->count--;
                    for( k = 0; k < 4; k++ )
                        if( q0->neighbors[k] == q )
                        {
                            q0->neighbors[k] = 0;
                            q0->count--;
                            break;
                        }
                    break;
                }
            }
        }

		// remove the quad by copying th last quad in the list into its place
        quad_group[min_box_area_index] = quad_group[quad_count-1];
        centers[min_box_area_index] = centers[quad_count-1];
    }

    icvCleanFoundConnectedQuads_EXIT:

    cvReleaseMemStorage( &temp_storage );
    cvFree( &centers );

    return quad_count;
}



//===========================================================================
// FIND CONNECTED QUADS
//===========================================================================
/*
	Function Name:      icvFindConnectedQuads
	Function Function : Find those quads connected with each other
	Input             : 
	    quad          : All quads generated by icvGenerateQuads()
	    quad_count    : Detected quad num
	    out_group     : Detected quad group
	    group_idx     : Label the quad group with an index
	Return            : 
	    quad_count    : The quad count of the group
	Note              :
	Revision History  : Author         Data             Changes

*/
int chessBoard_corner_detector::icvFindConnectedQuads( CvCBQuad *quad, int quad_count, CvCBQuad **out_group,
                       int group_idx, CvMemStorage* storage, int dilation )
{
//START TIMER
#if TIMER
	ofstream FindConnectedQuads;
    time_t  start_time = clock();
#endif

	// initializations
    CvMemStorage* temp_storage = cvCreateChildMemStorage( storage );
    CvSeq* stack = cvCreateSeq( 0, sizeof(*stack), sizeof(void*), temp_storage );
	int i, count = 0;


    // Scan the array for a first unlabeled quad
    for( i = 0; i < quad_count; i++ )
    {
        if( quad[i].count > 0 && quad[i].group_idx < 0)
            break;
    }


    // Recursively find a group of connected quads starting from the seed
	// quad[i]
    if( i < quad_count )
    {
        CvCBQuad* q = &quad[i];
        cvSeqPush( stack, &q );
        out_group[count++] = q;
        q->group_idx = group_idx;

        while( stack->total )
        {
            cvSeqPop( stack, &q );
            for( i = 0; i < 4; i++ )
            {
                CvCBQuad *neighbor = q->neighbors[i];


				// If he neighbor exists and the neighbor has more than 0 
				// neighbors and the neighbor has not been classified yet.
                if( neighbor && neighbor->count > 0 && neighbor->group_idx < 0 )
                {
                    cvSeqPush( stack, &neighbor );
                    out_group[count++] = neighbor;
                    neighbor->group_idx = group_idx;
                }
            }
        }
    }

    cvReleaseMemStorage( &temp_storage );
	
// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindConnectedQuads.open("timer/FindConnectedQuads.txt", ofstream::app);
	FindConnectedQuads << "Time for cvFindConnectedQuads was " << time << " seconds." << endl;
	FindConnectedQuads.close();
#endif

    return count;
}



/*
	Function Name:      mrLabelQuadGroup
	Function Function : Label corners with row and column
	Input             : 
	         count    : Detected quad num
		quad_group    : Detected quad group
		pattern_size  : the size of pattern required
		firstRun      : Whether this is the first time running
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/16        modify change needneighbor flag
*/
void chessBoard_corner_detector::mrLabelQuadGroup( CvCBQuad **quad_group, int count, CvSize pattern_size, bool firstRun )
{
//START TIMER
#if TIMER
	ofstream LabelQuadGroup;
    time_t  start_time = clock();
#endif

	// If this is the first function call, a seed quad needs to be selected
	if (firstRun == true)
	{
		// Search for the (first) quad with the maximum number of neighbors
		// (usually 4). This will be our starting point.
		int max_id = -1;
		int max_number = -1;
		for(int i = 0; i < count; i++ )
		{
			CvCBQuad* q = quad_group[i];
			//printf("camid: %d  neighbors: %d %d %d %d\n", i, q->neighbors[0], q->neighbors[0], q->neighbors[0], q->neighbors[0]);
			if( q->count > max_number)
			{
				max_number = q->count;
				max_id = i;

				if (max_number == 4)
					break;
			}
		}


		// Mark the starting quad's (per definition) upper left corner with
		//(0,0) and then proceed clockwise
		// The following labeling sequence ensures a "right coordinate system"
		(quad_group[max_id])->labeled = true;

		(quad_group[max_id])->corners[0]->row = 0;
        (quad_group[max_id])->corners[0]->column = 0;
		(quad_group[max_id])->corners[1]->row = 0;
		(quad_group[max_id])->corners[1]->column = 1;
		(quad_group[max_id])->corners[2]->row = 1;
		(quad_group[max_id])->corners[2]->column = 1;
		(quad_group[max_id])->corners[3]->row = 1;
		(quad_group[max_id])->corners[3]->column = 0;
	}

	// Mark all other corners with their respective row and column
	bool flag_changed = true;
	while( flag_changed == true )
	{
		// First reset the flag to "false"
		flag_changed = false;

		// Go through all quads top down is faster, since unlabeled quads will
		// be inserted at the end of the list
		for( int i = (count-1); i >= 0; i-- )
		{
			// Check whether quad "i" has been labeled already
 			if ( (quad_group[i])->labeled == false )
			{
				// Check its neighbors, whether some of them have been labeled
				// already
				for( int j = 0; j < 4; j++ )
				{
					// Check whether the neighbor exists (i.e. is not the NULL
					// pointer)
					if( (quad_group[i])->neighbors[j] )
					{
						CvCBQuad *quadNeighborJ = (quad_group[i])->neighbors[j];
						
						
						// Only proceed, if neighbor "j" was labeled
						if( quadNeighborJ->labeled == true)
						{
							// For every quad it could happen to pass here 
							// multiple times. We therefore "break" later.
							// Check which of the neighbors corners is 
							// connected to the current quad
							int connectedNeighborCornerId = -1;
							for( int k = 0; k < 4; k++)
							{
								if( quadNeighborJ->neighbors[k] == quad_group[i] )
								{
									connectedNeighborCornerId = k;
									
									
									// there is only one, therefore
									break;
								}
							}


							// For the following calculations we need the row 
							// and column of the connected neighbor corner and 
							// all other corners of the connected quad "j", 
							// clockwise (CW)
							CvCBCorner *conCorner	 = quadNeighborJ->corners[connectedNeighborCornerId];
							CvCBCorner *conCornerCW1 = quadNeighborJ->corners[(connectedNeighborCornerId+1)%4];
							CvCBCorner *conCornerCW2 = quadNeighborJ->corners[(connectedNeighborCornerId+2)%4];
							CvCBCorner *conCornerCW3 = quadNeighborJ->corners[(connectedNeighborCornerId+3)%4];
							
							(quad_group[i])->corners[j]->row			=	conCorner->row;
							(quad_group[i])->corners[j]->column			=	conCorner->column;
							(quad_group[i])->corners[(j+1)%4]->row		=	conCorner->row - conCornerCW2->row + conCornerCW3->row;
							(quad_group[i])->corners[(j+1)%4]->column	=	conCorner->column - conCornerCW2->column + conCornerCW3->column;
							(quad_group[i])->corners[(j+2)%4]->row		=	conCorner->row + conCorner->row - conCornerCW2->row;
							(quad_group[i])->corners[(j+2)%4]->column	=	conCorner->column + conCorner->column - conCornerCW2->column;
							(quad_group[i])->corners[(j+3)%4]->row		=	conCorner->row - conCornerCW2->row + conCornerCW1->row;
							(quad_group[i])->corners[(j+3)%4]->column	=	conCorner->column - conCornerCW2->column + conCornerCW1->column;
							

							// Mark this quad as labeled
							(quad_group[i])->labeled = true;
							

							// Changes have taken place, set the flag
							flag_changed = true;


							// once is enough!
							break;
						}
					}
				}
			}
		}
	}


	// All corners are marked with row and column
	// Record the minimal and maximal row and column indices
	// It is unlikely that more than 8bit checkers are used per dimension, if there are
	// an error would have been thrown at the beginning of "cvFindChessboardCorners2"
	int min_row		=  127;
	int max_row		= -127;
	int min_column	=  127;
	int max_column	= -127;

	for(int i = 0; i < count; i++ )
    {
		CvCBQuad* q = quad_group[i];
		
		for(int j = 0; j < 4; j++ )
		{
			if( (q->corners[j])->row > max_row)
				max_row = (q->corners[j])->row;

			if( (q->corners[j])->row < min_row)
				min_row = (q->corners[j])->row;

			if( (q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;

			if( (q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}

	// Label all external corners with "needsNeighbor" = true,
	// except if in a given dimension the pattern size is reached
	for(int i = min_row; i <= max_row; i++)
	{
		for(int j = min_column; j <= max_column; j++)
		{
			// A flag that indicates, whether a row/column combination is
			// executed multiple times
			bool flagg = false;


			// Remember corner and quad
			int cornerID;
			int quadID;

			for(int k = 0; k < count; k++)
			{
				for(int l = 0; l < 4; l++)
				{
					if( ((quad_group[k])->corners[l]->row == i) && ((quad_group[k])->corners[l]->column == j) )
					{

						if (flagg == true)
						{
							// Passed at least twice through here
							(quad_group[k])->corners[l]->needsNeighbor = true;
							(quad_group[quadID])->corners[cornerID]->needsNeighbor = true;
						}
						else
						{
							// Mark with needs a neighbor, but note the
							// address
							(quad_group[k])->corners[l]->needsNeighbor = false;
							cornerID = l;
							quadID = k;
						}


						// set the flag to true
						flagg = true;
					}
				}
			}
		}
	}

	// Corner bilinear interpolation , commented by YanShuo
	// Complete Linking:
	// sometimes not all corners were properly linked in "mrFindQuadNeighbors2",
	// but after labeling each corner with its respective row and column, it is 
	// possible to match them anyway.
	for(int i = min_row; i <= max_row; i++)
	{
		for(int j = min_column; j <= max_column; j++)
		{
			// the following "number" indicates the number of corners which 
			// correspond to the given (i,j) value
			// 1	is a border corner or a conrer which still needs a neighbor
			// 2	is a fully connected internal corner
			// >2	something went wrong during labeling, report a warning
			int number = 1;


			// remember corner and quad
			int cornerID;
			int quadID;

			for(int k = 0; k < count; k++)
			{
				for(int l = 0; l < 4; l++)
				{
					if( ((quad_group[k])->corners[l]->row == i) && ((quad_group[k])->corners[l]->column == j) )
					{

						if (number == 1)
						{
							// First corner, note its ID
							cornerID = l;
							quadID = k;
						}

						else if (number == 2)
						{
							// Second corner, check whether this and the 
							// first one have equal coordinates, else 
							// interpolate
							float delta_x = (quad_group[k])->corners[l]->pt.x - (quad_group[quadID])->corners[cornerID]->pt.x;
							float delta_y = (quad_group[k])->corners[l]->pt.y - (quad_group[quadID])->corners[cornerID]->pt.y;

							if (delta_x != 0 || delta_y != 0)
							{
								// Interpolate
								(quad_group[k])->corners[l]->pt.x = (quad_group[k])->corners[l]->pt.x - delta_x/2;
								(quad_group[quadID])->corners[cornerID]->pt.x = (quad_group[quadID])->corners[cornerID]->pt.x + delta_x/2;
								(quad_group[k])->corners[l]->pt.y = (quad_group[k])->corners[l]->pt.y - delta_y/2;
								(quad_group[quadID])->corners[cornerID]->pt.y = (quad_group[quadID])->corners[cornerID]->pt.y + delta_y/2;
							}
						}
						else if (number > 2)
						{
							// Something went wrong during row/column labeling
							// Report a Warning
							// ->Implemented in the function "mrWriteCorners"
						}

						// increase the number by one
						number = number + 1;
					}
				}
			}
		}
	}


	// Border corners don't need any neighbors, if the pattern size in the 
	// respective direction is reached
	// The only time we can make sure that the target pattern size is reached in a given
	// dimension, is when the larger side has reached the target size in the maximal
	// direction, or if the larger side is larger than the smaller target size and the 
	// smaller side equals the smaller target size
	int largerDimPattern = max(pattern_size.height,pattern_size.width);
	int smallerDimPattern = min(pattern_size.height,pattern_size.width);
	bool flagSmallerDim1 = false;
	bool flagSmallerDim2 = false;

	if((largerDimPattern + 1) == max_column - min_column)
	{
		flagSmallerDim1 = true;
		// We found out that in the column direction the target pattern size is reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for( int k = 0; k < count; k++ )
		{
			for( int l = 0; l < 4; l++ )
			{
				if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}		
	}

	if((largerDimPattern + 1) == max_row - min_row)
	{
		flagSmallerDim2 = true;
		// We found out that in the column direction the target pattern size is reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for( int k = 0; k < count; k++ )
		{
			for( int l = 0; l < 4; l++ )
			{
				if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}		
	}


	// Check the two flags: 
	//	-	If one is true and the other false, then the pattern target 
	//		size was reached in in one direction -> We can check, whether the target 
	//		pattern size is also reached in the other direction
	//  -	If both are set to true, then we deal with a square board -> do nothing
	//  -	If both are set to false -> There is a possibility that the larger side is
	//		larger than the smaller target size -> Check and if true, then check whether
	//		the other side has the same size as the smaller target size
	if( (flagSmallerDim1 == false && flagSmallerDim2 == true) )
	{
		// Larger target pattern size is in row direction, check wheter smaller target
		// pattern size is reached in column direction
		if((smallerDimPattern + 1) == max_column - min_column)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == true && flagSmallerDim2 == false) )
	{
		// Larger target pattern size is in column direction, check wheter smaller target
		// pattern size is reached in row direction
		if((smallerDimPattern + 1) == max_row - min_row)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_column - min_column )
	{
		// Larger target pattern size is in column direction, check whether smaller target
		// pattern size is reached in row direction
		if((smallerDimPattern + 1) == max_row - min_row)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->row == min_row || (quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if( (flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_row - min_row )
	{
		// Larger target pattern size is in row direction, check whether smaller target
		// pattern size is reached in column direction
		if((smallerDimPattern + 1) == max_column - min_column)
		{
			for( int k = 0; k < count; k++ )
			{
				for( int l = 0; l < 4; l++ )
				{
					if ( (quad_group[k])->corners[l]->column == min_column || (quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	LabelQuadGroup.open("timer/LabelQuadGroup.txt", ofstream::app);
	LabelQuadGroup << "Time for mrLabelQuadGroup was " << time << " seconds." << endl;
	LabelQuadGroup.close();
#endif

}


/*
	Function Name:      mrCopyQuadGroup
	Function Function : Copies all necessary information of every quad of the largest found group into a new Quad struct array. 
	Input             : 
	  temp_quad_group : source quad group
   for_out_quad_group : The output quad group
	     	    count : the valid quad num
	Return            : Void
	Note              : This information is then again needed in PART 2 of the MAIN LOOP
	Revision History  : Author         Data             Changes
*/
void chessBoard_corner_detector::mrCopyQuadGroup( CvCBQuad **temp_quad_group, CvCBQuad **for_out_quad_group, int count )
{
	for (int i = 0; i < count; i++)
	{
		for_out_quad_group[i]				= new CvCBQuad;
		for_out_quad_group[i]->count		= temp_quad_group[i]->count;
		for_out_quad_group[i]->min_edge_len = temp_quad_group[i]->min_edge_len;
		for_out_quad_group[i]->max_edge_len = temp_quad_group[i]->max_edge_len;
		for_out_quad_group[i]->group_idx	= temp_quad_group[i]->group_idx;
		for_out_quad_group[i]->labeled		= temp_quad_group[i]->labeled;
		
		for (int j = 0; j < 4; j++)
		{
			for_out_quad_group[i]->corners[j]					= new CvCBCorner;
			for_out_quad_group[i]->corners[j]->pt.x				= temp_quad_group[i]->corners[j]->pt.x;
			for_out_quad_group[i]->corners[j]->pt.y				= temp_quad_group[i]->corners[j]->pt.y;
			for_out_quad_group[i]->corners[j]->row				= temp_quad_group[i]->corners[j]->row;
			for_out_quad_group[i]->corners[j]->column			= temp_quad_group[i]->corners[j]->column;
			for_out_quad_group[i]->corners[j]->needsNeighbor	= temp_quad_group[i]->corners[j]->needsNeighbor;
			for_out_quad_group[i]->neighbors[j]                 = temp_quad_group[i]->neighbors[j];
			for_out_quad_group[i]->corners[j]->connected_way    = temp_quad_group[i]->corners[j]->connected_way;
			for_out_quad_group[i]->corners[j]->max_edge_len_ratio = temp_quad_group[i]->corners[j]->max_edge_len_ratio;
			for_out_quad_group[i]->corners[j]->min_edge_len_ratio = temp_quad_group[i]->corners[j]->min_edge_len_ratio;
		}
		//printf("quadid: %d needsNeighbors: %d %d %d %d \n", i, for_out_quad_group[i]->corners[0]->needsNeighbor, for_out_quad_group[i]->corners[1]->needsNeighbor, 
		//	for_out_quad_group[i]->corners[2]->needsNeighbor, for_out_quad_group[i]->corners[3]->needsNeighbor);
	}
}



//===========================================================================
// GIVE A GROUP IDX
//===========================================================================
// This function replaces mrFindQuadNeighbors, which in turn replaced
// icvFindQuadNeighbors
/*
	Function Name:      mrFindQuadNeighbors2
	Function Function : find neighbor quads of each quad and wirte neighbor information to each quad's neighbor structure
	Input             : 
	    quads         : All quads generated by icvGenerateQuads()
	    quad_count    : Detected quad num
	    dilation      :
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
*/
void chessBoard_corner_detector::mrFindQuadNeighbors2( CvCBQuad *quads, int quad_count, int dilation)
{
//START TIMER
#if TIMER
	ofstream FindQuadNeighbors2;
    time_t  start_time = clock();
#endif

	// Thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is 
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	const float thresh_dilation = (float)(2*dilation+3)*(2*dilation+3)*2;	// the "*2" is for the x and y component
    int idx, i, k, j;														// the "3" is for initial corner mismatch
    float dx, dy, dist;
	int cur_quad_group = -1;


    // Find quad neighbors
	for( idx = 0; idx < quad_count; idx++ )
	{
		CvCBQuad* cur_quad = &quads[idx];


		// Go through all quadrangles and label them in groups
		// For each corner of this quadrangle
		for( i = 0; i < 4; i++ )
		{
			CvPoint2D32f pt;
			float min_dist = FLT_MAX;
			int closest_corner_idx = -1;
			CvCBQuad *closest_quad = 0;
			CvCBCorner *closest_corner = 0;

			if( cur_quad->neighbors[i] )
				continue;

			pt = cur_quad->corners[i]->pt;

			j = (i + 2) % 4;

			// Find the closest corner in all other quadrangles
			for( k = 0; k < quad_count; k++ )
			{
				if( k == idx )
					continue;

				// If it already has a neighbor
				if( quads[k].neighbors[j] )
					continue;

				if (quads[k].neighbors[(j + 1) % 4] == &quads[idx]
					|| quads[k].neighbors[(j + 2) % 4] == &quads[idx]
					|| quads[k].neighbors[(j + 3) % 4] == &quads[idx])
					continue;

				dx = pt.x - quads[k].corners[j]->pt.x;
				dy = pt.y - quads[k].corners[j]->pt.y;
				dist = dx * dx + dy * dy;



				// The following "if" checks, whether "dist" is the
				// shortest so far and smaller than the smallest
				// edge length of the current and target quads
				if( dist < min_dist && 
					dist <= (cur_quad->min_edge_len + thresh_dilation) &&
					dist <= (quads[k].min_edge_len + thresh_dilation)    )
				{
					// First Check everything from the viewpoint of the current quad
					// compute midpoints of "parallel" quad sides 1
					float x1 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+1)%4]->pt.x)/2;
					float y1 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+1)%4]->pt.y)/2;				
					float x2 = (cur_quad->corners[(i+2)%4]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
					float y2 = (cur_quad->corners[(i+2)%4]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;	
					// compute midpoints of "parallel" quad sides 2
					float x3 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
					float y3 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;				
					float x4 = (cur_quad->corners[(i+1)%4]->pt.x + cur_quad->corners[(i+2)%4]->pt.x)/2;
					float y4 = (cur_quad->corners[(i+1)%4]->pt.y + cur_quad->corners[(i+2)%4]->pt.y)/2;	


					// MARTIN: Heuristic
					// For the corner "j" of quad "k" to be considered, 
					// it needs to be on the same side of the two lines as 
					// corner "i". This is given, if the cross product has 
					// the same sign for both computations below:
					float a1 = x1 - x2;
					float b1 = y1 - y2;
					// the current corner
					float c11 = cur_quad->corners[i]->pt.x - x2;
					float d11 = cur_quad->corners[i]->pt.y - y2;
					// the candidate corner
					float c12 = quads[k].corners[j]->pt.x - x2;
					float d12 = quads[k].corners[j]->pt.y - y2;
					float sign11 = a1*d11 - c11*b1;
					float sign12 = a1*d12 - c12*b1;

					float a2 = x3 - x4;
					float b2 = y3 - y4;
					// the current corner
					float c21 = cur_quad->corners[i]->pt.x - x4;
					float d21 = cur_quad->corners[i]->pt.y - y4;
					// the candidate corner
					float c22 = quads[k].corners[j]->pt.x - x4;
					float d22 = quads[k].corners[j]->pt.y - y4;
					float sign21 = a2*d21 - c21*b2;
					float sign22 = a2*d22 - c22*b2;



					// Then make shure that two border quads of the same row or
					// column don't link. Check from the current corner's view,
					// whether the corner diagonal from the candidate corner
					// is also on the same side of the two lines as the current
					// corner and the candidate corner.
					float c13 = quads[k].corners[(j+2)%4]->pt.x - x2;
					float d13 = quads[k].corners[(j+2)%4]->pt.y - y2;
					float c23 = quads[k].corners[(j+2)%4]->pt.x - x4;
					float d23 = quads[k].corners[(j+2)%4]->pt.y - y4;
					float sign13 = a1*d13 - c13*b1;
					float sign23 = a2*d23 - c23*b2;

					// Then check everything from the viewpoint of the candidate quad
					// compute midpoints of "parallel" quad sides 1
					float u1 = (quads[k].corners[j]->pt.x + quads[k].corners[(j+1)%4]->pt.x)/2;
					float v1 = (quads[k].corners[j]->pt.y + quads[k].corners[(j+1)%4]->pt.y)/2;				
					float u2 = (quads[k].corners[(j+2)%4]->pt.x + quads[k].corners[(j+3)%4]->pt.x)/2;
					float v2 = (quads[k].corners[(j+2)%4]->pt.y + quads[k].corners[(j+3)%4]->pt.y)/2;	
					// compute midpoints of "parallel" quad sides 2
					float u3 = (quads[k].corners[j]->pt.x + quads[k].corners[(j+3)%4]->pt.x)/2;
					float v3 = (quads[k].corners[j]->pt.y + quads[k].corners[(j+3)%4]->pt.y)/2;				
					float u4 = (quads[k].corners[(j+1)%4]->pt.x + quads[k].corners[(j+2)%4]->pt.x)/2;
					float v4 = (quads[k].corners[(j+1)%4]->pt.y + quads[k].corners[(j+2)%4]->pt.y)/2;	

					// MARTIN: Heuristic
					// for the corner "j" of quad "k" to be considered, it 
					// needs to be on the same side of the two lines as 
					// corner "i". This is again given, if the cross
					//product has the same sign for both computations below:
					float a3 = u1 - u2;
					float b3 = v1 - v2;
					// the current corner
					float c31 = cur_quad->corners[i]->pt.x - u2;
					float d31 = cur_quad->corners[i]->pt.y - v2;
					// the candidate corner
					float c32 = quads[k].corners[j]->pt.x - u2;
					float d32 = quads[k].corners[j]->pt.y - v2;
					float sign31 = a3*d31-c31*b3;
					float sign32 = a3*d32-c32*b3;


					float a4 = u3 - u4;
					float b4 = v3 - v4;
					// the current corner
					float c41 = cur_quad->corners[i]->pt.x - u4;
					float d41 = cur_quad->corners[i]->pt.y - v4;
					// the candidate corner
					float c42 = quads[k].corners[j]->pt.x - u4;
					float d42 = quads[k].corners[j]->pt.y - v4;
					float sign41 = a4*d41-c41*b4;
					float sign42 = a4*d42-c42*b4;

					// Then make shure that two border quads of the same row or
					// column don't link. Check from the candidate corner's view,
					// whether the corner diagonal from the current corner
					// is also on the same side of the two lines as the current
					// corner and the candidate corner.
					float c33 = cur_quad->corners[(i+2)%4]->pt.x - u2;
					float d33 = cur_quad->corners[(i+2)%4]->pt.y - v2;
					float c43 = cur_quad->corners[(i+2)%4]->pt.x - u4;
					float d43 = cur_quad->corners[(i+2)%4]->pt.y - v4;
					float sign33 = a3*d33-c33*b3;
					float sign43 = a4*d43-c43*b4;


					// Check whether conditions are fulfilled
					if ( ((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0))  && 
						((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0))  &&
						((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0))  &&   
						((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0))  &&
						((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0))  &&   
						((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0))  &&
						((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0))  &&   
						((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0))    )

					{
						closest_corner_idx = j;
						closest_quad = &quads[k];
						min_dist = dist;
					}
				}
			}

            // Have we found a matching corner point?
            if( closest_corner_idx >= 0 && min_dist < FLT_MAX )
            {
                closest_corner = closest_quad->corners[closest_corner_idx];


                // Make sure that the closest quad does not have the current
				// quad as neighbor already
                for( j = 0; j < 4; j++ )
                {
                    if( closest_quad->neighbors[j] == cur_quad )
                        break;
                }
                if( j < 4 )
                    continue;


				// We've found one more corner - remember it
                closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
                closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;

                cur_quad->count++;
                cur_quad->neighbors[i] = closest_quad;
                cur_quad->corners[i] = closest_corner;

                closest_quad->count++;
                closest_quad->neighbors[closest_corner_idx] = cur_quad;
				closest_quad->corners[closest_corner_idx] = closest_corner;

				if (i == 0 || i == 2)
				{
					cur_quad->corners[i]->connected_way = 2;
					closest_quad->corners[closest_corner_idx]->connected_way = 2;
				}
				else
				{
					cur_quad->corners[i]->connected_way = 3;
					closest_quad->corners[closest_corner_idx]->connected_way = 3;
				}

				if (closest_quad->max_edge_len > cur_quad->max_edge_len)
				{
					cur_quad->corners[i]->max_edge_len_ratio
						= sqrt(cur_quad->max_edge_len) / sqrt(closest_quad->max_edge_len);
					closest_quad->corners[closest_corner_idx]->max_edge_len_ratio
						= sqrt(cur_quad->max_edge_len) / sqrt(closest_quad->max_edge_len);
				}
				else
				{
					cur_quad->corners[i]->max_edge_len_ratio
						= sqrt(closest_quad->max_edge_len) / sqrt(cur_quad->max_edge_len);
					closest_quad->corners[closest_corner_idx]->max_edge_len_ratio
						= sqrt(closest_quad->max_edge_len) / sqrt(cur_quad->max_edge_len);
				}

				if (closest_quad->min_edge_len > cur_quad->min_edge_len)
				{
					cur_quad->corners[i]->min_edge_len_ratio
						= sqrt(cur_quad->min_edge_len) / sqrt(closest_quad->min_edge_len);
					closest_quad->corners[closest_corner_idx]->min_edge_len_ratio
						= sqrt(cur_quad->min_edge_len) / sqrt(closest_quad->min_edge_len);
				}
				else
				{
					cur_quad->corners[i]->min_edge_len_ratio
						= sqrt(closest_quad->min_edge_len) / sqrt(cur_quad->min_edge_len);
					closest_quad->corners[closest_corner_idx]->max_edge_len_ratio
						= sqrt(closest_quad->min_edge_len) / sqrt(cur_quad->min_edge_len);
				}
            }
			else
			{
				cur_quad->corners[i]->connected_way = -1;
				cur_quad->corners[i]->max_edge_len_ratio = 0;
				cur_quad->corners[i]->min_edge_len_ratio = 0;
			}
        }
    }

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	FindQuadNeighbors2.open("timer/FindQuadNeighbors2.txt", ofstream::app);
	FindQuadNeighbors2 << "Time for mrFindQuadNeighbors2 was " << time << " seconds." << endl;
	FindQuadNeighbors2.close();
#endif
}



//===========================================================================
// AUGMENT PATTERN WITH ADDITIONAL QUADS
//===========================================================================
// The first part of the function is basically a copy of 
// "mrFindQuadNeighbors2"
// The comparisons between two points and two lines could be computed in their
// own function
int chessBoard_corner_detector::mrAugmentBestRun( CvCBQuad *new_quads, int new_quad_count, int new_dilation, 
							  CvCBQuad **old_quads, int old_quad_count, int old_dilation )
{
//START TIMER
#if TIMER
	ofstream AugmentBestRun;
    time_t  start_time = clock();
#endif

	// thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is 
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	const float thresh_dilation = (float)(2*new_dilation+3)*(2*old_dilation+3)*2;	// the "*2" is for the x and y component
    int idx, i, k, j;																// the "3" is for initial corner mismatch
    float dx, dy, dist;
	

    // Search all old quads which have a neighbor that needs to be linked
    for( idx = 0; idx < old_quad_count; idx++ )
    {
        CvCBQuad* cur_quad = old_quads[idx];


        // For each corner of this quadrangle
        for( i = 0; i < 4; i++ )
        {
            CvPoint2D32f pt;
            float min_dist = FLT_MAX;
            int closest_corner_idx = -1;
            CvCBQuad *closest_quad = 0;
            CvCBCorner *closest_corner = 0;


			// If cur_quad corner[i] is already linked, continue
            if( cur_quad->corners[i]->needsNeighbor == false )
                continue;

            pt = cur_quad->corners[i]->pt;


            // Look for a match in all new_quads' corners
            for( k = 0; k < new_quad_count; k++ )
            {
				// Only look at unlabeled new quads
				if( new_quads[k].labeled == true)
					continue;

                for( j = 0; j < 4; j++ )
                {

					// Only proceed if they are less than dist away from each
					// other
                    dx = pt.x - new_quads[k].corners[j]->pt.x;
                    dy = pt.y - new_quads[k].corners[j]->pt.y;
                    dist = dx * dx + dy * dy;

                    if( (dist < min_dist) && 
						dist <= (cur_quad->min_edge_len + thresh_dilation) &&
                        dist <= (new_quads[k].min_edge_len + thresh_dilation) )
                    {
						// First Check everything from the viewpoint of the 
						// current quad compute midpoints of "parallel" quad 
						// sides 1
						float x1 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+1)%4]->pt.x)/2;
						float y1 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+1)%4]->pt.y)/2;				
						float x2 = (cur_quad->corners[(i+2)%4]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y2 = (cur_quad->corners[(i+2)%4]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;	
						// compute midpoints of "parallel" quad sides 2
						float x3 = (cur_quad->corners[i]->pt.x + cur_quad->corners[(i+3)%4]->pt.x)/2;
						float y3 = (cur_quad->corners[i]->pt.y + cur_quad->corners[(i+3)%4]->pt.y)/2;				
						float x4 = (cur_quad->corners[(i+1)%4]->pt.x + cur_quad->corners[(i+2)%4]->pt.x)/2;
						float y4 = (cur_quad->corners[(i+1)%4]->pt.y + cur_quad->corners[(i+2)%4]->pt.y)/2;	
						
						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered, 
						// it needs to be on the same side of the two lines as 
						// corner "i". This is given, if the cross product has 
						// the same sign for both computations below:
						float a1 = x1 - x2;
						float b1 = y1 - y2;
						// the current corner
						float c11 = cur_quad->corners[i]->pt.x - x2;
						float d11 = cur_quad->corners[i]->pt.y - y2;
						// the candidate corner
						float c12 = new_quads[k].corners[j]->pt.x - x2;
						float d12 = new_quads[k].corners[j]->pt.y - y2;
						float sign11 = a1*d11 - c11*b1;
						float sign12 = a1*d12 - c12*b1;

						float a2 = x3 - x4;
						float b2 = y3 - y4;
						// the current corner
						float c21 = cur_quad->corners[i]->pt.x - x4;
						float d21 = cur_quad->corners[i]->pt.y - y4;
						// the candidate corner
						float c22 = new_quads[k].corners[j]->pt.x - x4;
						float d22 = new_quads[k].corners[j]->pt.y - y4;
						float sign21 = a2*d21 - c21*b2;
						float sign22 = a2*d22 - c22*b2;

						// Also make shure that two border quads of the same row or
						// column don't link. Check from the current corner's view,
						// whether the corner diagonal from the candidate corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c13 = new_quads[k].corners[(j+2)%4]->pt.x - x2;
						float d13 = new_quads[k].corners[(j+2)%4]->pt.y - y2;
						float c23 = new_quads[k].corners[(j+2)%4]->pt.x - x4;
						float d23 = new_quads[k].corners[(j+2)%4]->pt.y - y4;
						float sign13 = a1*d13 - c13*b1;
						float sign23 = a2*d23 - c23*b2;


						// Second: Then check everything from the viewpoint of
						// the candidate quad. Compute midpoints of "parallel"
						// quad sides 1
						float u1 = (new_quads[k].corners[j]->pt.x + new_quads[k].corners[(j+1)%4]->pt.x)/2;
						float v1 = (new_quads[k].corners[j]->pt.y + new_quads[k].corners[(j+1)%4]->pt.y)/2;				
						float u2 = (new_quads[k].corners[(j+2)%4]->pt.x + new_quads[k].corners[(j+3)%4]->pt.x)/2;
						float v2 = (new_quads[k].corners[(j+2)%4]->pt.y + new_quads[k].corners[(j+3)%4]->pt.y)/2;	
						// compute midpoints of "parallel" quad sides 2
						float u3 = (new_quads[k].corners[j]->pt.x + new_quads[k].corners[(j+3)%4]->pt.x)/2;
						float v3 = (new_quads[k].corners[j]->pt.y + new_quads[k].corners[(j+3)%4]->pt.y)/2;				
						float u4 = (new_quads[k].corners[(j+1)%4]->pt.x + new_quads[k].corners[(j+2)%4]->pt.x)/2;
						float v4 = (new_quads[k].corners[(j+1)%4]->pt.y + new_quads[k].corners[(j+2)%4]->pt.y)/2;	
						
						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered, 
						// it needs to be on the same side of the two lines as 
						// corner "i". This is given, if the cross product has 
						// the same sign for both computations below:
						float a3 = u1 - u2;
						float b3 = v1 - v2;
						// the current corner
						float c31 = cur_quad->corners[i]->pt.x - u2;
						float d31 = cur_quad->corners[i]->pt.y - v2;
						// the candidate corner
						float c32 = new_quads[k].corners[j]->pt.x - u2;
						float d32 = new_quads[k].corners[j]->pt.y - v2;
						float sign31 = a3*d31-c31*b3;
						float sign32 = a3*d32-c32*b3;

						float a4 = u3 - u4;
						float b4 = v3 - v4;
						// the current corner
						float c41 = cur_quad->corners[i]->pt.x - u4;
						float d41 = cur_quad->corners[i]->pt.y - v4;
						// the candidate corner
						float c42 = new_quads[k].corners[j]->pt.x - u4;
						float d42 = new_quads[k].corners[j]->pt.y - v4;
						float sign41 = a4*d41-c41*b4;
						float sign42 = a4*d42-c42*b4;

						// Also make shure that two border quads of the same row or
						// column don't link. Check from the candidate corner's view,
						// whether the corner diagonal from the current corner
						// is also on the same side of the two lines as the current
						// corner and the candidate corner.
						float c33 = cur_quad->corners[(i+2)%4]->pt.x - u2;
						float d33 = cur_quad->corners[(i+2)%4]->pt.y - v2;
						float c43 = cur_quad->corners[(i+2)%4]->pt.x - u4;
						float d43 = cur_quad->corners[(i+2)%4]->pt.y - v4;
						float sign33 = a3*d33-c33*b3;
						float sign43 = a4*d43-c43*b4;

						
						// This time we also need to make shure, that no quad
						// is linked to a quad of another dilation run which 
						// may lie INSIDE it!!!
						// Third: Therefore check everything from the viewpoint
						// of the current quad compute midpoints of "parallel" 
						// quad sides 1
						float x5 = cur_quad->corners[i]->pt.x;
						float y5 = cur_quad->corners[i]->pt.y;				
						float x6 = cur_quad->corners[(i+1)%4]->pt.x;
						float y6 = cur_quad->corners[(i+1)%4]->pt.y;	
						// compute midpoints of "parallel" quad sides 2
						float x7 = x5;
						float y7 = y5;				
						float x8 = cur_quad->corners[(i+3)%4]->pt.x;
						float y8 = cur_quad->corners[(i+3)%4]->pt.y;	
						
						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered, 
						// it needs to be on the other side of the two lines than 
						// corner "i". This is given, if the cross product has 
						// a different sign for both computations below:
						float a5 = x6 - x5;
						float b5 = y6 - y5;
						// the current corner
						float c51 = cur_quad->corners[(i+2)%4]->pt.x - x5;
						float d51 = cur_quad->corners[(i+2)%4]->pt.y - y5;
						// the candidate corner
						float c52 = new_quads[k].corners[j]->pt.x - x5;
						float d52 = new_quads[k].corners[j]->pt.y - y5;
						float sign51 = a5*d51 - c51*b5;
						float sign52 = a5*d52 - c52*b5;

						float a6 = x8 - x7;
						float b6 = y8 - y7;
						// the current corner
						float c61 = cur_quad->corners[(i+2)%4]->pt.x - x7;
						float d61 = cur_quad->corners[(i+2)%4]->pt.y - y7;
						// the candidate corner
						float c62 = new_quads[k].corners[j]->pt.x - x7;
						float d62 = new_quads[k].corners[j]->pt.y - y7;
						float sign61 = a6*d61 - c61*b6;
						float sign62 = a6*d62 - c62*b6;


						// Fourth: Then check everything from the viewpoint of 
						// the candidate quad compute midpoints of "parallel" 
						// quad sides 1
						float u5 = new_quads[k].corners[j]->pt.x;
						float v5 = new_quads[k].corners[j]->pt.y;				
						float u6 = new_quads[k].corners[(j+1)%4]->pt.x;
						float v6 = new_quads[k].corners[(j+1)%4]->pt.y;	
						// compute midpoints of "parallel" quad sides 2
						float u7 = u5;
						float v7 = v5;				
						float u8 = new_quads[k].corners[(j+3)%4]->pt.x;
						float v8 = new_quads[k].corners[(j+3)%4]->pt.y;	
						
						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered, 
						// it needs to be on the other side of the two lines than 
						// corner "i". This is given, if the cross product has 
						// a different sign for both computations below:
						float a7 = u6 - u5;
						float b7 = v6 - v5;
						// the current corner
						float c71 = cur_quad->corners[i]->pt.x - u5;
						float d71 = cur_quad->corners[i]->pt.y - v5;
						// the candidate corner
						float c72 = new_quads[k].corners[(j+2)%4]->pt.x - u5;
						float d72 = new_quads[k].corners[(j+2)%4]->pt.y - v5;
						float sign71 = a7*d71-c71*b7;
						float sign72 = a7*d72-c72*b7;

						float a8 = u8 - u7;
						float b8 = v8 - v7;
						// the current corner
						float c81 = cur_quad->corners[i]->pt.x - u7;
						float d81 = cur_quad->corners[i]->pt.y - v7;
						// the candidate corner
						float c82 = new_quads[k].corners[(j+2)%4]->pt.x - u7;
						float d82 = new_quads[k].corners[(j+2)%4]->pt.y - v7;
						float sign81 = a8*d81-c81*b8;
						float sign82 = a8*d82-c82*b8;





						// Check whether conditions are fulfilled
						if ( ((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0))  && 
							 ((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0))  &&
							 ((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0))  &&   
							 ((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0))	 &&	
							 ((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0))  &&   
							 ((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0))  &&
							 ((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0))  &&   
							 ((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0))  &&
							 ((sign51 < 0 && sign52 > 0) || (sign51 > 0 && sign52 < 0))  && 
							 ((sign61 < 0 && sign62 > 0) || (sign61 > 0 && sign62 < 0))  &&
							 ((sign71 < 0 && sign72 > 0) || (sign71 > 0 && sign72 < 0))  &&   
							 ((sign81 < 0 && sign82 > 0) || (sign81 > 0 && sign82 < 0)) )						
						{
							closest_corner_idx = j;
							closest_quad = &new_quads[k];
							min_dist = dist;
						}
                    }
                }
            }

            // Have we found a matching corner point?
            if( closest_corner_idx >= 0 && min_dist < FLT_MAX )
            {
                closest_corner = closest_quad->corners[closest_corner_idx];
                closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
                closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;


                // We've found one more corner - remember it
				// ATTENTION: write the corner x and y coordinates separately, 
				// else the crucial row/column entries will be overwritten !!!
                cur_quad->corners[i]->pt.x = closest_corner->pt.x;
				cur_quad->corners[i]->pt.y = closest_corner->pt.y;
				cur_quad->neighbors[i] = closest_quad;
				closest_quad->corners[closest_corner_idx]->pt.x = closest_corner->pt.x;
				closest_quad->corners[closest_corner_idx]->pt.y = closest_corner->pt.y;
				
				
				// Label closest quad as labeled. In this way we exclude it
				// being considered again during the next loop iteration
				closest_quad->labeled = true;


				// We have a new member of the final pattern, copy it over
				old_quads[old_quad_count]				= new CvCBQuad;
				old_quads[old_quad_count]->count		= 1;
				old_quads[old_quad_count]->min_edge_len = closest_quad->min_edge_len;
				old_quads[old_quad_count]->max_edge_len = closest_quad->max_edge_len;
				old_quads[old_quad_count]->group_idx	= cur_quad->group_idx;	//the same as the current quad
				old_quads[old_quad_count]->labeled		= false;				//do it right afterwards
				
				
				// We only know one neighbor for shure, initialize rest with 
				// the NULL pointer
				old_quads[old_quad_count]->neighbors[closest_corner_idx]		= cur_quad;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+1)%4]	= NULL;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+2)%4]	= NULL;
				old_quads[old_quad_count]->neighbors[(closest_corner_idx+3)%4]	= NULL;

				for (int j = 0; j < 4; j++)
				{
					old_quads[old_quad_count]->corners[j]					= new CvCBCorner;
					old_quads[old_quad_count]->corners[j]->pt.x				= closest_quad->corners[j]->pt.x;
					old_quads[old_quad_count]->corners[j]->pt.y				= closest_quad->corners[j]->pt.y;
				}

				cur_quad->neighbors[i] = old_quads[old_quad_count];


				// Start the function again
				return -1;
            }
        }
    }
	
// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	AugmentBestRun.open("timer/AugmentBestRun.txt", ofstream::app);
	AugmentBestRun << "Time for mrAugmentBestRun was " << time << " seconds." << endl;
	AugmentBestRun.close();
#endif

	// Finished, don't start the function again
	return 1;
}



/*
	Function Name:      icvGenerateQuads
	Function Function : generate quad based on binary image
	Input             : 
	    output_quads  : The detected quad group
		out_corners   : The detected corner structure
		storage       : 
		image         : binary image
		flags         : 
		dilation      :
        firstRun      : Whether this function is called for the first time in the dilation loop
	Return            : the found quad count
	Note              : 
	Revision History  : Author         Data             Changes
*/
int chessBoard_corner_detector::icvGenerateQuads( CvCBQuad **out_quads, CvCBCorner **out_corners,
	CvMemStorage *storage, IplImage *image, int flags, int dilation, bool firstRun )
{
//START TIMER
#if TIMER
	ofstream GenerateQuads;
    time_t  start_time = clock();
#endif

	// Initializations
    int quad_count = 0;
    CvMemStorage *temp_storage = 0;

    if( out_quads )
        *out_quads = 0;

    if( out_corners )
        *out_corners = 0;


    CvSeq *src_contour = 0;
    CvSeq *root;
    CvContourEx* board = 0;
    CvContourScanner scanner;
    int i, j, idx, min_size;

	assert(out_corners && out_quads );



    // Empiric sower bound for the size of allowable quadrangles.
    // MARTIN, modified: Added "*0.1" in order to find smaller quads.	
	min_size = (int)( image->width * image->height * 0.0003*0.5 + 0.5);


    // Create temporary storage for contours and the sequence of pointers to
	// found quadrangles
    temp_storage = cvCreateChildMemStorage( storage );
    root = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSeq*), temp_storage );


    // Initialize contour retrieving routine
    scanner = cvStartFindContours( image, temp_storage, sizeof(CvContourEx),
                                            CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
#ifdef DEBUG_RESULT_SAVE
	char file_name[200];
	sprintf(file_name, "%s/selected_corner_%dx%d.GREY", DEBUG_FILE_PATH, image->width, image->height);
	FILE* fp_tmp = fopen(file_name, "wb");
	uchar* write_img = (uchar*)malloc(sizeof(uchar) * temp_img->height * temp_img->width);
	memset(write_img, 0, sizeof(uchar) * temp_img->height * temp_img->width);
#endif

    // Get all the contours one by one
    while( (src_contour = cvFindNextContour( scanner )) != 0 )
    {
        CvSeq *dst_contour = 0;
        CvRect rect = ((CvContour*)src_contour)->rect;
			
		// Reject contours with a too small perimeter and contours which are 
		// completely surrounded by another contour
		// MARTIN: If this function is called during PART 1, then the parameter "first run"
		// is set to "true". This guarantees, that only "nice" squares are detected.
		// During PART 2, we allow the polygonial approximation function below to
		// approximate more freely, which can result in recognized "squares" that are in
		// reality multiple blurred and sticked together squares.
        if( CV_IS_SEQ_HOLE(src_contour) && rect.width*rect.height >= min_size )
        {
            int min_approx_level = 2, max_approx_level;
			if (firstRun == true)
				max_approx_level = 4;
			else
				max_approx_level = MAX_CONTOUR_APPROX;
            int approx_level;

			double area_src = 0;
			double p_src = 0;
            for( approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++ )
            {
				p_src = cvArcLength(src_contour, CV_WHOLE_SEQ, 1);
				area_src = fabs(cvContourArea(src_contour, CV_WHOLE_SEQ));

                dst_contour = cvApproxPoly( src_contour, sizeof(CvContour), temp_storage,
                                            CV_POLY_APPROX_DP, (float)approx_level );
                
				
				// We call this again on its own output, because sometimes
                // cvApproxPoly() does not simplify as much as it should.
				dst_contour = cvApproxPoly(dst_contour, sizeof(CvContour), temp_storage,
					CV_POLY_APPROX_DP, (float)approx_level);

				if (dst_contour->total == 4)
					break;
				else
					area_src = 0;
             }

            // Reject non-quadrangles
            if(dst_contour->total == 4 && cvCheckContourConvexity(dst_contour) )
            {
				CvPoint pt[4];
                double /*d1, d2,*/ p = cvArcLength(dst_contour, CV_WHOLE_SEQ, 1);//length
                double area = fabs(cvContourArea(dst_contour, CV_WHOLE_SEQ));//area
               /* double dx, dy;*/

                for( i = 0; i < 4; i++ )
                    pt[i] = *(CvPoint*)cvGetSeqElem(dst_contour, i);

				float64_t rect_center[2];
				rect_center[0] =  rect.x + (rect.width >> 1);
				rect_center[1] =  rect.y + (rect.height >> 1);

				float64_t w_div_h_ceil = 7.0, w_div_h_floor = 0.2;
				if(rect_center[0] < temp_img->width * 0.3 || rect_center[0] > temp_img->width * 0.7)
				{
					w_div_h_ceil *= 0.75;
					w_div_h_floor *= 1.5;
				}

				bool is_valid;
				switch (station_type)
				{
				case STATION_TYPE_JAC:
					is_valid = (rect.height > 5 && rect.width > 5 && rect.width * rect.height > 150);
					break;
				case STATION_TYPE_OFILM:
				case STATION_TYPE_GAC:
					is_valid =
						(rect.height > 6 && rect.width > 6 // rectangle edge length selection
						&& rect.width / (float64_t)rect.height < w_div_h_ceil && rect.width / (float64_t)rect.height > w_div_h_floor // rectangle edge ratio selection
						&& rect_center[1] > temp_img->height * 0.25 && rect_center[1] < temp_img->height * 0.9 // rectangle center y direction pos selection
						&& rect_center[0] > temp_img->width * 0.06 && rect_center[0] < temp_img->width * 0.94 // rectangle center x direction pos selection					
						|| (corner_config.board_size.height * corner_config.board_size.width == 1));
					break;
				default:
					break;
				}

				if (is_valid)
                {
					CvContourEx* parent = (CvContourEx*)(src_contour->v_prev);
					parent->counter++;
					if( !board || board->counter < parent->counter )
						board = parent;
					dst_contour->v_prev = (CvSeq*)parent;
					cvSeqPush( root, &dst_contour );
#ifdef DEBUG_RESULT_SAVE

					for (int i = rect.y; i < rect.y + rect.height; i++)
					{
						write_img[i * temp_img->width + rect.x] = 128;
						write_img[i * temp_img->width + rect.x + rect.width] = 128;
					}
					for (int j = rect.x; j < rect.x + rect.width; j++)
					{
						write_img[rect.y * temp_img->width + j] = 128;
						write_img[(rect.y + rect.height) * temp_img->width + j] = 128;

					}
#endif
                }
            }
        }
    }

#ifdef DEBUG_RESULT_SAVE
	fwrite(write_img, sizeof(uchar) * temp_img->height * temp_img->width, 1, fp_tmp);
	fclose(fp_tmp);
#endif
    // Finish contour retrieving
    cvEndFindContours( &scanner );


    // Allocate quad & corner buffers
    *out_quads = (CvCBQuad*)cvAlloc(root->total * sizeof((*out_quads)[0]));
    *out_corners = (CvCBCorner*)cvAlloc(root->total * 4 * sizeof((*out_corners)[0]));
	if (out_quads == NULL || out_corners == NULL)
	{
		if( out_quads )
            cvFree( out_quads );
        if( out_corners )
            cvFree( out_corners );
        quad_count = 0;
		return quad_count;
	}


    // Create array of quads structures
    for( idx = 0; idx < root->total; idx++ )
    {
        CvCBQuad* q = &(*out_quads)[quad_count];
        src_contour = *(CvSeq**)cvGetSeqElem( root, idx );
        //if( flags && src_contour->v_prev != (CvSeq*)board )//CV_CALIB_CB_FILTER_QUADS
        //    continue;


        // Reset group ID
        memset( q, 0, sizeof(*q) );
        q->group_idx = -1;
        assert( src_contour->total == 4 );
		CvPoint2D32f QPts[4];
		int index[4];
		for ( i = 0; i < 4; i++ )
		{
			index[i] = i;
			QPts[i].x = (float32_t)(*(CvPoint*)cvGetSeqElem(src_contour, i)).x;
			QPts[i].y = (float32_t)(*(CvPoint*)cvGetSeqElem(src_contour, i)).y;

		}
		
		for ( i = 0; i < 4; i++)
		{
			for ( j = i + 1; j < 4; j++)
			{
				if (QPts[i].y > QPts[j].y)
				{
					CvPoint2D32f temp;
					temp = QPts[i];
					QPts[i] = QPts[j];
					QPts[j] = temp;

					int temp_idx;
					temp_idx = index[i];
					index[i] = index[j];
					index[j] = temp_idx;
				}
			}
		}
		int k = (QPts[0].x < QPts[1].x)? index[0]: index[1];
		CvPoint2D32f Pt[4];
		for (i = 0; i < 4; i++)
		{
			float32_t x = (float32_t)(*(CvPoint*)cvGetSeqElem(src_contour, k % 4)).x;
			float32_t y = (float32_t)(*(CvPoint*)cvGetSeqElem(src_contour, k % 4)).y;
			CvPoint2D32f pt = cvPoint2D32f(x, y);
			Pt[i] = pt;
			CvCBCorner* corner = &(*out_corners)[quad_count * 4 + i];

			memset(corner, 0, sizeof(*corner));
			corner->pt = pt;
			q->corners[i] = corner;
			k++;
		}

		float32_t dis_01, dis_12, dis_32, dis_03;
		float32_t cos_hor = 0.0, cos_ver = 0.0;
		float32_t vec_01[2], vec_32[2], vec_03[2], vec_12[2];

		vec_01[0] = Pt[1].y - Pt[0].y;
		vec_01[1] = Pt[1].x - Pt[0].x;
		vec_32[0] = Pt[2].y - Pt[3].y;
		vec_32[1] = Pt[2].x - Pt[3].x;

		vec_03[0] = Pt[3].y - Pt[0].y;
		vec_03[1] = Pt[3].x - Pt[0].x;
		vec_12[0] = Pt[2].y - Pt[1].y;
		vec_12[1] = Pt[2].x - Pt[1].x;

		dis_01 = sqrt(vec_01[0] * vec_01[0] + vec_01[1] * vec_01[1]);
		dis_12 = sqrt(vec_12[0] * vec_12[0] + vec_12[1] * vec_12[1]);
		dis_32 = sqrt(vec_32[0] * vec_32[0] + vec_32[1] * vec_32[1]);
		dis_03 = sqrt(vec_03[0] * vec_03[0] + vec_03[1] * vec_03[1]);


		cos_hor = (vec_01[0] * vec_32[0] + vec_01[1] * vec_32[1])
			/ sqrt(vec_01[0] * vec_01[0] + vec_01[1] * vec_01[1])
			/ sqrt(vec_32[0] * vec_32[0] + vec_32[1] * vec_32[1]);
		cos_ver = (vec_03[0] * vec_12[0] + vec_03[1] * vec_12[1])
			/ sqrt(vec_03[0] * vec_03[0] + vec_03[1] * vec_03[1])
			/ sqrt(vec_12[0] * vec_12[0] + vec_12[1] * vec_12[1]);

		if (dis_01 / dis_32 < 0.5 || dis_32 / dis_01 < 0.5)
		{
			continue;
		}

		if (dis_03 / dis_12 < 0.5 || dis_12 / dis_03 < 0.5)
		{
			continue;
		}

		if (cos_hor < 0.8 || cos_ver < 0.75)
		{
			continue;
		}
		
		q->min_edge_len = FLT_MAX;
		q->max_edge_len = FLT_MIN;
		for (i = 0; i < 4; i++)
		{
			float dx = q->corners[i]->pt.x - q->corners[(i + 1) & 3]->pt.x;
			float dy = q->corners[i]->pt.y - q->corners[(i + 1) & 3]->pt.y;
			float d = dx*dx + dy*dy;
			if (q->min_edge_len > d)
				q->min_edge_len = d;
			if (q->max_edge_len < d)
				q->max_edge_len = d;
		}
        quad_count++;
    }

    cvReleaseMemStorage( &temp_storage );

// EVALUATE TIMER
#if TIMER
	float time = (float) (clock() - start_time) / CLOCKS_PER_SEC;
	GenerateQuads.open("timer/GenerateQuads.txt", ofstream::app);
	GenerateQuads << "Time for icvGenerateQuads was " << time << " seconds." << endl;
	GenerateQuads.close();
#endif

    return quad_count;
}



/*
	Function Name:      mrWriteCorners
	Function Function : Write corners to file
	Input             : 
	    output_quads  : The detected quad group
		pattern_size  : The pattern size need to be detected
		count         : valid quad num in quad_group
        min_number_of_corners : the min valid corner num
	Return            : Whether desired corners are found
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/16        Create
*/
int chessBoard_corner_detector::mrWriteCorners( CvCBQuad **output_quads, int count, CvSize pattern_size, int min_number_of_corners )
{
	// Initialize
	int corner_count = 0;
	bool flagRow = false;
	bool flagColumn = false;
	int maxPattern_sizeRow = -1;
	int maxPattern_sizeColumn = -1;
	Point2f corner_temp[36];
	int corner_temp_flag[36]={0};
	int corner_temp_count=0;


	// Return variable
	int internal_found = 0;
	

	// Compute the minimum and maximum row / column ID
	// (it is unlikely that more than 8bit checkers are used per dimension)
	int min_row		=  127;
	int max_row		= -127;
	int min_column	=  127;
	int max_column	= -127;

	for(int i = 0; i < count; i++ )
    {
		CvCBQuad* q = output_quads[i];
		
		for(int j = 0; j < 4; j++ )
		{
			if( (q->corners[j])->row > max_row)
				max_row = (q->corners[j])->row;
			if( (q->corners[j])->row < min_row)
				min_row = (q->corners[j])->row;
			if( (q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;
			if( (q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}

	Point2f tmp_corner_point[CB_MAX_ROW][CB_MAX_COL];
	int32_t tmp_flag[CB_MAX_ROW][CB_MAX_COL];
	memset(tmp_corner_point, (0, 0), sizeof(Point2f)*CB_MAX_ROW*CB_MAX_COL);
	memset(tmp_flag, 0, sizeof(int32_t)*CB_MAX_ROW*CB_MAX_COL);
	// If in a given direction the target pattern size is reached, we know exactly how
	// the checkerboard is oriented.
	// Else we need to prepare enough "dummy" corners for the worst case.
	if (station_type == STATION_TYPE_GAC || station_type == STATION_TYPE_OFILM)
	{
		for (int i = 0; i < count; i++)
		{
			CvCBQuad* q = output_quads[i];

			for (int j = 0; j < 4; j++)
			{
				if ((q->corners[j])->column == max_column && (q->corners[j])->row != min_row && (q->corners[j])->row != max_row)
				{
					if ((q->corners[j]->needsNeighbor) == false)
					{
						// We know, that the target pattern size is reached
						// in column direction
						flagColumn = true;
					}
				}
				if ((q->corners[j])->row == max_row && (q->corners[j])->column != min_column && (q->corners[j])->column != max_column)
				{
					if ((q->corners[j]->needsNeighbor) == false)
					{
						// We know, that the target pattern size is reached
						// in row direction
						flagRow = true;
					}
				}
			}
		}

		if (flagColumn == true)
		{
			if (max_column - min_column == pattern_size.width + 1)
			{
				maxPattern_sizeColumn = pattern_size.width;
				maxPattern_sizeRow = pattern_size.height;
			}
			else
			{
				maxPattern_sizeColumn = pattern_size.height;
				maxPattern_sizeRow = pattern_size.width;
			}
		}
		else if (flagRow == true)
		{
			if (max_row - min_row == pattern_size.width + 1)
			{
				maxPattern_sizeRow = pattern_size.width;
				maxPattern_sizeColumn = pattern_size.height;
			}
			else
			{
				maxPattern_sizeRow = pattern_size.height;
				maxPattern_sizeColumn = pattern_size.width;
			}
		}
		else
		{
			// If target pattern size is not reached in at least one of the two
			// directions,  then we do not know where the remaining corners are
			// located. Account for this.
			maxPattern_sizeColumn = max(pattern_size.width, pattern_size.height);
			maxPattern_sizeRow = max(pattern_size.width, pattern_size.height);
		}
	}
	if (station_type == STATION_TYPE_JAC)
	{
		maxPattern_sizeColumn = pattern_size.width;
		maxPattern_sizeRow = pattern_size.height;
	}


	//int m=0,n=0;
	//int mm=0,nn=0;
	// Write the corners in increasing order to the output file
	for(int i = min_row + 1; i < maxPattern_sizeRow + min_row + 1; i++)
	{
		for(int j = min_column + 1; j < maxPattern_sizeColumn + min_column + 1; j++)
		{
			// Reset the iterator
			int iter = 1;

			for(int k = 0; k < count; k++)
			{
				for(int l = 0; l < 4; l++)
				{
					if(((output_quads[k])->corners[l]->row == i) && ((output_quads[k])->corners[l]->column == j) )
					{
						// Only write corners to the output file, which are connected
						// i.e. only if iter == 2
						if( iter == 2)
						{
							// The respective row and column have been found, print it to
							// the output file, only do this once

							pattern_corner[0].rows = pattern_size.height;
							pattern_corner[0].cols = pattern_size.width;
							pattern_corner[0].corner_point[i-min_row -1][j-min_column - 1] = Point2f((output_quads[k])->corners[l]->pt.x,(output_quads[k])->corners[l]->pt.y);
							pattern_corner[0].flag[i-min_row -1][j-min_column - 1] = 1;
							pattern_corner[0].connected_type[i - min_row - 1][j - min_column - 1] = (output_quads[k])->corners[l]->connected_way;
							pattern_corner[0].max_edge_len_ratio[i - min_row - 1][j - min_column - 1] = (output_quads[k])->corners[l]->max_edge_len_ratio;
							pattern_corner[0].min_edge_len_ratio[i - min_row - 1][j - min_column - 1] = (output_quads[k])->corners[l]->min_edge_len_ratio;
							//n++;
							//nn++;

							corner_count++;

							tmp_corner_point[i - min_row - 1][j - min_column - 1] = Point2f((output_quads[k])->corners[l]->pt.x, (output_quads[k])->corners[l]->pt.y);
							tmp_flag[i - min_row - 1][j - min_column - 1] = 1;
							//corner_temp[corner_temp_count]=Point2f((output_quads[k])->corners[l]->pt.x,(output_quads[k])->corners[l]->pt.y);
							//corner_temp_flag[corner_temp_count]=1;
							//corner_temp_count++;
						}
						

						// If the iterator is larger than two, this means that more than
						// two corners have the same row / column entries. Then some
						// linking errors must have occurred and we should not use the found
						// pattern
						if (iter > 2)
							return -1;

						iter++;
					}
				}
			}

			// If the respective row / column is non - existent or is a border corner
			if (iter == 1 || iter == 2)
			{
				pattern_corner[0].corner_point[i-min_row -1][j-min_column - 1] = Point2f(0, 0);
				pattern_corner[0].flag[i-min_row -1][j-min_column - 1] = 0;
				tmp_corner_point[i - min_row - 1][j - min_column - 1] = Point2f(0, 0);
				tmp_flag[i - min_row - 1][j - min_column - 1] = 0;
				//nn++;

			}
		}
		//m++;
		//n=0;
		//mm++;
		//nn=0;
	}

	if (station_type == STATION_TYPE_JAC)
	{
		int biar_x, biar_y;
		int selected_corner_row_ID = -1;
		int selected_corner_column_ID = -1;
		float min_corner_x = FLT_MAX;
		float max_corner_x = FLT_MIN;
		float max_corner_y = FLT_MIN;
		float related_x, related_y;
		float min_edge_len_ratio = -1;
		float max_edge_len_ratio = -1;
		if ((corner_config.camid == 0 && corner_config.board_size.height == 2)
			|| (corner_config.camid == 1 && corner_config.board_size.height == 3))
		{
			for (int i = 0; i < maxPattern_sizeRow; i++)
			{
				for (int j = 0; j < maxPattern_sizeColumn; j++)
				{
					if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
						&& pattern_corner[0].max_edge_len_ratio[i][j] > max_edge_len_ratio)
					{
						selected_corner_row_ID = i;
						selected_corner_column_ID = j;
						max_edge_len_ratio = pattern_corner[0].max_edge_len_ratio[i][j];
					}
				}
			}
			if (corner_config.camid == 0 && corner_config.board_size.height == 2)
			{
				switch (selected_corner_row_ID)
				{
				case 0:
					pattern_corner[0].selected_corner_x = 0;
					pattern_corner[0].selected_corner_y = 1;
					break;
				case 1:
					pattern_corner[0].selected_corner_x = 1;
					pattern_corner[0].selected_corner_y = 0;
					break;
				default:
					break;
				}
			}
			
			memset(pattern_corner[0].corner_point, (0, 0), sizeof(Point2f)*CB_MAX_ROW*CB_MAX_COL);
			memset(pattern_corner[0].flag, 0, sizeof(int32_t)*CB_MAX_ROW*CB_MAX_COL);
			if (selected_corner_row_ID != -1 && selected_corner_column_ID != -1 && max_edge_len_ratio > 0.5)
			{
				internal_found = 1;
				biar_x = pattern_corner[0].selected_corner_x - selected_corner_row_ID;
				biar_y = pattern_corner[0].selected_corner_y - selected_corner_column_ID;
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (tmp_flag[i][j] == 1
							&& (i + biar_x >= 0) && (i + biar_x < maxPattern_sizeRow)
							&& (j + biar_y >= 0) && (j + biar_y < maxPattern_sizeColumn))
						{
							pattern_corner[0].corner_point[i + biar_x][j + biar_y] = tmp_corner_point[i][j];
							pattern_corner[0].flag[i + biar_x][j + biar_y] = tmp_flag[i][j];
						}
					}
				}
			}
			else
			{
				internal_found = 0;
			}
		}

		if ((corner_config.camid == 0 && corner_config.board_size.height == 3)
			|| (corner_config.camid == 1 && corner_config.board_size.height == 4))
		{
			for (int i = 0; i < maxPattern_sizeRow; i++)
			{
				for (int j = 0; j < maxPattern_sizeColumn; j++)
				{
					if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
						&& pattern_corner[0].min_edge_len_ratio[i][j] > min_edge_len_ratio)
					{
						selected_corner_row_ID = i;
						selected_corner_column_ID = j;
						min_edge_len_ratio = pattern_corner[0].min_edge_len_ratio[i][j];
					}
				}
			}
			if (corner_config.camid == 1 && corner_config.board_size.height == 4)
			{
				int valid_quad_count = 0;
				int neighborId_1, neighborId_2;
				for (int i = 0; i < count; i++)
				{
					valid_quad_count = 0;	
					CvCBQuad* q = output_quads[i];
					
					for (int neighbor_id = 0; neighbor_id < 4; neighbor_id++)//test how many neighbors the current quad has
					{
						if (q->neighbors[neighbor_id] != 0)
						{
							valid_quad_count++;
							if (valid_quad_count == 1)
							{
								neighborId_1 = neighbor_id;
							}
							if (valid_quad_count == 2)
							{
								neighborId_2 = neighbor_id;
							}
						}
					}
	
				}
				if (valid_quad_count == 2 && neighborId_1 == 0 && neighborId_2 == 1)
				{
					pattern_corner[0].selected_corner_x = 2;
					pattern_corner[0].selected_corner_y = 1;
				}

				else
				{
					switch (selected_corner_row_ID)
					{
					case 0:
						pattern_corner[0].selected_corner_x = 0;
						pattern_corner[0].selected_corner_y = 1;
						break;
					case 1:
						pattern_corner[0].selected_corner_x = 1;
						pattern_corner[0].selected_corner_y = 2;
						break;
					case 2:
						pattern_corner[0].selected_corner_x = 2;
						pattern_corner[0].selected_corner_y = 1;
						break;
					case 3:
						pattern_corner[0].selected_corner_x = 3;
						pattern_corner[0].selected_corner_y = 2;
						break;
					default:
						break;
					}
				}
				
			}
			memset(pattern_corner[0].corner_point, (0, 0), sizeof(Point2f)*CB_MAX_ROW*CB_MAX_COL);
			memset(pattern_corner[0].flag, 0, sizeof(int32_t)*CB_MAX_ROW*CB_MAX_COL);
			if (selected_corner_row_ID != -1 && selected_corner_column_ID != -1 && min_edge_len_ratio > 0.5)
			{
				internal_found = 1;
				biar_x = pattern_corner[0].selected_corner_x - selected_corner_row_ID;
				biar_y = pattern_corner[0].selected_corner_y - selected_corner_column_ID;
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (tmp_flag[i][j] == 1
							&& (i + biar_x >= 0) && (i + biar_x < maxPattern_sizeRow)
							&& (j + biar_y >= 0) && (j + biar_y < maxPattern_sizeColumn))
						{
							pattern_corner[0].corner_point[i + biar_x][j + biar_y] = tmp_corner_point[i][j];
							pattern_corner[0].flag[i + biar_x][j + biar_y] = tmp_flag[i][j];
						}
					}
				}
			}
			else
			{
				internal_found = 0;
			}
		}

		if (corner_config.camid == 2)
		{
			for (int i = 0; i < maxPattern_sizeRow; i++)
			{
				for (int j = 0; j < maxPattern_sizeColumn; j++)
				{
					if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
						&& pattern_corner[0].corner_point[i][j].y > max_corner_y)
					{
						selected_corner_row_ID = i;
						selected_corner_column_ID = j;
						related_x = pattern_corner[0].corner_point[i][j].x;
						max_corner_y = pattern_corner[0].corner_point[i][j].y;
					}
				}
			}
			if (selected_corner_row_ID !=2)
			{
				related_x = 0;
			}
			if (related_x < 200 || related_x > 350)
			{
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
							&& pattern_corner[0].corner_point[i][j].x > max_corner_x)
						{
							selected_corner_row_ID = i;
							selected_corner_column_ID = j;
							max_corner_x = pattern_corner[0].corner_point[i][j].x;
						}
					}
				}
				switch (selected_corner_row_ID)
				{
				case 0:
					pattern_corner[0].selected_corner_x = 0;
					pattern_corner[0].selected_corner_y = 12;
					break;
				case 1:
					pattern_corner[0].selected_corner_x = 1;
					pattern_corner[0].selected_corner_y = 13;
					break;
				case 2:
					pattern_corner[0].selected_corner_x = 2;
					pattern_corner[0].selected_corner_y = 12;
					break;
				default:
					pattern_corner[0].selected_corner_x = 2;
					pattern_corner[0].selected_corner_y = 12;
					break;
				}

			}
			memset(pattern_corner[0].corner_point, (0, 0), sizeof(Point2f)*CB_MAX_ROW*CB_MAX_COL);
			memset(pattern_corner[0].flag, 0, sizeof(int32_t)*CB_MAX_ROW*CB_MAX_COL);
			if (selected_corner_row_ID != -1 && selected_corner_column_ID != -1)
			{
				internal_found = 1;
				biar_x = pattern_corner[0].selected_corner_x - selected_corner_row_ID;
				biar_y = pattern_corner[0].selected_corner_y - selected_corner_column_ID;
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (tmp_flag[i][j] == 1
							&& (i + biar_x >= 0) && (i + biar_x < maxPattern_sizeRow)
							&& (j + biar_y >= 0) && (j + biar_y < maxPattern_sizeColumn))
						{
							pattern_corner[0].corner_point[i + biar_x][j + biar_y] = tmp_corner_point[i][j];
							pattern_corner[0].flag[i + biar_x][j + biar_y] = tmp_flag[i][j];
						}
					}
				}
			}
			else
			{
				internal_found = 0;
			}
		}

		if (corner_config.camid == 3)
		{
			for (int i = 0; i < maxPattern_sizeRow; i++)
			{
				for (int j = 0; j < maxPattern_sizeColumn; j++)
				{
					if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
						&& pattern_corner[0].corner_point[i][j].y > max_corner_y)
					{
						selected_corner_row_ID = i;
						selected_corner_column_ID = j;
						related_x = pattern_corner[0].corner_point[i][j].x;
						max_corner_y = pattern_corner[0].corner_point[i][j].y;
					}
				}
			}
			if (related_x < 300 || related_x > 450)
			{			
				
				min_corner_x = FLT_MAX;
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (pattern_corner[0].connected_type[i][j] == pattern_corner[0].selected_corner_connected_type
							&& pattern_corner[0].corner_point[i][j].x < min_corner_x)
						{
							selected_corner_row_ID = i;
							selected_corner_column_ID = j;
							related_y = pattern_corner[0].corner_point[i][j].y;
							min_corner_x = pattern_corner[0].corner_point[i][j].x;
						}
					}
				}
				switch (selected_corner_row_ID)
				{
				case 0:
					pattern_corner[0].selected_corner_x = 0;
					pattern_corner[0].selected_corner_y = 2;
					break;
				case 1:
					pattern_corner[0].selected_corner_x = 1;
					pattern_corner[0].selected_corner_y = 1;
					break;
				case 2:
					pattern_corner[0].selected_corner_x = 2;
					pattern_corner[0].selected_corner_y = 2;
					break;
				default:
					pattern_corner[0].selected_corner_x = 2;
					pattern_corner[0].selected_corner_y = 2;
					break;
				}
			}
			memset(pattern_corner[0].corner_point, (0, 0), sizeof(Point2f)*CB_MAX_ROW*CB_MAX_COL);
			memset(pattern_corner[0].flag, 0, sizeof(int32_t)*CB_MAX_ROW*CB_MAX_COL);
			if (selected_corner_row_ID != -1 && selected_corner_column_ID != -1)
			{
				internal_found = 1;
				biar_x = pattern_corner[0].selected_corner_x - selected_corner_row_ID;
				biar_y = pattern_corner[0].selected_corner_y - selected_corner_column_ID;
				for (int i = 0; i < maxPattern_sizeRow; i++)
				{
					for (int j = 0; j < maxPattern_sizeColumn; j++)
					{
						if (tmp_flag[i][j] == 1
							&& (i + biar_x >= 0) && (i + biar_x < maxPattern_sizeRow)
							&& (j + biar_y >= 0) && (j + biar_y < maxPattern_sizeColumn))
						{
							pattern_corner[0].corner_point[i + biar_x][j + biar_y] = tmp_corner_point[i][j];
							pattern_corner[0].flag[i + biar_x][j + biar_y] = tmp_flag[i][j];
						}
					}
				}
			}
			else
			{
				internal_found = 0;
			}
		}
	}
	//int count_temp=0;;
	//for(int i = 0;i<pattern_size.height;i++)
	//	for(int j=0;j<pattern_size.width;j++)
	//	{
	//		pattern_corner[0].corner_point[i][j]=corner_temp[count_temp];
	//		pattern_corner[0].flag[i][j]=corner_temp_flag[count_temp];
	//		count_temp++;
	//	}
	// check whether enough corners have been found
	if (station_type == STATION_TYPE_GAC || station_type == STATION_TYPE_OFILM)
	{
		if (corner_count >= min_number_of_corners)
			internal_found = 1;
		else
			internal_found = 0;
	}

	// pattern found, or not found?
	return internal_found;
}

/*
	Function Name:      removeInvalidQuads
	Function Function : Remove invalid pattern
	Input             : 
	    quad_group    : The detected quad group
		board_size    : The pattern size need to be detected
		count         : valid quad num in quad_group
	Return            : Void
	Note              : this function remove quad by the flowing criterion.
	                       1. 0.25*height < y center of the pattern < 0.75*height;
						   2. if pattern is  bigger than 2 * 2, 0.18*width < x center of the pattern < 0.82*width;
						   3. if pattern is 2*2, 0.35*width < x center of the pattern || x center of the pattern < 0.65*width;
						   4. if pattern is 2*2, the ave x of left corners <  the ave x of right corners
						   5. if pattern is 2*2, the ave x of up corners <  the ave y of down corners
						   6. the max y of up corners < the min y of down corners
						   7. add two 2*2 pattern do not at the same side selection on 2017/3/21

	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/16        Create
					  YanShuo        2017/3/21        Revision
*/
void chessBoard_corner_detector::removeInvalidQuads(CvCBQuad** quad_group, CvSize board_size, int& count)
{
	float64_t sum[2] = {0, 0}, ave[2] = {0, 0};
	for(int i = 0; i < count; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			sum[0] += quad_group[i]->corners[j]->pt.x;
			sum[1] += quad_group[i]->corners[j]->pt.y;
		}				
	}
	ave[0] = sum[0] / (count * 4);
	ave[1] = sum[1] / (count * 4);
	if(ave[1] < temp_img->height * 0.25 || ave[1] > temp_img->height * 0.75)
	{
		count = -1;
		return;
	}
	/* End of this add*/

	if (station_type == STATION_TYPE_GAC || station_type == STATION_TYPE_OFILM)
	{
		if (4 < board_size.height * board_size.width)
		{
			if (ave[0] < temp_img->width * 0.18 || ave[0] > temp_img->width * 0.82)
			{
				count = -1;
				return;
			}
		}

		/*For 2*2 pattern, do this selection */
		if (4 == board_size.height * board_size.width)
		{
			if (ave[0] > temp_img->width * 0.35 && ave[0] < temp_img->width * 0.65)
			{
				count = -1;
				return;
			}

			if (SIDE_LEFT == pattern_side && ave[0] > temp_img->width * 0.5)
			{
				count = -1;
				return;
			}

			if (SIDE_RIGHT == pattern_side && ave[0] < temp_img->width * 0.5)
			{
				count = -1;
				return;
			}

			float64_t ave_y_cord_up = 0, ave_y_cord_down = 0;
			float64_t ave_x_cord_left = 0, ave_x_cord_right = 0;
			float64_t max_y_up = 0, min_y_down = 10000;
			/* each quad should have two connected neighbors*/
			for (int i = 0; i < count; i++)
			{
				int neighbor_label = 0;
				for (int j = 0; j < 4; j++)
				{
					if (quad_group[i]->neighbors[j])
					{
						neighbor_label |= (1) << j;
					}
				}
				if (3 != neighbor_label && 6 != neighbor_label &&
					9 != neighbor_label && 12 != neighbor_label)
				{
					count = -1;
					return;
				}
				if (3 == neighbor_label)
				{
					ave_y_cord_down += quad_group[i]->corners[0]->pt.y * 0.5;
					ave_y_cord_down += quad_group[i]->corners[1]->pt.y * 0.5;
					if (min_y_down > quad_group[i]->corners[0]->pt.y)
					{
						min_y_down = quad_group[i]->corners[0]->pt.y;
					}
					if (min_y_down > quad_group[i]->corners[1]->pt.y)
					{
						min_y_down = quad_group[i]->corners[1]->pt.y;
					}
				}
				if (6 == neighbor_label)
				{
					ave_x_cord_left += quad_group[i]->corners[1]->pt.x * 0.5;
					ave_x_cord_left += quad_group[i]->corners[2]->pt.x * 0.5;
				}
				if (9 == neighbor_label)
				{
					ave_x_cord_right += quad_group[i]->corners[0]->pt.x * 0.5;
					ave_x_cord_right += quad_group[i]->corners[3]->pt.x * 0.5;
				}
				if (12 == neighbor_label)
				{
					ave_y_cord_up += quad_group[i]->corners[2]->pt.y * 0.5;
					ave_y_cord_up += quad_group[i]->corners[3]->pt.y * 0.5;
					if (max_y_up < quad_group[i]->corners[2]->pt.y)
					{
						max_y_up = quad_group[i]->corners[2]->pt.y;
					}
					if (max_y_up < quad_group[i]->corners[3]->pt.y)
					{
						max_y_up = quad_group[i]->corners[3]->pt.y;
					}
				}
			}
			if (ave_y_cord_up > ave_y_cord_down || ave_x_cord_left > ave_x_cord_right)
			{
				count = -1;
				return;
			}
			//if(max_y_up >= min_y_down)
			//{
			//	count = -1;
			//	return;
			//}
		}
	}

	if (station_type == STATION_TYPE_JAC)
	{
		if (SIDE_LEFT == pattern_side && ave[0] > temp_img->width * 0.5)
		{
			count = -1;
			return;
		}

		if (SIDE_RIGHT == pattern_side && ave[0] < temp_img->width * 0.5)
		{
			count = -1;
			return;
		}

		if (SIDE_BOTH == pattern_side && (ave[0] < temp_img->width * 0.28 || ave[0] > temp_img->width * 0.65))
		{
			count = -1;
			return;
		}
	}
}
bool SaveRawFile1(const char* sfilename, char *pBuffer, int iSize)
{
	FILE* p_file = fopen(sfilename, "wb");
	if (NULL == p_file)
	{
		return 1;
	}
	fwrite(pBuffer, sizeof(uchar) * iSize, 1, p_file);
	fclose(p_file);
	return 0;
}
//===========================================================================
// END OF FILE
//===========================================================================
