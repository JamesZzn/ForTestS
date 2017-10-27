#include<algorithm>
#include "eol_image_process.h"

/*
    Function Name:      bilinear_interpolation
    Function Function : finish image interpolation
    Input             :
    	    srcImg    : rgb image
    	       rgb    : interpolation result
		   xSrcLoc    : source x location (double)
		   ySrcLoc    : source y location (double)
    Return            : void
    Note              :
    Revision History  : Author         Data             Changes
						YanShuo        2017/04/7        Create
*/
void bilinear_interpolation(IplImage* srcImg, 
							int32_t rgb[3], 
							float64_t xSrcLoc, 
							float64_t ySrcLoc 
						    )
{
	float64_t weightUpLeft, weightUpRight, weightDownLeft, weightDownRight;
	int32_t x0 = (int32_t)xSrcLoc;
	int32_t x1 = (int32_t)xSrcLoc + 1;
	int32_t y0 = (int32_t)ySrcLoc;
	int32_t y1 = (int32_t)ySrcLoc + 1;

	x0 = x0 < 0 ? 0 : x0;
	x1 = x1 < 0 ? 0 : x1;
	y0 = y0 < 0 ? 0 : y0;
	y1 = y1 < 0 ? 0 : y1;

	x0 = x0 > srcImg->width - 1 ? srcImg->width - 1 : x0;
	x1 = x1 > srcImg->width - 1 ? srcImg->width - 1 : x1;
	y0 = y0 > srcImg->height - 1 ? srcImg->height - 1 : y0;
	y1 = y1 > srcImg->height - 1 ? srcImg->height - 1 : y1;

	weightUpLeft = (x1 - xSrcLoc) * (y1 - ySrcLoc);
	weightUpRight = (xSrcLoc - x0) * (y1 - ySrcLoc);
	weightDownLeft = (x1 - xSrcLoc) * (ySrcLoc - y0);
	weightDownRight = 1 - weightUpLeft - weightUpRight - weightDownLeft;

	uchar* p = (uchar*)(srcImg->imageData + y0*srcImg->widthStep);
	uchar* q = (uchar*)(srcImg->imageData + y1*srcImg->widthStep);

	for(int i = 0; i <= 2; i++)
	{
		if(x0 == 0 || y0 == 0 || x0 == srcImg->width - 1 || y0 == srcImg->height - 1)
			rgb[i] = p[x0 * srcImg->nChannels + i];
		else
			rgb[i] =	weightUpLeft * p[x0 * srcImg->nChannels + i] + 
			            weightUpRight * p[x1 * srcImg->nChannels + i] + 
					    weightDownLeft * q[x0 * srcImg->nChannels + i] + 
					    weightDownRight * q[x1 * srcImg->nChannels + i];
	}
}

/*
    Function Name:      bgr_to_gray
    Function Function : change bgr image into gray image
    Input             :
    	rgb_img       : rgb image
    	gray_img      : i420 image
    Return            : void
    Note              :
    Revision History  : Author         Data             Changes
						YanShuo        2017/03/21       Create
*/
void bgr_to_gray(IplImage* rgb_img, IplImage* gray_img)
{
	uchar b, g, r, y, u, v;

	int32_t Y, U, V;
	int32_t height, width;
	height = gray_img->height;
	width = gray_img->width;

	uchar* pBgr = (uchar*)rgb_img->imageData;
	uchar* pGray = (uchar*)gray_img->imageData;

	/*convert bgr image to yuv420 image*/
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			b = pBgr[i * rgb_img->widthStep + j * rgb_img->nChannels + 0];
			g = pBgr[i * rgb_img->widthStep + j * rgb_img->nChannels + 1];
			r = pBgr[i * rgb_img->widthStep + j * rgb_img->nChannels + 2];

			Y = (int32_t)(0.299 * r + 0.587 * g + 0.114 * b);
			Y = Y < 0 ? 0 : (Y > 255 ? 255 : Y);
			y = (uchar)Y;
			pGray[i * gray_img->widthStep + j] = Y;
		}
	}
}

/*
	Function Name:      bgr_to_yuv420
	Function Function : change bgr image into yuv420 image
	Input             : 
	    rgb_img:        the rgb image
		yuv420_img:     i420 image
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
						YanShuo        2017/02/07       Create
*/
void bgr_to_yuv420(const IplImage* rgb_img, uchar* yuv420_img)
{
	uchar b, g, r, y, u, v;
	
	int32_t Y, U, V;
	int32_t height, width, half_height, half_width, pitch;
	FILE* p_file = NULL;
	height = rgb_img->height;
	width = rgb_img->width;
	half_height = height >> 1;
	half_width = width >> 1;
	pitch = height * width;

	uchar* pData = (uchar*)rgb_img->imageData;

	/*convert bgr image to yuv420 image*/
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if(1 == rgb_img->nChannels)
			{
				yuv420_img[i * width + j] = pData[i * rgb_img->widthStep + j];
				if ( !(i & 0x1) && (!(j & 0x1)) )
				{
					yuv420_img[pitch + (i >> 1) * half_width + (j >> 1)] = 128;
					yuv420_img[(pitch * 5 >> 2) + (i >> 1) * half_width + (j >> 1)] = 128;
				}
			}
			else
			{
				b = pData[i * rgb_img->widthStep + j * rgb_img->nChannels + 0];
				g = pData[i * rgb_img->widthStep + j * rgb_img->nChannels + 1];
				r = pData[i * rgb_img->widthStep + j * rgb_img->nChannels + 2];

				Y = (int32_t)( 0.299 * r + 0.587 * g + 0.114 * b);
				Y = Y < 0 ? 0 : (Y > 255 ? 255 : Y);
				y = (uchar)Y;
				yuv420_img[i * width + j] = Y;
				if (!(i & 0x1) && (!(j & 0x1)))
				{
					U = (int32_t)(-0.169 * r - 0.331 * g + 0.500 * b + 128);
					V = (int32_t)( 0.500 * r - 0.418 * g - 0.082 * b + 128);
					U = U < 0 ? 0 : (U > 255 ? 255 : U);
					V = V < 0 ? 0 : (V > 255 ? 255 : V);
					u = (uchar)U;
					v = (uchar)V;
					yuv420_img[pitch + (i >> 1) * half_width + (j >> 1)] = u;
					yuv420_img[(pitch * 5 >> 2) + (i >> 1) * half_width + (j >> 1)] = v;
				}
			}			
		}
	}
}

/*
	Function Name:      yuv420_to_bgr
	Function Function : Convert YUV420 image into IplImage* BGR image
	Input             : 
	    pBgrImg :       The converted BGR image
		pYuv420Img :    The source YUV420 image
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/14        Create
*/
void yuv420_to_bgr(IplImage *pBgrImg, uchar* pYuv420Img)
{
	int32_t	width	 = pBgrImg->width;
	int32_t	height    = pBgrImg->height;
	int32_t halfWidth = width >> 1;
	int32_t halfHeight = height >> 1;
	int32_t	y_pitch = width * height;
	int32_t	uv_pitch = y_pitch >> 2;
	int32_t	i, j, iHalf, jHalf;

	uchar* pBrgData = (uchar*)pBgrImg->imageData;
	uchar tempY, tempU, tempV;

	for (i = 0; i < height; i++)
	{
		iHalf = i >> 1;
		for (j = 0; j < width; j++)
		{
			jHalf = j >> 1;
			tempY = pYuv420Img[i * width + j];
			tempU = pYuv420Img[y_pitch + iHalf * halfWidth + jHalf];
			tempV = pYuv420Img[y_pitch + uv_pitch + iHalf * halfWidth + jHalf];

			int32_t Y = tempY;
			int32_t U = tempU - 128;
			int32_t V = tempV - 128;

			int32_t R = (int32_t)(Y + 1.4022 * V);
			int32_t G = (int32_t)(Y - 0.3456 * U - 0.7145 * V);
			int32_t B = (int32_t)(Y + 1.771  * U);

			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 0] = 
				B < 0 ? 0 : (B > 255 ? 255 : (uchar)B);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 1] = 
				G < 0 ? 0 : (G > 255 ? 255 : (uchar)G);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 2] = 
				R < 0 ? 0 : (R > 255 ? 255 : (uchar)R);
		}
	}
}

/*
	Function Name:      yuv422_to_bgr
	Function Function : Convert YUV422 image into IplImage* BGR image
	Input             : 
	    pBgrImg :       The converted BGR image
		pYuv422Img :    The source YUV422 image
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/14        Create
*/
void yuv422_to_bgr(IplImage *pBgrImg, uchar* pYuv422Img)
{
	int32_t	width	 = pBgrImg->width;
	int32_t	height    = pBgrImg->height;
	int32_t halfWidth = width >> 1;
	int32_t halfHeight = height >> 1;
	int32_t	y_pitch = width * height;
	int32_t	uv_pitch = y_pitch >> 1;
	int32_t	i, j, iHalf, jHalf;

	uchar* pBrgData = (uchar*)pBgrImg->imageData;
	uchar tempY, tempU, tempV;

	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			jHalf = j >> 1;
			tempY = pYuv422Img[i * width + j];
			tempU = pYuv422Img[y_pitch + i * halfWidth + jHalf];
			tempV = pYuv422Img[y_pitch + uv_pitch + i * halfWidth + jHalf];

			int32_t Y = tempY;
			int32_t U = tempU - 128;
			int32_t V = tempV - 128;

			int32_t R = (int32_t)(Y + 1.4022 * V);
			int32_t G = (int32_t)(Y - 0.3456 * U - 0.7145 * V);
			int32_t B = (int32_t)(Y + 1.771  * U);

			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 0] = 
				B < 0 ? 0 : (B > 255 ? 255 : (uchar)B);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 1] = 
				G < 0 ? 0 : (G > 255 ? 255 : (uchar)G);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 2] = 
				R < 0 ? 0 : (R > 255 ? 255 : (uchar)R);
		}
	}
}

/*
	Function Name:      nv12_to_bgr
	Function Function : Convert Nv12 image into IplImage* BGR image
	Input             : 
	    pBgrImg :       The converted BGR image
		pYuv420Img :    The source Nv12 image
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/8         Create
*/
void nv12_to_bgr(IplImage *pBgrImg, uchar* pNv12Img)
{
	int32_t	width	 = pBgrImg->width;
	int32_t	height    = pBgrImg->height;
	int32_t halfWidth = width >> 1;
	int32_t halfHeight = height >> 1;
	int32_t	y_pitch = width * height;
	int32_t	uv_pitch = y_pitch >> 2;
	int32_t	i, j, iHalf, jHalf;

	uchar* pBrgData = (uchar*)pBgrImg->imageData;
	uchar tempY, tempU, tempV;

	for (i = 0; i < height; i++)
	{
		iHalf = i >> 1;
		for (j = 0; j < width; j++)
		{
			jHalf = j >> 1;
			tempY = pNv12Img[i * width + j];
			tempU = pNv12Img[y_pitch + iHalf * width + (jHalf << 1)];
			tempV = pNv12Img[y_pitch + iHalf * width + (jHalf << 1) + 1];

			int32_t Y = tempY;
			int32_t U = tempU - 128;
			int32_t V = tempV - 128;

			int32_t R = (int32_t)(Y + 1.4022 * V);
			int32_t G = (int32_t)(Y - 0.3456 * U - 0.7145 * V);
			int32_t B = (int32_t)(Y + 1.771  * U);

			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 0] = 
				B < 0 ? 0 : (B > 255 ? 255 : (uchar)B);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 1] = 
				G < 0 ? 0 : (G > 255 ? 255 : (uchar)G);
			pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + 2] = 
				R < 0 ? 0 : (R > 255 ? 255 : (uchar)R);
		}
	}
}

/*
	Function Name:      yuv420_cut
	Function Function : Cut image
	Input             : 
	    pDstYuv422Img : The cut YUV420 image
		pSrcYuv422Img : The source YUV420 image
		src_height    : source image height
		src_width     : source image width
		dst_height    : cut image height
		dst_height    : cut image width
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/14        Create
*/
void yuv420_cut(uchar* pDstYuv420Img, uchar* pSrcYuv420Img, 
				int32_t src_height, int32_t src_width, 
				int32_t dst_height, int32_t dst_width)
{
	int32_t halfSrcWidth = src_width >> 1;
	int32_t halfSrcHeight = src_height >> 1;
	int32_t halfDstWidth = dst_width >> 1;
	int32_t halfDstHeight = dst_height >> 1;
	int32_t	y_src_pitch = src_width * src_height;
	int32_t	y_dst_pitch = dst_width * dst_height;
	int32_t	uv_src_pitch = y_src_pitch >> 2;
	int32_t	uv_dst_pitch = y_dst_pitch >> 2;
	int32_t	i, j, iHalf, jHalf;

	uchar tempY, tempU, tempV;

	for (i = 0; i < src_height; i++)
	{
		iHalf = i >> 1;
		for (j = 0; j < src_width; j++)
		{
			jHalf = j >> 1;
			if(j < dst_width && i < dst_height)
			{
				pDstYuv420Img[i * dst_width + j] = pSrcYuv420Img[i * src_width + j];
				pDstYuv420Img[y_dst_pitch + iHalf * halfDstWidth + jHalf] = 
					pSrcYuv420Img[y_src_pitch + iHalf * halfSrcWidth + jHalf];
				pDstYuv420Img[y_dst_pitch + uv_dst_pitch + iHalf * halfDstWidth + jHalf] = 
					pSrcYuv420Img[y_src_pitch + uv_src_pitch + iHalf * halfSrcWidth + jHalf];
			}
		}
	}
}

/*
	Function Name:      yuv422_cut
	Function Function : Cut image
	Input             : 
	    pDstYuv422Img : The cut YUV422 image
		pSrcYuv422Img : The source YUV422 image
		src_height    : source image height
		src_width     : source image width
		dst_height    : cut image height
		dst_height    : cut image width
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/14        Create
*/
void yuv422_cut(uchar* pDstYuv422Img, uchar* pSrcYuv422Img, 
				int32_t src_height, int32_t src_width, 
				int32_t dst_height, int32_t dst_width)
{
	int32_t halfSrcWidth = src_width >> 1;
	int32_t halfSrcHeight = src_height >> 1;
	int32_t halfDstWidth = dst_width >> 1;
	int32_t halfDstHeight = dst_height >> 1;
	int32_t	y_src_pitch = src_width * src_height;
	int32_t	y_dst_pitch = dst_width * dst_height;
	int32_t	uv_src_pitch = y_src_pitch >> 1;
	int32_t	uv_dst_pitch = y_dst_pitch >> 1;
	int32_t	i, j, iHalf, jHalf;

	uchar tempY, tempU, tempV;

	for (i = 0; i < src_height; i++)
	{
		for (j = 0; j < src_width; j++)
		{
			jHalf = j >> 1;
			if(j < dst_width && i < dst_height)
			{
				pDstYuv422Img[i * dst_width + j] = pSrcYuv422Img[i * src_width + j];
				pDstYuv422Img[y_dst_pitch + i * halfDstWidth + jHalf] = 
					pSrcYuv422Img[y_src_pitch + i * halfSrcWidth + jHalf];
				pDstYuv422Img[y_dst_pitch + uv_dst_pitch + i * halfDstWidth + jHalf] = 
					pSrcYuv422Img[y_src_pitch + uv_src_pitch + i * halfSrcWidth + jHalf];
			}
		}
	}
}

/*
	Function Name:      nv12_cut
	Function Function : Cut image
	Input             : 
	    pDstNv12Img   : The cut NV12 image
		pSrcNv12Img   : The source NV12 image
		src_height    : source image height
		src_width     : source image width
		dst_height    : cut image height
		dst_height    : cut image width
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/8        Create
*/
void nv12_cut(uchar* pDstNv12Img, uchar* pSrcNv12Img, 
			  int32_t src_height, int32_t src_width, 
			  int32_t dst_height, int32_t dst_width)
{
	int32_t halfSrcWidth = src_width >> 1;
	int32_t halfSrcHeight = src_height >> 1;
	int32_t halfDstWidth = dst_width >> 1;
	int32_t halfDstHeight = dst_height >> 1;
	int32_t	y_src_pitch = src_width * src_height;
	int32_t	y_dst_pitch = dst_width * dst_height;

	int32_t	i, j, iHalf, jHalf;

	uchar tempY, tempU, tempV;

	for (i = 0; i < src_height; i++)
	{
		iHalf = i >> 1;
		for (j = 0; j < src_width; j++)
		{
			jHalf = j >> 1;
			if(j < dst_width && i < dst_height)
			{
				pDstNv12Img[i * dst_width + j] = pSrcNv12Img[i * src_width + j];
				pDstNv12Img[y_dst_pitch + iHalf * dst_width + j] = 
					pSrcNv12Img[y_src_pitch + iHalf * src_width + j];
			}
		}
	}
}

/*
	Function Name:      rgba_cut
	Function Function : Cut image
	Input             : 
	    pDstNv12Img   : The cut NV12 image
		pSrcNv12Img   : The source NV12 image
		src_height    : source image height
		src_width     : source image width
		dst_height    : cut image height
		dst_height    : cut image width
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/8        Create
*/
void rgba_cut(uchar* pDstRgbaImg, uchar* pSrcRgbaImg, 
			  int32_t src_height, int32_t src_width, 
			  int32_t dst_height, int32_t dst_width)
{
	int32_t	y_src_pitch = src_width * src_height;
	int32_t	y_dst_pitch = dst_width * dst_height;

	int32_t	i, j, k, iHalf, jHalf;

	uchar tempY, tempU, tempV;

	for (i = 0; i < src_height; i++)
	{
		for (j = 0; j < src_width; j++)
		{
			if(j < dst_width && i < dst_height)
			{
				for(k = 0; k < 4; k++)
				{
					pDstRgbaImg[i * dst_width * 4 + j * 4 + k] = pSrcRgbaImg[i * src_width * 4 + j * 4 + k];
				}
			}
		}
	}
}

/*
	Function Name:      flip_image
	Function Function : flip image
	Input             : 
	    pBgrImg :       The BGR image to be flipped
		is_horizon :    Horizontal flip or vertical flip
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/7         Create
*/
void flip_image(IplImage *pBgrImg, bool is_horizon)
{
	int32_t	width	 = pBgrImg->width;
	int32_t	height    = pBgrImg->height;
	int32_t	i, j, k, iHalf, jHalf;

	uchar* pBrgData = (uchar*)pBgrImg->imageData;
	uchar tempY, tempU, tempV;
	if(is_horizon)
	{
		for (i = 0; i < height; i++)
		{
			for (j = 0; j < (width >> 1); j++)
			{
				for(k = 0; k < pBgrImg->nChannels; k++)
				{
					uchar tmp = pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + k];
					pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + k] = 
						pBrgData[i*pBgrImg->widthStep + (width - 1 - j)*pBgrImg->nChannels + k];
					pBrgData[i*pBgrImg->widthStep + (width - 1 - j)*pBgrImg->nChannels + k] = tmp;
				}
			}
		}
	}
	else
	{
		for (j = 0; j < (width >> 1); j++)
		{
			for (i = 0; i < height; i++)
			{
				for(k = 0; k < pBgrImg->nChannels; k++)
				{
					uchar tmp = pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + k];
					pBrgData[i*pBgrImg->widthStep + j*pBgrImg->nChannels + k] = 
						pBrgData[(height - 1 - i)*pBgrImg->widthStep + j*pBgrImg->nChannels + k];
					pBrgData[(height - 1 - i)*pBgrImg->widthStep + j*pBgrImg->nChannels + k] = tmp;
				}
			}
		}
	}
	
}


/*
	Function Name:      raw_to_bgr
	Function Function : Convert raw data into IplImage* BGR image
	Input             : 
	    pBgrImg :       The converted BGR image
		pRawImg :       The raw data
	Return            : Void
	Note              : it's incorrect now, need to be updated
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/23        Create
*/
/*
void raw_to_bgr(IplImage *pBgrImg, uchar* pRawImg)
{
	uchar* pData = (uchar*)pBgrImg->imageData;
	for(int i = 0; i < pBgrImg->height; i++)
	{
		for(int j = 0; j < pBgrImg->width; j++)
		{
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 0] = 
				pRawImg[i * pBgrImg->width + 0];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 1] = 
				pRawImg[i * pBgrImg->width + 1];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 2] = 
				pRawImg[i * pBgrImg->width + 2];
		}
	}
}
*/
void raw_to_bgr(IplImage *pBgrImg, uchar* pRawImg)
{
	uchar* pData = (uchar*)pBgrImg->imageData;
	for (int i = 0; i < pBgrImg->height; i++)
	{
		for (int j = 0; j < pBgrImg->width; j++)
		{
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 0] =
				pRawImg[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 0];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 1] =
				pRawImg[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 1];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 2] =
				pRawImg[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 2];
		}
	}
}

/*
	Function Name:      rgba_to_bgr
	Function Function : Convert raw data into IplImage* BGR image
	Input             : 
	    pBgrImg :       The converted BGR image
		pRawImg :       The raw data
	Return            : Void
	Note              : it's incorrect now, need to be updated
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/23        Create
*/
void rgba_to_bgr(IplImage *pBgrImg, uchar* pRawImg)
{
	int32_t pitch = pBgrImg->width * pBgrImg->height;
	uchar* pData = (uchar*)pBgrImg->imageData;
	for(int i = 0; i < pBgrImg->height; i++)
	{
		for(int j = 0; j < pBgrImg->width; j++)
		{
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 0] = 
				pRawImg[i * pBgrImg->width * 4 + j * 4 + 2];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 1] = 
				pRawImg[i * pBgrImg->width * 4 + j * 4 + 1];
			pData[i * pBgrImg->widthStep + j * pBgrImg->nChannels + 2] = 
				pRawImg[i * pBgrImg->width * 4 + j * 4 + 0];
		}
	}
}

/*
	Function Name:      load_image
	Function Function : Load file into memory
	Input             : 
	    pFile :         The file pointer
		pImage :        The image to be loaded
		height :        The image height
		width :         The image width
		image_type :    The image type is in one of yuv422, yuv420 and bgr
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/2/23        Create
*/
int load_image(FILE* pFile, uchar* pImage, int32_t height, 
			   int32_t width, EOL_Source_Image_Type_T image_type)
{	
	int ret;
	if(!pFile)
	{
		printf("Open file error.\n");
		return OPEN_FILE_ERROR;
	}
	switch (image_type)
	{
	case TYPE_YUV420:
		ret = fread(pImage, sizeof(uchar), height * width * 3 >> 1, pFile);
		break;
	case TYPE_YUV422:
		ret = fread(pImage, sizeof(uchar), height * width * 2, pFile);
		break;
	case TYPE_NV12:
		ret = fread(pImage, sizeof(uchar), height * width * 3 >> 1, pFile);
		break;
	case TYPE_BGR:
		ret = fread(pImage, sizeof(uchar), height * width * 3, pFile);
		break;
	case TYPE_RGBA:
		ret = fread(pImage, sizeof(uchar), height * width * 4, pFile);
		break;
	default:
		break;
	}
	return ret;
}

/*
	Function Name:      split_raw_data
	Function Function : split raw data into four camera's data and only choose pointed frames
	Input             : 
	 
	Return            : Error code
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/3/8         Create
*/
void split_raw_data(const char* file_name, const char* path_name, const int32_t start_frame, 
				    const int32_t frame_num, const int32_t height, const int32_t width)
{
	char front_name[200], rear_name[200], left_name[200], right_name[200];
	FILE* p_file_video = NULL, *p_file_front = NULL, *p_file_rear = NULL;
	FILE* p_file_left = NULL, *p_file_right = NULL;
	int32_t image_size = sizeof(uchar) * height * width * 3 >> 1;
	int i, j;
	
	uchar* front_image = (uchar*)malloc(image_size);
	uchar* rear_image = (uchar*)malloc(image_size);
	uchar* left_image = (uchar*)malloc(image_size);
	uchar* right_image = (uchar*)malloc(image_size);

	sprintf(front_name, "%s/Front.nv12", path_name);
	sprintf(rear_name, "%s/Rear.nv12", path_name);
	sprintf(left_name, "%s/Left.nv12", path_name);
	sprintf(right_name, "%s/Right.nv12", path_name);

	p_file_video = fopen(file_name, "rb");
	p_file_front = fopen(front_name, "wb");
	p_file_rear  = fopen(rear_name, "wb");
	p_file_left  = fopen(left_name, "wb");
	p_file_right = fopen(right_name, "wb");

	fseek(p_file_video, image_size * start_frame, 0);
//	for(i = 0; i < (frame_num >> 2); i++)
	{
		fread(front_image, image_size, 1, p_file_video);
		fread(rear_image, image_size, 1, p_file_video);
		fread(left_image, image_size, 1, p_file_video);
		fread(right_image, image_size, 1, p_file_video);

		fwrite(front_image, image_size, 1, p_file_front);
		fwrite(rear_image, image_size, 1, p_file_rear);
		fwrite(left_image, image_size, 1, p_file_left);
		fwrite(right_image, image_size, 1, p_file_right);
	}

	if(front_image)
	{
		free(front_image);
	}
	if(rear_image)
	{
		free(rear_image);
	}
	if(left_image)
	{
		free(left_image);
	}
	if(right_image)
	{
		free(right_image);
	}

	fclose(p_file_video);
	fclose(p_file_front);
	fclose(p_file_rear);
	fclose(p_file_left);
	fclose(p_file_right);
}

void split_raw_data2(const char* file_name, const char* path_name, const int32_t start_frame,
	const int32_t frame_num, const int32_t height, const int32_t width)
{
	FILE* p_file_video = NULL, *p_file_segment = NULL;
	int32_t image_size = sizeof(uchar) * height * width * 3 >> 1;
	int i, j;

	uchar* image = (uchar*)malloc(image_size);

	p_file_video = fopen(file_name, "rb");
	p_file_segment = fopen(path_name, "wb");

	fseek(p_file_video, image_size * start_frame, 0);
	for (i = 0; i < frame_num; i++)
	{
		fread(image, image_size, 1, p_file_video);

		fwrite(image, image_size, 1, p_file_segment);
	}

	if (image)
	{
		free(image);
	}

	fclose(p_file_video);
	fclose(p_file_segment);
}

/*
    Function Name:      image_copy_roi
    Function Function : Convert src image into dst image's roi region with scale
    Input             :
        pSrcImg       : The source image
        pDstImg       : The destination image
		roi           : The roi of dst image
    Return            : Void
    Note              :
    Revision History  : Author         Data             Changes
                        YanShuo        2017/3/21        Create
*/
void image_copy_roi(const IplImage* pSrcImg, IplImage* pDstImg, CvRect roi)
{
	int32_t	i, j, k;

	uchar* pSrc = (uchar*)pSrcImg->imageData;
	uchar* pDst = (uchar*)pDstImg->imageData;

	for (i = 0; i < roi.height; i++)
	{
		for (j = 0; j < roi.width; j++)
		{			
			Point2f srcPoint;
			
			srcPoint.x = j / roi.width * pSrcImg->width;
			srcPoint.y = i / roi.height * pSrcImg->height;

			if (srcPoint.x >= pSrcImg->width - 1)
				srcPoint.x -= 1;
			if (srcPoint.x >= pSrcImg->height - 1)
				srcPoint.y -= 1;

			CvPoint upLeftPoint;
			upLeftPoint.x = (int)srcPoint.x;
			upLeftPoint.y = (int)srcPoint.y;

			float64_t weight[4];
			weight[0] = (1 - (srcPoint.x - upLeftPoint.x)) * (1 - (srcPoint.y - upLeftPoint.y));
			weight[1] = (srcPoint.x - upLeftPoint.x) * (1 - (srcPoint.y - upLeftPoint.y));
			weight[2] = (1 - (srcPoint.x - upLeftPoint.x)) * (srcPoint.y - upLeftPoint.y);
			weight[3] = 1 - weight[0] - weight[1] - weight[2];

			float result[3];

			for (k = 0; k < min(pDstImg->nChannels, pSrcImg->nChannels); k++)
			{
				result[k] = pSrc[upLeftPoint.y * pSrcImg->widthStep + upLeftPoint.x * pSrcImg->nChannels + k] * weight[0] +
					pSrc[upLeftPoint.y * pSrcImg->widthStep + (upLeftPoint.x + 1) * pSrcImg->nChannels + k] * weight[1] +
					pSrc[(upLeftPoint.y + 1) * pSrcImg->widthStep + upLeftPoint.x * pSrcImg->nChannels + k] * weight[2] +
					pSrc[(upLeftPoint.y + 1) * pSrcImg->widthStep + (upLeftPoint.x + 1) * pSrcImg->nChannels + k] * weight[3];
			}
			
			for (k = 0; k < min(pDstImg->nChannels, pSrcImg->nChannels); k++)
			{ 
				pDst[i * pDstImg->widthStep + j * pDstImg->nChannels + k] = 
					result[k] < 0 ? 0 : (result[k] > 255 ? 255 : (uchar)result[k]);
			}
		}
	}
}

