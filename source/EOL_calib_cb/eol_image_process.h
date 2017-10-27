#include "eol_opencv_adapter.h"
#include "eol_data_type.h"
#include "eol_error.h"

void bilinear_interpolation(IplImage* srcImg, 
							int32_t rgb[3], 
							float64_t xSrcLoc, 
							float64_t ySrcLoc);

void bgr_to_gray(IplImage* rgb_img, IplImage* gray_img);

void bgr_to_yuv420(const IplImage* rgb_img, uchar* yuv420_img);

void yuv420_to_bgr(IplImage *pBgrImg, uchar* pYuv420Img);

void yuv422_to_bgr(IplImage *pBgrImg, uchar* pYuv422Img);

void nv12_to_bgr(IplImage *pBgrImg, uchar* pNv12Img);

void yuv420_cut(uchar* pDstYuv420Img, uchar* pSrcYuv420Img, 
				int32_t src_height, int32_t src_width, 
				int32_t dst_height, int32_t dst_width);

void yuv422_cut(uchar* pDstYuv422Img, uchar* pSrcYuv422Img, 
				int32_t src_height, int32_t src_width, 
				int32_t dst_height, int32_t dst_width);

void nv12_cut(uchar* pDstNv12Img, uchar* pSrcNv12Img, 
			  int32_t src_height, int32_t src_width, 
			  int32_t dst_height, int32_t dst_width);

void rgba_cut(uchar* pDstRgbaImg, uchar* pSrcRgbaImg, 
			  int32_t src_height, int32_t src_width, 
			  int32_t dst_height, int32_t dst_width);

void flip_image(IplImage *pBgrImg, bool is_horizon);

void raw_to_bgr(IplImage *pBgrImg, uchar* pRawImg);

void rgba_to_bgr(IplImage *pBgrImg, uchar* pRawImg);

int load_image(FILE* pFile, uchar* pImage, int32_t height, 
			   int32_t width, EOL_Source_Image_Type_T image_type);

void split_raw_data(const char* file_name, const char* path_name, 
					const int32_t start_frame, const int32_t frame_num, 
					const int32_t height, const int32_t width);

void image_copy_roi(const IplImage* pSrcImg, IplImage* pDstImg, CvRect roi);

void split_raw_data2(const char* file_name, const char* path_name,
	const int32_t start_frame, const int32_t frame_num,
	const int32_t height, const int32_t width);

