#include "eol_opencv_adapter.h"

#if OPENCV == 0
extern IplImage* temp_img_copy, *norm_img;
/* initializes 8-element array for fast access to 3x3 neighborhood of a pixel */
#define  CV_INIT_3X3_DELTAS( deltas, step, nch )            \
    ((deltas)[0] =  (nch),  (deltas)[1] = -(step) + (nch),  \
     (deltas)[2] = -(step), (deltas)[3] = -(step) - (nch),  \
     (deltas)[4] = -(nch),  (deltas)[5] =  (step) - (nch),  \
     (deltas)[6] =  (step), (deltas)[7] =  (step) + (nch))

//static const CvPoint icvCodeDeltas[8] =
//    { {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1} };

static const int icvCodeDeltas[8][2] =
{ {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1} };

static const schar icvPower2ShiftTab[] =
{
    0, 1, -1, 2, -1, -1, -1, 3, -1, -1, -1, -1, -1, -1, -1, 4,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 5
};
#define cmp_pts( pt1, pt2 )  \
        ((pt1)->x < (pt2)->x || (pt1)->x <= (pt2)->x && (pt1)->y < (pt2)->y)

#define ICV_FREE_PTR(storage)  \
    ((schar*)(storage)->top + (storage)->block_size - (storage)->free_space)
#define ICV_ALIGNED_SEQ_BLOCK_SIZE  \
    (int)cvAlign(sizeof(CvSeqBlock), CV_STRUCT_ALIGN)
#define CV_GET_LAST_ELEM( seq, block ) \
    ((block)->data + ((block)->count - 1)*((seq)->elem_size))

static CV_IMPLEMENT_QSORT( icvSortPointsByPointers_32s, CvPoint*, cmp_pts )
static CV_IMPLEMENT_QSORT( icvSortPointsByPointers_32f, CvPoint2D32f*, cmp_pts )

#define CV_WRITE_SEQ_ELEM( elem, writer )             \
{                                                     \
    assert( (writer).seq->elem_size == sizeof(elem)); \
    if( (writer).ptr >= (writer).block_max )          \
    {                                                 \
        cvCreateSeqBlock( &writer);                   \
    }                                                 \
    assert( (writer).ptr <= (writer).block_max - sizeof(elem));\
    memcpy((writer).ptr, &(elem), sizeof(elem));      \
    (writer).ptr += sizeof(elem);                     \
}

#define CV_NEXT_SEQ_ELEM( elem_size, reader )                 \
{                                                             \
    if( ((reader).ptr += (elem_size)) >= (reader).block_max ) \
    {                                                         \
        cvChangeSeqBlock( &(reader), 1 );                     \
    }                                                         \
}

#define CV_READ_SEQ_ELEM( elem, reader )                       \
{                                                              \
    assert( (reader).seq->elem_size == sizeof(elem));          \
    memcpy( &(elem), (reader).ptr, sizeof((elem)));            \
    CV_NEXT_SEQ_ELEM( sizeof(elem), reader )                   \
}

#define CV_READ_CHAIN_POINT( _pt, reader )                              \
{                                                                       \
    (_pt) = (reader).pt;                                                \
    if( (reader).ptr )                                                  \
    {                                                                   \
        CV_READ_SEQ_ELEM( (reader).code, (reader));                     \
        assert( ((reader).code & ~7) == 0 );                            \
        (reader).pt.x += (reader).deltas[(int)(reader).code][0];        \
        (reader).pt.y += (reader).deltas[(int)(reader).code][1];        \
    }                                                                   \
}

#define CV_MEMCPY_INT( dst, src, len )                                              \
{                                                                                   \
    size_t _icv_memcpy_i_, _icv_memcpy_len_ = (len);                                \
    int* _icv_memcpy_dst_ = (int*)(dst);                                            \
    const int* _icv_memcpy_src_ = (const int*)(src);                                \
    assert( ((size_t)_icv_memcpy_src_&(sizeof(int)-1)) == 0 &&                      \
            ((size_t)_icv_memcpy_dst_&(sizeof(int)-1)) == 0 );                      \
                                                                                    \
    for(_icv_memcpy_i_=0;_icv_memcpy_i_<_icv_memcpy_len_;_icv_memcpy_i_++)          \
        _icv_memcpy_dst_[_icv_memcpy_i_] = _icv_memcpy_src_[_icv_memcpy_i_];        \
}

#define CV_MEMCPY_AUTO( dst, src, len )                                             \
{                                                                                   \
    size_t _icv_memcpy_i_, _icv_memcpy_len_ = (len);                                \
    char* _icv_memcpy_dst_ = (char*)(dst);                                          \
    const char* _icv_memcpy_src_ = (const char*)(src);                              \
    if( (_icv_memcpy_len_ & (sizeof(int)-1)) == 0 )                                 \
    {                                                                               \
        assert( ((size_t)_icv_memcpy_src_&(sizeof(int)-1)) == 0 &&                  \
                ((size_t)_icv_memcpy_dst_&(sizeof(int)-1)) == 0 );                  \
        for( _icv_memcpy_i_ = 0; _icv_memcpy_i_ < _icv_memcpy_len_;                 \
            _icv_memcpy_i_+=sizeof(int) )                                           \
        {                                                                           \
            *(int*)(_icv_memcpy_dst_+_icv_memcpy_i_) =                              \
            *(const int*)(_icv_memcpy_src_+_icv_memcpy_i_);                         \
        }                                                                           \
    }                                                                               \
    else                                                                            \
    {                                                                               \
        for(_icv_memcpy_i_ = 0; _icv_memcpy_i_ < _icv_memcpy_len_; _icv_memcpy_i_++)\
            _icv_memcpy_dst_[_icv_memcpy_i_] = _icv_memcpy_src_[_icv_memcpy_i_];    \
    }                                                                               \
}


#if OPENCV

#else
///////////////////////////////// Range /////////////////////////////////

inline
Range::Range()
    : start(0), end(0) {}

inline
Range::Range(int _start, int _end)
    : start(_start), end(_end) {}

inline
int Range::size() const
{
    return end - start;
}

inline
bool Range::empty() const
{
    return start == end;
}

inline
Range Range::all()
{
    return Range(INT_MIN, INT_MAX);
}


static inline
bool operator == (const Range& r1, const Range& r2)
{
    return r1.start == r2.start && r1.end == r2.end;
}

static inline
bool operator != (const Range& r1, const Range& r2)
{
    return !(r1 == r2);
}

static inline
bool operator !(const Range& r)
{
    return r.start == r.end;
}

static inline
Range operator & (const Range& r1, const Range& r2)
{
	Range r(MAX(r1.start, r2.start), MIN(r1.end, r2.end));
    r.end = MAX(r.end, r.start);
    return r;
}

static inline
Range& operator &= (Range& r1, const Range& r2)
{
    r1 = r1 & r2;
    return r1;
}

static inline
Range operator + (const Range& r1, int delta)
{
    return Range(r1.start + delta, r1.end + delta);
}

static inline
Range operator + (int delta, const Range& r1)
{
    return Range(r1.start + delta, r1.end + delta);
}

static inline
Range operator - (const Range& r1, int delta)
{
    return r1 + (-delta);
}

/////////////////////////////// AutoBuffer implementation ////////////////////////////////////////

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::AutoBuffer()
{
    ptr = buf;
    sz = fixed_size;
}

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::AutoBuffer(size_t _size)
{
    ptr = buf;
    sz = fixed_size;
    allocate(_size);
}

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::AutoBuffer(const AutoBuffer<_Tp, fixed_size>& abuf )
{
    ptr = buf;
    sz = fixed_size;
    allocate(abuf.size());
    for( size_t i = 0; i < sz; i++ )
        ptr[i] = abuf.ptr[i];
}

template<typename _Tp, size_t fixed_size> inline AutoBuffer<_Tp, fixed_size>&
AutoBuffer<_Tp, fixed_size>::operator = (const AutoBuffer<_Tp, fixed_size>& abuf)
{
    if( this != &abuf )
    {
        deallocate();
        allocate(abuf.size());
        for( size_t i = 0; i < sz; i++ )
            ptr[i] = abuf.ptr[i];
    }
    return *this;
}

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::~AutoBuffer()
{ deallocate(); }

template<typename _Tp, size_t fixed_size> inline void
AutoBuffer<_Tp, fixed_size>::allocate(size_t _size)
{
    if(_size <= sz)
    {
        sz = _size;
        return;
    }
    deallocate();
    if(_size > fixed_size)
    {
        ptr = new _Tp[_size];
        sz = _size;
    }
}

template<typename _Tp, size_t fixed_size> inline void
AutoBuffer<_Tp, fixed_size>::deallocate()
{
    if( ptr != buf )
    {
        delete[] ptr;
        ptr = buf;
        sz = fixed_size;
    }
}

template<typename _Tp, size_t fixed_size> inline void
AutoBuffer<_Tp, fixed_size>::resize(size_t _size)
{
    if(_size <= sz)
    {
        sz = _size;
        return;
    }
    size_t i, prevsize = sz, minsize = MIN(prevsize, _size);
    _Tp* prevptr = ptr;

    ptr = _size > fixed_size ? new _Tp[_size] : buf;
    sz = _size;

    if( ptr != prevptr )
        for( i = 0; i < minsize; i++ )
            ptr[i] = prevptr[i];
    for( i = prevsize; i < _size; i++ )
        ptr[i] = _Tp();

    if( prevptr != buf )
        delete[] prevptr;
}

template<typename _Tp, size_t fixed_size> inline size_t
AutoBuffer<_Tp, fixed_size>::size() const
{ return sz; }

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::operator _Tp* ()
{ return ptr; }

template<typename _Tp, size_t fixed_size> inline
AutoBuffer<_Tp, fixed_size>::operator const _Tp* () const
{ return ptr; }

CvSlice  cvSlice( int32_t start, int32_t end )
{
    CvSlice slice;
    slice.start_index = start;
    slice.end_index = end;

    return slice;
}

CvPoint  cvPoint( int32_t x, int32_t y )
{
	CvPoint p;
	p.x = x;
	p.y = y;
	return p;
}

CvPoint2D32f  cvPoint2D32f( float32_t x, float32_t y )
{
	CvPoint2D32f p;

	p.x = (float)x;
	p.y = (float)y;

	return p;
}


CvScalar  cvScalar( float32_t val0, float32_t val1 ,
				   float32_t val2 , float32_t val3 )
{
	CvScalar scalar;
	scalar.val[0] = val0; scalar.val[1] = val1;
	scalar.val[2] = val2; scalar.val[3] = val3;
	return scalar;
}

CvRect  cvRect( int32_t x, int32_t y, int32_t width, int32_t height )
{
	CvRect r;

	r.x = x;
	r.y = y;
	r.width = width;
	r.height = height;

	return r;
}

CvSize  cvSize( int32_t width, int32_t height )
{
	CvSize s;

	s.width = width;
	s.height = height;

	return s;
}



IplImage *cvCreateImage(CvSize size, int32_t depth, int32_t channels)
{
	IplImage *psimageheader = cvCreateImageHeader(size, depth ,channels);//
#if (1)//TEST_PCEM==1)

	int8_t* data = (int8_t *)malloc(size.width*size.height*depth/8*channels);
#else
	//IplImage *psimageheader = (IplImage *)MallocDynMemory(sizeof(IplImage));
	int8_t* data = (int8_t *)MallocDynMemory(size.width*size.height*depth/8*channels);
#endif
	psimageheader->width = size.width;
	psimageheader->height =  size.height;
	psimageheader->imageData = (char*)data;
	psimageheader->nChannels = channels;
	psimageheader->depth = depth;
	psimageheader->widthStep = size.width*channels*depth/8;
	psimageheader->origin=0;
	psimageheader->imageSize = psimageheader->widthStep* size.height;

	return psimageheader;
}

IplImage *cvCreateImage(Size size, int32_t depth, int32_t channels)
{

	IplImage *psimageheader = (IplImage *)malloc(sizeof(IplImage));
#if (1)//TEST_PCEM==1)

	int8_t* data = (int8_t *)malloc(size.width*size.height*depth/8*channels);
#else
	//IplImage *psimageheader = (IplImage *)MallocDynMemory(sizeof(IplImage));
	int8_t* data = (int8_t *)MallocDynMemory(size.width*size.height*depth/8*channels);
#endif
	psimageheader->width = size.width;
	psimageheader->height =  size.height;
	psimageheader->imageData = (char*)data;
	psimageheader->nChannels = channels;
	psimageheader->depth = depth;
	psimageheader->widthStep = size.width*channels*depth/8;
	psimageheader->origin=0;
	psimageheader->imageSize = psimageheader->widthStep* size.height;

	return psimageheader;
}

IplImage *cvCreateImageHeader( CvSize size, int32_t depth, int32_t channels )
{


#if (1)//TEST_PCEM==1)
	IplImage *psimageheader = (IplImage *)malloc(sizeof(IplImage));
#else
	IplImage *psimageheader = (IplImage *)MallocDynMemory(sizeof(IplImage));
#endif
	psimageheader->width = size.width;
	psimageheader->height =  size.height;
	psimageheader->nChannels = channels;
	psimageheader->depth = depth;
	psimageheader->widthStep = size.width*channels*depth/8;
	psimageheader->origin=0;
	psimageheader->imageSize = psimageheader->widthStep* size.height;

	return psimageheader;
}

void cvReleaseImageHeader(IplImage **image)
{
	if( !image )
		return;
	if( image )
	{
		IplImage *img = *image;
		*image = 0;
		free(img);
	}
}


void cvReleaseImage(IplImage **image)
{
	if( !image )
		return;
	if( image )
	{
		IplImage *img = *image;
		*image = 0;
		free(img->imageData);
		free(img);
	}
}

void cvSetZero(IplImage* dst)
{
	if (NULL == dst)
		return;

	int i,j,k;
	int height = dst->height;
	int width = dst->width;
	int step = dst->widthStep;
	int channels = dst->nChannels;
	uchar* data = (uchar*)dst->imageData;
	for (i=0; i<height; ++i)
	{
		for (j=0; j<width; ++j)
		{
			for (k=0; k<channels; ++k)
			{
				data[i*step+j*channels+k] = 0;
			}
		}
	}
}

void cvCopy(const IplImage* src, IplImage* dst)
{
	if( dst->depth != src->depth )
    {
		printf("Error\n");
		return;
	}

	if( dst->nChannels != src->nChannels)
	{
		printf("Error\n");
		return;
	}

    if( src->imageSize != dst->imageSize )
    {
		printf("Error\n");
		return;
	}

	int i,j,k;
	int height = src->height;
	int width = src->width;
	int depth = src->depth;
	int step = src->widthStep;
	int channels = src->nChannels;
	CvSize size = cvSize(width, height);
	uchar* data = (uchar*)src->imageData;
	uchar* data_copy = (uchar*)dst->imageData;
	for (i=0; i<height; ++i)
	{
		for (j=0; j<width; ++j)
		{
			for (k=0; k<channels; ++k)
			{
				data_copy[i*step+j*channels+k] 
				      = data[i*step+j*channels+k];
			}
		}
	}
}

void cvCvtColor( const IplImage* src, IplImage* dst, int code )
{
	if( dst->depth != IPL_DEPTH_8U )
    {
		printf("Error\n");
		return;
	}

	if( dst->nChannels != 1)
	{
		printf("Error\n");
		return;
	}

    if( (src->width != dst->width)||(src->height != dst->height) )
    {
		printf("Error\n");
		return;
	}

	int i,j;
	int height = src->height;
	int width = src->width;
	int step = src->widthStep;
	int step1 = dst->widthStep;
	int channels = src->nChannels;
	uchar* data_rgb = (uchar*)src->imageData;
	uchar* data_gray = (uchar*)dst->imageData;
	for (i=0; i<height; ++i)
	{
		for (j=0; j<width; ++j)
		{
			uchar B = data_rgb[i*step+j*channels];
			uchar G = data_rgb[i*step+j*channels+1];
			uchar R = data_rgb[i*step+j*channels+2];
			data_gray[i*step1+j] = 
				(int)(0.299*R + 0.587*G + 0.114*B);
		}
	}
}

//void GenerateKernel(double *kernel, int ksize, double sigma)
//{
//	int i, j;
//	double sum = 0.0;
//	double x2 = 0;
//	double y2 = 0;
//	int center = (ksize - 1)/2;
//	for(i=0; i<ksize; ++i)
//	{
//		x2 = pow(double(i - center), 2);
//		for(j=0; j<ksize; ++j)
//		{
//			y2 = pow(double(j - center), 2);
//			kernel[i*ksize + j] 
//			= exp(-(x2 + y2)/(2.0*sigma*sigma));
//			sum += kernel[i*ksize + j];
//		}
//	}
//	for(i=0; i<ksize; ++i)
//	{
//		for(j=0; j<ksize; j++)
//		{
//			kernel[i*ksize + j] /= sum;
//		}
//	}
//}
//
//void cvGaussianSmooth(const IplImage* src, IplImage* dst, int ksize, double sigma)
//{
//	if( ksize <= 1 || (ksize&1) == 0 )
//    {
//		printf("kernel size must be >=3 and odd (3, 5, 7, ...)\n");
//		return;
//
//	}
//
//	if( dst->imageSize != src->imageSize )
//    {
//		printf("UnmatchedSizes\n");
//		return;
//	}
//
//	if( dst->nChannels != src->nChannels)
//	{
//		printf("UnsupportedFormat\n");
//		return;
//	}
//
//	double *kernel = (double *)malloc(ksize*ksize*sizeof(double));
//	GenerateKernel(kernel, ksize, sigma);
//	int i,j,k,index;
//	double sum;
//	int height = dst->height;
//	int width = dst->width;
//	int step = dst->widthStep;
//	int channels = dst->nChannels;
//	uchar* data1 = (uchar*)src->imageData;
//	uchar* data2 = (uchar*)dst->imageData;
//	for(i=(ksize-1)/2; i<(height-(ksize-1)/2); ++i)
//	{
//		for(j=(ksize-1)/2; j<(width-(ksize-1)/2); ++j)
//		{
//			for(k=0; k<channels; ++k)
//			{
//				sum = 0.0;
//				index = 0;
//				for(int m=i-(ksize-1)/2; m<i+(ksize-1)/2+1; ++m)
//				{
//					for(int n=j-(ksize-1)/2; n<j+(ksize-1)/2+1; ++n)
//					{
//						sum += data1[m*step + n*channels + k]
//						* kernel[index++];
//					}
//				}
//				data2[i*step + j*channels + k] = (uchar)sum;
//			}
//			
//		}
//	}
//	free(kernel);
//}

void GenerateKernel(double *kernel, int ksize, double sigma)
{
	int i;
	double sum = 0.0;
	double d = 0.0;
	int center = (ksize - 1)/2;
	for(i=0; i<ksize; ++i)
	{
		d = pow(double(i - center), 2);
		kernel[i] 
		= exp(-d / (2.0*sigma*sigma));
		sum += kernel[i];
	}
	for(i=0; i<ksize; ++i)
	{
		kernel[i] /= sum;
	}
}

void cvGaussianSmooth(const IplImage* src, IplImage* dst, int ksize, double sigma)
{
	if( ksize <= 1 || (ksize&1) == 0 )
    {
		printf("kernel size must be >=3 and odd (3, 5, 7, ...)\n");
		return;

	}

	if( dst->imageSize != src->imageSize )
    {
		printf("UnmatchedSizes\n");
		return;
	}

	if( dst->nChannels != src->nChannels)
	{
		printf("UnsupportedFormat\n");
		return;
	}

	double *kernel;
	if ((kernel = (double *)malloc(ksize*sizeof(double))) == NULL)
	{
		printf("malloc memory for kernel, failed!");  
		return;
	}

	GenerateKernel(kernel, ksize, sigma);
	int i,j,k,index;
	double sum;
	int height = src->height;
	int width = src->width;
	int depth = src->depth;
	int step = src->widthStep;
	int channels = src->nChannels;
	CvSize Size = cvSize(width, height);
	IplImage *tmp = temp_img_copy;
	cvCopy(src, tmp);
	uchar* data1 = (uchar*)src->imageData;
	uchar* data2 = (uchar*)tmp->imageData;
	uchar* data3 = (uchar*)dst->imageData;

	for(i=0; i<height; ++i)
	{
		for(j=(ksize-1)/2; j<(width-(ksize-1)/2); ++j)
		{
			for(k=0; k<channels; ++k)
			{
				sum = 0.0;
				index = 0;
				for(int n=j-(ksize-1)/2; n<j+(ksize-1)/2+1; ++n)
				{
					sum += data1[i*step + n*channels + k]
					* kernel[index++];
				}
				data2[i*step + j*channels + k] = (uchar)sum;
			}
		}
	}

	for(j=0; j<width; ++j)
	{
		for(i=(ksize-1)/2; i<(height-(ksize-1)/2); ++i)
		{
			for(k=0; k<channels; ++k)
			{
				sum = 0.0;
				index = 0;
				for(int n=i-(ksize-1)/2; n<i+(ksize-1)/2+1; ++n)
				{
					sum += data2[n*step + j*channels + k]
					* kernel[index++];
				}
				data3[i*step + j*channels + k] = (uchar)sum;
			}
		}
	}
	free(kernel);
}

void adaptiveThreshold( const IplImage* src, IplImage* dst,
					    double maxValue, int type, int blockSize, double delta )
{
    assert( src->depth == IPL_DEPTH_8U );
    assert( blockSize % 2 == 1 && blockSize > 1 );

	int i, j, height, width;
	height = src->height;
    width = src->width;

    if( maxValue < 0 )
    {
        cvSetZero(dst);
        return;
    }

    IplImage* mean = 0;

    if( src->imageData != dst->imageData )
        mean = dst;
    else
        mean = cvCreateImage( CvSize(width, height), IPL_DEPTH_8U, 1 );

    cvGaussianSmooth(src, mean, blockSize, ((blockSize-1)*0.5 - 1)*0.9 + 0.8);

    uchar imaxval;
	if( maxValue > 255 )
        imaxval = 255;
	else if ( maxValue < 0)
		imaxval = 0;
	else
		imaxval = maxValue;

    int idelta = type == CV_THRESH_BINARY ? cvCeil(delta) : cvFloor(delta);
    uchar tab[768];

    if( type == CV_THRESH_BINARY )
        for( i = 0; i < 768; i++ )
            tab[i] = (uchar)(i - 255 > -idelta ? imaxval : 0);
    else if( type == CV_THRESH_BINARY_INV )
        for( i = 0; i < 768; i++ )
            tab[i] = (uchar)(i - 255 <= -idelta ? imaxval : 0);
    else
	{
	   printf("Unknown/unsupported threshold type\n" );
	   return;
	}

	for( i = 0; i < height; i++ )
    {
        const uchar* sdata = (uchar*)src->imageData + i*src->widthStep;
        const uchar* mdata = (uchar*)mean->imageData + i*mean->widthStep;
        uchar* ddata = (uchar*)dst->imageData + i*dst->widthStep;

        for( j = 0; j < width; j++ )
            ddata[j] = tab[sdata[j] - mdata[j] + 255];
    }

}

void cvAdaptiveThreshold( const IplImage* srcIm, IplImage* dstIm, double maxValue, int adaptive_method CV_DEFAULT(CV_ADAPTIVE_THRESH_MEAN_C),
						 int type CV_DEFAULT(CV_THRESH_BINARY), int blockSize CV_DEFAULT(3), double delta CV_DEFAULT(5))
{
    assert( srcIm->imageSize == dstIm->imageSize && srcIm->depth == dstIm->depth );
    adaptiveThreshold( srcIm, dstIm, maxValue, type, blockSize, delta );
}

//void cvDilate( const IplImage* src, IplImage* dst, int blockSize, int mode, int iterations )
//{
//	if ((mode != CV_SHAPE_RECT) && (mode != CV_SHAPE_CROSS))
//	{
//		printf("Error kernel mode\n");
//		return;
//	}
//
//	int height = src->height;
//	int width = src->width;
//	int depth = src->depth;
//	int step = src->widthStep;
//	int channels = src->nChannels;
//	int mid = (blockSize - 1) / 2;
//    uchar val = 0;
//	CvSize Size = cvSize(width, height);
//	IplImage *tmp = temp_img_copy;
//	cvCopy(src, tmp);
//	uchar* data1 = (uchar*)tmp->imageData;
//	uchar* data2 = (uchar*)dst->imageData;
//
//	if (mode == CV_SHAPE_RECT)
//	{
//		for (int k = 0; k < iterations; k++)
//	    {
//		    for (int i = mid; i < height - mid; i++)
//		    {
//			    for (int j = mid; j < width - mid; j++)
//			    {
//				    for (int m = -mid; m < mid+1; m++)
//				    {
//					    for (int n = -mid; n < mid+1; n++)
//					    {
//						    val |= data1[(i + m) * width + j + n];
//					    }
//				    }
//				
//				    data2[i * width + j] = val;
//				    val = 0;
//			    }
//		    }
//			cvCopy(dst, tmp);
//	    }
//	}
//
//	if (mode == CV_SHAPE_CROSS)
//	{
//		for (int k = 0; k < iterations; k++)
//	    {
//		    for (int i = mid; i < height - mid; i++)
//		    {
//			    for (int j = mid; j < width - mid; j++)
//			    {
//				    for (int m = -mid; m < mid+1; m++)
//				    {
//					    for (int n = -mid; n < mid+1; n++)
//					    {
//							if (m == 0 || n == 0)
//								val |= data1[(i + m) * width + j + n];
//					    }
//				    }
//				
//				    data2[i * width + j] = val;
//				    val = 0;
//			    }
//		    }
//			cvCopy(dst, tmp);
//	    }
//	}
//	cvCopy(tmp, dst);
//}

void cvDilate( const IplImage* src, IplImage* dst, int blockSize, int mode, int iterations )
{
	if ((mode != CV_SHAPE_RECT) && 
		(mode != CV_SHAPE_CROSS) && 
		(mode != CV_SHAPE_BAR_VERT_ALL) &&
		(mode != CV_SHAPE_BAR_VERT_UP) &&
		(mode != CV_SHAPE_BAR_VERT_DOWN) &&
		(mode != CV_SHAPE_BAR_HORI_ALL) &&
		(mode != CV_SHAPE_BAR_HORI_LEFT) &&
		(mode != CV_SHAPE_BAR_HORI_RIGHT) &&
		(mode != CV_SHAPE_BAR_INCL_048) &&
		(mode != CV_SHAPE_BAR_INCL_04) &&
		(mode != CV_SHAPE_BAR_INCL_48) &&
		(mode != CV_SHAPE_BAR_INCL_246) &&
		(mode != CV_SHAPE_BAR_INCL_24) &&
		(mode != CV_SHAPE_BAR_INCL_46)
		)
	{
		printf("Error kernel mode\n");
		return;
	}

	if((mode == CV_SHAPE_RECT || 
	    mode == CV_SHAPE_CROSS || 
	    mode == CV_SHAPE_BAR_VERT_ALL || 
	    mode == CV_SHAPE_BAR_HORI_ALL ||
		mode == CV_SHAPE_BAR_INCL_048 ||
		mode == CV_SHAPE_BAR_INCL_246) &&
		blockSize & 1 != 1 )
	{
		printf("blockSize should be odd in this mode\n");
		return;
	}

	int height = src->height;
	int width = src->width;
	int depth = src->depth;
	int step = src->widthStep;
	int channels = src->nChannels;
	int mid = (blockSize - 1) / 2;
	uchar val = 0;
	CvSize Size = cvSize(width, height);
	IplImage *tmp = norm_img;
	cvCopy(src, tmp);
	uchar* data1 = (uchar*)tmp->imageData;
	uchar* data2 = (uchar*)dst->imageData;

	switch(mode)
	{
	case CV_SHAPE_RECT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						for (int n = - mid; n < mid + 1; n++)
						{
							val |= data1[(i + m) * width + j + n];
						}
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_CROSS:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = -mid; m < mid+1; m++)
					{
						for (int n = -mid; n < mid+1; n++)
						{
							if (m == 0 || n == 0)
								val |= data1[(i + m) * width + j + n];
						}
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_ALL:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val |= data1[(i + m) * width + j];
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_UP:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					for (int m = - (blockSize - 1); m < 1; m++)
					{
						val |= data1[(i + m) * width + j];						
					}

					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_DOWN:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = 0; j < width; j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val |= data1[(i + m) * width + j];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_ALL:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int n = - mid; n < mid + 1; n++)
					{
						val |= data1[i * width + j + n];
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_LEFT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height; i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int n = - (blockSize - 1); n < 1; n++)
					{
						val |= data1[i * width + j + n];						
					}

					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_RIGHT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int n = 0; n < blockSize; n++)
					{
						val |= data1[i * width + j + n];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_048:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val |= data1[(i + m) * width + j + m];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_04:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int m = - (blockSize - 1); m < 1; m++)
					{
						val |= data1[(i + m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_48:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val |= data1[(i + m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_246:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val |= data1[(i + m) * width + j - m];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_24:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int m = 0; m < (blockSize - 1); m++)
					{
						val |= data1[(i - m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_46:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val |= data1[(i + m) * width + (j - m)];						
					}
					data2[i * width + j] = val;
					val = 0;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	}
	cvCopy(tmp, dst);
}

void cvErode( const IplImage* src, IplImage* dst, int blockSize, int mode, int iterations )
{
		if ((mode != CV_SHAPE_RECT) && 
		(mode != CV_SHAPE_CROSS) && 
		(mode != CV_SHAPE_BAR_VERT_ALL) &&
		(mode != CV_SHAPE_BAR_VERT_UP) &&
		(mode != CV_SHAPE_BAR_VERT_DOWN) &&
		(mode != CV_SHAPE_BAR_HORI_ALL) &&
		(mode != CV_SHAPE_BAR_HORI_LEFT) &&
		(mode != CV_SHAPE_BAR_HORI_RIGHT) &&
		(mode != CV_SHAPE_BAR_INCL_048) &&
		(mode != CV_SHAPE_BAR_INCL_04) &&
		(mode != CV_SHAPE_BAR_INCL_48) &&
		(mode != CV_SHAPE_BAR_INCL_246) &&
		(mode != CV_SHAPE_BAR_INCL_24) &&
		(mode != CV_SHAPE_BAR_INCL_46)
		)
	{
		printf("Error kernel mode\n");
		return;
	}

	if((mode == CV_SHAPE_RECT || 
	    mode == CV_SHAPE_CROSS || 
	    mode == CV_SHAPE_BAR_VERT_ALL || 
	    mode == CV_SHAPE_BAR_HORI_ALL ||
		mode == CV_SHAPE_BAR_INCL_048 ||
		mode == CV_SHAPE_BAR_INCL_246) &&
		blockSize & 1 != 1 )
	{
		printf("blockSize should be odd in this mode\n");
		return;
	}

	int height = src->height;
	int width = src->width;
	int depth = src->depth;
	int step = src->widthStep;
	int channels = src->nChannels;
	int mid = (blockSize - 1) / 2;
    uchar val = 255;
	CvSize Size = cvSize(width, height);
	IplImage *tmp = temp_img_copy;
	cvCopy(src, tmp);
	uchar* data1 = (uchar*)tmp->imageData;
	uchar* data2 = (uchar*)dst->imageData;

	switch(mode)
	{
	case CV_SHAPE_RECT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						for (int n = - mid; n < mid + 1; n++)
						{
							val &= data1[(i + m) * width + j + n];
						}
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_CROSS:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = -mid; m < mid+1; m++)
					{
						for (int n = -mid; n < mid+1; n++)
						{
							if (m == 0 || n == 0)
								val &= data1[(i + m) * width + j + n];
						}
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_ALL:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val &= data1[(i + m) * width + j];
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_UP:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					for (int m = - (blockSize - 1); m < 1; m++)
					{
						val &= data1[(i + m) * width + j];						
					}

					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_VERT_DOWN:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = 0; j < width; j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val &= data1[(i + m) * width + j];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_ALL:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int n = - mid; n < mid + 1; n++)
					{
						val &= data1[i * width + j + n];
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_LEFT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height; i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int n = - (blockSize - 1); n < 1; n++)
					{
						val &= data1[i * width + j + n];						
					}

					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_HORI_RIGHT:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int n = 0; n < blockSize; n++)
					{
						val &= data1[i * width + j + n];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_048:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val &= data1[(i + m) * width + j + m];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_04:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int m = - (blockSize - 1); m < 1; m++)
					{
						val &= data1[(i + m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_48:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val &= data1[(i + m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_246:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = mid; i < height - mid; i++)
			{
				for (int j = mid; j < width - mid; j++)
				{
					for (int m = - mid; m < mid + 1; m++)
					{
						val &= data1[(i + m) * width + j - m];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_24:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = blockSize - 1; i < height; i++)
			{
				for (int j = 0; j < width - (blockSize - 1); j++)
				{
					for (int m = 0; m < (blockSize - 1); m++)
					{
						val &= data1[(i - m) * width + (j + m)];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	case CV_SHAPE_BAR_INCL_46:
		for (int k = 0; k < iterations; k++)
		{
			for (int i = 0; i < height - (blockSize - 1); i++)
			{
				for (int j = blockSize - 1; j < width; j++)
				{
					for (int m = 0; m < blockSize; m++)
					{
						val &= data1[(i + m) * width + (j - m)];						
					}
					data2[i * width + j] = val;
					val = 255;
				}
			}
			cvCopy(dst, tmp);
		}
		break;
	}
	cvCopy(tmp, dst);
}

void myDilateV4(const IplImage *src,IplImage *dst,int radius)
{
    int i, j;
    unsigned char maxDist=254;
	int height = src->height;
	int width = src->width;
	int depth = src->depth;
	int step = src->widthStep;
	int channels = src->nChannels;
	CvSize Size = cvSize(width, height);
	IplImage *tmp = temp_img_copy;
	cvCopy(src, tmp);
	uchar* data1 = (uchar*)tmp->imageData;
	uchar* data2 = (uchar*)dst->imageData;

    //step1 forward
    //the top left pixel
    data2[0] = (data1[0] == 255) ? 0 : maxDist;
    //the first row
    for (i = 1; i < src->width; ++i)
    {
		data2[i] = (data1[i]==255) ? 0 : MIN(maxDist, data2[i-1] + 1);
	}
    //rest pixels
    for (i=1; i<src->height; ++i) // traverse from top left to bottom right
    {
        //left-most pixel
        data2[i * src->widthStep] = (data1[i * src->widthStep] == 255) ? 0 : 
			  MIN(MIN(data2[(i - 1) * src->widthStep],data2[(i - 1) * src->widthStep + 1]) + 1,maxDist);
        for (j = 1;j < src->width - 1; ++j)
        {
            if (data1[i * src->widthStep + j] == 255)
            {
                data2[i * src->widthStep + j] = 0;//first pass and pixel was on, it gets a zero
            }
            else
            {
                // pixel was off
                data2[i * src->widthStep + j] = MIN(maxDist, data2[(i - 1) * src->widthStep + j] + 1);
                data2[i * src->widthStep + j] = MIN(MIN(data2[i * src->widthStep + (j - 1)] + 1, data2[i * src->widthStep + j]),
					                                 MIN(data2[(i - 1) * src->widthStep + (j - 1)] + 1, data2[(i - 1) * src->widthStep + (j + 1) ] + 1));
            }
        }
        //right-most pixle
        data2[(i + 1) * src->widthStep - 1] = (data1[(i + 1) * src->widthStep - 1]==255) ? 0 : MIN
                                               (MIN(data2[i * src->widthStep - 1] + 1,data2[i * src->widthStep - 2] + 1),
											    MIN(data2[(i + 1) * src->widthStep - 2] + 1, maxDist));
    }
    //step2 backward
    //    the bottom right pixel
    data2[src->widthStep * src->height - 1] = (data1[src->widthStep * src->height - 1] == 255) ? 0 : data2[src->widthStep * src->height - 1];
    //the last row
    for (i = src->width - 2; i >= 0; --i)
    {
		data2[(src->height - 1) * src->widthStep + i] = (data1[(src->height - 1) * src->widthStep + i] == 255) ? 0 : 
			MIN(data2[(src->height - 1) * src->widthStep + i], data2[(src->height - 1) * src->widthStep + (i + 1)] + 1);
	}
    //rest pixles
    for (i = src->height - 2; i >= 0; --i)
	{
        //right-most pixel
        data2[(i + 1) * src->widthStep - 1] = (data1[(i + 1) * src->widthStep - 1] == 255) ? 0 : 
			MIN(MIN(data2[(i + 2) * src->widthStep - 1], data2[(i + 2) * src->widthStep - 2]) + 1, data2[(i + 1) * src->widthStep - 1]);
        for (j=src->width-2; j>0; --j)
        {
            if(data1[i*src->widthStep+j]!=255)
            {
                data2[i*src->widthStep+j] = MIN(data2[i*src->widthStep+j], data2[i*src->widthStep+j+1]+1);
                data2[i*src->widthStep+j] = MIN(
                                     MIN(data2[i*src->widthStep+j], data2[(i+1)*src->widthStep+j]+1),
                                     MIN(data2[(i+1)*src->widthStep+j+1]+1, data2[(i+1)*src->widthStep+j-1]+1)
                                     );
            }
        }
        //left-most pixel
        data2[i*src->widthStep]=(data1[i*src->widthStep]==255)?0:MIN
        (MIN(data2[i*src->widthStep+1],data2[(i+1)*src->widthStep])+1,
         MIN(data2[(i+1)*src->widthStep+1]+1,data2[i*src->widthStep]));
    }
    for (i=0;i<src->height;++i)
    {
        for (j=0;j<src->width;++j)
        {
            data2[i*src->widthStep+j] = ((data2[i*src->widthStep+j]<=radius)?255:data1[i*src->widthStep+j]);
        }
    }
}

/****************************************************************************************\
*                               Sequence implementation                                  *
\****************************************************************************************/

/* Create empty sequence: */
CvSeq *cvCreateSeq( int seq_flags, size_t header_size, size_t elem_size, CvMemStorage* storage )
{
    CvSeq *seq = 0;

    if( !storage )
        return 0;
    if( header_size < sizeof( CvSeq ) || elem_size <= 0 )
        return 0;

    /* allocate sequence header */
    seq = (CvSeq*)cvMemStorageAlloc( storage, header_size );
    memset( seq, 0, header_size );

    seq->header_size = (int)header_size;
    seq->flags = (seq_flags & ~CV_MAGIC_MASK) | CV_SEQ_MAGIC_VAL;
    {
        int elemtype = CV_MAT_TYPE(seq_flags);
        int typesize = CV_ELEM_SIZE(elemtype);

        if( elemtype != CV_SEQ_ELTYPE_GENERIC && elemtype != CV_USRTYPE1 &&
            typesize != 0 && typesize != (int)elem_size )
		{
			printf("Specified element size doesn't match to the size of the specified element type "
                "(try to use 0 for element type)\n");
			return 0;
		}
    }
    seq->elem_size = (int)elem_size;
    seq->storage = storage;

    cvSetSeqBlockSize( seq, (int)((1 << 10)/elem_size) );

    return seq;
}

/* adjusts <delta_elems> field of sequence. It determines how much the sequence
   grows if there are no free space inside the sequence buffers */
void cvSetSeqBlockSize( CvSeq *seq, int delta_elements )
{
    int elem_size;
    int useful_block_size;

    if( !seq || !seq->storage )
        return;
    if( delta_elements < 0 )
        return;

    useful_block_size = cvAlignLeft(seq->storage->block_size - sizeof(CvMemBlock) -
                                    sizeof(CvSeqBlock), CV_STRUCT_ALIGN);
    elem_size = seq->elem_size;

    if( delta_elements == 0 )
    {
        delta_elements = (1 << 10) / elem_size;
        delta_elements = MAX( delta_elements, 1 );
    }
    if( delta_elements * elem_size > useful_block_size )
    {
        delta_elements = useful_block_size / elem_size;
        if( delta_elements == 0 )
            printf("Storage block size is too small "
				"to fit the sequence elements\n");
			return;
    }

    seq->delta_elems = delta_elements;
}

/* Find a sequence element by its index: */
schar* cvGetSeqElem( const CvSeq *seq, int index )
{
    CvSeqBlock *block;
    int count, total = seq->total;

    if( (unsigned)index >= (unsigned)total )
    {
        index += index < 0 ? total : 0;
        index -= index >= total ? total : 0;
        if( (unsigned)index >= (unsigned)total )
            return 0;
    }

    block = seq->first;
    if( index + index <= total )
    {
        while( index >= (count = block->count) )
        {
            block = block->next;
            index -= count;
        }
    }
    else
    {
        do
        {
            block = block->prev;
            total -= block->count;
        }
        while( index < total );
        index -= total;
    }

    return block->data + index * seq->elem_size;
}

/* Calculate index of a sequence element: */
int cvSeqElemIdx( const CvSeq* seq, const void* _element, CvSeqBlock** _block )
{
    const schar *element = (const schar *)_element;
    int elem_size;
    int id = -1;
    CvSeqBlock *first_block;
    CvSeqBlock *block;

    if( !seq || !element )
        return -1;

    block = first_block = seq->first;
    elem_size = seq->elem_size;

    for( ;; )
    {
        if( (unsigned)(element - block->data) < (unsigned) (block->count * elem_size) )
        {
            if( _block )
                *_block = block;
            if( elem_size <= ICV_SHIFT_TAB_MAX && (id = icvPower2ShiftTab[elem_size - 1]) >= 0 )
                id = (int)((size_t)(element - block->data) >> id);
            else
                id = (int)((size_t)(element - block->data) / elem_size);
            id += block->start_index - seq->first->start_index;
            break;
        }
        block = block->next;
        if( block == first_block )
            break;
    }

    return id;
}

int cvSliceLength( CvSlice slice, const CvSeq* seq )
{
    int total = seq->total;
    int length = slice.end_index - slice.start_index;

    if( length != 0 )
    {
        if( slice.start_index < 0 )
            slice.start_index += total;
        if( slice.end_index <= 0 )
            slice.end_index += total;

        length = slice.end_index - slice.start_index;
    }

    while( length < 0 )
        length += total;
    if( length > total )
        length = total;

    return length;
}

/* Copy all sequence elements into single continuous array: */
void* cvCvtSeqToArray( const CvSeq *seq, void *array, CvSlice slice )
{
    int elem_size, total;
    CvSeqReader reader;
    char *dst = (char*)array;

    if( !seq || !array )
        return 0;

    elem_size = seq->elem_size;
    total = cvSliceLength( slice, seq )*elem_size;

    if( total == 0 )
        return 0;

    cvStartReadSeq( seq, &reader, 0 );
    cvSetSeqReaderPos( &reader, slice.start_index, 0 );

    do
    {
        int count = (int)(reader.block_max - reader.ptr);
        if( count > total )
            count = total;

        memcpy( dst, reader.ptr, count );
        dst += count;
        reader.block = reader.block->next;
        reader.ptr = reader.block->data;
        reader.block_max = reader.ptr + reader.block->count*elem_size;
        total -= count;
    }
    while( total > 0 );

    return array;
}

/* The function allocates space for at least one more sequence element.
   If there are free sequence blocks (seq->free_blocks != 0)
   they are reused, otherwise the space is allocated in the storage: */
static void icvGrowSeq( CvSeq *seq, int in_front_of )
{
    CvSeqBlock *block;

    if( !seq )
        return;
    block = seq->free_blocks;

    if( !block )
    {
        int elem_size = seq->elem_size;
        int delta_elems = seq->delta_elems;
        CvMemStorage *storage = seq->storage;

        if( seq->total >= delta_elems*4 )
            cvSetSeqBlockSize( seq, delta_elems*2 );

        if( !storage )
        {
			printf("The sequence has NULL storage pointer\n" );
			return;
		}

        /* If there is a free space just after last allocated block
           and it is big enough then enlarge the last block.
           This can happen only if the new block is added to the end of sequence: */
        if( (size_t)(ICV_FREE_PTR(storage) - seq->block_max) < CV_STRUCT_ALIGN &&
            storage->free_space >= seq->elem_size && !in_front_of )
        {
            int delta = storage->free_space / elem_size;

            delta = MIN( delta, delta_elems ) * elem_size;
            seq->block_max += delta;
            storage->free_space = cvAlignLeft((int)(((schar*)storage->top + storage->block_size) -
                                              seq->block_max), CV_STRUCT_ALIGN );
            return;
        }
        else
        {
            int delta = elem_size * delta_elems + ICV_ALIGNED_SEQ_BLOCK_SIZE;

            /* Try to allocate <delta_elements> elements: */
            if( storage->free_space < delta )
            {
                int small_block_size = MAX(1, delta_elems/3)*elem_size +
                                       ICV_ALIGNED_SEQ_BLOCK_SIZE;
                /* try to allocate smaller part */
                if( storage->free_space >= small_block_size + CV_STRUCT_ALIGN )
                {
                    delta = (storage->free_space - ICV_ALIGNED_SEQ_BLOCK_SIZE)/seq->elem_size;
                    delta = delta*seq->elem_size + ICV_ALIGNED_SEQ_BLOCK_SIZE;
                }
                else
                {
                    icvGoNextMemBlock( storage );
                    assert( storage->free_space >= delta );
                }
            }

            block = (CvSeqBlock*)cvMemStorageAlloc( storage, delta );
            block->data = (schar*)cvAlignPtr( block + 1, CV_STRUCT_ALIGN );
            block->count = delta - ICV_ALIGNED_SEQ_BLOCK_SIZE;
            block->prev = block->next = 0;
        }
    }
    else
    {
        seq->free_blocks = block->next;
    }

    if( !(seq->first) )
    {
        seq->first = block;
        block->prev = block->next = block;
    }
    else
    {
        block->prev = seq->first->prev;
        block->next = seq->first;
        block->prev->next = block->next->prev = block;
    }

    /* For free blocks the <count> field means
     * total number of bytes in the block.
     *
     * For used blocks it means current number
     * of sequence elements in the block:
     */
    assert( block->count % seq->elem_size == 0 && block->count > 0 );

    if( !in_front_of )
    {
        seq->ptr = block->data;
        seq->block_max = block->data + block->count;
        block->start_index = block == block->prev ? 0 :
            block->prev->start_index + block->prev->count;
    }
    else
    {
        int delta = block->count / seq->elem_size;
        block->data += block->count;

        if( block != block->prev )
        {
            assert( seq->first->start_index == 0 );
            seq->first = block;
        }
        else
        {
            seq->block_max = seq->ptr = block->data;
        }

        block->start_index = 0;

        for( ;; )
        {
            block->start_index += delta;
            block = block->next;
            if( block == seq->first )
                break;
        }
    }

    block->count = 0;
}

/* Recycle a sequence block: */
static void icvFreeSeqBlock( CvSeq *seq, int in_front_of )
{
    CvSeqBlock *block = seq->first;

    assert( (in_front_of ? block : block->prev)->count == 0 );

    if( block == block->prev )  /* single block case */
    {
        block->count = (int)(seq->block_max - block->data) + block->start_index * seq->elem_size;
        block->data = seq->block_max - block->count;
        seq->first = 0;
        seq->ptr = seq->block_max = 0;
        seq->total = 0;
    }
    else
    {
        if( !in_front_of )
        {
            block = block->prev;
            assert( seq->ptr == block->data );

            block->count = (int)(seq->block_max - seq->ptr);
            seq->block_max = seq->ptr = block->prev->data +
                block->prev->count * seq->elem_size;
        }
        else
        {
            int delta = block->start_index;

            block->count = delta * seq->elem_size;
            block->data -= block->count;

            /* Update start indices of sequence blocks: */
            for( ;; )
            {
                block->start_index -= delta;
                block = block->next;
                if( block == seq->first )
                    break;
            }

            seq->first = block->next;
        }

        block->prev->next = block->next;
        block->next->prev = block->prev;
    }

    assert( block->count > 0 && block->count % seq->elem_size == 0 );
    block->next = seq->free_blocks;
    seq->free_blocks = block;
}

/****************************************************************************************\
*                             Sequence Writer implementation                             *
\****************************************************************************************/

/* Initialize sequence writer: */
void cvStartAppendToSeq( CvSeq *seq, CvSeqWriter * writer )
{
    if( !seq || !writer )
        return;

    memset( writer, 0, sizeof( *writer ));
    writer->header_size = sizeof( CvSeqWriter );

    writer->seq = seq;
    writer->block = seq->first ? seq->first->prev : 0;
    writer->ptr = seq->ptr;
    writer->block_max = seq->block_max;
}


/* Initialize sequence writer: */
void cvStartWriteSeq( int seq_flags, int header_size,
                 int elem_size, CvMemStorage * storage, CvSeqWriter * writer )
{
    if( !storage || !writer )
        return;

    CvSeq* seq = cvCreateSeq( seq_flags, header_size, elem_size, storage );
    cvStartAppendToSeq( seq, writer );
}


/* Update sequence header: */
void cvFlushSeqWriter( CvSeqWriter * writer )
{
    if( !writer )
        return;

    CvSeq* seq = writer->seq;
    seq->ptr = writer->ptr;

    if( writer->block )
    {
        int total = 0;
        CvSeqBlock *first_block = writer->seq->first;
        CvSeqBlock *block = first_block;

        writer->block->count = (int)((writer->ptr - writer->block->data) / seq->elem_size);
        assert( writer->block->count > 0 );

        do
        {
            total += block->count;
            block = block->next;
        }
        while( block != first_block );

        writer->seq->total = total;
    }
}


/* Calls icvFlushSeqWriter and finishes writing process: */
CvSeq *cvEndWriteSeq( CvSeqWriter * writer )
{
    if( !writer )
        return 0;

    cvFlushSeqWriter( writer );
    CvSeq* seq = writer->seq;

    /* Truncate the last block: */
    if( writer->block && writer->seq->storage )
    {
        CvMemStorage *storage = seq->storage;
        schar *storage_block_max = (schar *) storage->top + storage->block_size;

        assert( writer->block->count > 0 );

        if( (unsigned)((storage_block_max - storage->free_space)
            - seq->block_max) < CV_STRUCT_ALIGN )
        {
            storage->free_space = cvAlignLeft((int)(storage_block_max - seq->ptr), CV_STRUCT_ALIGN);
            seq->block_max = seq->ptr;
        }
    }

    writer->ptr = 0;
    return seq;
}


/* Create new sequence block: */
void cvCreateSeqBlock( CvSeqWriter * writer )
{
    if( !writer || !writer->seq )
        return;

    CvSeq* seq = writer->seq;

    cvFlushSeqWriter( writer );

    icvGrowSeq( seq, 0 );

    writer->block = seq->first->prev;
    writer->ptr = seq->ptr;
    writer->block_max = seq->block_max;
}

/****************************************************************************************\
*                               Sequence Reader implementation                           *
\****************************************************************************************/

/* Initialize sequence reader: */
void cvStartReadSeq( const CvSeq *seq, CvSeqReader * reader, int reverse )
{
    CvSeqBlock *first_block;
    CvSeqBlock *last_block;

    if( reader )
    {
        reader->seq = 0;
        reader->block = 0;
        reader->ptr = reader->block_max = reader->block_min = 0;
    }

    if( !seq || !reader )
        return;

    reader->header_size = sizeof( CvSeqReader );
    reader->seq = (CvSeq*)seq;

    first_block = seq->first;

    if( first_block )
    {
        last_block = first_block->prev;
        reader->ptr = first_block->data;
        reader->prev_elem = CV_GET_LAST_ELEM( seq, last_block );
        reader->delta_index = seq->first->start_index;

        if( reverse )
        {
            schar *temp = reader->ptr;

            reader->ptr = reader->prev_elem;
            reader->prev_elem = temp;

            reader->block = last_block;
        }
        else
        {
            reader->block = first_block;
        }

        reader->block_min = reader->block->data;
        reader->block_max = reader->block_min + reader->block->count * seq->elem_size;
    }
    else
    {
        reader->delta_index = 0;
        reader->block = 0;

        reader->ptr = reader->prev_elem = reader->block_min = reader->block_max = 0;
    }
}


/* Change the current reading block
 * to the previous or to the next:
 */
void cvChangeSeqBlock( void* _reader, int direction )
{
    CvSeqReader* reader = (CvSeqReader*)_reader;

    if( !reader )
        return;

    if( direction > 0 )
    {
        reader->block = reader->block->next;
        reader->ptr = reader->block->data;
    }
    else
    {
        reader->block = reader->block->prev;
        reader->ptr = CV_GET_LAST_ELEM( reader->seq, reader->block );
    }
    reader->block_min = reader->block->data;
    reader->block_max = reader->block_min + reader->block->count * reader->seq->elem_size;
}


/* Return the current reader position: */
int cvGetSeqReaderPos( CvSeqReader* reader )
{
    int elem_size;
    int index = -1;

    if( !reader || !reader->ptr )
        return -1;

    elem_size = reader->seq->elem_size;
    if( elem_size <= ICV_SHIFT_TAB_MAX && (index = icvPower2ShiftTab[elem_size - 1]) >= 0 )
        index = (int)((reader->ptr - reader->block_min) >> index);
    else
        index = (int)((reader->ptr - reader->block_min) / elem_size);

    index += reader->block->start_index - reader->delta_index;

    return index;
}


/* Set reader position to given position,
 * either absolute or relative to the
 *  current one:
 */
void cvSetSeqReaderPos( CvSeqReader* reader, int index, int is_relative )
{
    CvSeqBlock *block;
    int elem_size, count, total;

    if( !reader || !reader->seq )
        return;

    total = reader->seq->total;
    elem_size = reader->seq->elem_size;

    if( !is_relative )
    {
        if( index < 0 )
        {
            if( index < -total )
                return;
            index += total;
        }
        else if( index >= total )
        {
            index -= total;
            if( index >= total )
                return;
        }

        block = reader->seq->first;
        if( index >= (count = block->count) )
        {
            if( index + index <= total )
            {
                do
                {
                    block = block->next;
                    index -= count;
                }
                while( index >= (count = block->count) );
            }
            else
            {
                do
                {
                    block = block->prev;
                    total -= block->count;
                }
                while( index < total );
                index -= total;
            }
        }
        reader->ptr = block->data + index * elem_size;
        if( reader->block != block )
        {
            reader->block = block;
            reader->block_min = block->data;
            reader->block_max = block->data + block->count * elem_size;
        }
    }
    else
    {
        schar* ptr = reader->ptr;
        index *= elem_size;
        block = reader->block;

        if( index > 0 )
        {
            while( ptr + index >= reader->block_max )
            {
                int delta = (int)(reader->block_max - ptr);
                index -= delta;
                reader->block = block = block->next;
                reader->block_min = ptr = block->data;
                reader->block_max = block->data + block->count*elem_size;
            }
            reader->ptr = ptr + index;
        }
        else
        {
            while( ptr + index < reader->block_min )
            {
                int delta = (int)(ptr - reader->block_min);
                index += delta;
                reader->block = block = block->prev;
                reader->block_min = block->data;
                reader->block_max = ptr = block->data + block->count*elem_size;
            }
            reader->ptr = ptr + index;
        }
    }
}


/* Push element onto the sequence: */
schar* cvSeqPush( CvSeq *seq, const void *element )
{
    schar *ptr = 0;
    size_t elem_size;

    if( !seq )
        return 0;

    elem_size = seq->elem_size;
    ptr = seq->ptr;

    if( ptr >= seq->block_max )
    {
        icvGrowSeq( seq, 0 );

        ptr = seq->ptr;
        assert( ptr + elem_size <= seq->block_max /*&& ptr == seq->block_min */  );
    }

    if( element )
        memcpy( ptr, element, elem_size );
    seq->first->prev->count++;
    seq->total++;
    seq->ptr = ptr + elem_size;

    return ptr;
}


/* Pop last element off of the sequence: */
void cvSeqPop( CvSeq *seq, void *element )
{
    schar *ptr;
    int elem_size;

    if( !seq )
        return;
    if( seq->total <= 0 )
        return;

    elem_size = seq->elem_size;
    seq->ptr = ptr = seq->ptr - elem_size;

    if( element )
        memcpy( element, ptr, elem_size );
    seq->ptr = ptr;
    seq->total--;

    if( --(seq->first->prev->count) == 0 )
    {
        icvFreeSeqBlock( seq, 0 );
        assert( seq->ptr == seq->block_max );
    }
}

/* Add several elements to the beginning or end of a sequence: */
void cvSeqPushMulti( CvSeq *seq, const void *_elements, int count, int front )
{
    char *elements = (char *) _elements;

    if( !seq )
    {
		printf("NULL sequence pointer\n" );
		return;
	}
    if( count < 0 )
    {
		printf("number of removed elements is negative\n" );
		return;
	}

    int elem_size = seq->elem_size;

    if( !front )
    {
        while( count > 0 )
        {
            int delta = (int)((seq->block_max - seq->ptr) / elem_size);

            delta = MIN( delta, count );
            if( delta > 0 )
            {
                seq->first->prev->count += delta;
                seq->total += delta;
                count -= delta;
                delta *= elem_size;
                if( elements )
                {
                    memcpy( seq->ptr, elements, delta );
                    elements += delta;
                }
                seq->ptr += delta;
            }

            if( count > 0 )
                icvGrowSeq( seq, 0 );
        }
    }
    else
    {
        CvSeqBlock* block = seq->first;

        while( count > 0 )
        {
            int delta;

            if( !block || block->start_index == 0 )
            {
                icvGrowSeq( seq, 1 );

                block = seq->first;
                assert( block->start_index > 0 );
            }

            delta = MIN( block->start_index, count );
            count -= delta;
            block->start_index -= delta;
            block->count += delta;
            seq->total += delta;
            delta *= elem_size;
            block->data -= delta;

            if( elements )
                memcpy( block->data, elements + count*elem_size, delta );
        }
    }
}


/* Remove several elements from the end of sequence: */
void cvSeqPopMulti( CvSeq *seq, void *_elements, int count, int front )
{
    char *elements = (char *) _elements;

    if( !seq )
    {
		printf("NULL sequence pointer\n" );
		return;
	}
    if( count < 0 )
    {
		printf("number of removed elements is negative\n" );
		return;
	}

    count = MIN( count, seq->total );

    if( !front )
    {
        if( elements )
            elements += count * seq->elem_size;

        while( count > 0 )
        {
            int delta = seq->first->prev->count;

            delta = MIN( delta, count );
            assert( delta > 0 );

            seq->first->prev->count -= delta;
            seq->total -= delta;
            count -= delta;
            delta *= seq->elem_size;
            seq->ptr -= delta;

            if( elements )
            {
                elements -= delta;
                memcpy( elements, seq->ptr, delta );
            }

            if( seq->first->prev->count == 0 )
                icvFreeSeqBlock( seq, 0 );
        }
    }
    else
    {
        while( count > 0 )
        {
            int delta = seq->first->count;

            delta = MIN( delta, count );
            assert( delta > 0 );

            seq->first->count -= delta;
            seq->total -= delta;
            count -= delta;
            seq->first->start_index += delta;
            delta *= seq->elem_size;

            if( elements )
            {
                memcpy( elements, seq->first->data, delta );
                elements += delta;
            }

            seq->first->data += delta;
            if( seq->first->count == 0 )
                icvFreeSeqBlock( seq, 1 );
        }
    }
}

/* Remove all elements from a sequence: */
void cvClearSeq( CvSeq *seq )
{
    if( !seq )
        return;
    cvSeqPopMulti( seq, 0, seq->total );
}

/****************************************************************************************\
*                                      Set implementation                                *
\****************************************************************************************/

/* Creates empty set: */
CvSet* cvCreateSet( int set_flags, int header_size, int elem_size, CvMemStorage * storage )
{
    if( !storage )
        return 0;
    if( header_size < (int)sizeof( CvSet ) ||
        elem_size < (int)sizeof(void*)*2 ||
        (elem_size & (sizeof(void*)-1)) != 0 )
        return 0;

    CvSet* set = (CvSet*) cvCreateSeq( set_flags, header_size, elem_size, storage );
    set->flags = (set->flags & ~CV_MAGIC_MASK) | CV_SET_MAGIC_VAL;

    return set;
}

/* Add new element to the set: */
int cvSetAdd( CvSet* set, CvSetElem* element, CvSetElem** inserted_element )
{
    int id = -1;
    CvSetElem *free_elem;

    if( !set )
        return -1;

    if( !(set->free_elems) )
    {
        int count = set->total;
        int elem_size = set->elem_size;
        schar *ptr;
        icvGrowSeq( (CvSeq *) set, 0 );

        set->free_elems = (CvSetElem*) (ptr = set->ptr);
        for( ; ptr + elem_size <= set->block_max; ptr += elem_size, count++ )
        {
            ((CvSetElem*)ptr)->flags = count | CV_SET_ELEM_FREE_FLAG;
            ((CvSetElem*)ptr)->next_free = (CvSetElem*)(ptr + elem_size);
        }
        assert( count <= CV_SET_ELEM_IDX_MASK+1 );
        ((CvSetElem*)(ptr - elem_size))->next_free = 0;
        set->first->prev->count += count - set->total;
        set->total = count;
        set->ptr = set->block_max;
    }

    free_elem = set->free_elems;
    set->free_elems = free_elem->next_free;

    id = free_elem->flags & CV_SET_ELEM_IDX_MASK;
    if( element )
        memcpy( free_elem, element, set->elem_size );

    free_elem->flags = id;
    set->active_count++;

    if( inserted_element )
        *inserted_element = free_elem;

    return id;
}

/* Remove all elements from a set: */
void cvClearSet( CvSet* set )
{
    cvClearSeq( (CvSeq*)set );
    set->free_elems = 0;
    set->active_count = 0;
}


/****************************************************************************************\
*                                     Contours Tracing                                   *
\****************************************************************************************/
/*
   Initializes scanner structure.
   Prepare image for scanning ( clear borders and convert all pixels to 0-1.
*/
CvContourScanner cvStartFindContours( IplImage* _img, CvMemStorage* storage,
                     int  header_size, int mode, int  method, CvPoint offset )
{
    if( !storage )
        return 0;

    if( _img->depth == IPL_DEPTH_32S && _img->nChannels == 1 && mode == CV_RETR_CCOMP )
        mode = CV_RETR_FLOODFILL;

    if( !((_img->depth == IPL_DEPTH_8U && _img->nChannels == 1 && mode < CV_RETR_FLOODFILL) ||
          (_img->depth == IPL_DEPTH_32S && _img->nChannels == 1 && mode == CV_RETR_FLOODFILL)) )
    {
		printf("[Start]FindContours supports only CV_8UC1 images when mode != CV_RETR_FLOODFILL "
                  "otherwise supports CV_32SC1 images only" );
		return 0;
	}

    CvSize size = cvSize( _img->width, _img->height );
    int step = _img->widthStep;
    uchar* img = (uchar*)(_img->imageData);

    if( method < 0 || method > CV_CHAIN_APPROX_TC89_KCOS )
        return 0;

    if( header_size < (int) (method == CV_CHAIN_CODE ? sizeof( CvChain ) : sizeof( CvContour )))
        return 0;

    CvContourScanner scanner = (CvContourScanner)cvAlloc( sizeof( *scanner ));
    memset( scanner, 0, sizeof(*scanner) );

    scanner->storage1 = scanner->storage2 = storage;
    scanner->img0 = (schar *) img;
    scanner->img = (schar *) (img + step);
    scanner->img_step = step;
    scanner->img_size.width = size.width - 1;   /* exclude rightest column */
    scanner->img_size.height = size.height - 1; /* exclude bottomost row */
    scanner->mode = mode;
    scanner->offset = offset;
    scanner->pt.x = scanner->pt.y = 1;
    scanner->lnbd.x = 0;
    scanner->lnbd.y = 1;
    scanner->nbd = 2;
    scanner->mode = (int) mode;
    scanner->frame_info.contour = &(scanner->frame);
    scanner->frame_info.is_hole = 1;
    scanner->frame_info.next = 0;
    scanner->frame_info.parent = 0;
    scanner->frame_info.rect = cvRect( 0, 0, size.width, size.height );
    scanner->l_cinfo = 0;
    scanner->subst_flag = 0;

    scanner->frame.flags = CV_SEQ_FLAG_HOLE;

    scanner->approx_method2 = scanner->approx_method1 = method;

    if( method == CV_CHAIN_APPROX_TC89_L1 || method == CV_CHAIN_APPROX_TC89_KCOS )
        scanner->approx_method1 = CV_CHAIN_CODE;

    if( scanner->approx_method1 == CV_CHAIN_CODE )
    {
        scanner->seq_type1 = CV_SEQ_CHAIN_CONTOUR;
        scanner->header_size1 = scanner->approx_method1 == scanner->approx_method2 ?
            header_size : sizeof( CvChain );
        scanner->elem_size1 = sizeof( char );
    }
    else
    {
        scanner->seq_type1 = CV_SEQ_POLYGON;
        scanner->header_size1 = scanner->approx_method1 == scanner->approx_method2 ?
            header_size : sizeof( CvContour );
        scanner->elem_size1 = sizeof( CvPoint );
    }

    scanner->header_size2 = header_size;

    if( scanner->approx_method2 == CV_CHAIN_CODE )
    {
        scanner->seq_type2 = scanner->seq_type1;
        scanner->elem_size2 = scanner->elem_size1;
    }
    else
    {
        scanner->seq_type2 = CV_SEQ_POLYGON;
        scanner->elem_size2 = sizeof( CvPoint );
    }

    scanner->seq_type1 = scanner->approx_method1 == CV_CHAIN_CODE ?
        CV_SEQ_CHAIN_CONTOUR : CV_SEQ_POLYGON;

    scanner->seq_type2 = scanner->approx_method2 == CV_CHAIN_CODE ?
        CV_SEQ_CHAIN_CONTOUR : CV_SEQ_POLYGON;

    cvSaveMemStoragePos( storage, &(scanner->initial_pos) );

    if( method > CV_CHAIN_APPROX_SIMPLE )
    {
        scanner->storage1 = cvCreateChildMemStorage( scanner->storage2 );
    }

    if( mode > CV_RETR_LIST )
    {
        scanner->cinfo_storage = cvCreateChildMemStorage( scanner->storage2 );
        scanner->cinfo_set = cvCreateSet( 0, sizeof( CvSet ), sizeof( _CvContourInfo ),
                                          scanner->cinfo_storage );
    }

    /* make zero borders */
    int esz = _img->nChannels;
    memset( img, 0, size.width*esz );
    memset( img + step * (size.height - 1), 0, size.width*esz );

    img += step;
    for( int y = 1; y < size.height - 1; y++, img += step )
    {
        for( int k = 0; k < esz; k++ )
            img[k] = img[(size.width - 1)*esz + k] = (schar)0;
    }

    /* converts all pixels to 0 or 1 */
    if( _img->depth != IPL_DEPTH_32S )
        //cvThreshold( mat, mat, 0, 1, CV_THRESH_BINARY );
	{
		img = (uchar*)(_img->imageData);
		for (int i = 0; i < size.height; ++i)
	    {
		    for (int j = 0; j < size.width; ++j)
		    {
			    if (img[i*step+j] > 0)
			    {
				    img[i*step+j] = 1;
			    }
			    else
			    {
				    img[i*step+j] = 0;
			    }
		    }
	    }
	}

    return scanner;
}

static void icvEndProcessContour( CvContourScanner scanner )
{
    _CvContourInfo *l_cinfo = scanner->l_cinfo;

    if( l_cinfo )
    {
        if( scanner->subst_flag )
        {
            CvMemStoragePos temp;

            cvSaveMemStoragePos( scanner->storage2, &temp );

            if( temp.top == scanner->backup_pos2.top &&
                temp.free_space == scanner->backup_pos2.free_space )
            {
                cvRestoreMemStoragePos( scanner->storage2, &scanner->backup_pos );
            }
            scanner->subst_flag = 0;
        }

        if( l_cinfo->contour )
        {
            cvInsertNodeIntoTree( l_cinfo->contour, l_cinfo->parent->contour,
                                  &(scanner->frame) );
        }
        scanner->l_cinfo = 0;
    }
}

/* replaces one contour with another */
void cvSubstituteContour( CvContourScanner scanner, CvSeq * new_contour )
{
    _CvContourInfo *l_cinfo;

    if( !scanner )
        return;

    l_cinfo = scanner->l_cinfo;
    if( l_cinfo && l_cinfo->contour && l_cinfo->contour != new_contour )
    {
        l_cinfo->contour = new_contour;
        scanner->subst_flag = 1;
    }
}


/*
    marks domain border with +/-<constant> and stores the contour into CvSeq.
        method:
            <0  - chain
            ==0 - direct
            >0  - simple approximation
*/
static void icvFetchContour( schar *ptr, int step, CvPoint pt, CvSeq* contour, int _method )
{
    const schar     nbd = 2;
    int             deltas[16];
    CvSeqWriter     writer;
    schar           *i0 = ptr, *i1, *i3, *i4 = 0;
    int             prev_s = -1, s, s_end;
    int             method = _method - 1;

    assert( (unsigned) _method <= CV_CHAIN_APPROX_SIMPLE );

    /* initialize local state */
    CV_INIT_3X3_DELTAS( deltas, step, 1 );
    memcpy( deltas + 8, deltas, 8 * sizeof( deltas[0] ));

    /* initialize writer */
    cvStartAppendToSeq( contour, &writer );

    if( method < 0 )
        ((CvChain *) contour)->origin = pt;

    s_end = s = CV_IS_SEQ_HOLE( contour ) ? 0 : 4;

    do
    {
        s = (s - 1) & 7;
        i1 = i0 + deltas[s];
        if( *i1 != 0 )
            break;
    }
    while( s != s_end );

    if( s == s_end )            /* single pixel domain */
    {
        *i0 = (schar) (nbd | -128);
        if( method >= 0 )
        {
            CV_WRITE_SEQ_ELEM( pt, writer );
        }
    }
    else
    {
        i3 = i0;
        prev_s = s ^ 4;

        /* follow border */
        for( ;; )
        {
            s_end = s;

            for( ;; )
            {
                i4 = i3 + deltas[++s];
                if( *i4 != 0 )
                    break;
            }
            s &= 7;

            /* check "right" bound */
            if( (unsigned) (s - 1) < (unsigned) s_end )
            {
                *i3 = (schar) (nbd | -128);
            }
            else if( *i3 == 1 )
            {
                *i3 = nbd;
            }

            if( method < 0 )
            {
                schar _s = (schar) s;

                CV_WRITE_SEQ_ELEM( _s, writer );
            }
            else
            {
                if( s != prev_s || method == 0 )
                {
                    CV_WRITE_SEQ_ELEM( pt, writer );
                    prev_s = s;
                }

                //pt.x += icvCodeDeltas[s].x;
                //pt.y += icvCodeDeltas[s].y;
				pt.x += icvCodeDeltas[s][0];
				pt.y += icvCodeDeltas[s][1];

            }

            if( i4 == i0 && i3 == i1 )
                break;

            i3 = i4;
            s = (s + 4) & 7;
        }                       /* end of border following loop */
    }

    cvEndWriteSeq( &writer );

    if( _method != CV_CHAIN_CODE )
        cvBoundingRect( contour, 1 );

    assert( (writer.seq->total == 0 && writer.seq->first == 0) ||
            writer.seq->total > writer.seq->first->count ||
            (writer.seq->first->prev == writer.seq->first &&
             writer.seq->first->next == writer.seq->first) );
}



/*
   trace contour until certain point is met.
   returns 1 if met, 0 else.
*/
static int icvTraceContour( schar *ptr, int step, schar *stop_ptr, int is_hole )
{
    int deltas[16];
    schar *i0 = ptr, *i1, *i3, *i4;
    int s, s_end;

    /* initialize local state */
    CV_INIT_3X3_DELTAS( deltas, step, 1 );
    memcpy( deltas + 8, deltas, 8 * sizeof( deltas[0] ));

    assert( (*i0 & -2) != 0 );

    s_end = s = is_hole ? 0 : 4;

    do
    {
        s = (s - 1) & 7;
        i1 = i0 + deltas[s];
        if( *i1 != 0 )
            break;
    }
    while( s != s_end );

    i3 = i0;

    /* check single pixel domain */
    if( s != s_end )
    {
        /* follow border */
        for( ;; )
        {
            s_end = s;

            for( ;; )
            {
                i4 = i3 + deltas[++s];
                if( *i4 != 0 )
                    break;
            }

            if( i3 == stop_ptr || (i4 == i0 && i3 == i1) )
                break;

            i3 = i4;
            s = (s + 4) & 7;
        }                       /* end of border following loop */
    }
    return i3 == stop_ptr;
}


static void icvFetchContourEx( schar* ptr, int step, CvPoint pt, CvSeq* contour, int _method, int nbd, CvRect* _rect )
{
    int         deltas[16];
    CvSeqWriter writer;
    schar        *i0 = ptr, *i1, *i3, *i4;
    CvRect      rect;
    int         prev_s = -1, s, s_end;
    int         method = _method - 1;

    assert( (unsigned) _method <= CV_CHAIN_APPROX_SIMPLE );
    assert( 1 < nbd && nbd < 128 );

    /* initialize local state */
    CV_INIT_3X3_DELTAS( deltas, step, 1 );
    memcpy( deltas + 8, deltas, 8 * sizeof( deltas[0] ));

    /* initialize writer */
    cvStartAppendToSeq( contour, &writer );

    if( method < 0 )
        ((CvChain *)contour)->origin = pt;

    rect.x = rect.width = pt.x;
    rect.y = rect.height = pt.y;

    s_end = s = CV_IS_SEQ_HOLE( contour ) ? 0 : 4;

    do
    {
        s = (s - 1) & 7;
        i1 = i0 + deltas[s];
        if( *i1 != 0 )
            break;
    }
    while( s != s_end );

    if( s == s_end )            /* single pixel domain */
    {
        *i0 = (schar) (nbd | 0x80);
        if( method >= 0 )
        {
            CV_WRITE_SEQ_ELEM( pt, writer );
        }
    }
    else
    {
        i3 = i0;

        prev_s = s ^ 4;

        /* follow border */
        for( ;; )
        {
            s_end = s;

            for( ;; )
            {
                i4 = i3 + deltas[++s];
                if( *i4 != 0 )
                    break;
            }
            s &= 7;

            /* check "right" bound */
            if( (unsigned) (s - 1) < (unsigned) s_end )
            {
                *i3 = (schar) (nbd | 0x80);
            }
            else if( *i3 == 1 )
            {
                *i3 = (schar) nbd;
            }

            if( method < 0 )
            {
                schar _s = (schar) s;
                CV_WRITE_SEQ_ELEM( _s, writer );
            }
            else if( s != prev_s || method == 0 )
            {
                CV_WRITE_SEQ_ELEM( pt, writer );
            }

            if( s != prev_s )
            {
                /* update bounds */
                if( pt.x < rect.x )
                    rect.x = pt.x;
                else if( pt.x > rect.width )
                    rect.width = pt.x;

                if( pt.y < rect.y )
                    rect.y = pt.y;
                else if( pt.y > rect.height )
                    rect.height = pt.y;
            }

            prev_s = s;
            //pt.x += icvCodeDeltas[s].x;
            //pt.y += icvCodeDeltas[s].y;
			pt.x += icvCodeDeltas[s][0];
			pt.y += icvCodeDeltas[s][1];

            if( i4 == i0 && i3 == i1 )  break;

            i3 = i4;
            s = (s + 4) & 7;
        }                       /* end of border following loop */
    }

    rect.width -= rect.x - 1;
    rect.height -= rect.y - 1;

    cvEndWriteSeq( &writer );

    if( _method != CV_CHAIN_CODE )
        ((CvContour*)contour)->rect = rect;

    assert( (writer.seq->total == 0 && writer.seq->first == 0) ||
            writer.seq->total > writer.seq->first->count ||
            (writer.seq->first->prev == writer.seq->first &&
             writer.seq->first->next == writer.seq->first) );

    if( _rect )  *_rect = rect;
}


static int icvTraceContour_32s( int *ptr, int step, int *stop_ptr, int is_hole )
{
    int deltas[16];
    int *i0 = ptr, *i1, *i3, *i4;
    int s, s_end;
    const int   right_flag = INT_MIN;
    const int   new_flag = (int)((unsigned)INT_MIN >> 1);
    const int   value_mask = ~(right_flag | new_flag);
    const int   ccomp_val = *i0 & value_mask;

    /* initialize local state */
    CV_INIT_3X3_DELTAS( deltas, step, 1 );
    memcpy( deltas + 8, deltas, 8 * sizeof( deltas[0] ));

    s_end = s = is_hole ? 0 : 4;

    do
    {
        s = (s - 1) & 7;
        i1 = i0 + deltas[s];
        if( (*i1 & value_mask) == ccomp_val )
            break;
    }
    while( s != s_end );

    i3 = i0;

    /* check single pixel domain */
    if( s != s_end )
    {
        /* follow border */
        for( ;; )
        {
            s_end = s;

            for( ;; )
            {
                i4 = i3 + deltas[++s];
                if( (*i4 & value_mask) == ccomp_val )
                    break;
            }

            if( i3 == stop_ptr || (i4 == i0 && i3 == i1) )
                break;

            i3 = i4;
            s = (s + 4) & 7;
        }                       /* end of border following loop */
    }
    return i3 == stop_ptr;
}


static void icvFetchContourEx_32s( int* ptr, int step, CvPoint pt, CvSeq* contour, int _method, CvRect* _rect )
{
    int         deltas[16];
    CvSeqWriter writer;
    int        *i0 = ptr, *i1, *i3, *i4;
    CvRect      rect;
    int         prev_s = -1, s, s_end;
    int         method = _method - 1;
    const int   right_flag = INT_MIN;
    const int   new_flag = (int)((unsigned)INT_MIN >> 1);
    const int   value_mask = ~(right_flag | new_flag);
    const int   ccomp_val = *i0 & value_mask;
    const int   nbd0 = ccomp_val | new_flag;
    const int   nbd1 = nbd0 | right_flag;

    assert( (unsigned) _method <= CV_CHAIN_APPROX_SIMPLE );

    /* initialize local state */
    CV_INIT_3X3_DELTAS( deltas, step, 1 );
    memcpy( deltas + 8, deltas, 8 * sizeof( deltas[0] ));

    /* initialize writer */
    cvStartAppendToSeq( contour, &writer );

    if( method < 0 )
        ((CvChain *)contour)->origin = pt;

    rect.x = rect.width = pt.x;
    rect.y = rect.height = pt.y;

    s_end = s = CV_IS_SEQ_HOLE( contour ) ? 0 : 4;

    do
    {
        s = (s - 1) & 7;
        i1 = i0 + deltas[s];
        if( (*i1 & value_mask) == ccomp_val )
            break;
    }
    while( s != s_end );

    if( s == s_end )            /* single pixel domain */
    {
        *i0 = nbd1;
        if( method >= 0 )
        {
            CV_WRITE_SEQ_ELEM( pt, writer );
        }
    }
    else
    {
        i3 = i0;
        prev_s = s ^ 4;

        /* follow border */
        for( ;; )
        {
            s_end = s;

            for( ;; )
            {
                i4 = i3 + deltas[++s];
                if( (*i4 & value_mask) == ccomp_val )
                    break;
            }
            s &= 7;

            /* check "right" bound */
            if( (unsigned) (s - 1) < (unsigned) s_end )
            {
                *i3 = nbd1;
            }
            else if( *i3 == ccomp_val )
            {
                *i3 = nbd0;
            }

            if( method < 0 )
            {
                schar _s = (schar) s;
                CV_WRITE_SEQ_ELEM( _s, writer );
            }
            else if( s != prev_s || method == 0 )
            {
                CV_WRITE_SEQ_ELEM( pt, writer );
            }

            if( s != prev_s )
            {
                /* update bounds */
                if( pt.x < rect.x )
                    rect.x = pt.x;
                else if( pt.x > rect.width )
                    rect.width = pt.x;

                if( pt.y < rect.y )
                    rect.y = pt.y;
                else if( pt.y > rect.height )
                    rect.height = pt.y;
            }

            prev_s = s;
            //pt.x += icvCodeDeltas[s].x;
            //pt.y += icvCodeDeltas[s].y;
			pt.x += icvCodeDeltas[s][0];
			pt.y += icvCodeDeltas[s][1];

            if( i4 == i0 && i3 == i1 )  break;

            i3 = i4;
            s = (s + 4) & 7;
        }                       /* end of border following loop */
    }

    rect.width -= rect.x - 1;
    rect.height -= rect.y - 1;

    cvEndWriteSeq( &writer );

    if( _method != CV_CHAIN_CODE )
        ((CvContour*)contour)->rect = rect;

    assert( (writer.seq->total == 0 && writer.seq->first == 0) ||
           writer.seq->total > writer.seq->first->count ||
           (writer.seq->first->prev == writer.seq->first &&
            writer.seq->first->next == writer.seq->first) );

    if( _rect )  *_rect = rect;
}

CvSeq *cvFindNextContour( CvContourScanner scanner )
{
    if( !scanner )
        return 0;
    icvEndProcessContour( scanner );

    /* initialize local state */
    schar* img0 = scanner->img0;
    schar* img = scanner->img;
    int step = scanner->img_step;
    int step_i = step / sizeof(int);
    int x = scanner->pt.x;
    int y = scanner->pt.y;
    int width = scanner->img_size.width;
    int height = scanner->img_size.height;
    int mode = scanner->mode;
    CvPoint lnbd = scanner->lnbd;
    int nbd = scanner->nbd;
    int prev = img[x - 1];
    int new_mask = -2;

    if( mode == CV_RETR_FLOODFILL )
    {
        prev = ((int*)img)[x - 1];
        new_mask = INT_MIN / 2;
    }

    for( ; y < height; y++, img += step )
    {
        int* img0_i = 0;
        int* img_i = 0;
        int p = 0;

        if( mode == CV_RETR_FLOODFILL )
        {
            img0_i = (int*)img0;
            img_i = (int*)img;
        }

        for( ; x < width; x++ )
        {
            if( img_i )
            {
                for( ; x < width && ((p = img_i[x]) == prev || (p & ~new_mask) == (prev & ~new_mask)); x++ )
                    prev = p;
            }
            else
            {
                for( ; x < width && (p = img[x]) == prev; x++ )
                    ;
            }

            if( x >= width )
                break;

            {
                _CvContourInfo *par_info = 0;
                _CvContourInfo *l_cinfo = 0;
                CvSeq *seq = 0;
                int is_hole = 0;
                CvPoint origin;

                /* if not external contour */
                if( (!img_i && !(prev == 0 && p == 1)) ||
                    (img_i && !(((prev & new_mask) != 0 || prev == 0) && (p & new_mask) == 0)) )
                {
                    /* check hole */
                    if( (!img_i && (p != 0 || prev < 1)) ||
                        (img_i && ((prev & new_mask) != 0 || (p & new_mask) != 0)))
                        goto resume_scan;

                    if( prev & new_mask )
                    {
                        lnbd.x = x - 1;
                    }
                    is_hole = 1;
                }

                if( mode == 0 && (is_hole || img0[lnbd.y * step + lnbd.x] > 0) )
                    goto resume_scan;

                origin.y = y;
                origin.x = x - is_hole;

                /* find contour parent */
                if( mode <= 1 || (!is_hole && (mode == CV_RETR_CCOMP || mode == CV_RETR_FLOODFILL)) || lnbd.x <= 0 )
                {
                    par_info = &(scanner->frame_info);
                }
                else
                {
                    int lval = (img0_i ?
                        img0_i[lnbd.y * step_i + lnbd.x] :
                        (int)img0[lnbd.y * step + lnbd.x]) & 0x7f;
                    _CvContourInfo *cur = scanner->cinfo_table[lval];

                    /* find the first bounding contour */
                    while( cur )
                    {
                        if( (unsigned) (lnbd.x - cur->rect.x) < (unsigned) cur->rect.width &&
                            (unsigned) (lnbd.y - cur->rect.y) < (unsigned) cur->rect.height )
                        {
                            if( par_info )
                            {
                                if( (img0_i &&
                                     icvTraceContour_32s( img0_i + par_info->origin.y * step_i +
                                                          par_info->origin.x, step_i, img_i + lnbd.x,
                                                          par_info->is_hole ) > 0) ||
                                    (!img0_i &&
                                     icvTraceContour( img0 + par_info->origin.y * step +
                                                      par_info->origin.x, step, img + lnbd.x,
                                                      par_info->is_hole ) > 0) )
                                    break;
                            }
                            par_info = cur;
                        }
                        cur = cur->next;
                    }

                    assert( par_info != 0 );

                    /* if current contour is a hole and previous contour is a hole or
                       current contour is external and previous contour is external then
                       the parent of the contour is the parent of the previous contour else
                       the parent is the previous contour itself. */
                    if( par_info->is_hole == is_hole )
                    {
                        par_info = par_info->parent;
                        /* every contour must have a parent
                           (at least, the frame of the image) */
                        if( !par_info )
                            par_info = &(scanner->frame_info);
                    }

                    /* hole flag of the parent must differ from the flag of the contour */
                    assert( par_info->is_hole != is_hole );
                    if( par_info->contour == 0 )        /* removed contour */
                        goto resume_scan;
                }

                lnbd.x = x - is_hole;

                cvSaveMemStoragePos( scanner->storage2, &(scanner->backup_pos) );

                seq = cvCreateSeq( scanner->seq_type1, scanner->header_size1,
                                   scanner->elem_size1, scanner->storage1 );
                seq->flags |= is_hole ? CV_SEQ_FLAG_HOLE : 0;

                /* initialize header */
                if( mode <= 1 )
                {
                    l_cinfo = &(scanner->cinfo_temp);
                    icvFetchContour( img + x - is_hole, step,
                                     cvPoint( origin.x + scanner->offset.x,
                                              origin.y + scanner->offset.y),
                                     seq, scanner->approx_method1 );
                }
                else
                {
                    union { _CvContourInfo* ci; CvSetElem* se; } v;
                    v.ci = l_cinfo;
                    cvSetAdd( scanner->cinfo_set, 0, &v.se );
                    l_cinfo = v.ci;
                    int lval;

                    if( img_i )
                    {
                        lval = img_i[x - is_hole] & 127;
                        icvFetchContourEx_32s(img_i + x - is_hole, step_i,
                                              cvPoint( origin.x + scanner->offset.x,
                                                       origin.y + scanner->offset.y),
                                              seq, scanner->approx_method1,
                                              &(l_cinfo->rect) );
                    }
                    else
                    {
                        lval = nbd;
                        // change nbd
                        nbd = (nbd + 1) & 127;
                        nbd += nbd == 0 ? 3 : 0;
                        icvFetchContourEx( img + x - is_hole, step,
                                           cvPoint( origin.x + scanner->offset.x,
                                                    origin.y + scanner->offset.y),
                                           seq, scanner->approx_method1,
                                           lval, &(l_cinfo->rect) );
                    }
                    l_cinfo->rect.x -= scanner->offset.x;
                    l_cinfo->rect.y -= scanner->offset.y;

                    l_cinfo->next = scanner->cinfo_table[lval];
                    scanner->cinfo_table[lval] = l_cinfo;
                }

                l_cinfo->is_hole = is_hole;
                l_cinfo->contour = seq;
                l_cinfo->origin = origin;
                l_cinfo->parent = par_info;

                if( scanner->approx_method1 != scanner->approx_method2 )
                {
                    l_cinfo->contour = icvApproximateChainTC89( (CvChain *) seq,
                                                      scanner->header_size2,
                                                      scanner->storage2,
                                                      scanner->approx_method2 );
                    cvClearMemStorage( scanner->storage1 );
                }

                l_cinfo->contour->v_prev = l_cinfo->parent->contour;

                if( par_info->contour == 0 )
                {
                    l_cinfo->contour = 0;
                    if( scanner->storage1 == scanner->storage2 )
                    {
                        cvRestoreMemStoragePos( scanner->storage1, &(scanner->backup_pos) );
                    }
                    else
                    {
                        cvClearMemStorage( scanner->storage1 );
                    }
                    p = img[x];
                    goto resume_scan;
                }

                cvSaveMemStoragePos( scanner->storage2, &(scanner->backup_pos2) );
                scanner->l_cinfo = l_cinfo;
                scanner->pt.x = !img_i ? x + 1 : x + 1 - is_hole;
                scanner->pt.y = y;
                scanner->lnbd = lnbd;
                scanner->img = (schar *) img;
                scanner->nbd = nbd;
                return l_cinfo->contour;

            resume_scan:

                prev = p;
                /* update lnbd */
                if( prev & -2 )
                {
                    lnbd.x = x;
                }
            }                   /* end of prev != p */
        }                       /* end of loop on x */

        lnbd.x = 0;
        lnbd.y = y + 1;
        x = 1;
        prev = 0;
    }                           /* end of loop on y */

    return 0;
}


/*
   The function add to tree the last retrieved/substituted contour,
   releases temp_storage, restores state of dst_storage (if needed), and
   returns pointer to root of the contour tree */
CvSeq *cvEndFindContours( CvContourScanner * _scanner )
{
    CvContourScanner scanner;
    CvSeq *first = 0;

    if( !_scanner )
        return 0;
    scanner = *_scanner;

    if( scanner )
    {
        icvEndProcessContour( scanner );

        if( scanner->storage1 != scanner->storage2 )
            cvReleaseMemStorage( &(scanner->storage1) );

        if( scanner->cinfo_storage )
            cvReleaseMemStorage( &(scanner->cinfo_storage) );

        first = scanner->frame.v_next;
        cvFree( _scanner );
    }

    return first;
}

/****************************************************************************************\
*                                  Chain Approximation                                   *
\****************************************************************************************/

typedef struct _CvPtInfo
{
    CvPoint pt;
    int k;                      /* support region */
    int s;                      /* curvature value */
    struct _CvPtInfo *next;
}
_CvPtInfo;

void cvStartReadChainPoints( CvChain * chain, CvChainPtReader * reader )
{
    int i;

    if( !chain || !reader )
        return;

    if( chain->elem_size != 1 || chain->header_size < (int)sizeof(CvChain))
        return;

    cvStartReadSeq( (CvSeq *) chain, (CvSeqReader *) reader, 0 );

    reader->pt = chain->origin;
    for( i = 0; i < 8; i++ )
    {
        //reader->deltas[i][0] = (schar) icvCodeDeltas[i].x;
        //reader->deltas[i][1] = (schar) icvCodeDeltas[i].y;
		reader->deltas[i][0] = (schar) icvCodeDeltas[i][0];
		reader->deltas[i][1] = (schar) icvCodeDeltas[i][1];
    }
}

/* curvature: 0 - 1-curvature, 1 - k-cosine curvature. */
CvSeq* icvApproximateChainTC89( CvChain* chain, int header_size, CvMemStorage* storage, int method )
{
    static const int abs_diff[] = { 1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1 };

	AutoBuffer<_CvPtInfo> buf(chain->total + 8);

    _CvPtInfo       temp;
    _CvPtInfo       *array = buf, *first = 0, *current = 0, *prev_current = 0;
    int             i, j, i1, i2, s, len;
    int             count = chain->total;

    CvChainPtReader reader;
    CvSeqWriter     writer;
    CvPoint         pt = chain->origin;

    assert( CV_IS_SEQ_CHAIN_CONTOUR( chain ));
    assert( header_size >= (int)sizeof(CvContour) );

    cvStartWriteSeq( (chain->flags & ~CV_SEQ_ELTYPE_MASK) | CV_SEQ_ELTYPE_POINT,
                     header_size, sizeof( CvPoint ), storage, &writer );

    if( chain->total == 0 )
    {
        CV_WRITE_SEQ_ELEM( pt, writer );
        return cvEndWriteSeq( &writer );
    }

    cvStartReadChainPoints( chain, &reader );

    temp.next = 0;
    current = &temp;
    /* Pass 0.
       Restores all the digital curve points from the chain code.
       Removes the points (from the resultant polygon)
       that have zero 1-curvature */
    for( i = 0; i < count; i++ )
    {
        int prev_code = *reader.prev_elem;

        reader.prev_elem = reader.ptr;
        CV_READ_CHAIN_POINT( pt, reader );

        /* calc 1-curvature */
        s = abs_diff[reader.code - prev_code + 7];

        if( method <= CV_CHAIN_APPROX_SIMPLE )
        {
            if( method == CV_CHAIN_APPROX_NONE || s != 0 )
            {
                CV_WRITE_SEQ_ELEM( pt, writer );
            }
        }
        else
        {
            if( s != 0 )
                current = current->next = array + i;
            array[i].s = s;
            array[i].pt = pt;
        }
    }

    //assert( pt.x == chain->origin.x && pt.y == chain->origin.y );

    if( method <= CV_CHAIN_APPROX_SIMPLE )
        return cvEndWriteSeq( &writer );

    current->next = 0;

    len = i;
    current = temp.next;

    assert( current );

    /* Pass 1.
       Determines support region for all the remained points */
    do
    {
        CvPoint pt0;
        int k, l = 0, d_num = 0;

        i = (int)(current - array);
        pt0 = array[i].pt;

        /* determine support region */
        for( k = 1;; k++ )
        {
            int lk, dk_num;
            int dx, dy;
            Cv32suf d;

            assert( k <= len );

            /* calc indices */
            i1 = i - k;
            i1 += i1 < 0 ? len : 0;
            i2 = i + k;
            i2 -= i2 >= len ? len : 0;

            dx = array[i2].pt.x - array[i1].pt.x;
            dy = array[i2].pt.y - array[i1].pt.y;

            /* distance between p_(i - k) and p_(i + k) */
            lk = dx * dx + dy * dy;

            /* distance between p_i and the line (p_(i-k), p_(i+k)) */
            dk_num = (pt0.x - array[i1].pt.x) * dy - (pt0.y - array[i1].pt.y) * dx;
            d.f = (float) (((double) d_num) * lk - ((double) dk_num) * l);

            if( k > 1 && (l >= lk || ((d_num > 0 && d.i <= 0) || (d_num < 0 && d.i >= 0))))
                break;

            d_num = dk_num;
            l = lk;
        }

        current->k = --k;

        /* determine cosine curvature if it should be used */
        if( method == CV_CHAIN_APPROX_TC89_KCOS )
        {
            /* calc k-cosine curvature */
            for( j = k, s = 0; j > 0; j-- )
            {
                double temp_num;
                int dx1, dy1, dx2, dy2;
                Cv32suf sk;

                i1 = i - j;
                i1 += i1 < 0 ? len : 0;
                i2 = i + j;
                i2 -= i2 >= len ? len : 0;

                dx1 = array[i1].pt.x - pt0.x;
                dy1 = array[i1].pt.y - pt0.y;
                dx2 = array[i2].pt.x - pt0.x;
                dy2 = array[i2].pt.y - pt0.y;

                if( (dx1 | dy1) == 0 || (dx2 | dy2) == 0 )
                    break;

                temp_num = dx1 * dx2 + dy1 * dy2;
                temp_num =
                    (float) (temp_num /
                             sqrt( ((double)dx1 * dx1 + (double)dy1 * dy1) *
                                   ((double)dx2 * dx2 + (double)dy2 * dy2) ));
                sk.f = (float) (temp_num + 1.1);

                assert( 0 <= sk.f && sk.f <= 2.2 );
                if( j < k && sk.i <= s )
                    break;

                s = sk.i;
            }
            current->s = s;
        }
        current = current->next;
    }
    while( current != 0 );

    prev_current = &temp;
    current = temp.next;

    /* Pass 2.
       Performs non-maxima suppression */
    do
    {
        int k2 = current->k >> 1;

        s = current->s;
        i = (int)(current - array);

        for( j = 1; j <= k2; j++ )
        {
            i2 = i - j;
            i2 += i2 < 0 ? len : 0;

            if( array[i2].s > s )
                break;

            i2 = i + j;
            i2 -= i2 >= len ? len : 0;

            if( array[i2].s > s )
                break;
        }

        if( j <= k2 )           /* exclude point */
        {
            prev_current->next = current->next;
            current->s = 0;     /* "clear" point */
        }
        else
            prev_current = current;
        current = current->next;
    }
    while( current != 0 );

    /* Pass 3.
       Removes non-dominant points with 1-length support region */
    current = temp.next;
    assert( current );
    prev_current = &temp;

    do
    {
        if( current->k == 1 )
        {
            s = current->s;
            i = (int)(current - array);

            i1 = i - 1;
            i1 += i1 < 0 ? len : 0;

            i2 = i + 1;
            i2 -= i2 >= len ? len : 0;

            if( s <= array[i1].s || s <= array[i2].s )
            {
                prev_current->next = current->next;
                current->s = 0;
            }
            else
                prev_current = current;
        }
        else
            prev_current = current;
        current = current->next;
    }
    while( current != 0 );

    if( method == CV_CHAIN_APPROX_TC89_KCOS )
        goto copy_vect;

    /* Pass 4.
       Cleans remained couples of points */
    assert( temp.next );

    if( array[0].s != 0 && array[len - 1].s != 0 )      /* specific case */
    {
        for( i1 = 1; i1 < len && array[i1].s != 0; i1++ )
        {
            array[i1 - 1].s = 0;
        }
        if( i1 == len )
            goto copy_vect;     /* all points survived */
        i1--;

        for( i2 = len - 2; i2 > 0 && array[i2].s != 0; i2-- )
        {
            array[i2].next = 0;
            array[i2 + 1].s = 0;
        }
        i2++;

        if( i1 == 0 && i2 == len - 1 )  /* only two points */
        {
            i1 = (int)(array[0].next - array);
            array[len] = array[0];      /* move to the end */
            array[len].next = 0;
            array[len - 1].next = array + len;
        }
        temp.next = array + i1;
    }

    current = temp.next;
    first = prev_current = &temp;
    count = 1;

    /* do last pass */
    do
    {
        if( current->next == 0 || current->next - current != 1 )
        {
            if( count >= 2 )
            {
                if( count == 2 )
                {
                    int s1 = prev_current->s;
                    int s2 = current->s;

                    if( s1 > s2 || (s1 == s2 && prev_current->k <= current->k) )
                        /* remove second */
                        prev_current->next = current->next;
                    else
                        /* remove first */
                        first->next = current;
                }
                else
                    first->next->next = current;
            }
            first = current;
            count = 1;
        }
        else
            count++;
        prev_current = current;
        current = current->next;
    }
    while( current != 0 );

copy_vect:

    // gather points
    current = temp.next;
    assert( current );

    do
    {
        CV_WRITE_SEQ_ELEM( current->pt, writer );
        current = current->next;
    }
    while( current != 0 );

    return cvEndWriteSeq( &writer );
}

int cvRound( double value )
{
    return (int)(value + (value >= 0 ? 0.5 : -0.5));
}

int cvFloor( double value )
{
   int i = cvRound(value);
   float diff = (float)(value - i);
   return i - (diff < 0);
}

int cvCeil( double value )
{
   int i = cvRound(value);
   float diff = (float)(i - value);
   return i + (diff < 0);
}

/****************************************************************************************\
*                               Polygonal Approximation                                  *
\****************************************************************************************/

/* Ramer-Douglas-Peucker algorithm for polygon simplification */

static int approxPolyDP_32s( const CvPoint* src_contour, int count0, CvPoint* dst_contour, bool is_closed0, double eps, AutoBuffer<Range>* _stack )
{
    #define PUSH_SLICE(slice) \
        if( top >= stacksz ) \
        { \
            _stack->resize(stacksz*3/2); \
            stack = *_stack; \
            stacksz = _stack->size(); \
        } \
        stack[top++] = slice

    #define READ_PT(pt, pos) \
        pt = src_contour[pos]; \
        if( ++pos >= count ) pos = 0

    #define READ_DST_PT(pt, pos) \
        pt = dst_contour[pos]; \
        if( ++pos >= count ) pos = 0

    #define WRITE_PT(pt) \
        dst_contour[new_count++] = pt

    typedef CvPoint PT;
    int             init_iters = 3;
    Range           slice(0, 0), right_slice(0, 0);
	PT              start_pt = { -1000000, -1000000 }, end_pt = {0, 0}, pt = {0, 0};//start_pt((T)-1000000, (T)-1000000), end_pt(0, 0), pt(0,0);
    int             i = 0, j, pos = 0, wpos, count = count0, new_count=0;
    int             is_closed = is_closed0;
    bool            le_eps = false;
    size_t top = 0, stacksz = _stack->size();
    Range*          stack = *_stack;

    if( count == 0  )
        return 0;

    eps *= eps;

    if( !is_closed )
    {
        right_slice.start = count;
        end_pt = src_contour[0];
        start_pt = src_contour[count-1];

        if( start_pt.x != end_pt.x || start_pt.y != end_pt.y )
        {
            slice.start = 0;
            slice.end = count - 1;
            PUSH_SLICE(slice);
        }
        else
        {
            is_closed = 1;
            init_iters = 1;
        }
    }

    if( is_closed )
    {
        // 1. Find approximately two farthest points of the contour
        right_slice.start = 0;

        for( i = 0; i < init_iters; i++ )
        {
            double dist, max_dist = 0;
            pos = (pos + right_slice.start) % count;
            READ_PT(start_pt, pos);

            for( j = 1; j < count; j++ )
            {
                double dx, dy;

                READ_PT(pt, pos);
                dx = pt.x - start_pt.x;
                dy = pt.y - start_pt.y;

                dist = dx * dx + dy * dy;

                if( dist > max_dist )
                {
                    max_dist = dist;
                    right_slice.start = j;
                }
            }

            le_eps = max_dist <= eps;
        }

        // 2. initialize the stack
        if( !le_eps )
        {
            right_slice.end = slice.start = pos % count;
            slice.end = right_slice.start = (right_slice.start + slice.start) % count;

            PUSH_SLICE(right_slice);
            PUSH_SLICE(slice);
        }
        else
            WRITE_PT(start_pt);
    }

    // 3. run recursive process
    while( top > 0 )
    {
        slice = stack[--top];
        end_pt = src_contour[slice.end];
        pos = slice.start;
        READ_PT(start_pt, pos);

        if( pos != slice.end )
        {
            double dx, dy, dist, max_dist = 0;

            dx = end_pt.x - start_pt.x;
            dy = end_pt.y - start_pt.y;

            assert( dx != 0 || dy != 0 );

            while( pos != slice.end )
            {
                READ_PT(pt, pos);
                dist = fabs((pt.y - start_pt.y) * dx - (pt.x - start_pt.x) * dy);

                if( dist > max_dist )
                {
                    max_dist = dist;
                    right_slice.start = (pos+count-1)%count;
                }
            }

            le_eps = max_dist * max_dist <= eps * (dx * dx + dy * dy);
        }
        else
        {
            le_eps = true;
            // read starting point
            start_pt = src_contour[slice.start];
        }

        if( le_eps )
        {
            WRITE_PT(start_pt);
        }
        else
        {
            right_slice.end = slice.end;
            slice.end = right_slice.start;
            PUSH_SLICE(right_slice);
            PUSH_SLICE(slice);
        }
    }

    if( !is_closed )
        WRITE_PT( src_contour[count-1] );

    // last stage: do final clean-up of the approximated contour -
    // remove extra points on the [almost] stright lines.
    is_closed = is_closed0;
    count = new_count;
    pos = is_closed ? count - 1 : 0;
    READ_DST_PT(start_pt, pos);
    wpos = pos;
    READ_DST_PT(pt, pos);

    for( i = !is_closed; i < count - !is_closed && new_count > 2; i++ )
    {
        double dx, dy, dist, successive_inner_product;
        READ_DST_PT( end_pt, pos );

        dx = end_pt.x - start_pt.x;
        dy = end_pt.y - start_pt.y;
        dist = fabs((pt.x - start_pt.x)*dy - (pt.y - start_pt.y)*dx);
        successive_inner_product = (pt.x - start_pt.x) * (end_pt.x - pt.x) +
        (pt.y - start_pt.y) * (end_pt.y - pt.y);

        if( dist * dist <= 0.5*eps*(dx*dx + dy*dy) && dx != 0 && dy != 0 &&
           successive_inner_product >= 0 )
        {
            new_count--;
            dst_contour[wpos] = start_pt = end_pt;
            if(++wpos >= count) wpos = 0;
            READ_DST_PT(pt, pos);
            i++;
            continue;
        }
        dst_contour[wpos] = start_pt = pt;
        if(++wpos >= count) wpos = 0;
        pt = end_pt;
    }

    if( !is_closed )
        dst_contour[wpos] = pt;

    return new_count;
}

static int approxPolyDP_32f( const CvPoint2D32f* src_contour, int count0, CvPoint2D32f* dst_contour, bool is_closed0, double eps, AutoBuffer<Range>* _stack )
{
    #define PUSH_SLICE(slice) \
        if( top >= stacksz ) \
        { \
            _stack->resize(stacksz*3/2); \
            stack = *_stack; \
            stacksz = _stack->size(); \
        } \
        stack[top++] = slice

    #define READ_PT(pt, pos) \
        pt = src_contour[pos]; \
        if( ++pos >= count ) pos = 0

    #define READ_DST_PT(pt, pos) \
        pt = dst_contour[pos]; \
        if( ++pos >= count ) pos = 0

    #define WRITE_PT(pt) \
        dst_contour[new_count++] = pt

    typedef CvPoint2D32f PT;
    int             init_iters = 3;
    Range           slice(0, 0), right_slice(0, 0);
	PT              start_pt = { -1000000, -1000000 }, end_pt = {0, 0}, pt = {0, 0};//start_pt((T)-1000000, (T)-1000000), end_pt(0, 0), pt(0,0);
    int             i = 0, j, pos = 0, wpos, count = count0, new_count=0;
    int             is_closed = is_closed0;
    bool            le_eps = false;
    size_t top = 0, stacksz = _stack->size();
    Range*          stack = *_stack;

    if( count == 0  )
        return 0;

    eps *= eps;

    if( !is_closed )
    {
        right_slice.start = count;
        end_pt = src_contour[0];
        start_pt = src_contour[count-1];

        if( start_pt.x != end_pt.x || start_pt.y != end_pt.y )
        {
            slice.start = 0;
            slice.end = count - 1;
            PUSH_SLICE(slice);
        }
        else
        {
            is_closed = 1;
            init_iters = 1;
        }
    }

    if( is_closed )
    {
        // 1. Find approximately two farthest points of the contour
        right_slice.start = 0;

        for( i = 0; i < init_iters; i++ )
        {
            double dist, max_dist = 0;
            pos = (pos + right_slice.start) % count;
            READ_PT(start_pt, pos);

            for( j = 1; j < count; j++ )
            {
                double dx, dy;

                READ_PT(pt, pos);
                dx = pt.x - start_pt.x;
                dy = pt.y - start_pt.y;

                dist = dx * dx + dy * dy;

                if( dist > max_dist )
                {
                    max_dist = dist;
                    right_slice.start = j;
                }
            }

            le_eps = max_dist <= eps;
        }

        // 2. initialize the stack
        if( !le_eps )
        {
            right_slice.end = slice.start = pos % count;
            slice.end = right_slice.start = (right_slice.start + slice.start) % count;

            PUSH_SLICE(right_slice);
            PUSH_SLICE(slice);
        }
        else
            WRITE_PT(start_pt);
    }

    // 3. run recursive process
    while( top > 0 )
    {
        slice = stack[--top];
        end_pt = src_contour[slice.end];
        pos = slice.start;
        READ_PT(start_pt, pos);

        if( pos != slice.end )
        {
            double dx, dy, dist, max_dist = 0;

            dx = end_pt.x - start_pt.x;
            dy = end_pt.y - start_pt.y;

            assert( dx != 0 || dy != 0 );

            while( pos != slice.end )
            {
                READ_PT(pt, pos);
                dist = fabs((pt.y - start_pt.y) * dx - (pt.x - start_pt.x) * dy);

                if( dist > max_dist )
                {
                    max_dist = dist;
                    right_slice.start = (pos+count-1)%count;
                }
            }

            le_eps = max_dist * max_dist <= eps * (dx * dx + dy * dy);
        }
        else
        {
            le_eps = true;
            // read starting point
            start_pt = src_contour[slice.start];
        }

        if( le_eps )
        {
            WRITE_PT(start_pt);
        }
        else
        {
            right_slice.end = slice.end;
            slice.end = right_slice.start;
            PUSH_SLICE(right_slice);
            PUSH_SLICE(slice);
        }
    }

    if( !is_closed )
        WRITE_PT( src_contour[count-1] );

    // last stage: do final clean-up of the approximated contour -
    // remove extra points on the [almost] stright lines.
    is_closed = is_closed0;
    count = new_count;
    pos = is_closed ? count - 1 : 0;
    READ_DST_PT(start_pt, pos);
    wpos = pos;
    READ_DST_PT(pt, pos);

    for( i = !is_closed; i < count - !is_closed && new_count > 2; i++ )
    {
        double dx, dy, dist, successive_inner_product;
        READ_DST_PT( end_pt, pos );

        dx = end_pt.x - start_pt.x;
        dy = end_pt.y - start_pt.y;
        dist = fabs((pt.x - start_pt.x)*dy - (pt.y - start_pt.y)*dx);
        successive_inner_product = (pt.x - start_pt.x) * (end_pt.x - pt.x) +
        (pt.y - start_pt.y) * (end_pt.y - pt.y);

        if( dist * dist <= 0.5*eps*(dx*dx + dy*dy) && dx != 0 && dy != 0 &&
           successive_inner_product >= 0 )
        {
            new_count--;
            dst_contour[wpos] = start_pt = end_pt;
            if(++wpos >= count) wpos = 0;
            READ_DST_PT(pt, pos);
            i++;
            continue;
        }
        dst_contour[wpos] = start_pt = pt;
        if(++wpos >= count) wpos = 0;
        pt = end_pt;
    }

    if( !is_closed )
        dst_contour[wpos] = pt;

    return new_count;
}

CvSeq* cvApproxPoly( const CvSeq* src_seq, int header_size, CvMemStorage* storage, int method, double parameter, int parameter2 )
{
	AutoBuffer<CvPoint> _buf;
    AutoBuffer<Range> stack(100);
    CvSeq* dst_seq = 0;
    CvSeq *prev_contour = 0, *parent = 0;
    int recursive = 0;

    if( !CV_IS_SEQ_POLYLINE( src_seq ))
    {
		printf("Unsupported sequence type\n" );
		return 0;
	}

    recursive = parameter2;

    if( !storage )
        storage = src_seq->storage;

    if( !storage )
    {
		printf("NULL storage pointer\n" );
		return 0;
	}

    if( header_size < 0 )
    {
		printf("header_size is negative. "
                 "Pass 0 to make the destination header_size == input header_size\n" );
		return 0;
	}

    if( header_size == 0 )
        header_size = src_seq->header_size;

    if( !CV_IS_SEQ_POLYLINE( src_seq ))
    {
        if( CV_IS_SEQ_CHAIN( src_seq ))
        {
            printf("Input curves are not polygonal. "
                     "Use cvApproxChains first\n" );
			return 0;
        }
        else
        {
            printf("Input curves have uknown type\n" );
			return 0;
        }
    }

    if( header_size == 0 )
        header_size = src_seq->header_size;

    if( header_size < (int)sizeof(CvContour) )
    {
		printf("New header size must be non-less than sizeof(CvContour)\n" );
		return 0;
	}

    if( method != CV_POLY_APPROX_DP )
    {
		printf("Unknown approximation method\n" );
		return 0;
	}

    while( src_seq != 0 )
    {
        CvSeq *contour = 0;

        switch (method)
        {
        case CV_POLY_APPROX_DP:
            if( parameter < 0 )
            {
				printf("Accuracy must be non-negative\n" );
				return 0;
			}

             assert( CV_SEQ_ELTYPE(src_seq) == CV_32SC2 ||
                      CV_SEQ_ELTYPE(src_seq) == CV_32FC2 );

            {
            int npoints = src_seq->total, nout = 0;
            _buf.allocate(npoints*2);
            CvPoint *src = _buf, *dst = src + npoints;
            bool closed = CV_IS_SEQ_CLOSED(src_seq);

            if( src_seq->first->next == src_seq->first )
                src = (CvPoint*)src_seq->first->data;
			else
                cvCvtSeqToArray(src_seq, src);

            if( CV_SEQ_ELTYPE(src_seq) == CV_32SC2 )
                nout = approxPolyDP_32s(src, npoints, dst, closed, parameter, &stack);
            else if( CV_SEQ_ELTYPE(src_seq) == CV_32FC2 )
                nout = approxPolyDP_32f((CvPoint2D32f*)src, npoints,
                                         (CvPoint2D32f*)dst, closed, parameter, &stack);
            else
                return 0;

            contour = cvCreateSeq( src_seq->flags, header_size,
                                  src_seq->elem_size, storage );
            cvSeqPushMulti(contour, dst, nout);
            }
            break;
        default:
			{
				printf("Invalid approximation method" );
				return 0;
			}
        }

        assert( contour );

        if( header_size >= (int)sizeof(CvContour))
            cvBoundingRect( contour, 1 );

        contour->v_prev = parent;
        contour->h_prev = prev_contour;

        if( prev_contour )
            prev_contour->h_next = contour;
        else if( parent )
            parent->v_next = contour;
        prev_contour = contour;
        if( !dst_seq )
            dst_seq = prev_contour;

        if( !recursive )
            break;

        if( src_seq->v_next )
        {
            assert( prev_contour != 0 );
            parent = prev_contour;
            prev_contour = 0;
            src_seq = src_seq->v_next;
        }
        else
        {
            while( src_seq->h_next == 0 )
            {
                src_seq = src_seq->v_prev;
                if( src_seq == 0 )
                    break;
                prev_contour = parent;
                if( parent )
                    parent = parent->v_prev;
            }
            if( src_seq )
                src_seq = src_seq->h_next;
        }
    }

    return dst_seq;
}

/****************************************************************************************\
*                            Contour Processing and Shape Analysis                       *
\****************************************************************************************/

double cvArcLength( const CvSeq *contour, CvSlice slice, int is_closed )
{
    double perimeter = 0;

    int i, j = 0, count;
    const int N = 16;
    float buf[N];
    CvSeqReader reader;

    if( !CV_IS_SEQ_POLYLINE( contour ))
    {
		printf("Unsupported sequence type\n" );
		return 0;
	}
    if( is_closed < 0 )
        is_closed = CV_IS_SEQ_CLOSED( contour );

    if( contour->total > 1 )
    {
        int is_float = CV_SEQ_ELTYPE( contour ) == CV_32FC2;

        cvStartReadSeq( contour, &reader, 0 );
        cvSetSeqReaderPos( &reader, slice.start_index );
        count = cvSliceLength( slice, contour );

        count -= !is_closed && count == contour->total;

        // scroll the reader by 1 point
        reader.prev_elem = reader.ptr;
        CV_NEXT_SEQ_ELEM( sizeof(CvPoint), reader );

        for( i = 0; i < count; i++ )
        {
            float dx, dy;

            if( !is_float )
            {
                CvPoint* pt = (CvPoint*)reader.ptr;
                CvPoint* prev_pt = (CvPoint*)reader.prev_elem;

                dx = (float)pt->x - (float)prev_pt->x;
                dy = (float)pt->y - (float)prev_pt->y;
            }
            else
            {
                CvPoint2D32f* pt = (CvPoint2D32f*)reader.ptr;
                CvPoint2D32f* prev_pt = (CvPoint2D32f*)reader.prev_elem;

                dx = pt->x - prev_pt->x;
                dy = pt->y - prev_pt->y;
            }

            reader.prev_elem = reader.ptr;
            CV_NEXT_SEQ_ELEM( contour->elem_size, reader );
            // Bugfix by Axel at rubico.com 2010-03-22, affects closed slices only
            // wraparound not handled by CV_NEXT_SEQ_ELEM
            if( is_closed && i == count - 2 )
                cvSetSeqReaderPos( &reader, slice.start_index );

            buf[j] = sqrt(dx * dx + dy * dy);
            if( ++j == N || i == count - 1 )
            {
                for( ; j > 0; j-- )
                    perimeter += buf[j-1];
            }
        }
    }

    return perimeter;
}

CvRect cvBoundingRect( const CvSeq* ptseq, int update )
{
    CvSeqReader reader;
    CvRect  rect = { 0, 0, 0, 0 };
	int  xmin = 0, ymin = 0, xmax = -1, ymax = -1, i;
    int calculate = update;

    if( !CV_IS_SEQ_POINT_SET( ptseq ))
    {
		printf("Unsupported sequence type\n" );
		return rect;
	}

    if( ptseq->header_size < (int)sizeof(CvContour))
    {
        update = 0;
        calculate = 1;
    }

    if( !calculate )
        return ((CvContour*)ptseq)->rect;

    if (ptseq->total)
	{
		int  is_float = CV_SEQ_ELTYPE(ptseq) == CV_32FC2;
        cvStartReadSeq( ptseq, &reader, 0 );

        if( !is_float )
        {
            CvPoint pt;
            /* init values */
            CV_READ_SEQ_ELEM( pt, reader );
            xmin = xmax = pt.x;
            ymin = ymax = pt.y;

            for( i = 1; i < ptseq->total; i++ )
            {            
                CV_READ_SEQ_ELEM( pt, reader );
        
                if( xmin > pt.x )
                    xmin = pt.x;
        
                if( xmax < pt.x )
                    xmax = pt.x;

                if( ymin > pt.y )
                    ymin = pt.y;

                if( ymax < pt.y )
                    ymax = pt.y;
            }
        }
        else
        {
            CvPoint pt;
            Cv32suf v;
            /* init values */
            CV_READ_SEQ_ELEM( pt, reader );
            xmin = xmax = CV_TOGGLE_FLT(pt.x);
            ymin = ymax = CV_TOGGLE_FLT(pt.y);

            for( i = 1; i < ptseq->total; i++ )
            {            
                CV_READ_SEQ_ELEM( pt, reader );
                pt.x = CV_TOGGLE_FLT(pt.x);
                pt.y = CV_TOGGLE_FLT(pt.y);
        
                if( xmin > pt.x )
                    xmin = pt.x;
        
                if( xmax < pt.x )
                    xmax = pt.x;

                if( ymin > pt.y )
                    ymin = pt.y;

                if( ymax < pt.y )
                    ymax = pt.y;
            }

            v.i = CV_TOGGLE_FLT(xmin); xmin = cvFloor(v.f);
            v.i = CV_TOGGLE_FLT(ymin); ymin = cvFloor(v.f);
            /* because right and bottom sides of
               the bounding rectangle are not inclusive
               (note +1 in width and height calculation below),
               cvFloor is used here instead of cvCeil */
            v.i = CV_TOGGLE_FLT(xmax); xmax = cvFloor(v.f);
            v.i = CV_TOGGLE_FLT(ymax); ymax = cvFloor(v.f);
        }
	}

	rect.x = xmin;
    rect.y = ymin;
    rect.width = xmax - xmin + 1;
    rect.height = ymax - ymin + 1;

    if( update )
        ((CvContour*)ptseq)->rect = rect;
    return rect;
}

double contourArea( const CvSeq* contour, bool oriented )
{
	double a00 = 0;
    if( contour->total )
    {
        CvSeqReader reader;
        int lpt = contour->total;
        double xi_1, yi_1;
        int is_float = CV_SEQ_ELTYPE(contour) == CV_32FC2;

        cvStartReadSeq( contour, &reader, 0 );

        if( !is_float )
        {
            xi_1 = ((CvPoint*)(reader.ptr))->x;
            yi_1 = ((CvPoint*)(reader.ptr))->y;
        }
        else
        {
            xi_1 = ((CvPoint2D32f*)(reader.ptr))->x;
            yi_1 = ((CvPoint2D32f*)(reader.ptr))->y;
        }
        CV_NEXT_SEQ_ELEM( contour->elem_size, reader );
        
        while( lpt-- > 0 )
        {
            double dxy, xi, yi;

            if( !is_float )
            {
                xi = ((CvPoint*)(reader.ptr))->x;
                yi = ((CvPoint*)(reader.ptr))->y;
            }
            else
            {
                xi = ((CvPoint2D32f*)(reader.ptr))->x;
                yi = ((CvPoint2D32f*)(reader.ptr))->y;
            }
            CV_NEXT_SEQ_ELEM( contour->elem_size, reader );

            dxy = xi_1 * yi - xi * yi_1;
            a00 += dxy;
            xi_1 = xi;
            yi_1 = yi;
        }

        a00 *= 0.5;
    }
    else
        a00 = 0.;
    if( !oriented )
        a00 = fabs(a00);

    return a00;
}

static double icvContourSecArea( const CvSeq * contour, CvSlice slice )
{
    CvPoint pt;                 /*  pointer to points   */
    CvPoint pt_s, pt_e;         /*  first and last points  */
    CvSeqReader reader;         /*  points reader of contour   */

    int p_max = 2, p_ind;
    int lpt, flag, i;
    double a00;                 /* unnormalized moments m00    */
    double xi, yi, xi_1, yi_1, x0, y0, dxy, sk, sk1, t;
    double x_s, y_s, nx, ny, dx, dy, du, dv;
    double eps = 1.e-5;
    double *p_are1, *p_are2, *p_are;
    double area = 0;

    assert( contour != NULL && CV_IS_SEQ_POINT_SET( contour ));

    lpt = cvSliceLength( slice, contour );
    /*if( n2 >= n1 )
        lpt = n2 - n1 + 1;
    else
        lpt = contour->total - n1 + n2 + 1;*/

    if( contour->total <= 0 || lpt <= 2 )
        return 0.;

    a00 = x0 = y0 = xi_1 = yi_1 = 0;
    sk1 = 0;
    flag = 0;
    dxy = 0;
    p_are1 = (double *) cvAlloc( p_max * sizeof( double ));

    p_are = p_are1;
    p_are2 = NULL;

    cvStartReadSeq( contour, &reader, 0 );
    cvSetSeqReaderPos( &reader, slice.start_index );
    CV_READ_SEQ_ELEM( pt_s, reader );
    p_ind = 0;
    cvSetSeqReaderPos( &reader, slice.end_index );
    CV_READ_SEQ_ELEM( pt_e, reader );

/*    normal coefficients    */
    nx = pt_s.y - pt_e.y;
    ny = pt_e.x - pt_s.x;
    cvSetSeqReaderPos( &reader, slice.start_index );

    while( lpt-- > 0 )
    {
        CV_READ_SEQ_ELEM( pt, reader );

        if( flag == 0 )
        {
            xi_1 = (double) pt.x;
            yi_1 = (double) pt.y;
            x0 = xi_1;
            y0 = yi_1;
            sk1 = 0;
            flag = 1;
        }
        else
        {
            xi = (double) pt.x;
            yi = (double) pt.y;

/****************   edges intersection examination   **************************/
            sk = nx * (xi - pt_s.x) + ny * (yi - pt_s.y);
            if( (fabs( sk ) < eps && lpt > 0) || sk * sk1 < -eps )
            {
                if( fabs( sk ) < eps )
                {
                    dxy = xi_1 * yi - xi * yi_1;
                    a00 = a00 + dxy;
                    dxy = xi * y0 - x0 * yi;
                    a00 = a00 + dxy;

                    if( p_ind >= p_max )
                        icvMemCopy( &p_are1, &p_are2, &p_are, &p_max );

                    p_are[p_ind] = a00 / 2.;
                    p_ind++;
                    a00 = 0;
                    sk1 = 0;
                    x0 = xi;
                    y0 = yi;
                    dxy = 0;
                }
                else
                {
/*  define intersection point    */
                    dv = yi - yi_1;
                    du = xi - xi_1;
                    dx = ny;
                    dy = -nx;
                    if( fabs( du ) > eps )
                        t = ((yi_1 - pt_s.y) * du + dv * (pt_s.x - xi_1)) /
                            (du * dy - dx * dv);
                    else
                        t = (xi_1 - pt_s.x) / dx;
                    if( t > eps && t < 1 - eps )
                    {
                        x_s = pt_s.x + t * dx;
                        y_s = pt_s.y + t * dy;
                        dxy = xi_1 * y_s - x_s * yi_1;
                        a00 += dxy;
                        dxy = x_s * y0 - x0 * y_s;
                        a00 += dxy;
                        if( p_ind >= p_max )
                            icvMemCopy( &p_are1, &p_are2, &p_are, &p_max );

                        p_are[p_ind] = a00 / 2.;
                        p_ind++;

                        a00 = 0;
                        sk1 = 0;
                        x0 = x_s;
                        y0 = y_s;
                        dxy = x_s * yi - xi * y_s;
                    }
                }
            }
            else
                dxy = xi_1 * yi - xi * yi_1;

            a00 += dxy;
            xi_1 = xi;
            yi_1 = yi;
            sk1 = sk;

        }
    }

    xi = x0;
    yi = y0;
    dxy = xi_1 * yi - xi * yi_1;

    a00 += dxy;

    if( p_ind >= p_max )
        icvMemCopy( &p_are1, &p_are2, &p_are, &p_max );

    p_are[p_ind] = a00 / 2.;
    p_ind++;

    // common area calculation
    area = 0;
    for( i = 0; i < p_ind; i++ )
        area += fabs( p_are[i] );

    if( p_are1 != NULL )
        cvFree( &p_are1 );
    else if( p_are2 != NULL )
        cvFree( &p_are2 );

    return area;
}

double cvContourArea( const CvSeq* contour, CvSlice slice, int oriented )
{
    double area = 0;

    if( !CV_IS_SEQ_POLYLINE( contour ))
    {
		printf("Unsupported sequence type\n" );
		return 0;
	}

    if( cvSliceLength( slice, contour ) == contour->total )
    {
        return contourArea( contour, oriented !=0 );
    }

    if( CV_SEQ_ELTYPE( contour ) != CV_32SC2 )
    {
		printf("Only curves with integer coordinates are supported in case of contour slice\n" );
		return 0;
	}
    area = icvContourSecArea( contour, slice );
    return oriented ? area : fabs(area);
}

static int
icvSklansky_32s( CvPoint** array, int start, int end, int* stack, int nsign, int sign2 )
{
    int incr = end > start ? 1 : -1;
    /* prepare first triangle */
    int pprev = start, pcur = pprev + incr, pnext = pcur + incr;
    int stacksize = 3;

    if( start == end ||
        (array[start]->x == array[end]->x &&
         array[start]->y == array[end]->y) )
    {
        stack[0] = start;
        return 1;
    }

    stack[0] = pprev;
    stack[1] = pcur;
    stack[2] = pnext;

    end += incr; /* make end = afterend */

    while( pnext != end )
    {
        /* check the angle p1,p2,p3 */
        int cury = array[pcur]->y;
        int nexty = array[pnext]->y;
        int by = nexty - cury;

        if( CV_SIGN(by) != nsign )
        {
            int ax = array[pcur]->x - array[pprev]->x;
            int bx = array[pnext]->x - array[pcur]->x;
            int ay = cury - array[pprev]->y;
            int convexity = ay*bx - ax*by;/* if >0 then convex angle */

            if( CV_SIGN(convexity) == sign2 && (ax != 0 || ay != 0) )
            {
                pprev = pcur;
                pcur = pnext;
                pnext += incr;
                stack[stacksize] = pnext;
                stacksize++;
            }
            else
            {
                if( pprev == start )
                {
                    pcur = pnext;
                    stack[1] = pcur;
                    pnext += incr;
                    stack[2] = pnext;
                }
                else
                {
                    stack[stacksize-2] = pnext;
                    pcur = pprev;
                    pprev = stack[stacksize-4];
                    stacksize--;
                }
            }
        }
        else
        {
            pnext += incr;
            stack[stacksize-1] = pnext;
        }
    }

    return --stacksize;
}


static int
icvSklansky_32f( CvPoint2D32f** array, int start, int end, int* stack, int nsign, int sign2 )
{
    int incr = end > start ? 1 : -1;
    /* prepare first triangle */
    int pprev = start, pcur = pprev + incr, pnext = pcur + incr;
    int stacksize = 3;

    if( start == end ||
        (array[start]->x == array[end]->x &&
         array[start]->y == array[end]->y) )
    {
        stack[0] = start;
        return 1;
    }

    stack[0] = pprev;
    stack[1] = pcur;
    stack[2] = pnext;

    end += incr; /* make end = afterend */

    while( pnext != end )
    {
        /* check the angle p1,p2,p3 */
        float cury = array[pcur]->y;
        float nexty = array[pnext]->y;
        float by = nexty - cury;

        if( CV_SIGN( by ) != nsign )
        {
            float ax = array[pcur]->x - array[pprev]->x;
            float bx = array[pnext]->x - array[pcur]->x;
            float ay = cury - array[pprev]->y;
            float convexity = ay*bx - ax*by;/* if >0 then convex angle */

            if( CV_SIGN( convexity ) == sign2 && (ax != 0 || ay != 0) )
            {
                pprev = pcur;
                pcur = pnext;
                pnext += incr;
                stack[stacksize] = pnext;
                stacksize++;
            }
            else
            {
                if( pprev == start )
                {
                    pcur = pnext;
                    stack[1] = pcur;
                    pnext += incr;
                    stack[2] = pnext;

                }
                else
                {
                    stack[stacksize-2] = pnext;
                    pcur = pprev;
                    pprev = stack[stacksize-4];
                    stacksize--;
                }
            }
        }
        else
        {
            pnext += incr;
            stack[stacksize-1] = pnext;
        }
    }

    return --stacksize;
}

static void
icvCalcAndWritePtIndices( CvPoint** pointer, int* stack, int start, int end,
                          const CvSeq* ptseq, CvSeqWriter* writer )
{
    int i, incr = start < end ? 1 : -1;
    int idx, first_idx = ptseq->first->start_index;

    for( i = start; i != end; i += incr )
    {
        CvPoint* ptr = (CvPoint*)pointer[stack[i]];
        CvSeqBlock* block = ptseq->first;
        while( (unsigned)(idx = (int)(ptr - (CvPoint*)block->data)) >= (unsigned)block->count )
        {
            block = block->next;
            if( block == ptseq->first )
                return;
        }
        idx += block->start_index - first_idx;
        CV_WRITE_SEQ_ELEM( idx, *writer );
    }
}

CvSeq* cvConvexHull2( const CvSeq* ptseq, void* hull_storage, int orientation, int return_points )
{
    union { CvContour* c; CvSeq* s; } hull;
    CvPoint** pointer = 0;
    CvPoint2D32f** pointerf = 0;
    int* stack = 0;
    hull.s = 0;
    CvSeqReader reader;
    CvSeqWriter writer;
    union { CvContour c; CvSeq s; } hull_header;
    CvSeqBlock hullblock;
    CvSeq* hullseq = 0;
    int is_float;
    int* t_stack;
    int t_count;
    int i, miny_ind = 0, maxy_ind = 0, total;
    int hulltype;
    int stop_idx;
    sklansky_func sklansky;

    if( !CV_IS_SEQ_POINT_SET( ptseq ))
        return 0;
    if( hull_storage == 0 )
        hull_storage = ptseq->storage;

    if( CV_IS_STORAGE( hull_storage ))
    {
        if( return_points )
        {
            hullseq = cvCreateSeq(
                CV_SEQ_KIND_CURVE|CV_SEQ_ELTYPE(ptseq)|
                CV_SEQ_FLAG_CLOSED|CV_SEQ_FLAG_CONVEX,
                sizeof(CvContour), sizeof(CvPoint),(CvMemStorage*)hull_storage );
        }
        else
        {
            hullseq = cvCreateSeq(
                CV_SEQ_KIND_CURVE|CV_SEQ_ELTYPE_PPOINT|
                CV_SEQ_FLAG_CLOSED|CV_SEQ_FLAG_CONVEX,
                sizeof(CvContour), sizeof(CvPoint*), (CvMemStorage*)hull_storage );
        }
    }

    total = ptseq->total;
    if( total == 0 )
    {
       return 0;
    }

    cvStartAppendToSeq( hullseq, &writer );

    is_float = CV_SEQ_ELTYPE(ptseq) == CV_32FC2;
    hulltype = CV_SEQ_ELTYPE(hullseq);
    sklansky = !is_float ? (sklansky_func)icvSklansky_32s :
                           (sklansky_func)icvSklansky_32f;

    pointer = (CvPoint**)cvAlloc( ptseq->total*sizeof(pointer[0]) );
    stack = (int*)cvAlloc( (ptseq->total + 2)*sizeof(stack[0]) );
    pointerf = (CvPoint2D32f**)pointer;

    cvStartReadSeq( ptseq, &reader );

    for( i = 0; i < total; i++ )
    {
        pointer[i] = (CvPoint*)reader.ptr;
        CV_NEXT_SEQ_ELEM( ptseq->elem_size, reader );
    }

    // sort the point set by x-coordinate, find min and max y
    if( !is_float )
    {
        icvSortPointsByPointers_32s( pointer, total, 0 );
        for( i = 1; i < total; i++ )
        {
            int y = pointer[i]->y;
            if( pointer[miny_ind]->y > y )
                miny_ind = i;
            if( pointer[maxy_ind]->y < y )
                maxy_ind = i;
        }
    }
    else
    {
        icvSortPointsByPointers_32f( pointerf, total, 0 );
        for( i = 1; i < total; i++ )
        {
            float y = pointerf[i]->y;
            if( pointerf[miny_ind]->y > y )
                miny_ind = i;
            if( pointerf[maxy_ind]->y < y )
                maxy_ind = i;
        }
    }

    if( pointer[0]->x == pointer[total-1]->x &&
        pointer[0]->y == pointer[total-1]->y )
    {
        if( hulltype == CV_SEQ_ELTYPE_PPOINT )
        {
            CV_WRITE_SEQ_ELEM( pointer[0], writer );
        }
        else if( hulltype == CV_SEQ_ELTYPE_INDEX )
        {
            int index = 0;
            CV_WRITE_SEQ_ELEM( index, writer );
        }
        else
        {
            CvPoint pt = pointer[0][0];
            CV_WRITE_SEQ_ELEM( pt, writer );
        }
        goto finish_hull;
    }

    /*upper half */
    {
        int *tl_stack = stack;
        int tl_count = sklansky( pointer, 0, maxy_ind, tl_stack, -1, 1 );
        int *tr_stack = tl_stack + tl_count;
        int tr_count = sklansky( pointer, ptseq->total - 1, maxy_ind, tr_stack, -1, -1 );

        /* gather upper part of convex hull to output */
        if( orientation == CV_COUNTER_CLOCKWISE )
        {
            CV_SWAP( tl_stack, tr_stack, t_stack );
            CV_SWAP( tl_count, tr_count, t_count );
        }

        if( hulltype == CV_SEQ_ELTYPE_PPOINT )
        {
            for( i = 0; i < tl_count - 1; i++ )
                CV_WRITE_SEQ_ELEM( pointer[tl_stack[i]], writer );

            for( i = tr_count - 1; i > 0; i-- )
                CV_WRITE_SEQ_ELEM( pointer[tr_stack[i]], writer );
        }
        else if( hulltype == CV_SEQ_ELTYPE_INDEX )
        {
            icvCalcAndWritePtIndices( pointer, tl_stack,
                                               0, tl_count-1, ptseq, &writer );
            icvCalcAndWritePtIndices( pointer, tr_stack,
                                               tr_count-1, 0, ptseq, &writer );
        }
        else
        {
            for( i = 0; i < tl_count - 1; i++ )
                CV_WRITE_SEQ_ELEM( pointer[tl_stack[i]][0], writer );

            for( i = tr_count - 1; i > 0; i-- )
                CV_WRITE_SEQ_ELEM( pointer[tr_stack[i]][0], writer );
        }
        stop_idx = tr_count > 2 ? tr_stack[1] : tl_count > 2 ? tl_stack[tl_count - 2] : -1;
    }

    /* lower half */
    {
        int *bl_stack = stack;
        int bl_count = sklansky( pointer, 0, miny_ind, bl_stack, 1, -1 );
        int *br_stack = stack + bl_count;
        int br_count = sklansky( pointer, ptseq->total - 1, miny_ind, br_stack, 1, 1 );

        if( orientation != CV_COUNTER_CLOCKWISE )
        {
            CV_SWAP( bl_stack, br_stack, t_stack );
            CV_SWAP( bl_count, br_count, t_count );
        }

        if( stop_idx >= 0 )
        {
            int check_idx = bl_count > 2 ? bl_stack[1] :
                            bl_count + br_count > 2 ? br_stack[2-bl_count] : -1;
            if( check_idx == stop_idx || check_idx >= 0 &&
                pointer[check_idx]->x == pointer[stop_idx]->x &&
                pointer[check_idx]->y == pointer[stop_idx]->y )
            {
                /* if all the points lie on the same line, then
                   the bottom part of the convex hull is the mirrored top part
                   (except the exteme points).*/
                bl_count = MIN( bl_count, 2 );
                br_count = MIN( br_count, 2 );
            }
        }

        if( hulltype == CV_SEQ_ELTYPE_PPOINT )
        {
            for( i = 0; i < bl_count - 1; i++ )
                CV_WRITE_SEQ_ELEM( pointer[bl_stack[i]], writer );

            for( i = br_count - 1; i > 0; i-- )
                CV_WRITE_SEQ_ELEM( pointer[br_stack[i]], writer );
        }
        else if( hulltype == CV_SEQ_ELTYPE_INDEX )
        {
            icvCalcAndWritePtIndices( pointer, bl_stack,
                                               0, bl_count-1, ptseq, &writer );
            icvCalcAndWritePtIndices( pointer, br_stack,
                                               br_count-1, 0, ptseq, &writer );
        }
        else
        {
            for( i = 0; i < bl_count - 1; i++ )
                CV_WRITE_SEQ_ELEM( pointer[bl_stack[i]][0], writer );

            for( i = br_count - 1; i > 0; i-- )
                CV_WRITE_SEQ_ELEM( pointer[br_stack[i]][0], writer );
        }
    }

finish_hull:
    cvEndWriteSeq( &writer );

    
    hull.s = hullseq;
    hull.c->rect = cvBoundingRect( ptseq, ptseq->header_size < (int)sizeof(CvContour) );
        
    /*if( ptseq != (CvSeq*)&contour_header )
        hullseq->v_prev = ptseq;*/

    cvFree( &pointer );
    cvFree( &stack );

    return hull.s;
}

bool isContourConvex( const CvSeq* contour )
{
    int flag = -1;
    int i;
    int orientation = 0;
    CvSeqReader reader;
    
    cvStartReadSeq( contour, &reader, 0 );
    
    flag = 1;

    if( CV_SEQ_ELTYPE( contour ) == CV_32SC2 )
    {
        CvPoint *prev_pt = (CvPoint*)reader.prev_elem;
        CvPoint *cur_pt = (CvPoint*)reader.ptr;
    
        int dx0 = cur_pt->x - prev_pt->x;
        int dy0 = cur_pt->y - prev_pt->y;

        for( i = 0; i < contour->total; i++ )
        {
            int dxdy0, dydx0;
            int dx, dy;

            /*int orient; */
            CV_NEXT_SEQ_ELEM( sizeof(CvPoint), reader );
            prev_pt = cur_pt;
            cur_pt = (CvPoint *) reader.ptr;

            dx = cur_pt->x - prev_pt->x;
            dy = cur_pt->y - prev_pt->y;
            dxdy0 = dx * dy0;
            dydx0 = dy * dx0;

            /* find orientation */
            /*orient = -dy0 * dx + dx0 * dy;
               orientation |= (orient > 0) ? 1 : 2;
             */
            orientation |= (dydx0 > dxdy0) ? 1 : ((dydx0 < dxdy0) ? 2 : 3);

            if( orientation == 3 )
            {
                flag = 0;
                break;
            }

            dx0 = dx;
            dy0 = dy;
        }
    }
    else
    {
        assert( CV_SEQ_ELTYPE(contour) == CV_32FC2 );

        CvPoint2D32f *prev_pt = (CvPoint2D32f*)reader.prev_elem;
        CvPoint2D32f *cur_pt = (CvPoint2D32f*)reader.ptr;
    
        float dx0 = cur_pt->x - prev_pt->x;
        float dy0 = cur_pt->y - prev_pt->y;

        for( i = 0; i < contour->total; i++ )
        {
            float dxdy0, dydx0;
            float dx, dy;

            /*int orient; */
            CV_NEXT_SEQ_ELEM( sizeof(CvPoint2D32f), reader );
            prev_pt = cur_pt;
            cur_pt = (CvPoint2D32f*) reader.ptr;

            dx = cur_pt->x - prev_pt->x;
            dy = cur_pt->y - prev_pt->y;
            dxdy0 = dx * dy0;
            dydx0 = dy * dx0;

            /* find orientation */
            /*orient = -dy0 * dx + dx0 * dy;
               orientation |= (orient > 0) ? 1 : 2;
             */
            orientation |= (dydx0 > dxdy0) ? 1 : ((dydx0 < dxdy0) ? 2 : 3);

            if( orientation == 3 )
            {
                flag = 0;
                break;
            }

            dx0 = dx;
            dy0 = dy;
        }
    }

    return flag;
}

int cvCheckContourConvexity( const CvSeq* contour )
{
    if( !CV_IS_SEQ_POINT_SET(contour))
    {
		printf("Input sequence must be polygon (closed 2d curve)\n" );
		return -1;
	}

    if( contour->total == 0 )
        return -1;

    return isContourConvex(contour) ? 1 : 0;
}

/****************************************************************************************\
*            Functions for manipulating memory storage - list of memory blocks           *
\****************************************************************************************/

/* Initialize allocated storage: */
static void icvInitMemStorage( CvMemStorage* storage, int block_size )
{
    if( !storage )
        return;

    if( block_size <= 0 )
        block_size = CV_STORAGE_BLOCK_SIZE;

    block_size = cvAlign( block_size, CV_STRUCT_ALIGN );
    assert( sizeof(CvMemBlock) % CV_STRUCT_ALIGN == 0 );

    memset( storage, 0, sizeof( *storage ));
    storage->signature = CV_STORAGE_MAGIC_VAL;
    storage->block_size = block_size;
}


/* Create root memory storage: */
CvMemStorage* cvCreateMemStorage( int block_size )
{
    CvMemStorage* storage = (CvMemStorage *)cvAlloc( sizeof( CvMemStorage ));
    icvInitMemStorage( storage, block_size );
    return storage;
}


/* Create child memory storage: */
CvMemStorage *cvCreateChildMemStorage( CvMemStorage * parent )
{
    if( !parent )
        return 0;

    CvMemStorage* storage = cvCreateMemStorage(parent->block_size);
    storage->parent = parent;

    return storage;
}


/* Release all blocks of the storage (or return them to parent, if any): */
static void icvDestroyMemStorage( CvMemStorage* storage )
{
    int k = 0;

    CvMemBlock *block;
    CvMemBlock *dst_top = 0;

    if( !storage )
        return;

    if( storage->parent )
        dst_top = storage->parent->top;

    for( block = storage->bottom; block != 0; k++ )
    {
        CvMemBlock *temp = block;

        block = block->next;
        if( storage->parent )
        {
            if( dst_top )
            {
                temp->prev = dst_top;
                temp->next = dst_top->next;
                if( temp->next )
                    temp->next->prev = temp;
                dst_top = dst_top->next = temp;
            }
            else
            {
                dst_top = storage->parent->bottom = storage->parent->top = temp;
                temp->prev = temp->next = 0;
                storage->free_space = storage->block_size - sizeof( *temp );
            }
        }
        else
        {
            cvFree( &temp );
        }
    }

    storage->top = storage->bottom = 0;
    storage->free_space = 0;
}


/* Release memory storage: */
void cvReleaseMemStorage( CvMemStorage** storage )
{
    if( !storage )
        return;

    CvMemStorage* st = *storage;
    *storage = 0;
    if( st )
    {
        icvDestroyMemStorage( st );
        cvFree( &st );
    }
}


/* Clears memory storage (return blocks to the parent, if any): */
void cvClearMemStorage( CvMemStorage * storage )
{
    if( !storage )
        return;

    if( storage->parent )
        icvDestroyMemStorage( storage );
    else
    {
        storage->top = storage->bottom;
        storage->free_space = storage->bottom ? storage->block_size - sizeof(CvMemBlock) : 0;
    }
}


/* Moves stack pointer to next block.
   If no blocks, allocate new one and link it to the storage: */
static void icvGoNextMemBlock( CvMemStorage * storage )
{
    if( !storage )
        return;

    if( !storage->top || !storage->top->next )
    {
        CvMemBlock *block;

        if( !(storage->parent) )
        {
            block = (CvMemBlock *)cvAlloc( storage->block_size );
        }
        else
        {
            CvMemStorage *parent = storage->parent;
            CvMemStoragePos parent_pos;

            cvSaveMemStoragePos( parent, &parent_pos );
            icvGoNextMemBlock( parent );

            block = parent->top;
            cvRestoreMemStoragePos( parent, &parent_pos );

            if( block == parent->top )  /* the single allocated block */
            {
                assert( parent->bottom == block );
                parent->top = parent->bottom = 0;
                parent->free_space = 0;
            }
            else
            {
                /* cut the block from the parent's list of blocks */
                parent->top->next = block->next;
                if( block->next )
                    block->next->prev = parent->top;
            }
        }

        /* link block */
        block->next = 0;
        block->prev = storage->top;

        if( storage->top )
            storage->top->next = block;
        else
            storage->top = storage->bottom = block;
    }

    if( storage->top->next )
        storage->top = storage->top->next;
    storage->free_space = storage->block_size - sizeof(CvMemBlock);
    assert( storage->free_space % CV_STRUCT_ALIGN == 0 );
}


/* Remember memory storage position: */
void cvSaveMemStoragePos( const CvMemStorage * storage, CvMemStoragePos * pos )
{
    if( !storage || !pos )
        return;

    pos->top = storage->top;
    pos->free_space = storage->free_space;
}


/* Restore memory storage position: */
void cvRestoreMemStoragePos( CvMemStorage * storage, CvMemStoragePos * pos )
{
    if( !storage || !pos )
        return;
    if( pos->free_space > storage->block_size )
        return;

    /*
    // this breaks icvGoNextMemBlock, so comment it off for now
    if( storage->parent && (!pos->top || pos->top->next) )
    {
        CvMemBlock* save_bottom;
        if( !pos->top )
            save_bottom = 0;
        else
        {
            save_bottom = storage->bottom;
            storage->bottom = pos->top->next;
            pos->top->next = 0;
            storage->bottom->prev = 0;
        }
        icvDestroyMemStorage( storage );
        storage->bottom = save_bottom;
    }*/

    storage->top = pos->top;
    storage->free_space = pos->free_space;

    if( !storage->top )
    {
        storage->top = storage->bottom;
        storage->free_space = storage->top ? storage->block_size - sizeof(CvMemBlock) : 0;
    }
}


/* Allocate continuous buffer of the specified size in the storage: */
void* cvMemStorageAlloc( CvMemStorage* storage, size_t size )
{
    schar *ptr = 0;
    if( !storage )
    {
		printf("NULL storage pointer\n" );
		return NULL;
	}

    if( size > INT_MAX )
	{
        printf("Too large memory block is requested\n" );
		return NULL;
	}

    assert( storage->free_space % CV_STRUCT_ALIGN == 0 );

    if( (size_t)storage->free_space < size )
    {
        size_t max_free_space = cvAlignLeft(storage->block_size - sizeof(CvMemBlock), CV_STRUCT_ALIGN);
        if( max_free_space < size )
		{
            printf("requested size is negative or too big\n" );
			return NULL;
		}

        icvGoNextMemBlock( storage );
    }

    ptr = ICV_FREE_PTR(storage);
    assert( (size_t)ptr % CV_STRUCT_ALIGN == 0 );
    storage->free_space = cvAlignLeft(storage->free_space - (int)size, CV_STRUCT_ALIGN );

    return ptr;
}

static void icvMemCopy( double **buf1, double **buf2, double **buf3, int *b_max )
{
    assert( (*buf1 != NULL || *buf2 != NULL) && *buf3 != NULL );

    int bb = *b_max;
    if( *buf2 == NULL )
    {
        *b_max = 2 * (*b_max);
        *buf2 = (double *)cvAlloc( (*b_max) * sizeof( double ));

        memcpy( *buf2, *buf3, bb * sizeof( double ));

        *buf3 = *buf2;
        cvFree( buf1 );
        *buf1 = NULL;
    }
    else
    {
        *b_max = 2 * (*b_max);
        *buf1 = (double *) cvAlloc( (*b_max) * sizeof( double ));

        memcpy( *buf1, *buf3, bb * sizeof( double ));

        *buf3 = *buf1;
        cvFree( buf2 );
        *buf2 = NULL;
    }
}

/****************************************************************************************\
*                                 Working with sequence tree                             *
\****************************************************************************************/

// Insert contour into tree given certain parent sequence.
// If parent is equal to frame (the most external contour),
// then added contour will have null pointer to parent:
void cvInsertNodeIntoTree( void* _node, void* _parent, void* _frame )
{
    CvTreeNode* node = (CvTreeNode*)_node;
    CvTreeNode* parent = (CvTreeNode*)_parent;

    if( !node || !parent )
        return;

    node->v_prev = _parent != _frame ? parent : 0;
    node->h_next = parent->v_next;

    assert( parent->v_next != node );

    if( parent->v_next )
        parent->v_next->h_prev = node;
    parent->v_next = node;
}

static inline void* cvAlignPtr( const void* ptr, int align )
{
    assert ( (align & (align-1)) == 0 );
    return (void*)( ((size_t)ptr + align - 1) & ~(size_t)(align-1) );
}

inline int cvAlignLeft( int size, int align )
{
    return size & -align;
}

static inline int cvAlign( int size, int align )
{
    assert( (align & (align-1)) == 0 && size < INT_MAX );
    return (size + align - 1) & -align;
}

template<typename _Tp> static inline _Tp* alignPtr(_Tp* ptr, int n=(int)sizeof(_Tp))
{
    return (_Tp*)(((size_t)ptr + n-1) & -n);
}

/****************************************************************************************\
*          Array allocation, deallocation, initialization and access to elements         *
\****************************************************************************************/

void* fastMalloc( size_t size )
{
    uchar* udata = (uchar*)malloc(size + sizeof(void*) + CV_MALLOC_ALIGN);
    if(!udata)
        return NULL;
    uchar** adata = alignPtr((uchar**)udata + 1, CV_MALLOC_ALIGN);
    adata[-1] = udata;
    return adata;
}

void fastFree(void* ptr)
{
    if(ptr)
    {
        uchar* udata = ((uchar**)ptr)[-1];
        assert(udata < (uchar*)ptr &&
               ((uchar*)ptr - udata) <= (ptrdiff_t)(sizeof(void*)+CV_MALLOC_ALIGN));
        free(udata);
    }
}

void* cvAlloc( size_t size )
{
    return fastMalloc( size );
}

void cvFree_( void* ptr )
{
    fastFree( ptr );
}

#endif

#endif