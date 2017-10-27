#ifndef _OPENCV_ADAPTER_HEADER
#define _OPENCV_ADAPTER_HEADER

#include <math.h>
#include <vector>
//#include "dyn_memory.h"
#include <malloc.h>
#include <assert.h>
#include<iostream>
#include<stddef.h>
using namespace std;
#include "../common/reuse.h"

#if OPENCV
	#include <opencv/cv.h>
	#include <opencv/cxcore.h>
	#include <opencv/highgui.h>
	#include "opencv2\opencv.hpp"
	using namespace cv;
#else
    #define uchar unsigned char
    #define CV_DEFAULT(val) = val
    #define CV_CLOCKWISE         1
    #define CV_COUNTER_CLOCKWISE 2
    #define CV_RGB( r, g, b )  cvScalar( (b), (g), (r), 0 )
    #define CV_WHOLE_SEQ_END_INDEX 0x3fffffff
    #define CV_WHOLE_SEQ  cvSlice(0, CV_WHOLE_SEQ_END_INDEX)
    #define CV_POLY_APPROX_DP 0
    #define CV_MAGIC_MASK 0xFFFF0000 
    #define CV_STRUCT_ALIGN sizeof(double)
    #define CV_MAX_ALLOC_SIZE (((size_t)1 << (sizeof(size_t)*8-2)))
    #define  CV_MALLOC_ALIGN 32
    #define ICV_SHIFT_TAB_MAX 32
    #define CV_STORAGE_BLOCK_SIZE ((1<<16) - 128)
    #define cvFree(ptr) (cvFree_(*(ptr)), *(ptr)=0)
    #define CV_TOGGLE_FLT(x) ((x)^((int)(x) < 0 ? 0x7fffffff : 0))

    /****************************************************************************************\
    *                                  Image type (IplImage)                                 *
    \****************************************************************************************/
    #define IPL_DEPTH_SIGN 0x80000000

    #define IPL_DEPTH_1U     1
    #define IPL_DEPTH_8U     8
    #define IPL_DEPTH_16U   16
    #define IPL_DEPTH_32F   32

    #define IPL_DEPTH_8S  (IPL_DEPTH_SIGN| 8)
    #define IPL_DEPTH_16S (IPL_DEPTH_SIGN|16)
    #define IPL_DEPTH_32S (IPL_DEPTH_SIGN|32)

	#define CV_IMAGE_ELEM( image, elemtype, row, col )       \
		(((elemtype*)((image)->imageData + (image)->widthStep*(row)))[(col)])

	/* Types of filtering */
	#define CV_BLUR_NO_SCALE 0
    #define CV_BLUR  1
    #define CV_GAUSSIAN  2
    #define CV_MEDIAN 3
    #define CV_BILATERAL 4

	/* Types of thresholding */
    #define CV_THRESH_BINARY      0  /* value = value > threshold ? max_value : 0       */
    #define CV_THRESH_BINARY_INV  1  /* value = value > threshold ? 0 : max_value       */
    #define CV_THRESH_TRUNC       2  /* value = value > threshold ? threshold : value   */
    #define CV_THRESH_TOZERO      3  /* value = value > threshold ? value : 0           */
    #define CV_THRESH_TOZERO_INV  4  /* value = value > threshold ? 0 : value           */
    #define CV_THRESH_MASK        7

    #define CV_THRESH_OTSU        8  /* use Otsu algorithm to choose the optimal threshold value;
                                    combine the flag with one of the above CV_THRESH_* values */

    #define CV_STORAGE_MAGIC_VAL    0x42890000

    #define CV_IS_STORAGE(storage)  \
    ((storage) != NULL &&       \
    (((CvMemStorage*)(storage))->signature & CV_MAGIC_MASK) == CV_STORAGE_MAGIC_VAL)

    #define CV_TYPE_NAME_SEQ             "opencv-sequence"
    #define CV_TYPE_NAME_SEQ_TREE        "opencv-sequence-tree"

    #define CV_SET_ELEM_IDX_MASK   ((1 << 26) - 1)
    #define CV_SET_ELEM_FREE_FLAG  (1 << (sizeof(int)*8-1))

    /** Checks whether the element pointed by ptr belongs to a set or not */
    #define CV_IS_SET_ELEM( ptr )  (((CvSetElem*)(ptr))->flags >= 0)

    /****************************************************************************************\
    *                                    Sequence types                                      *
    \****************************************************************************************/

    #define CV_SEQ_MAGIC_VAL             0x42990000

    #define CV_IS_SEQ(seq) \
        ((seq) != NULL && (((CvSeq*)(seq))->flags & CV_MAGIC_MASK) == CV_SEQ_MAGIC_VAL)

    #define CV_SET_MAGIC_VAL             0x42980000
    #define CV_IS_SET(set) \
    ((set) != NULL && (((CvSeq*)(set))->flags & CV_MAGIC_MASK) == CV_SET_MAGIC_VAL)

    #define CV_SEQ_ELTYPE_BITS           12
    #define CV_SEQ_ELTYPE_MASK           ((1 << CV_SEQ_ELTYPE_BITS) - 1)

    #define CV_SEQ_ELTYPE_POINT          CV_32SC2  /**< (x,y) */
    #define CV_SEQ_ELTYPE_CODE           CV_8UC1   /**< freeman code: 0..7 */
    #define CV_SEQ_ELTYPE_GENERIC        0
    #define CV_SEQ_ELTYPE_PTR            CV_USRTYPE1
    #define CV_SEQ_ELTYPE_PPOINT         CV_SEQ_ELTYPE_PTR  /**< &(x,y) */
    #define CV_SEQ_ELTYPE_INDEX          CV_32SC1  /**< #(x,y) */
    #define CV_SEQ_ELTYPE_GRAPH_EDGE     0  /**< &next_o, &next_d, &vtx_o, &vtx_d */
    #define CV_SEQ_ELTYPE_GRAPH_VERTEX   0  /**< first_edge, &(x,y) */
    #define CV_SEQ_ELTYPE_TRIAN_ATR      0  /**< vertex of the binary tree   */
    #define CV_SEQ_ELTYPE_CONNECTED_COMP 0  /**< connected component  */
    #define CV_SEQ_ELTYPE_POINT3D        CV_32FC3  /**< (x,y,z)  */

    #define CV_SEQ_KIND_BITS        2
    #define CV_SEQ_KIND_MASK        (((1 << CV_SEQ_KIND_BITS) - 1)<<CV_SEQ_ELTYPE_BITS)

    /** types of sequences */
    #define CV_SEQ_KIND_GENERIC     (0 << CV_SEQ_ELTYPE_BITS)
    #define CV_SEQ_KIND_CURVE       (1 << CV_SEQ_ELTYPE_BITS)
    #define CV_SEQ_KIND_BIN_TREE    (2 << CV_SEQ_ELTYPE_BITS)

    /** types of sparse sequences (sets) */
    #define CV_SEQ_KIND_GRAPH       (1 << CV_SEQ_ELTYPE_BITS)
    #define CV_SEQ_KIND_SUBDIV2D    (2 << CV_SEQ_ELTYPE_BITS)

    #define CV_SEQ_FLAG_SHIFT       (CV_SEQ_KIND_BITS + CV_SEQ_ELTYPE_BITS)

    /** flags for curves */
    #define CV_SEQ_FLAG_CLOSED     (1 << CV_SEQ_FLAG_SHIFT)
    #define CV_SEQ_FLAG_SIMPLE     (0 << CV_SEQ_FLAG_SHIFT)
    #define CV_SEQ_FLAG_CONVEX     (0 << CV_SEQ_FLAG_SHIFT)
    #define CV_SEQ_FLAG_HOLE       (2 << CV_SEQ_FLAG_SHIFT)

    /** flags for graphs */
    #define CV_GRAPH_FLAG_ORIENTED (1 << CV_SEQ_FLAG_SHIFT)

    #define CV_GRAPH               CV_SEQ_KIND_GRAPH
    #define CV_ORIENTED_GRAPH      (CV_SEQ_KIND_GRAPH|CV_GRAPH_FLAG_ORIENTED)

    /** point sets */
    #define CV_SEQ_POINT_SET       (CV_SEQ_KIND_GENERIC| CV_SEQ_ELTYPE_POINT)
    #define CV_SEQ_POINT3D_SET     (CV_SEQ_KIND_GENERIC| CV_SEQ_ELTYPE_POINT3D)
    #define CV_SEQ_POLYLINE        (CV_SEQ_KIND_CURVE  | CV_SEQ_ELTYPE_POINT)
    #define CV_SEQ_POLYGON         (CV_SEQ_FLAG_CLOSED | CV_SEQ_POLYLINE )
    #define CV_SEQ_CONTOUR         CV_SEQ_POLYGON
    #define CV_SEQ_SIMPLE_POLYGON  (CV_SEQ_FLAG_SIMPLE | CV_SEQ_POLYGON  )

    /** chain-coded curves */
    #define CV_SEQ_CHAIN           (CV_SEQ_KIND_CURVE  | CV_SEQ_ELTYPE_CODE)
    #define CV_SEQ_CHAIN_CONTOUR   (CV_SEQ_FLAG_CLOSED | CV_SEQ_CHAIN)

    /** binary tree for the contour */
    #define CV_SEQ_POLYGON_TREE    (CV_SEQ_KIND_BIN_TREE  | CV_SEQ_ELTYPE_TRIAN_ATR)

    /** sequence of the connected components */
    #define CV_SEQ_CONNECTED_COMP  (CV_SEQ_KIND_GENERIC  | CV_SEQ_ELTYPE_CONNECTED_COMP)

    /** sequence of the integer numbers */
    #define CV_SEQ_INDEX           (CV_SEQ_KIND_GENERIC  | CV_SEQ_ELTYPE_INDEX)

    #define CV_SEQ_ELTYPE( seq )   ((seq)->flags & CV_SEQ_ELTYPE_MASK)
    #define CV_SEQ_KIND( seq )     ((seq)->flags & CV_SEQ_KIND_MASK )

    /** flag checking */
    #define CV_IS_SEQ_INDEX( seq )      ((CV_SEQ_ELTYPE(seq) == CV_SEQ_ELTYPE_INDEX) && \
                                     (CV_SEQ_KIND(seq) == CV_SEQ_KIND_GENERIC))

    #define CV_IS_SEQ_CURVE( seq )      (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE)
    #define CV_IS_SEQ_CLOSED( seq )     (((seq)->flags & CV_SEQ_FLAG_CLOSED) != 0)
    #define CV_IS_SEQ_CONVEX( seq )     0
    #define CV_IS_SEQ_HOLE( seq )       (((seq)->flags & CV_SEQ_FLAG_HOLE) != 0)
    #define CV_IS_SEQ_SIMPLE( seq )     1

    /** type checking macros */
    #define CV_IS_SEQ_POINT_SET( seq ) \
        ((CV_SEQ_ELTYPE(seq) == CV_32SC2 || CV_SEQ_ELTYPE(seq) == CV_32FC2))

    #define CV_IS_SEQ_POINT_SUBSET( seq ) \
        (CV_IS_SEQ_INDEX( seq ) || CV_SEQ_ELTYPE(seq) == CV_SEQ_ELTYPE_PPOINT)

    #define CV_IS_SEQ_POLYLINE( seq )   \
        (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE && CV_IS_SEQ_POINT_SET(seq))

    #define CV_IS_SEQ_POLYGON( seq )   \
        (CV_IS_SEQ_POLYLINE(seq) && CV_IS_SEQ_CLOSED(seq))

    #define CV_IS_SEQ_CHAIN( seq )   \
        (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE && (seq)->elem_size == 1)

    #define CV_IS_SEQ_CONTOUR( seq )   \
        (CV_IS_SEQ_CLOSED(seq) && (CV_IS_SEQ_POLYLINE(seq) || CV_IS_SEQ_CHAIN(seq)))

    #define CV_IS_SEQ_CHAIN_CONTOUR( seq ) \
        (CV_IS_SEQ_CHAIN( seq ) && CV_IS_SEQ_CLOSED( seq ))

    #define CV_IS_SEQ_POLYGON_TREE( seq ) \
        (CV_SEQ_ELTYPE (seq) ==  CV_SEQ_ELTYPE_TRIAN_ATR &&    \
        CV_SEQ_KIND( seq ) ==  CV_SEQ_KIND_BIN_TREE )

    #define CV_IS_GRAPH( seq )    \
        (CV_IS_SET(seq) && CV_SEQ_KIND((CvSet*)(seq)) == CV_SEQ_KIND_GRAPH)

    #define CV_IS_GRAPH_ORIENTED( seq )   \
        (((seq)->flags & CV_GRAPH_FLAG_ORIENTED) != 0)

    #define CV_IS_SUBDIV2D( seq )  \
        (CV_IS_SET(seq) && CV_SEQ_KIND((CvSet*)(seq)) == CV_SEQ_KIND_SUBDIV2D)


    /****************************************************************************************\
    *                                  Matrix type (Mat)                                     *
    \****************************************************************************************/

    #define CV_CN_MAX     512
    #define CV_CN_SHIFT   3
    #define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)

    #define CV_8U   0
    #define CV_8S   1
    #define CV_16U  2
    #define CV_16S  3
    #define CV_32S  4
    #define CV_32F  5
    #define CV_64F  6
    #define CV_USRTYPE1 7

    #define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
    #define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)

    #define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
    #define CV_MAKE_TYPE CV_MAKETYPE

    #define CV_8UC1 CV_MAKETYPE(CV_8U,1)
    #define CV_8UC2 CV_MAKETYPE(CV_8U,2)
    #define CV_8UC3 CV_MAKETYPE(CV_8U,3)
    #define CV_8UC4 CV_MAKETYPE(CV_8U,4)
    #define CV_8UC(n) CV_MAKETYPE(CV_8U,(n))

    #define CV_8SC1 CV_MAKETYPE(CV_8S,1)
    #define CV_8SC2 CV_MAKETYPE(CV_8S,2)
    #define CV_8SC3 CV_MAKETYPE(CV_8S,3)
    #define CV_8SC4 CV_MAKETYPE(CV_8S,4)
    #define CV_8SC(n) CV_MAKETYPE(CV_8S,(n))

    #define CV_16UC1 CV_MAKETYPE(CV_16U,1)
    #define CV_16UC2 CV_MAKETYPE(CV_16U,2)
    #define CV_16UC3 CV_MAKETYPE(CV_16U,3)
    #define CV_16UC4 CV_MAKETYPE(CV_16U,4)
    #define CV_16UC(n) CV_MAKETYPE(CV_16U,(n))

    #define CV_16SC1 CV_MAKETYPE(CV_16S,1)
    #define CV_16SC2 CV_MAKETYPE(CV_16S,2)
    #define CV_16SC3 CV_MAKETYPE(CV_16S,3)
    #define CV_16SC4 CV_MAKETYPE(CV_16S,4)
    #define CV_16SC(n) CV_MAKETYPE(CV_16S,(n))

    #define CV_32SC1 CV_MAKETYPE(CV_32S,1)
    #define CV_32SC2 CV_MAKETYPE(CV_32S,2)
    #define CV_32SC3 CV_MAKETYPE(CV_32S,3)
    #define CV_32SC4 CV_MAKETYPE(CV_32S,4)
    #define CV_32SC(n) CV_MAKETYPE(CV_32S,(n))

    #define CV_32FC1 CV_MAKETYPE(CV_32F,1)
    #define CV_32FC2 CV_MAKETYPE(CV_32F,2)
    #define CV_32FC3 CV_MAKETYPE(CV_32F,3)
    #define CV_32FC4 CV_MAKETYPE(CV_32F,4)
    #define CV_32FC(n) CV_MAKETYPE(CV_32F,(n))

    #define CV_64FC1 CV_MAKETYPE(CV_64F,1)
    #define CV_64FC2 CV_MAKETYPE(CV_64F,2)
    #define CV_64FC3 CV_MAKETYPE(CV_64F,3)
    #define CV_64FC4 CV_MAKETYPE(CV_64F,4)
    #define CV_64FC(n) CV_MAKETYPE(CV_64F,(n))

    #define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
    #define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
    #define CV_MAT_TYPE_MASK        (CV_DEPTH_MAX*CV_CN_MAX - 1)
    #define CV_MAT_TYPE(flags)      ((flags) & CV_MAT_TYPE_MASK)
    #define CV_MAT_CONT_FLAG_SHIFT  14
    #define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
    #define CV_IS_MAT_CONT(flags)   ((flags) & CV_MAT_CONT_FLAG)
    #define CV_IS_CONT_MAT          CV_IS_MAT_CONT
    #define CV_SUBMAT_FLAG_SHIFT    15
    #define CV_SUBMAT_FLAG          (1 << CV_SUBMAT_FLAG_SHIFT)
    #define CV_IS_SUBMAT(flags)     ((flags) & CV_MAT_SUBMAT_FLAG)

    /* Size of each channel item,
       0x124489 = 1000 0100 0100 0010 0010 0001 0001 ~ array of sizeof(arr_type_elem) */
    #define CV_ELEM_SIZE1(type) \
        ((((sizeof(size_t)<<28)|0x8442211) >> CV_MAT_DEPTH(type)*4) & 15)

    /* 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
    #define CV_ELEM_SIZE(type) \
        (CV_MAT_CN(type) << ((((sizeof(size_t)/4+1)*16384|0x3a50) >> CV_MAT_DEPTH(type)*2) & 3))

    #ifndef MIN
    #  define MIN(a,b)  ((a) > (b) ? (b) : (a))
    #endif

    #ifndef MAX
    #  define MAX(a,b)  ((a) < (b) ? (b) : (a))
    #endif

    #define  CV_CMP(a,b)    (((a) > (b)) - ((a) < (b)))
    #define  CV_SIGN(a)     CV_CMP((a),0)
    #define CV_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))

    #define CV_IMPLEMENT_QSORT_EX( func_name, T, LT, user_data_type )                   \
    void func_name( T *array, size_t total, user_data_type aux )                        \
    {                                                                                   \
        int isort_thresh = 7;                                                           \
        T t;                                                                            \
        int sp = 0;                                                                     \
                                                                                    \
        struct                                                                          \
        {                                                                               \
            T *lb;                                                                      \
            T *ub;                                                                      \
        }                                                                               \
        stack[48];                                                                      \
                                                                                    \
        aux = aux;                                                                      \
                                                                                    \
        if( total <= 1 )                                                                \
            return;                                                                     \
                                                                                    \
        stack[0].lb = array;                                                            \
        stack[0].ub = array + (total - 1);                                              \
                                                                                    \
        while( sp >= 0 )                                                                \
        {                                                                               \
            T* left = stack[sp].lb;                                                     \
            T* right = stack[sp--].ub;                                                  \
                                                                                    \
            for(;;)                                                                     \
            {                                                                           \
                int i, n = (int)(right - left) + 1, m;                                  \
                T* ptr;                                                                 \
                T* ptr2;                                                                \
                                                                                    \
                if( n <= isort_thresh )                                                 \
                {                                                                       \
                insert_sort:                                                            \
                    for( ptr = left + 1; ptr <= right; ptr++ )                          \
                    {                                                                   \
                        for( ptr2 = ptr; ptr2 > left && LT(ptr2[0],ptr2[-1]); ptr2--)   \
                            CV_SWAP( ptr2[0], ptr2[-1], t );                            \
                    }                                                                   \
                    break;                                                              \
                }                                                                       \
                else                                                                    \
                {                                                                       \
                    T* left0;                                                           \
                    T* left1;                                                           \
                    T* right0;                                                          \
                    T* right1;                                                          \
                    T* pivot;                                                           \
                    T* a;                                                               \
                    T* b;                                                               \
                    T* c;                                                               \
                    int swap_cnt = 0;                                                   \
                                                                                    \
                    left0 = left;                                                       \
                    right0 = right;                                                     \
                    pivot = left + (n/2);                                               \
                                                                                    \
                    if( n > 40 )                                                        \
                    {                                                                   \
                        int d = n / 8;                                                  \
                        a = left, b = left + d, c = left + 2*d;                         \
                        left = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))     \
                                          : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                        a = pivot - d, b = pivot, c = pivot + d;                        \
                        pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                          : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                        a = right - 2*d, b = right - d, c = right;                      \
                        right = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                          : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                    }                                                                   \
                                                                                    \
                    a = left, b = pivot, c = right;                                     \
                    pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))        \
                                       : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));       \
                    if( pivot != left0 )                                                \
                    {                                                                   \
                        CV_SWAP( *pivot, *left0, t );                                   \
                        pivot = left0;                                                  \
                    }                                                                   \
                    left = left1 = left0 + 1;                                           \
                    right = right1 = right0;                                            \
                                                                                    \
                    for(;;)                                                             \
                    {                                                                   \
                        while( left <= right && !LT(*pivot, *left) )                    \
                        {                                                               \
                            if( !LT(*left, *pivot) )                                    \
                            {                                                           \
                                if( left > left1 )                                      \
                                    CV_SWAP( *left1, *left, t );                        \
                                swap_cnt = 1;                                           \
                                left1++;                                                \
                            }                                                           \
                            left++;                                                     \
                        }                                                               \
                                                                                    \
                        while( left <= right && !LT(*right, *pivot) )                   \
                        {                                                               \
                            if( !LT(*pivot, *right) )                                   \
                            {                                                           \
                                if( right < right1 )                                    \
                                    CV_SWAP( *right1, *right, t );                      \
                                swap_cnt = 1;                                           \
                                right1--;                                               \
                            }                                                           \
                            right--;                                                    \
                        }                                                               \
                                                                                    \
                        if( left > right )                                              \
                            break;                                                      \
                        CV_SWAP( *left, *right, t );                                    \
                        swap_cnt = 1;                                                   \
                        left++;                                                         \
                        right--;                                                        \
                    }                                                                   \
                                                                                    \
                    if( swap_cnt == 0 )                                                 \
                    {                                                                   \
                        left = left0, right = right0;                                   \
                        goto insert_sort;                                               \
                    }                                                                   \
                                                                                    \
                    n = MIN( (int)(left1 - left0), (int)(left - left1) );               \
                    for( i = 0; i < n; i++ )                                            \
                        CV_SWAP( left0[i], left[i-n], t );                              \
                                                                                    \
                    n = MIN( (int)(right0 - right1), (int)(right1 - right) );           \
                    for( i = 0; i < n; i++ )                                            \
                        CV_SWAP( left[i], right0[i-n+1], t );                           \
                    n = (int)(left - left1);                                            \
                    m = (int)(right1 - right);                                          \
                    if( n > 1 )                                                         \
                    {                                                                   \
                        if( m > 1 )                                                     \
                        {                                                               \
                            if( n > m )                                                 \
                            {                                                           \
                                stack[++sp].lb = left0;                                 \
                                stack[sp].ub = left0 + n - 1;                           \
                                left = right0 - m + 1, right = right0;                  \
                            }                                                           \
                            else                                                        \
                            {                                                           \
                                stack[++sp].lb = right0 - m + 1;                        \
                                stack[sp].ub = right0;                                  \
                                left = left0, right = left0 + n - 1;                    \
                            }                                                           \
                        }                                                               \
                        else                                                            \
                            left = left0, right = left0 + n - 1;                        \
                    }                                                                   \
                    else if( m > 1 )                                                    \
                        left = right0 - m + 1, right = right0;                          \
                    else                                                                \
                        break;                                                          \
                }                                                                       \
            }                                                                           \
        }                                                                               \
    }

    #define CV_IMPLEMENT_QSORT( func_name, T, cmp )  \
        CV_IMPLEMENT_QSORT_EX( func_name, T, cmp, int )

    /* IPP-compatible return codes */
    typedef enum CvStatus
    {         
        CV_BADMEMBLOCK_ERR          = -113,
        CV_INPLACE_NOT_SUPPORTED_ERR= -112,
        CV_UNMATCHED_ROI_ERR        = -111,
        CV_NOTFOUND_ERR             = -110,
        CV_BADCONVERGENCE_ERR       = -109,

        CV_BADDEPTH_ERR             = -107,
        CV_BADROI_ERR               = -106,
        CV_BADHEADER_ERR            = -105,
        CV_UNMATCHED_FORMATS_ERR    = -104,
        CV_UNSUPPORTED_COI_ERR      = -103,
        CV_UNSUPPORTED_CHANNELS_ERR = -102,
        CV_UNSUPPORTED_DEPTH_ERR    = -101,
        CV_UNSUPPORTED_FORMAT_ERR   = -100,

        CV_BADARG_ERR      = -49,  //ipp comp
        CV_NOTDEFINED_ERR  = -48,  //ipp comp

        CV_BADCHANNELS_ERR = -47,  //ipp comp
        CV_BADRANGE_ERR    = -44,  //ipp comp
        CV_BADSTEP_ERR     = -29,  //ipp comp

        CV_BADFLAG_ERR     =  -12,
        CV_DIV_BY_ZERO_ERR =  -11, //ipp comp
        CV_BADCOEF_ERR     =  -10,

        CV_BADFACTOR_ERR   =  -7,
        CV_BADPOINT_ERR    =  -6,
        CV_BADSCALE_ERR    =  -4,
        CV_OUTOFMEM_ERR    =  -3,
        CV_NULLPTR_ERR     =  -2,
        CV_BADSIZE_ERR     =  -1,
        CV_NO_ERR          =   0,
        CV_OK              =   CV_NO_ERR
    }
    CvStatus;

	/************************************* CvSlice ******************************************/

    typedef struct CvSlice
    {
        int  start_index, end_index;
    }
    CvSlice;

	class Scalar
	{
	public:
		Scalar(float a,float b,float c){};
		Scalar(){};
		~Scalar(){};

	private:
		float32_t val[4];
	};
	typedef struct Rect2f_
	{
		Rect2f_(float x_in=0.0f,float y_in=0.0f,float width_in=0.0f,float height_in=0.0f)
		{
			x = x_in;
			y = y_in;
			width = width_in;
			height = height_in;
		}
		float x;
		float y;
		float width;
		float height;
	}Rect2f;

	typedef struct Size_
	{
		Size_(int width_in=0,int height_in=0)
		{
			width = width_in;
			height = height_in;
		}
		int width;
		int height;
	}Size;

	typedef struct Point2f_
	{
		Point2f_(float32_t x_in=0.0f,float32_t y_in=0.0f)
		{
			x = x_in;
			y = y_in;
		}
		float32_t x;
		float32_t y;
	}Point2f;

	typedef struct CvPoint2D32f
	{
		float x;
		float y;

	}
	CvPoint2D32f;

	typedef struct CvPoint
	{
		int x;
		int y;
	}
	CvPoint;

	typedef union Cv32suf
    {
        int i;
        unsigned u;
        float f;
    }
    Cv32suf;

	typedef struct _IplImage
	{
		int  nSize;             /**< sizeof(IplImage) */
		int  ID;                /**< version (=0)*/
		int  nChannels;         /**< Most of OpenCV functions support 1,2,3 or 4 channels */
		unsigned int  timeStamp;//int  alphaChannel;      /**< Ignored by OpenCV */
		int  depth;             /**< Pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
								   IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported.  */
		char colorModel[4];     /**< Ignored by OpenCV */
		char channelSeq[4];     /**< ditto */
		int  dataOrder;         /**< 0 - interleaved color channels, 1 - separate color channels.
								   cvCreateImage can only create interleaved images */
		int  origin;            /**< 0 - top-left origin,
								   1 - bottom-left origin (Windows bitmaps style).  */
		int  align;             /**< Alignment of image rows (4 or 8).
								   OpenCV ignores it and uses widthStep instead.    */
		int  width;             /**< Image width in pixels.                           */
		int  height;            /**< Image height in pixels.                          */
		struct _IplROI *roi;    /**< Image ROI. If NULL, the whole image is selected. */
		struct _IplImage *maskROI;      /**< Must be NULL. */
		void  *imageId;                 /**< "           " */
		struct _IplTileInfo *tileInfo;  /**< "           " */
		int  imageSize;         /**< Image data size in bytes
								   (==image->height*image->widthStep
								   in case of interleaved data)*/
		char *imageData;        /**< Pointer to aligned image data.         */
		int  widthStep;         /**< Size of aligned image row in bytes.    */
		int  BorderMode[4];     /**< Ignored by OpenCV.                     */
		int  BorderConst[4];    /**< Ditto.                                 */
		char *imageDataOrigin;  /**< Pointer to very origin of image data
								   (not necessarily aligned) -
								   needed for correct deallocation */
	}
	IplImage;


	typedef struct CvScalar
	{
		float32_t val[4];
	}CvScalar;

	typedef struct CvRect
	{
		int x;
		int y;
		int width;
		int height;
		void cvRect(int init_x, int init_y, int rect_width, int rect_height)
		{
			x = init_x;
			y = init_y;
			width = rect_width;
			height = rect_height;
		}
	}
	CvRect;

	typedef struct CvSize
	{
		int width;
		int height;
		CvSize(int w,int h)
		{
			width = w;
			height = h;

		}
		CvSize(){}
	}CvSize;

	//CvSlice  cvSlice( int start, int end );
	//CvSize  cvSize( int width, int height );
	//CvPoint  cvPoint( int x, int y );
	//CvScalar  cvScalar( float32_t val0, float32_t val1 ,
	//				   float32_t val2 , float32_t val3 );
	//CvRect  cvRect( int x, int y, int width, int height );

	// Let 0 - 9 to represents 3*3 matrix, the meaning of each kernel is illustrated as
	enum
	{
		CV_SHAPE_RECT			  = 0, // 0, 1, 2, 3, 4, 5, 6, 7, 8
		CV_SHAPE_CROSS			  = 1, // 1, 3, 4, 5, 7
		CV_SHAPE_BAR_VERT_ALL	  = 2, // 1, 4, 7
		CV_SHAPE_BAR_VERT_UP	  = 3, // 1, 4
		CV_SHAPE_BAR_VERT_DOWN	  = 4, // 4, 7
		CV_SHAPE_BAR_HORI_ALL  	  = 5, // 3, 4, 5
		CV_SHAPE_BAR_HORI_LEFT	  = 6, // 3, 4
		CV_SHAPE_BAR_HORI_RIGHT	  = 7, // 4, 5
		CV_SHAPE_BAR_INCL_048     = 8, // 0, 4, 8
		CV_SHAPE_BAR_INCL_04      = 9, // 0, 4
		CV_SHAPE_BAR_INCL_48      = 10,// 4, 8
		CV_SHAPE_BAR_INCL_246     = 11,// 2, 4, 6
		CV_SHAPE_BAR_INCL_24      = 12,// 2, 4
		CV_SHAPE_BAR_INCL_46      = 13,// 4, 6
	};

    /****************************************************************************************\
    *                                   Dynamic Data structures                              *
    \****************************************************************************************/

    /******************************** Memory storage ****************************************/
    typedef signed char schar;
    typedef struct CvMemBlock
    {
        struct CvMemBlock*  prev;
        struct CvMemBlock*  next;
    }
    CvMemBlock;

    typedef struct CvMemStorage
    {
        int signature;
        CvMemBlock* bottom;           /**< First allocated block.                   */
        CvMemBlock* top;              /**< Current memory block - top of the stack. */
        struct  CvMemStorage* parent; /**< We get new blocks from parent as needed. */
        int block_size;               /**< Block size.                              */
        int free_space;               /**< Remaining free space in current block.   */
    }
    CvMemStorage;


    typedef struct CvMemStoragePos
    {
        CvMemBlock* top;
        int free_space;
    }
    CvMemStoragePos;


    typedef struct CvTreeNode
    {
        int       flags;         /* micsellaneous flags */         
        int       header_size;   /* size of sequence header */     
        struct    CvTreeNode* h_prev; /* previous sequence */      
        struct    CvTreeNode* h_next; /* next sequence */          
        struct    CvTreeNode* v_prev; /* 2nd previous sequence */  
        struct    CvTreeNode* v_next; /* 2nd next sequence */
    }
    CvTreeNode;


    /*********************************** Sequence *******************************************/

    typedef struct CvSeqBlock
    {
        struct  CvSeqBlock*  prev; /* previous sequence block */
        struct  CvSeqBlock*  next; /* next sequence block */
        int     start_index;       /* index of the first element in the block +
                                      sequence->first->start_index */
        int     count;             /* number of elements in the block */
        schar*  data;              /* pointer to the first element of the block */
    }
    CvSeqBlock;


    #define CV_TREE_NODE_FIELDS(node_type)                          \
        int       flags;         /* micsellaneous flags */          \
        int       header_size;   /* size of sequence header */      \
        struct    node_type* h_prev; /* previous sequence */        \
        struct    node_type* h_next; /* next sequence */            \
        struct    node_type* v_prev; /* 2nd previous sequence */    \
        struct    node_type* v_next  /* 2nd next sequence */

    /*
       Read/Write sequence.
       Elements can be dynamically inserted to or deleted from the sequence.
    */
    #define CV_SEQUENCE_FIELDS()                                            \
        CV_TREE_NODE_FIELDS(CvSeq);                                         \
        int        total;          /* total number of elements */            \
        int        elem_size;      /* size of sequence element in bytes */   \
        schar*     block_max;      /* maximal bound of the last block */     \
        schar*     ptr;            /* current write pointer */               \
        int        delta_elems;    /* how many elements allocated when the seq grows */  \
        CvMemStorage*  storage;    /* where the seq is stored */             \
        CvSeqBlock*  free_blocks;  /* free blocks list */                    \
        CvSeqBlock*  first; /* pointer to the first sequence block */

    typedef struct CvSeq
    {
        CV_SEQUENCE_FIELDS()
    }
    CvSeq;

    /*************************************** Set ********************************************/
    /** @brief Set
      Order is not preserved. There can be gaps between sequence elements.
      After the element has been inserted it stays in the same place all the time.
      The MSB(most-significant or sign bit) of the first field (flags) is 0 iff the element exists.
    */
    #define CV_SET_ELEM_FIELDS(elem_type)   \
        int  flags;                         \
        struct elem_type* next_free;

    typedef struct CvSetElem
    {
        CV_SET_ELEM_FIELDS(CvSetElem)
    }
    CvSetElem;

    #define CV_SET_FIELDS()      \
        CV_SEQUENCE_FIELDS()     \
        CvSetElem* free_elems;   \
        int active_count;

    typedef struct CvSet
    {
        CV_SET_FIELDS()
    }
    CvSet;

    /****************************************************************************************/
    /*                            Sequence writer & reader                                  */
    /****************************************************************************************/

    #define CV_SEQ_WRITER_FIELDS()                                     \
        int          header_size;                                      \
        CvSeq*       seq;        /* the sequence written */            \
        CvSeqBlock*  block;      /* current block */                   \
        schar*       ptr;        /* pointer to free space */           \
        schar*       block_min;  /* pointer to the beginning of block*/\
        schar*       block_max;  /* pointer to the end of block */

    typedef struct CvSeqWriter
    {
        CV_SEQ_WRITER_FIELDS()
    }
    CvSeqWriter;


    #define CV_SEQ_READER_FIELDS()                                      \
        int          header_size;                                       \
        CvSeq*       seq;        /* sequence, beign read */             \
        CvSeqBlock*  block;      /* current block */                    \
        schar*       ptr;        /* pointer to element be read next */  \
        schar*       block_min;  /* pointer to the beginning of block */\
        schar*       block_max;  /* pointer to the end of block */      \
        int          delta_index;/* = seq->first->start_index   */      \
        schar*       prev_elem;  /* pointer to previous element */


    typedef struct CvSeqReader
    {
        CV_SEQ_READER_FIELDS()
    }
    CvSeqReader;

    /* Freeman chain reader state */
    typedef struct CvChainPtReader
    {
        CV_SEQ_READER_FIELDS()
        char      code;
        CvPoint   pt;
        char      deltas[8][2];
    }
    CvChainPtReader;

    /****************************************************************************************\
    *                         Raster->Chain Tree (Suzuki algorithms)                         *
    \****************************************************************************************/

    typedef struct _CvContourInfo
    {
        int flags;
        struct _CvContourInfo *next;        /* next contour with the same mark value */
        struct _CvContourInfo *parent;      /* information about parent contour */
        CvSeq *contour;             /* corresponding contour (may be 0, if rejected) */
        CvRect rect;                /* bounding rectangle */
        CvPoint origin;             /* origin point (where the contour was traced from) */
        int is_hole;                /* hole flag */
    }
    _CvContourInfo;


    /*
      Structure that is used for sequential retrieving contours from the image.
      It supports both hierarchical and plane variants of Suzuki algorithm.
    */
    typedef struct _CvContourScanner
    {
        CvMemStorage *storage1;     /* contains fetched contours */
        CvMemStorage *storage2;     /* contains approximated contours
                                       (!=storage1 if approx_method2 != approx_method1) */
        CvMemStorage *cinfo_storage;        /* contains _CvContourInfo nodes */
        CvSet *cinfo_set;           /* set of _CvContourInfo nodes */
        CvMemStoragePos initial_pos;        /* starting storage pos */
        CvMemStoragePos backup_pos; /* beginning of the latest approx. contour */
        CvMemStoragePos backup_pos2;        /* ending of the latest approx. contour */
        schar *img0;                /* image origin */
        schar *img;                 /* current image row */
        int img_step;               /* image step */
        CvSize img_size;            /* ROI size */
        CvPoint offset;             /* ROI offset: coordinates, added to each contour point */
        CvPoint pt;                 /* current scanner position */
        CvPoint lnbd;               /* position of the last met contour */
        int nbd;                    /* current mark val */
        _CvContourInfo *l_cinfo;    /* information about latest approx. contour */
        _CvContourInfo cinfo_temp;  /* temporary var which is used in simple modes */
        _CvContourInfo frame_info;  /* information about frame */
        CvSeq frame;                /* frame itself */
        int approx_method1;         /* approx method when tracing */
        int approx_method2;         /* final approx method */
        int mode;                   /* contour scanning mode:
                                       0 - external only
                                       1 - all the contours w/o any hierarchy
                                       2 - connected components (i.e. two-level structure -
                                       external contours and holes),
                                       3 - full hierarchy;
                                       4 - connected components of a multi-level image
                                    */
        int subst_flag;
        int seq_type1;              /* type of fetched contours */
        int header_size1;           /* hdr size of fetched contours */
        int elem_size1;             /* elem size of fetched contours */
        int seq_type2;              /*                                       */
        int header_size2;           /*        the same for approx. contours  */
        int elem_size2;             /*                                       */
        _CvContourInfo *cinfo_table[128];
    }
    _CvContourScanner;

    #define _CV_FIND_CONTOURS_FLAGS_EXTERNAL_ONLY    1
    #define _CV_FIND_CONTOURS_FLAGS_HIERARCHIC       2

	enum
	{
		CV_ADAPTIVE_THRESH_MEAN_C  =0,
		CV_ADAPTIVE_THRESH_GAUSSIAN_C  =1
	};

    /** Contour retrieval modes */
    enum
    {
        CV_RETR_EXTERNAL=0,
        CV_RETR_LIST=1,
        CV_RETR_CCOMP=2,
        CV_RETR_TREE=3,
        CV_RETR_FLOODFILL=4
    };

    /** Contour approximation methods */
    enum
    {
        CV_CHAIN_CODE=0,
        CV_CHAIN_APPROX_NONE=1,
        CV_CHAIN_APPROX_SIMPLE=2,
        CV_CHAIN_APPROX_TC89_L1=3,
        CV_CHAIN_APPROX_TC89_KCOS=4,
        CV_LINK_RUNS=5
    };

    /*
    Internal structure that is used for sequential retrieving contours from the image.
    It supports both hierarchical and plane variants of Suzuki algorithm.
    */
    typedef struct _CvContourScanner* CvContourScanner;

    /*********************************** Chain/Countour *************************************/

    typedef struct CvChain
    {
        CV_SEQUENCE_FIELDS()
        CvPoint  origin;
    }
    CvChain;

    #define CV_CONTOUR_FIELDS()  \
        CV_SEQUENCE_FIELDS()     \
        CvRect rect;             \
        int color;               \
        int reserved[3];

    typedef struct CvContour
    {
        CV_CONTOUR_FIELDS()
    }
    CvContour;

	// Definition Contour Struct
    //typedef struct CvContourEx
    //{
    //    CV_CONTOUR_FIELDS()
    //    int counter;
    //}
    //CvContourEx;

    typedef CvContour CvPoint2DSeq;

    typedef int (*sklansky_func)( CvPoint** points, int start, int end,
                                  int* stack, int sign, int sign2 );

	class Range
    {
    public:
        Range();
        Range(int _start, int _end);
        int size() const;
        bool empty() const;
        static Range all();

        int start, end;
    };

	template<typename _Tp, size_t fixed_size = 1024/sizeof(_Tp)+8> class AutoBuffer
    {
    public:
        typedef _Tp value_type;

        //! the default constructor
        AutoBuffer();
        //! constructor taking the real buffer size
        AutoBuffer(size_t _size);

        //! the copy constructor
        AutoBuffer(const AutoBuffer<_Tp, fixed_size>& buf);
        //! the assignment operator
        AutoBuffer<_Tp, fixed_size>& operator = (const AutoBuffer<_Tp, fixed_size>& buf);

        //! destructor. calls deallocate()
        ~AutoBuffer();

        //! allocates the new buffer of size _size. if the _size is small enough, stack-allocated buffer is used
        void allocate(size_t _size);
        //! deallocates the buffer if it was dynamically allocated
        void deallocate();
        //! resizes the buffer and preserves the content
        void resize(size_t _size);
        //! returns the current buffer size
        size_t size() const;
        //! returns pointer to the real buffer, stack-allocated or head-allocated
        operator _Tp* ();
        //! returns read-only pointer to the real buffer, stack-allocated or head-allocated
        operator const _Tp* () const;

    protected:
        //! pointer to the real buffer, can point to buf if the buffer is small enough
        _Tp* ptr;
        //! size of the real buffer
        size_t sz;
        //! pre-allocated buffer. At least 1 element to confirm C++ standard reqirements
        _Tp buf[(fixed_size > 0) ? fixed_size : 1];
    };

	CvSlice cvSlice( int32_t start, int32_t end );
	CvPoint cvPoint( int32_t x, int32_t y );
	CvPoint2D32f cvPoint2D32f( float32_t x, float32_t y );
	CvRect cvRect( int32_t x, int32_t y, int32_t width, int32_t height );
	CvSize cvSize( int32_t width, int32_t height );

	IplImage *cvCreateImage(CvSize size, int32_t depth, int32_t channels);
	IplImage *cvCreateImage(Size size, int32_t depth, int32_t channels);
	IplImage *cvCreateImageHeader( CvSize size, int32_t depth, int32_t channels );
	void cvReleaseImageHeader(IplImage **image);
	void cvReleaseImage(IplImage **image);

	void cvSetZero(IplImage* dst);
	void cvCopy(const IplImage* src, IplImage* dst);
	void cvCvtColor(const IplImage* src, IplImage* dst, int code);
	void GenerateKernel(double *kernel, int ksize, double sigma);
	void cvGaussianSmooth(const IplImage* src, IplImage* dst, int ksize, double sigma);
	void adaptiveThreshold( const IplImage* src, IplImage* dst,
					    double maxValue, int type, int blockSize, double delta );
	void cvAdaptiveThreshold( const IplImage* srcIm, IplImage* dstIm, double maxValue, int adaptive_method,
		int type, int blockSize, double delta);
	void cvDilate( const IplImage* src, IplImage* dst, int blockSize, int mode, int iterations CV_DEFAULT(1) );
	void cvErode( const IplImage* src, IplImage* dst, int blockSize, int mode, int iterations CV_DEFAULT(1) );
	void myDilateV4(const IplImage *src,IplImage *dst,int radius);

	/****************************************************************************************\
    *                               Sequence implementation                                  *
    \****************************************************************************************/
	CvSeq *cvCreateSeq( int seq_flags, size_t header_size, size_t elem_size, CvMemStorage* storage );
	void cvSetSeqBlockSize( CvSeq *seq, int delta_elements );
	schar* cvGetSeqElem( const CvSeq *seq, int index );
	int cvSeqElemIdx( const CvSeq* seq, const void* element, CvSeqBlock** block CV_DEFAULT(NULL) );
	int cvSliceLength( CvSlice slice, const CvSeq* seq );
	void* cvCvtSeqToArray( const CvSeq* seq, void* elements, CvSlice slice CV_DEFAULT(CV_WHOLE_SEQ) );
	static void icvGrowSeq( CvSeq *seq, int in_front_of );
	static void icvFreeSeqBlock( CvSeq *seq, int in_front_of );

	/****************************************************************************************\
    *                             Sequence Writer implementation                             *
    \****************************************************************************************/
	void cvStartAppendToSeq( CvSeq *seq, CvSeqWriter * writer );
	void cvStartWriteSeq( int seq_flags, int header_size, int elem_size, CvMemStorage * storage, CvSeqWriter * writer );
	void cvFlushSeqWriter( CvSeqWriter * writer );
	CvSeq *cvEndWriteSeq( CvSeqWriter * writer );
	void cvCreateSeqBlock( CvSeqWriter * writer );

	/****************************************************************************************\
    *                               Sequence Reader implementation                           *
    \****************************************************************************************/
	void cvStartReadSeq( const CvSeq *seq, CvSeqReader * reader, int reverse CV_DEFAULT(0) );
	void cvChangeSeqBlock( void* reader, int direction );
	int cvGetSeqReaderPos( CvSeqReader* reader );
	void cvSetSeqReaderPos( CvSeqReader* reader, int index, int is_relative CV_DEFAULT(0) );
	schar* cvSeqPush( CvSeq* seq, const void* element CV_DEFAULT(NULL));
	void cvSeqPop( CvSeq* seq, void* element CV_DEFAULT(NULL));
	void cvSeqPushMulti( CvSeq* seq, const void* elements, int count, int in_front CV_DEFAULT(0) );
	void cvSeqPopMulti( CvSeq* seq, void* elements, int count, int in_front CV_DEFAULT(0) );
	void cvClearSeq( CvSeq *seq );

    /****************************************************************************************\
    *                                  Set implementation                                    *
    \****************************************************************************************/
	CvSet* cvCreateSet( int set_flags, int header_size, int elem_size, CvMemStorage* storage );
	int cvSetAdd( CvSet* set_header, CvSetElem* elem CV_DEFAULT(NULL), CvSetElem** inserted_elem CV_DEFAULT(NULL) );
	void cvClearSet( CvSet* set_header );

	/****************************************************************************************\
    *                                   Contours Tracing                                     *
    \****************************************************************************************/
	CvContourScanner cvStartFindContours( IplImage* image, CvMemStorage* storage,
                            int header_size CV_DEFAULT(sizeof(CvContour)),
                            int mode CV_DEFAULT(CV_RETR_LIST),
                            int method CV_DEFAULT(CV_CHAIN_APPROX_SIMPLE),
                            CvPoint offset CV_DEFAULT(cvPoint(0,0)));
	void icvEndProcessContour( CvContourScanner scanner );
	void cvSubstituteContour( CvContourScanner scanner, CvSeq* new_contour );
	void icvFetchContour( schar *ptr, int step, CvPoint pt, CvSeq* contour, int _method );
	static int icvTraceContour( schar *ptr, int step, schar *stop_ptr, int is_hole );
	static void icvFetchContourEx( schar* ptr, int step, CvPoint pt, CvSeq* contour, int _method, int nbd, CvRect* _rect );
	static int icvTraceContour_32s( int *ptr, int step, int *stop_ptr, int is_hole );
	static void icvFetchContourEx_32s( int* ptr, int step, CvPoint pt, CvSeq* contour, int _method, CvRect* _rect );
	CvSeq* cvFindNextContour( CvContourScanner scanner );
	CvSeq* cvEndFindContours( CvContourScanner * scanner );

	/****************************************************************************************\
    *                                  Chain Approximation                                   *
    \****************************************************************************************/
	void cvStartReadChainPoints( CvChain * chain, CvChainPtReader * reader );
	CvSeq* icvApproximateChainTC89( CvChain* chain, int header_size, CvMemStorage* storage, int method );
	int cvRound( double value );
	int cvFloor(double value);
	int cvCeil(double value);
	
	/****************************************************************************************\
    *                               Polygonal Approximation                                  *
    \****************************************************************************************/
	static int approxPolyDP_32s( const CvPoint* src_contour, int count0, CvPoint* dst_contour, bool is_closed0, double eps, AutoBuffer<Range>* _stack );
	static int approxPolyDP_32f( const CvPoint2D32f* src_contour, int count0, CvPoint2D32f* dst_contour, bool is_closed0, double eps, AutoBuffer<Range>* _stack );
	CvSeq* cvApproxPoly( const CvSeq* src_seq, int header_size, CvMemStorage* storage, int method, double eps, int recursive CV_DEFAULT(0));

	/****************************************************************************************\
    *                            Contour Processing and Shape Analysis                       *
    \****************************************************************************************/
	double cvArcLength( const CvSeq* curve, CvSlice slice CV_DEFAULT(CV_WHOLE_SEQ), int is_closed CV_DEFAULT(-1));
	//double cvContourPerimeter( const void* contour )
	//{
	//	return cvArcLength( contour, CV_WHOLE_SEQ, 1 );
	//}
	CvRect cvBoundingRect( const CvSeq* ptseq, int update );
	double contourArea( const CvSeq* contour, bool oriented );
	static double icvContourSecArea( const CvSeq * contour, CvSlice slice );
	double cvContourArea( const CvSeq* contour, CvSlice slice CV_DEFAULT(CV_WHOLE_SEQ), int oriented CV_DEFAULT(0));
	static int icvSklansky_32s( CvPoint** array, int start, int end, int* stack, int nsign, int sign2 );
	static int icvSklansky_32f( CvPoint2D32f** array, int start, int end, int* stack, int nsign, int sign2 );
	static void icvCalcAndWritePtIndices( CvPoint** pointer, int* stack, int start, int end, const CvSeq* ptseq, CvSeqWriter* writer );
	CvSeq* cvConvexHull2( const CvSeq* input, void* hull_storage CV_DEFAULT(NULL), int orientation CV_DEFAULT(CV_CLOCKWISE), int return_points CV_DEFAULT(0));
	bool isContourConvex( const CvSeq* contour );
	int cvCheckContourConvexity( const CvSeq* contour );

	/****************************************************************************************\
    *            Functions for manipulating memory storage - list of memory blocks           *
    \****************************************************************************************/
	static void icvInitMemStorage( CvMemStorage* storage, int block_size );
	CvMemStorage* cvCreateMemStorage( int block_size );
	CvMemStorage* cvCreateChildMemStorage( CvMemStorage * parent );
	static void icvDestroyMemStorage( CvMemStorage* storage );
	void cvReleaseMemStorage( CvMemStorage** storage );
	void cvClearMemStorage( CvMemStorage * storage );
	static void icvGoNextMemBlock( CvMemStorage * storage );
	void cvSaveMemStoragePos( const CvMemStorage * storage, CvMemStoragePos * pos );
	void cvRestoreMemStoragePos( CvMemStorage * storage, CvMemStoragePos * pos );
	void* cvMemStorageAlloc( CvMemStorage* storage, size_t size );
	static void icvMemCopy( double **buf1, double **buf2, double **buf3, int *b_max );
	
	/****************************************************************************************\
    *                                 Working with sequence tree                             *
    \****************************************************************************************/
	void cvInsertNodeIntoTree( void* _node, void* _parent, void* _frame );

	static inline void* cvAlignPtr( const void* ptr, int align = 32 );
	inline int cvAlignLeft( int size, int align );
	static inline int cvAlign( int size, int align );

	/****************************************************************************************\
     *          Array allocation, deallocation, initialization and access to elements         *
    \****************************************************************************************/
	void* fastMalloc( size_t size );
	void fastFree(void* ptr);
	void* cvAlloc( size_t size );
	void cvFree_( void* ptr );

#endif

#endif