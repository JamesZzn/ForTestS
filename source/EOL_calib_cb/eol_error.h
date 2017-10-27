#ifndef _EOL_ERROR_H_
#define _EOL_ERROR_H_

#include "utility\common\reuse.h"
#include "utility\common\bev_data_type.h"
#include "eol_opencv_adapter.h"

#define EOL_ERROR_CODE(x) (x)+(3<<8)		

#define CHECK_ERROR(status)			\
    if (status!=EOL_SUCCESS)			\
	{					\
		printf("[EOL_CALIB]check_error: file(%s), line(%d)\n", __FILE__, __LINE__);	 \
		return status; \
	} 

typedef enum EOL_Calib_Diag_Status_Tag
{
	EOL_SUCCESS                                = EOL_ERROR_CODE(0), 
	LOAD_INIT_IMG_FAIL                                            ,//769
	LOAD_SMC_FAIL                                                 ,//770
	SMC_CONFIG_ERROR                                              ,//771
	SMC_IMG_NOT_COMPATIBLE                                        ,//772
	LOAD_IMG_FAIL                                                 ,//773
	MEM_MALLOC_FAIL                                               ,//774
	UNDEFINED_VEHICLE_TYPE                                        ,//775
	OPEN_FILE_ERROR                                               ,//776
	FRONT_IMG_CORNER_NUM_WRONG                                    ,//777
	REAR_IMG_CORNER_NUM_WRONG                                     ,//778
	LEFT_IMG_CORNER_NUM_WRONG                                     ,//779
	RIGHT_IMG_CORNER_NUM_WRONG                                    ,//780
	EOL_CB_WRONG_CORNER_NUM                                       ,//781
	CONVERGENCE_FAIL                                              ,//782
	CALIB_DATA_BUFFER_SIZE_ERROR			                      ,//783
	LM_FRONT_ERROR_OVERSIZE                                       ,//784
	LM_REAR_ERROR_OVERSIZE                                        ,//785
	LM_LEFT_ERROR_OVERSIZE                                        ,//786
	LM_RIGHT_ERROR_OVERSIZE                                       ,//787
	LM_FRONT_TRANS_OVERSIZE                                       ,//788
	LM_REAR_TRANS_OVERSIZE                                        ,//789
	LM_LEFT_TRANS_OVERSIZE                                        ,//790
	LM_RIGHT_TRANS_OVERSIZE                                       ,//791
	LM_FRONT_CENTER_OVERSIZE                                      ,//792
	LM_REAR_CENTER_OVERSIZE                                       ,//793
	LM_LEFT_CENTER_OVERSIZE                                       ,//794
	LM_RIGHT_CENTER_OVERSIZE                                      ,//795
	LM_FRONT_FOV_OVERSIZE                                         ,//796
	LM_REAR_FOV_OVERSIZE                                          ,//797
	LM_LEFT_FOV_OVERSIZE                                          ,//798
	LM_RIGHT_FOV_OVERSIZE                                         ,//799
	INSUFFICIENT_CORNER_NUM                                        //800
}
EOL_Calib_Diag_Status_T;

#endif



