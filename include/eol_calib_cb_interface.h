#include "FrameworkBase\FrameworkBase.h"
#include "utility\common\commondef.h"
#include "utility\common\bev_data_type.h"
#include "eol_data_type.h"

#ifdef _cplusplus
extern "C"
{
#endif

	int32_t EOL_init(void** ppEolHandle, 
		             Smc_Cal_T* const pSMC);

	int32_t EOL_process(void* pImageHandle[CAMERA_NUM], 
		                Smc_Cal_T* const pSMC, 
						void** ppEolHandle);

	void EOL_get_result(str_avm_pose_t *cam_pose, 
		                float64_t *cam_center, 
						float64_t *cam_fov,
						eol_result *result);

	void EOL_deinit(void** ppEolHandle);

	int32_t save_rt_pose(const char *filename, 
		float *rt_front, 
		float *rt_right, 
		float *rt_rear, 
		float *rt_left);

#ifdef _cplusplus
}
#endif