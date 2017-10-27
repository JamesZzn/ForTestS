#ifndef _EOL_CONFIG_H_
#define _EOL_CONFIG_H_

#include <eol_data_type.h>

/*Save Corner Detect temp result*/
#define DEBUG_RESULT_SAVE 

/*Whether need to solve init RT*/
#define SOLVE_INIT_RT

/*Remove outliers of detected corners*/
//#define REMOVE_OUTLIERS

/*Draw detected corners*/
#define SAVE_CORNER_IMAGE

/*Whether need generate log.txt*/
#define SAVE_LOG

//#define CONVERT_SAMPLE

#define CURR_OPTIMIZE_TYPE OPTIMIZE_TYPE_NONE

/*Re-project image points to ground*/
#define RE_PROJECTION_ERROR


#define RAW_HEIGHT      480
#define RAW_WIDTH       736
#define IMAGE_HEIGHT	480
#define IMAGE_WIDTH		720
//#define CUR_IMAGE_TYPE	TYPE_BGR
#define CUR_IMAGE_TYPE TYPE_NV12

#define START_FRAME_ID 0 // The init frame id, start from 0
#define VALID_FRAME_COUNT 1

/* Convert raw image data to short image sequence*/
//#define CONVERT_SAMPLE

#endif