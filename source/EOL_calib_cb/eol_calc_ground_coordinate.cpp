#include "eol_calc_ground_coordinate.h"

/*
	Function Name:      EOL_calculate_ground_coordinate
	Function Function : calculate corner point's world coordinate via input smc config params
	Input             : 
	    p_Eol_param :   Eol config param
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2016/2/07        Create
*/
void EOL_calculate_ground_coordinate(P_EOL_Param p_Eol_param)
{
	int32_t cam_id, pattern_id; 
	float32_t station_height, station_width, board_2_veh_center_axis;
	int32_t i, j;
	float32_t init_x_trans, init_y_trans, init_x_world, init_y_world, cur_x_coord, cur_y_coord;

	station_height = p_Eol_param->station_height;
	station_width = p_Eol_param->station_width;
	board_2_veh_center_axis = p_Eol_param->board_2_veh_center_axis;

	for (cam_id = 0; cam_id < CAMERA_NUM; cam_id++)
	{
		int32_t valid_pattern_num = p_Eol_param->cb_pattern_group[cam_id].valid_num;
		for (pattern_id = 0; pattern_id < valid_pattern_num; pattern_id++)
		{
			Chessboard_Pattern* p_pattern = &p_Eol_param->cb_pattern_group[cam_id].pattern[pattern_id];
			EOL_pattern_config pattern_config = p_pattern->pattern_config;
			
			switch (cam_id)
			{
			case 0:
				// this pattern's up-left point in camera view is defined as origin point
				init_x_trans = pattern_config.top_left_corner_x;
				init_y_trans = pattern_config.top_left_corner_y;

				init_x_world = station_height * (float32_t)0.5 + pattern_config.pattern_height;
				init_y_world = pattern_config.left_boundary_to_axis;

			    for (i = 0; i < p_pattern->rows; i++)
				{
					for(j = 0; j < p_pattern->cols; j++)
					{
						// origin point is pattern's upleft point 
						cur_x_coord = init_x_trans + j * pattern_config.inner_quad_width;
						cur_y_coord = init_y_trans + i * pattern_config.inner_quad_height;

						p_pattern->ground_corner_point[i][j].x = init_x_world - cur_y_coord;
						p_pattern->ground_corner_point[i][j].y = init_y_world + cur_x_coord;
					}
				}
				break;
			case 1:
				init_x_trans = pattern_config.top_left_corner_x;
				init_y_trans = pattern_config.top_left_corner_y;

				init_x_world = - (station_height * (float32_t)0.5 + pattern_config.pattern_height);
				init_y_world = - pattern_config.left_boundary_to_axis;

				for (i = 0; i < p_pattern->rows; i++)
				{
					for(j = 0; j < p_pattern->cols; j++)
					{
						// origin point is pattern's upleft point 
						cur_x_coord = init_x_trans + j * pattern_config.inner_quad_width;
						cur_y_coord = init_y_trans + i * pattern_config.inner_quad_height;

						p_pattern->ground_corner_point[i][j].x = init_x_world + cur_y_coord;
						p_pattern->ground_corner_point[i][j].y = init_y_world - cur_x_coord;
					}
				}
				break;
			case 2:
				init_x_trans = pattern_config.top_left_corner_x;
				init_y_trans = pattern_config.top_left_corner_y;

				init_x_world = pattern_config.left_boundary_to_axis;
				init_y_world = - (board_2_veh_center_axis + pattern_config.pattern_height);

				for (i = 0; i < p_pattern->rows; i++)
				{
					for(j = 0; j < p_pattern->cols; j++)
					{
						// origin point is pattern's upleft point 
						cur_x_coord = init_x_trans + j * pattern_config.inner_quad_width;
						cur_y_coord = init_y_trans + i * pattern_config.inner_quad_height;

						p_pattern->ground_corner_point[i][j].x = init_x_world + cur_x_coord;
						p_pattern->ground_corner_point[i][j].y = init_y_world + cur_y_coord;
					}
				}
				break;
			case 3:
				init_x_trans = pattern_config.top_left_corner_x;
				init_y_trans = pattern_config.top_left_corner_y;

				init_x_world = - pattern_config.left_boundary_to_axis;
				init_y_world = station_width - board_2_veh_center_axis + pattern_config.pattern_height;

				for (i = 0; i < p_pattern->rows; i++)
				{
					for(j = 0; j < p_pattern->cols; j++)
					{
						// origin point is pattern's upleft point 
						cur_x_coord = init_x_trans + j * pattern_config.inner_quad_width;
						cur_y_coord = init_y_trans + i * pattern_config.inner_quad_height;

						p_pattern->ground_corner_point[i][j].x = (float32_t)init_x_world - cur_x_coord;
						p_pattern->ground_corner_point[i][j].y = (float32_t)init_y_world - cur_y_coord;
					}
				}
				break;				
			}
		}
	}
}