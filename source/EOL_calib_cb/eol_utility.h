#ifndef _EOL_UTILITY_H_
#define _EOL_UTILITY_H_
#include "eol_data_type.h"
#include "eol_error.h"

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

#ifndef PI
#define PI ((float64_t)3.14159265358979323846264338327950288)
#endif

int32_t get_mid_index(float64_t* value_array, int32_t low, int32_t high);

void quick_sort(float64_t* value_array, int32_t low, int32_t high);

int32_t find_median(float64_t* value_array, int32_t len, float64_t& median_value);

int32_t svd(float64_t a[][9], int32_t m, int32_t n, float64_t w[], float64_t v[][9]);

void homography_to_angle(float64_t homography[9], float64_t* angle);

int32_t get_valid_corner_num(const Chessboard_Pattern *p);

void mask_roi(Chessboard_Pattern *pattern, Rect2f *roi);

void calc_boundary_point(Chessboard_Pattern* pattern, Point2f points_array[4]);

void calc_boundary_point_jac(Chessboard_Pattern* pattern, Point2f points_array[4], int32_t camid, int pattern_id);

void refenrece_point(Chessboard_Pattern *pattern,Point2f *point);

void pattern_checking(Chessboard_Pattern *pattern, int32_t station_type, int32_t cam_id, int32_t pattern_id);

void get_order(int32_t* p_source, int32_t* p_index, int32_t num, bool b_is_ascending);

#endif