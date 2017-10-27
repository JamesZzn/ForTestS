#include "eol_utility.h"

/*
	Function Name:      get_mid_index
	Function Function : Get the min and max index that need to be exchanged in quick sort algorithm
	Input             : 
	   value_array :    The array need to be resorted
	   low :            The first element's subindex bigger than pivot
	   high :           The first element's subindex smaller than pivot
	Return            : current index
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/22       Create
*/
int32_t get_mid_index(float64_t* value_array, int32_t low, int32_t high)
{  
	float64_t tmp = value_array[low]; 
	while(low < high)
	{
		while(low < high && value_array[high] >= tmp)
		{ 
			high--;  
		}  
		value_array[low] = value_array[high];  
		         
		while(low < high && value_array[low] < tmp)
		{ 
			low++;  
		}  
		value_array[high] = value_array[low];  
	}  
	               
	value_array[low]=tmp;
	return low;
}  
 
/*
	Function Name:      quick_sort
	Function Function : do quick on current array
	Input             : 
	   value_array :    The array need to be resorted
	   low :            The first element's subindex bigger than pivot
	   high :           The first element's subindex smaller than pivot
	Return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/22       Create
*/
void quick_sort(float64_t* value_array, int32_t low, int32_t high)
{
	if(low < high)
	{
		int32_t mid = get_mid_index(value_array, low, high);   
		quick_sort(value_array, low, mid);  
		quick_sort(value_array, mid + 1, high);  
	}  
}  

/*
	Function Name:      find_median
	Function Function : find the median of value_array
	Input             : 
	   value_array :    The array need to be resorted
	   len :            The length of value_array
	Return            : median
	Note              : 
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/22       Create
*/
int32_t find_median(float64_t* value_array, int32_t len, float64_t& median_value) 
{  
	int32_t ret = EOL_SUCCESS;
	if(len == 0) 
	{
		ret = INSUFFICIENT_CORNER_NUM;
		return ret;
	}
	quick_sort(value_array, 0, len - 1);  
	median_value = value_array[(len - 1) / 2];
	return ret;
}

/*
	Function Name:      svd
	Function Function : realize svd decomposition
	Input             : 
	   a :              src array (m*n)
	   m :              sample nums
	   n :              parameter nums
	   w :              to save the solved eigen value
	   v :              the solved eigen vector (n*n), each col corresponding to a eigen value, and is the solution to be solved. 
	Return            : error code
	Note              : The function is limited to solve homogeneous linear equation group and a,w,v 's memory should be malloced beforehand 
	Revision History  : Author         Data             Changes
	                    YanShuo        2017/02/24       Create
*/

int32_t svd(float64_t a[][9], int32_t m, int32_t n, float64_t w[], float64_t v[][9])
{
	int32_t ret = EOL_SUCCESS;
    int32_t flag, i, its, j, jj, k, l, nm;
    float64_t c, f, h, s, x, y, z;
    float64_t anorm = 0.0, g = 0.0, scale = 0.0;
    float64_t *rv1;

    if (m < n) 
    {
        printf("The matrix is not full rank, has multi solution \n");
		ret = INSUFFICIENT_CORNER_NUM;
        return ret;
    }

    rv1 = (float64_t *)malloc(n*sizeof(float64_t));
	if(!rv1)
	{
		printf("Insufficient memory.\n");
		ret = MEM_MALLOC_FAIL;
		return ret;
	}

    /* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++) 
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m) 
        {
            for (k = i; k < m; k++) 
			{
                scale += fabs((float64_t)a[k][i]);
			}
            if (scale) 
            {
                for (k = i; k < m; k++) 
                {
                    a[k][i] = ((float64_t)a[k][i]/scale);
                    s += ((float64_t)a[k][i] * (float64_t)a[k][i]);
                }
                f = (float64_t)a[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][i] = (f - g);
                if (i != n - 1) 
                {
                    for (j = l; j < n; j++) 
                    {
                        for (s = 0.0, k = i; k < m; k++) 
						{
                            s += ((float64_t)a[k][i] * (float64_t)a[k][j]);
						}
                        f = s / h;
                        for (k = i; k < m; k++) 
						{
                            a[k][j] += (f * (float64_t)a[k][i]);
						}
                    }
                }
                for (k = i; k < m; k++) 
				{
                    a[k][i] = ((float64_t)a[k][i]*scale);
				}
            }
        }
        w[i] = (scale * g);

        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1) 
        {
            for (k = l; k < n; k++) 
			{
                scale += fabs((float64_t)a[i][k]);
			}
            if (scale) 
            {
                for (k = l; k < n; k++) 
                {
                    a[i][k] = ((float64_t)a[i][k]/scale);
                    s += ((float64_t)a[i][k] * (float64_t)a[i][k]);
                }
                f = (float64_t)a[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][l] = (f - g);
                for (k = l; k < n; k++) 
				{
                    rv1[k] = (float64_t)a[i][k] / h;
				}
                if (i != m - 1) 
                {
                    for (j = l; j < m; j++) 
                    {
                        for (s = 0.0, k = l; k < n; k++) 
						{
                            s += ((float64_t)a[j][k] * (float64_t)a[i][k]);
						}
                        for (k = l; k < n; k++) 
						{
                            a[j][k] += (s * rv1[k]);
						}
                    }
                }
                for (k = l; k < n; k++) 
				{
                    a[i][k] = ((float64_t)a[i][k]*scale);
				}
            }
        }
        anorm = MAX(anorm, (fabs((float64_t)w[i]) + fabs(rv1[i])));
    }

    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--) 
    {
        if (i < n - 1) 
        {
            if (g) 
            {
                for (j = l; j < n; j++)
				{
                    v[j][i] = (((float64_t)a[i][j] / (float64_t)a[i][l]) / g);
				}
                /* double division to avoid underflow */
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < n; k++) 
					{
                        s += ((float64_t)a[i][k] * (float64_t)v[k][j]);
					}
                    for (k = l; k < n; k++) 
					{
                        v[k][j] += (s * (float64_t)v[k][i]);
					}
                }
            }
            for (j = l; j < n; j++) 
			{
                v[i][j] = v[j][i] = 0.0;
			}
        }
        v[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }

    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--) 
    {
        l = i + 1;
        g = (float64_t)w[i];
        if (i < n - 1) 
		{
            for (j = l; j < n; j++) 
			{
                a[i][j] = 0.0;
			}
		}
        if (g) 
        {
            g = 1.0 / g;
            if (i != n - 1) 
            {
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < m; k++) 
					{
                        s += ((float64_t)a[k][i] * (float64_t)a[k][j]);
					}
                    f = (s / (float64_t)a[i][i]) * g;
                    for (k = i; k < m; k++) 
					{
                        a[k][j] += (f * (float64_t)a[k][i]);
					}
                }
            }
            for (j = i; j < m; j++) 
			{
                a[j][i] = ((float64_t)a[j][i] * g);
			}
        }
        else 
        {
            for (j = i; j < m; j++) 
			{
                a[j][i] = 0.0;
			}
        }
        ++a[i][i];
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--) 
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++) 
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--) 
            {                     /* test for splitting */
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm) 
                {
                    flag = 0;
                    break;
                }
                if (fabs((float64_t)w[nm]) + anorm == anorm) 
                    break;
            }
            if (flag) 
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++) 
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm) 
                    {
                        g = (float64_t)w[i];
                        h = sqrt(f*f+g*g);
                        w[i] = h; 
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++) 
                        {
                            y = (float64_t)a[j][nm];
                            z = (float64_t)a[j][i];
                            a[j][nm] = (y * c + z * s);
                            a[j][i] = (z * c - y * s);
                        }
                    }
                }
            }
            z = (float64_t)w[k];
            if (l == k) 
            {                  /* convergence */
                if (z < 0.0) 
                {              /* make singular value nonnegative */
                    w[k] = (-z);
                    for (j = 0; j < n; j++) 
					{
                        v[j][k] = (-v[j][k]);
					}
                }
                break;
            }
            if (its >= 30) 
			{
                free((void*) rv1);
                fprintf(stderr, "No convergence after 30,000! iterations \n");
				ret = CONVERGENCE_FAIL;
                return ret;
            }

            /* shift from bottom 2 x 2 minor */
            x = (float64_t)w[l];
            nm = k - 1;
            y = (float64_t)w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = sqrt(f*f + 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++) 
            {
                i = j + 1;
                g = rv1[i];
                y = (float64_t)w[i];
                h = s * g;
                g = c * g;
                z = sqrt(f*f+h*h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++) 
                {
                    x = (float64_t)v[jj][j];
                    z = (float64_t)v[jj][i];
                    v[jj][j] = (x * c + z * s);
                    v[jj][i] = (z * c - x * s);
                }
                z = sqrt(f*f+h*h);
                w[j] = z;
                if (z) 
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++) 
                {
                    y = (float64_t)a[jj][j];
                    z = (float64_t)a[jj][i];
                    a[jj][j] = (y * c + z * s);
                    a[jj][i] = (z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
    free((void*) rv1);
    return ret;
}

/*
	Function Name:      homography_to_angle
	Function Function : change the solved homography matrix to Euler angle
	Input             : 
	   homography :     The homography matrix is the transformation from vehicle coordinate system to camera coordinate system
	   angle :          The solved angle in arc, generally it is two group
	Return            : void
	Note              : Because the ground coordinate is set as 0, so the 3rd column is not used, 
		                thus the homography of world->cam is defined as:
		                Note: for x and z axis, anti-clock rotation is positive direction,
		                      for y axis, clock rotation is positive direction. 
		                [ cos(z)*cos(y), -cos(x)*sin(z)+sin(x)*sin(y)*cos(z),  sin(z)*sin(x)+cos(x)*sin(y)*cos(z), tx]
		                [ sin(z)*cos(y),  cos(x)*cos(z)+sin(x)*sin(y)*sin(z), -sin(x)*cos(z)+cos(x)*sin(y)*sin(z), ty]
		                [      - sin(y),                       sin(x)*cos(y),                       cos(x)*cos(y), tz]

						Reference article: "Computing Euler angles from a rotation matrix" 
						"http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf"
						Currently, the second group solution is not used and I don't know why
	Revision History  : Author         Data             Changes
	                   	YanShuo        2017/02/28       Create
*/
void homography_to_angle(float64_t homography[9], float64_t* angle)
{
	float64_t sin_x[2], cos_x[2];
	
	//sin(y) != 1
	if(homography[6] > -0.9999 && homography[6] < 0.9999)
	{
		angle[1] = - asin(homography[6]);
		angle[4] = - asin(homography[6]) + PI;
		angle[2] = atan2(homography[3] / cos(angle[1]), homography[0] / cos(angle[1]));
		angle[5] = atan2(homography[3] / cos(angle[4]), homography[0] / cos(angle[4]));
		
		sin_x[0] = homography[7] / cos(angle[1]);
		cos_x[0] = (homography[4] - sin(angle[2]) * sin(angle[1] * sin_x[0])) / cos(angle[2]);
		sin_x[1] = homography[7] / cos(angle[4]);
		cos_x[1] = (homography[4] - sin(angle[5]) * sin(angle[4] * sin_x[1])) / cos(angle[5]);
		
		angle[0] = atan2(sin_x[0], cos_x[0]);
		angle[3] = atan2(sin_x[1], cos_x[1]);		
	}
	else
	{
		angle[2] = 0;
		angle[5] = 0;
		angle[1] = PI / 2;
		angle[4] = - PI / 2;
		angle[0] = atan2(homography[1], homography[4]);
		angle[3] = - atan2(homography[1], homography[4]);
	}
}

/*
	Function Name:      get_valid_corner_num
	Function Function : calculate the valid corner num in current pattern
	Input             : 
	    p :             The pattern's point
	Return            : valid corner point's number in p
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    2017/12/18       Create
*/
int32_t get_valid_corner_num(const Chessboard_Pattern *p)
{
	int32_t num = 0;
	for(int32_t i = 0; i < p->rows; i++)
	{
		for(int32_t j = 0; j < p->cols; j++)
		{
			if(p->flag[i][j] > 0)
			{
				num++;
			}
		}
	}
	return num;
}

/*
	Function Name     : mask_roi
	Function Function : Mask reference point with a rectangle mask to avoid repeat detection
	Input:
		pattern :       Pattern structure
		roi:            Rectangle mask ROI
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    2016/12/19       Create
 */
void mask_roi(Chessboard_Pattern *pattern, Rect2f *roi)
{
	float32_t x_min = pattern->corner_point[0][0].x;
	float32_t x_max= pattern->corner_point[0][0].x;
	float32_t y_min= pattern->corner_point[0][0].y;
	float32_t y_max= pattern->corner_point[0][0].y;
	for(int32_t i = 0;i< pattern->rows;i++)
	{
		for(int32_t j = 0;j<pattern->cols;j++)
		{
			if(pattern->flag[i][j]==1)
			{
				if(x_min>pattern->corner_point[i][j].x)
				{
					x_min = pattern->corner_point[i][j].x;
				}
				if(x_max<pattern->corner_point[i][j].x)
				{
					x_max=pattern->corner_point[i][j].x;
				}
				if(y_min>pattern->corner_point[i][j].y)
				{
					y_min = pattern->corner_point[i][j].y;
				}
				if(y_max<pattern->corner_point[i][j].y)
				{
					y_max=pattern->corner_point[i][j].y;
				}
			}
		}
	}
	
	x_min-=5;
	y_min-=5;
	x_max+=5;
	y_max+=5;
	roi->x=x_min;
	roi->y=y_min;
	roi->width = x_max-x_min+1;
	roi->height=y_max-y_min+1;
}

/*
	Function Name     : boundary_point
	Function Function : Mask the detected chessboard with a quadrangle mask to avoid repeat detection
	                    Calculate the position of 4 boundary points of the quadrangle mask
	Input:
		pattern :       Pattern structure
		points_array:   4 vertexes of the quadrangle mask
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    WangLuyao      2017/3/22        Create
 */
void calc_boundary_point(Chessboard_Pattern* pattern, Point2f points_array[4])
{
	if ((pattern->rows == 1) && (pattern->cols == 1))
	{
		if (pattern->flag[0][0] == 1)
		{
			points_array[0].x = pattern->corner_point[0][0].x - 20;
			points_array[0].y = pattern->corner_point[0][0].y - 20;
			points_array[1].x = pattern->corner_point[0][0].x + 20;
			points_array[1].y = pattern->corner_point[0][0].y - 20;
			points_array[2].x = pattern->corner_point[0][0].x - 20;
			points_array[2].y = pattern->corner_point[0][0].y + 20;
			points_array[3].x = pattern->corner_point[0][0].x + 20;
			points_array[3].y = pattern->corner_point[0][0].y + 20;
		}
	}

	if(pattern->flag[0][0] == 1)
	{
		points_array[0].x = pattern->corner_point[0][0].x - 20;
		points_array[0].y = pattern->corner_point[0][0].y - 20;
	}

	if(pattern->flag[0][pattern->cols - 1] == 1)
	{
		points_array[1].x = pattern->corner_point[0][pattern->cols - 1].x + 20;
		points_array[1].y = pattern->corner_point[0][pattern->cols - 1].y - 20;
	}

	if(pattern->flag[pattern->rows - 1][0] == 1)
	{
		points_array[2].x = pattern->corner_point[pattern->rows - 1][0].x - 20;
		points_array[2].y = pattern->corner_point[pattern->rows - 1][0].y + 20;
	}

	if(pattern->flag[pattern->rows - 1][pattern->cols - 1] == 1)
	{
		points_array[3].x = pattern->corner_point[pattern->rows - 1][pattern->cols - 1].x + 20;
		points_array[3].y = pattern->corner_point[pattern->rows - 1][pattern->cols - 1].y + 20;
	}
}

/*
	Function Name     : calc_boundary_point_jac
	Function Function : Mask the detected chessboard with a quadrangle mask to avoid repeat detection
	Calculate the position of 4 boundary points of the quadrangle mask
	Input:
	pattern :           Pattern structure
	points_array:       4 vertexes of the quadrangle mask
	camid:              cam id
	pattern_id          pattern id
	Return            : Void
	Note              :
	Revision History  : Author         Data             Changes
	WangLuyao      2017/3/22        Create
*/
void calc_boundary_point_jac(Chessboard_Pattern* pattern, Point2f points_array[4], int32_t camid, int pattern_id)
{
	const int EDGE_NARROW = 50;
	const int EDGE_GENERAL = 80;
	switch (camid)
	{
	case 0:
		switch (pattern_id)
		{
		case 0:
			if (pattern->flag[0][1] == 1)
			{
				points_array[0].x = pattern->corner_point[0][1].x - EDGE_NARROW;
				points_array[0].y = pattern->corner_point[0][1].y - EDGE_NARROW;
				points_array[1].x = pattern->corner_point[0][1].x + EDGE_NARROW;
				points_array[1].y = pattern->corner_point[0][1].y - EDGE_NARROW;
				points_array[2].x = pattern->corner_point[0][1].x - EDGE_NARROW;
				points_array[2].y = pattern->corner_point[0][1].y + EDGE_NARROW;
				points_array[3].x = pattern->corner_point[0][1].x + EDGE_NARROW;
				points_array[3].y = pattern->corner_point[0][1].y + EDGE_NARROW;
			}
			break;
		case 1:
			if (pattern->flag[0][1] == 1)
			{
				points_array[0].x = pattern->corner_point[0][1].x - EDGE_GENERAL;
				points_array[0].y = pattern->corner_point[0][1].y;
				points_array[1].x = pattern->corner_point[0][1].x;
				points_array[1].y = pattern->corner_point[0][1].y;
				points_array[2].x = pattern->corner_point[0][1].x - EDGE_GENERAL;
				points_array[2].y = pattern->corner_point[0][1].y + EDGE_GENERAL;
				points_array[3].x = pattern->corner_point[0][1].x;
				points_array[3].y = pattern->corner_point[0][1].y + EDGE_GENERAL;
			}
			break;
		case 2:
			if (pattern->flag[0][1] == 1)
			{
				points_array[0].x = pattern->corner_point[0][1].x;
				points_array[0].y = pattern->corner_point[0][1].y;
				points_array[1].x = pattern->corner_point[0][1].x + EDGE_GENERAL;
				points_array[1].y = pattern->corner_point[0][1].y;
				points_array[2].x = pattern->corner_point[0][1].x;
				points_array[2].y = pattern->corner_point[0][1].y + EDGE_GENERAL;
				points_array[3].x = pattern->corner_point[0][1].x + EDGE_GENERAL;
				points_array[3].y = pattern->corner_point[0][1].y + EDGE_GENERAL;
			}
			break;
		}
		break;
	case 1:
		switch (pattern_id)
		{
		case 0:
			if (pattern->flag[1][1] == 1)
			{
				points_array[0].x = pattern->corner_point[1][1].x - EDGE_GENERAL;
				points_array[0].y = pattern->corner_point[1][1].y - EDGE_GENERAL;
				points_array[1].x = pattern->corner_point[1][1].x + EDGE_GENERAL;
				points_array[1].y = pattern->corner_point[1][1].y - EDGE_GENERAL;
				points_array[2].x = pattern->corner_point[1][1].x - EDGE_GENERAL;
				points_array[2].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
				points_array[3].x = pattern->corner_point[1][1].x + EDGE_GENERAL;
				points_array[3].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
			}
			break;
		case 1:
			if (pattern->flag[1][1] == 1)
			{
				points_array[0].x = pattern->corner_point[1][1].x - EDGE_GENERAL;
				points_array[0].y = pattern->corner_point[1][1].y;
				points_array[1].x = pattern->corner_point[1][1].x;
				points_array[1].y = pattern->corner_point[1][1].y;
				points_array[2].x = pattern->corner_point[1][1].x - EDGE_GENERAL;
				points_array[2].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
				points_array[3].x = pattern->corner_point[1][1].x;
				points_array[3].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
			}
			break;
		case 2:
			if (pattern->flag[1][1] == 1)
			{
				points_array[0].x = pattern->corner_point[1][1].x;
				points_array[0].y = pattern->corner_point[1][1].y;
				points_array[1].x = pattern->corner_point[1][1].x + EDGE_GENERAL;
				points_array[1].y = pattern->corner_point[1][1].y;
				points_array[2].x = pattern->corner_point[1][1].x;
				points_array[2].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
				points_array[3].x = pattern->corner_point[1][1].x + EDGE_GENERAL;
				points_array[3].y = pattern->corner_point[1][1].y + EDGE_GENERAL;
			}
			break;
		}
		break;
	case 2:
	case 3:
		break;
	default:
		break;
	}
}

/*
	Function Name     : refenrece_point
	Function Function : Calculate the pattern's reference point
	Input:
		pattern :       Pattern structure
		roi:            Rectangle mask ROI
	Return            : Void
	Note              : 
	Revision History  : Author         Data             Changes
	                    DaiXiaqiang    2016/12/19       Create
						YanShuo        2017/5/15        Code normalization
 */
void refenrece_point(Chessboard_Pattern* pattern, Point2f* point)
{
	float32_t x_sum = 0;
	float32_t y_sum = 0;
	int32_t count = 0;
	for(int32_t i = 0; i < pattern->rows; i++)
	{
		for(int32_t j = 0; j < pattern->cols; j++)
		{
			if(pattern->flag[i][j])
			{
				x_sum += pattern->corner_point[i][j].x;
				y_sum += pattern->corner_point[i][j].y;
				count++;
			}
		}
	}

	if(0 < count)
	{
		point->x = x_sum / count;
		point->y = y_sum / count;
	}
	else
	{
		point->x = 0;
		point->y = 0;
	}
}

/*
	Function Name:      pattern_checking
	Function Function : only when at least a full row and a full col are detected, the pattern is valid
	Input             : 
	    pattern :       Pattern structure
		station_type : station type
	Return            : Void
	Note              : Each pattern's valid corner num for optimizer is obtained in this function,
	                    the pattern is valid only when at least a full row and a full col are detected. 
	Revision History  : Author         Data             Changes
	                    WangLuyao      2016/2/22        Create
						YanShuo        2016/2/23        Modify
						YanShuo        2017/5/15        Normalization
*/
void pattern_checking(Chessboard_Pattern *pattern, int32_t station_type, int32_t cam_id, int32_t pattern_id)
{
	bool flag_row = false, flag_col = false;
	int32_t valid_corner_count = 0;

	switch (station_type)
	{
	case STATION_TYPE_JAC:
		if (cam_id == 0 && pattern_id == 1)
		{
			if (pattern->flag[0][0] == 1)
			{
				pattern->flag[0][0] = 0;
				pattern->corner_point[0][0].x = 0;
				pattern->corner_point[0][0].y = 0;
			}

			if (pattern->flag[0][2] == 1)
			{
				pattern->flag[0][2] = 0;
				pattern->corner_point[0][2].x = 0;
				pattern->corner_point[0][2].y = 0;
			}

			if (pattern->flag[1][2] == 1)
			{
				pattern->flag[1][2] = 0;
				pattern->corner_point[1][2].x = 0;
				pattern->corner_point[1][2].y = 0;
			}
		}

		if (cam_id == 0 && pattern_id == 2)
		{
			if (pattern->flag[0][0] == 1)
			{
				pattern->flag[0][0] = 0;
				pattern->corner_point[0][0].x = 0;
				pattern->corner_point[0][0].y = 0;
			}

			if (pattern->flag[0][2] == 1)
			{
				pattern->flag[0][2] = 0;
				pattern->corner_point[0][2].x = 0;
				pattern->corner_point[0][2].y = 0;
			}

			if (pattern->flag[1][0] == 1)
			{
				pattern->flag[1][0] = 0;
				pattern->corner_point[1][0].x = 0;
				pattern->corner_point[1][0].y = 0;
			}
		}

		if (cam_id == 1 && pattern_id == 0)
		{
			if (pattern->flag[0][0] == 1)
			{
				pattern->flag[0][0] = 0;
				pattern->corner_point[0][0].x = 0;
				pattern->corner_point[0][0].y = 0;
			}

			if (pattern->flag[2][2] == 1)
			{
				pattern->flag[2][2] = 0;
				pattern->corner_point[2][2].x = 0;
				pattern->corner_point[2][2].y = 0;
			}
		}

		if (cam_id == 1 && pattern_id == 1)
		{
			if (pattern->flag[0][2] == 1)
			{
				pattern->flag[0][2] = 0;
				pattern->corner_point[0][2].x = 0;
				pattern->corner_point[0][2].y = 0;
			}

			if (pattern->flag[1][2] == 1)
			{
				pattern->flag[1][2] = 0;
				pattern->corner_point[1][2].x = 0;
				pattern->corner_point[1][2].y = 0;
			}

			if (pattern->flag[2][2] == 1)
			{
				pattern->flag[2][2] = 0;
				pattern->corner_point[2][2].x = 0;
				pattern->corner_point[2][2].y = 0;
			}
		}

		if (cam_id == 1 && pattern_id == 2)
		{
			if (pattern->flag[0][0] == 1)
			{
				pattern->flag[0][0] = 0;
				pattern->corner_point[0][0].x = 0;
				pattern->corner_point[0][0].y = 0;
			}

			if (pattern->flag[1][0] == 1)
			{
				pattern->flag[1][0] = 0;
				pattern->corner_point[1][0].x = 0;
				pattern->corner_point[1][0].y = 0;
			}

			if (pattern->flag[2][0] == 1)
			{
				pattern->flag[2][0] = 0;
				pattern->corner_point[2][0].x = 0;
				pattern->corner_point[2][0].y = 0;
			}
		}

		if (cam_id == 2)
		{
			if (pattern->flag[2][0] == 1)
			{
				pattern->flag[2][0] = 0;
				pattern->corner_point[2][0].x = 0;
				pattern->corner_point[2][0].y = 0;
			}

			if (pattern->flag[2][1] == 1)
			{
				pattern->flag[2][1] = 0;
				pattern->corner_point[2][1].x = 0;
				pattern->corner_point[2][1].y = 0;
			}

			if (pattern->flag[2][2] == 1)
			{
				pattern->flag[2][2] = 0;
				pattern->corner_point[2][2].x = 0;
				pattern->corner_point[2][2].y = 0;
			}

			if (pattern->flag[2][13] == 1)
			{
				pattern->flag[2][13] = 0;
				pattern->corner_point[2][13].x = 0;
				pattern->corner_point[2][13].y = 0;
			}

			if (pattern->flag[2][14] == 1)
			{
				pattern->flag[2][14] = 0;
				pattern->corner_point[2][14].x = 0;
				pattern->corner_point[2][14].y = 0;
			}

			if (pattern->flag[0][14] == 1)
			{
				pattern->flag[0][14] = 0;
				pattern->corner_point[0][14].x = 0;
				pattern->corner_point[0][14].y = 0;
			}
		}

		if (cam_id == 3)
		{
			if (pattern->flag[0][0] == 1)
			{
				pattern->flag[0][0] = 0;
				pattern->corner_point[0][0].x = 0;
				pattern->corner_point[0][0].y = 0;
			}

			if (pattern->flag[2][0] == 1)
			{
				pattern->flag[2][0] = 0;
				pattern->corner_point[2][0].x = 0;
				pattern->corner_point[2][0].y = 0;
			}

			if (pattern->flag[2][1] == 1)
			{
				pattern->flag[2][1] = 0;
				pattern->corner_point[2][1].x = 0;
				pattern->corner_point[2][1].y = 0;
			}

			if (pattern->flag[2][12] == 1)
			{
				pattern->flag[2][12] = 0;
				pattern->corner_point[2][12].x = 0;
				pattern->corner_point[2][12].y = 0;
			}

			if (pattern->flag[2][13] == 1)
			{
				pattern->flag[2][13] = 0;
				pattern->corner_point[2][13].x = 0;
				pattern->corner_point[2][13].y = 0;
			}

			if (pattern->flag[2][14] == 1)
			{
				pattern->flag[2][14] = 0;
				pattern->corner_point[0][14].x = 0;
				pattern->corner_point[0][14].y = 0;
			}
		}

		for (int i = 0; i < pattern->rows; i++)
		{
			for (int j = 0; j < pattern->cols; j++)
			{
				if (pattern->flag[i][j])
				{
					valid_corner_count++;
				}
			}
		}
		break;
	case STATION_TYPE_OFILM:
	case STATION_TYPE_GAC:
		for (int32_t i = 0; i < pattern->rows; i++)
		{
			int32_t p = 1;
			for (int32_t j = 0; j < pattern->cols; j++)
			{
				p *= pattern->flag[i][j];
			}
			if (p)
			{
				flag_row = true;
				break;
			}
		}

		for (int32_t j = 0; j < pattern->cols; j++)
		{
			int32_t p = 1;
			for (int32_t i = 0; i < pattern->rows; i++)
			{
				p *= pattern->flag[i][j];
			}
			if (p)
			{
				flag_col = true;
				break;
			}
		}

		if (!(flag_row && flag_col))
		{
			for (int32_t i = 0; i < pattern->rows; i++)
			{
				for (int32_t j = 0; j < pattern->cols; j++)
				{
					pattern->flag[i][j] = 0;
					pattern->corner_point[i][j].x = 0;
					pattern->corner_point[i][j].y = 0;
				}
			}
		}
		else
		{
			for (int32_t i = 0; i < pattern->rows; i++)
			{
				for (int32_t j = 0; j < pattern->cols; j++)
				{
					if (pattern->flag[i][j])
					{
						valid_corner_count++;
					}
				}
			}
		}
		break;
	default:
		break;
	}
	

	pattern->valid_point_num = valid_corner_count;
}

/*
	Function Name:      get_order
	Function Function : reordering chessboard patterns in one camera group because of the corner detection 
	                    algorithm is designed to start from the largest pattern, my be removed in future  
	Input             : 
	  p_source        : the corner num of each pattern
	  p_index         : the index in assigned order
	  num : the num of valid patterns
	 b_is_ascending   : in ascending or descending order
	return            : void
	Note              : 
	Revision History  : Author         Data             Changes
	                    YanShuo        2016/02/07       Create
*/
void get_order(int32_t* p_source, int32_t* p_index, int32_t num, bool b_is_ascending)
{
	int i, j, tmpValue, tmpIndex;
	int ordered_array[100];

	for (i = 0; i < num; i++)
	{
		ordered_array[i] = i;
	}

	if(b_is_ascending)
	{
		for (i = 0; i < num; i++)
		{
			for (j = i + 1; j < num; j++)  
			{  
				if (p_source[i] > p_source[j])  
				{
					tmpValue = p_source[i];
					p_source[i] = p_source[j];
					p_source[j] = tmpValue;

					tmpIndex = ordered_array[i];
					ordered_array[i] = ordered_array[j];
					ordered_array[j] = tmpIndex;
				}
			}
		}
	}
	else
	{
		for (i = 0; i < num; i++)
		{
			for (j = i + 1; j < num; j++)  
			{  
				if (p_source[i] < p_source[j])  
				{
					tmpValue = p_source[i];
					p_source[i] = p_source[j];
					p_source[j] = tmpValue;

					tmpIndex = ordered_array[i];
					ordered_array[i] = ordered_array[j];
					ordered_array[j] = tmpIndex;
				}
			}
		}
	}
	
	for (i = 0; i < num; i++)
	{
		p_index[i] = ordered_array[i];
	}
}