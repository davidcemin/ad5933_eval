/********************************************************************************/
/**
 * \name stats.c
 *
 * This file implements the statistical analysis. All functions not related
 * to ad5933 whatsoever should be here.
 *
 * Supported OSs: Mac OSx
 *
 * Copyright 2013 David Cemin <davidcemin at gmail dot com>
  
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */
/********************************************************************************/

#include "stats.h"


/********************************************************************************/
/* Private Stuff */
/********************************************************************************/


/********************************************************************************/
/* Public Stuff */
/********************************************************************************/

double
dataMean(double *val, int n)
{	
	double ret;
	int i;

	ret = 0.0;

	for (i = 0; i < n; i++) {
		ret += val[i];
	}
	
	ret /= n;

	return (ret);
}

/******************************************************************************/

void 
maxValue(double *v, int n, double *max)
{
	int i;
	
	*max = fabs(v[0]);

	for (i = 0; i < n; i++) 
		if (fabs(v[i]) > *max) 
			*max = fabs(v[i]);
}

/******************************************************************************/

void
minValue(double *v, int n, double *min)
{
	int i;
	
	*min = fabs(v[0]);

	for (i = 0; i < n; i++) 
		if (fabs(v[i]) < *min) 
			*min = fabs(v[i]);
}

/******************************************************************************/

void
variance_stddev(double *v, int n, double *variance, double *stddev)
{
	int i;
	double sum = 0.0;
	double calc = 0.0;
	double mean = dataMean(v, n);

	for (i = 0; i < n; i++){
		calc = v[i] - mean;
		if(calc < CALCERROR)
			calc = 0;
		sum += pow(calc, 2);
	}
	*variance = (double)(sum / (n));
	*stddev = (double)sqrt(*variance);
}

/******************************************************************************/


