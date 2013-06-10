/**
 * \name stats.h
 *
 * This file implements the statistical analysis and other stuff not directly
 * related to ad5933 chip.
 *
 * Supported OSs: Mac OSx
 *
 * Copyright 2013 David Cemin <davidcemin at gmail dot com>
  
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */
/********************************************************************************/

#include <math.h>
#include <strings.h>

/********************************************************************************/


#define NMEMB(x)   	((sizeof(x))/(sizeof(*x)))

//! below this value I consider that it equals to zero
#define	CALCERROR	0.000001

/****************************************************************************/

/**
 * \brief Calculates max value of a vector
 * \param v pointer to period vector
 * \param n number of members in the vector
 * \param max pointer to max value
 * \return void
 */
void
maxValue(double *v, int n, double *max);

/********************************************************************************/

/**
 * \brief Calculates min value of a vector
 * \param period pointer to vector v
 * \param nmemb number of members in v
 * \param min pointer to min value
 * \return void
 */
void
minValue(double *v, int n, double *min);

/********************************************************************************/

/**
 * \brief  Calculates a vectors mean
 * \param  val Pointer to the vector
 * \param  n nmemb of val
 * \param  mean Pointer to mean result
 * \return Vectors mean
 */
double dataMean(double *val, int n);

/********************************************************************************/

/**
 * \brief  Calculates the variance and standard deviation
 * \param  v pointer to vector v
 * \param  n number of members
 * \param  variance pointer to the variance
 * \param  stddev pointer to standard deviation
 * \return void
 */
void variance_stddev(double *v, int n, double *variance, double *stddev);

/********************************************************************************/

