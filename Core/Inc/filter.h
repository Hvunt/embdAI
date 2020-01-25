/*
 * filter.h
 *
 *  Created on: Dec 26, 2019
 *      Author: hvunt
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "main.h"
#include <math.h>



void filter_median(float *input_data, uint32_t intput_data_length);
void filter(float *y, int degree, int length);

#endif /* INC_FILTER_H_ */
