/*
 * filter.c
 *
 *  Created on: Dec 26, 2019
 *      Author: hvunt
 */

#include "filter.h"

static const uint8_t sort_frame = 5;

static void sort_insertion(float *input_data, uint32_t length);

void filter_median(float *input_data, uint32_t input_data_length) {
	for (uint32_t i = 0; i < (input_data_length - sort_frame); i +=
			sort_frame) {
		float temp_array[sort_frame];
		for (uint8_t j = 0; j < sort_frame; j++) {
			temp_array[j] = input_data[i + j];
		}
		sort_insertion(temp_array, sort_frame);
		temp_array[sort_frame-1] = temp_array[sort_frame / 2];
		for(int8_t j = 0; j < sort_frame; j++){
			input_data[i+j] = temp_array[j];
		}
	}
}

static void sort_insertion(float *input_data, uint32_t length) {
	for (int i = 0; i < length; i++) {
		/*storing current element whose left side is checked for its
		 correct position .*/

		int temp = input_data[i];
		int j = i;

		/* check whether the adjacent element in left side is greater or
		 less than the current element. */

		while (j > 0 && temp < input_data[j - 1]) {

			// moving the left side element to one position forward.
			input_data[j] = input_data[j - 1];
			j = j - 1;

		}
		// moving current element to its  correct position.
		input_data[j] = temp;
	}

}

void filter(float *input_data, int degree, int length) {
	int x[length], x_mid;
	float Y_rez[length], XTY[degree + 1], x_n[length], X[length][degree + 1],
			XT[degree + 1][length], XT_X[degree + 1][degree + 1], tmpk, tmpi;
//	FILE *fout;
//	fout = fopen("Rez.txt", "a");

	for (uint32_t i = 0; i < length; i++) {
		x[i] = i + 1;
	}

	x_mid = (x[0] + x[length - 1]) / 2;	//middle poin of the first seria, all equal
	for (uint32_t j = 0; j < length; j++) {
		x_n[j] = (float) (x[j] - x_mid);//center number is null	for all serials
		for (uint32_t d = 0; d <= degree; d++) {
			X[j][d] = pow(x_n[j], d);
		}
	}

	//����������� ������������� A=(XT*X)^-1*XT*Y:
	//transp X:
	for (uint32_t i = 0; i < length; i++)
		for (uint32_t j = 0; j <= degree; j++)
			XT[j][i] = X[i][j];
	//XT*Y:
	for (uint32_t i = 0; i <= degree; i++) {
		XTY[i] = 0;
		for (uint32_t j = 0; j < length; j++) {
			XTY[i] += XT[i][j] * input_data[j];
		}
	}

	//XT*X:
	for (uint32_t i = 0; i <= degree; i++) {
		//printf("\n");
		for (uint32_t j = 0; j <= degree; j++) {
			XT_X[i][j] = 0;
			for (uint32_t k = 0; k < length; k++) {
				XT_X[i][j] += XT[i][k] * X[k][j];
			}
		}
	}

	//(XT*X)^-1
	//����� ������
	for (uint32_t k = 0; k <= degree; k++) {	//under diag
		tmpk = XT_X[k][k];
		for (uint32_t i = k + 1; i <= degree; i++) {
			tmpi = XT_X[i][k];
			XTY[i] -= XTY[k] * tmpi / tmpk;
			for (uint32_t j = 0; j <= degree; j++) {
				XT_X[i][j] -= XT_X[k][j] * tmpi / tmpk;
			}
		}
	}
	for (uint32_t i = 0; i <= degree; i++) {	//norm diag
		tmpk = XT_X[i][i];
		XTY[i] /= tmpk;
		for (uint32_t j = 0; j <= degree; j++) {
			XT_X[i][j] /= tmpk;
		}
	}
	for (int32_t k = degree; k > 0; k--) {		//up to diag
		for (int32_t i = k - 1; i >= 0; i--) {
			XTY[i] -= XTY[k] * XT_X[i][k];
			for (int32_t j = 1; j <= degree; j++) {
				XT_X[i][j] -= XT_X[k][j] * XT_X[i][k];
			}
		}
	}

	//X*A:

//		printf("\nY_rez:\t\tY_exp:\n");
	tmpk = 0;
	for (uint32_t i = 0; i < length; i++) {
		//printf("\n");
		Y_rez[i] = 0;
		for (uint32_t j = 0; j <= degree; j++) {
			Y_rez[i] += X[i][j] * XTY[j];		//XTY: Coeff-ts
		}
		if (Y_rez[i] < 0) {
			Y_rez[i] = 0;
		}
		input_data = Y_rez;
//		tmpk += fabs(Y_rez[i] - y[i]);
//			printf("%f\t%f\n", Y_rez[i],  y[i]);
//			fprintf(fout, "%f\n", Y_rez[i]);
	}

//	fclose(fout);
//	return tmpk;
}
