#ifndef FILTER_UTIL
#define FILTER_UTIL

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <dirent.h>

// void addArr(double *arr, double value, int arr_length);
void merge(double *A, int numA, double *B, int numB, double *C);
void mergesort(double *values, int num, double *sorted);
// double median(double *values, int num);
// double average(double *values, int num);
double std_int(int16_t *values, double mean, int num);
int16_t average_int(int16_t *values, int num);
int factorial(int n);
int combo(int n, int r);

#ifdef __cplusplus
extern "C" {
#endif
  void addArr(double *, double, int);
  double median(double *, int);
  double average(double *, int);
#ifdef __cplusplus
}
#endif

#endif
