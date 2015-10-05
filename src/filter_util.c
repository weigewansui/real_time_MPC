#include "../include/filter_util.h"

#define EXTERN

void addArr(double *arr, double value, int arr_length){
  int k = 1;
  for(; k < arr_length; k++){
    arr[k-1] = arr[k];
  }
  arr[arr_length-1] = value;
}

void merge(double *A, int numA, double *B, int numB, double *C){
  int i,j,k;
  i=0; j=0; k=0;
  while(i<numA && j<numB){
    if(A[i] <= B[j]){
      C[k] = A[i];
      i++;
      k++;
    } else{
      C[k] = B[j];
      j++;
      k++;
    }
  }

  //Move other elements into C
  while(i < numA){
    C[k] = A[i];
    i++;
    k++;
  }

  while(j<numB){
    C[k] = B[j];
    j++;
    k++;
  }
}

void mergesort(double *values, int num, double *sorted){
  if(num<2)
    return;

  //sizes of divided arrays
  int n1 = num/2;
  int n2 = num-n1;

  double *A1 = (double *) malloc(sizeof(double)*n1);
  double *A2 = (double *) malloc(sizeof(double)*n2);

  int i=0;
  for(; i<n1; i++){
    A1[i] = values[i];
  }

  for(i=0; i<n2; i++){
    A2[i] = values[i+n1];
  }

  mergesort(A1,n1,sorted);
  mergesort(A2,n2,sorted);

  merge(A1,n1,A2,n2,sorted);

  free(A1);
  free(A2);
}

//Call when getting new signal
double median(double *values, int num){
  double sorted[num];
  mergesort(values, num,sorted);

  if(num%2==1)
    return sorted[num/2];
  else
    return (sorted[num/2] + sorted[num/2-1])/2;
}

double average(double *values, int num){
  //NOTE:Assume index 0 gets shifted out
  double sum = 0;
  int i=0;
  for(; i<num; i++){
    sum += values[i];
  }
  return sum/num;
}

int16_t average_int(int16_t *values, int num){
  int16_t sum = 0;
  int i=0;
  for(; i<num; i++){
    sum += values[i];
  }

  int16_t result = sum/num;
  return result;
}

double std2(double *values, double mean, int num){
  double sum_dev = 0;
  int i=0;
  for (; i<num; i++){
    sum_dev+= (values[i]-mean)*(values[i]-mean);
  }

  return sqrt(sum_dev/num);
}

double std_int(int16_t *values, double mean, int num){
  double sum_dev = 0;
  int i=0;
  for (; i<num; i++){
    sum_dev+= (values[i]-mean)*(values[i]-mean);
  }

  return sqrt(sum_dev/num);
}

int factorial(int n){
  int fact = 1;
  int k=1;
  for(; k<=n; k++){
    fact *= k;
  }
  return fact;
}

int combo(int n, int r){
  return factorial(n)/(factorial(r)*factorial(n-r));
}

