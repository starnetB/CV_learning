#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <sys/time.h>
#include <stdio.h>
#include <math.h>

#define Row  1024
#define Col 1024

__global__ void addKernel(int **C,  int **A, int ** B)
{
    int idx = threadIdx.x + blockDim.x * blockIdx.x; //行
    int idy = threadIdx.y + blockDim.y * blockIdx.y;
    if (idx < Col && idy < Row) {   //注意行和列没有逻辑关系
        C[idy][idx] = A[idy][idx] + B[idy][idx];
    }
}

int main()
{
    struct timeval start, end;
    gettimeofday( &start, NULL );
    int **A = (int **)malloc(sizeof(int*) * Row);
    int **B = (int **)malloc(sizeof(int*) * Row);
    int **C = (int **)malloc(sizeof(int*) * Row);
    int *dataA = (int *)malloc(sizeof(int) * Row * Col);
    int *dataB = (int *)malloc(sizeof(int) * Row * Col);
    int *dataC = (int *)malloc(sizeof(int) * Row * Col);

    int **d_A;
    int **d_B;
    int **d_C;
    int *d_dataA;
    int *d_dataB;
    int *d_dataC;

    //malloc device_memory
}