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
    cudaMalloc((void**)&d_A, sizeof(int *) * Row);
    cudaMalloc((void**)&d_B, sizeof(int *) * Row);
    cudaMalloc((void**)&d_C, sizeof(int *) * Row);
    cudaMalloc((void**)&d_dataA, sizeof(int)*Row*Col);
    cudaMalloc((void**)&d_dataB, sizeof(int)*Row*Col);
    cudaMalloc((void**)&d_dataC, sizeof(int)*Row*Col);

    //set value
    for (int i = 0; i < Row*Col; i++) {
        dataA[i] = 90;
        dataB[i] = 10;
    }

    //将主机指针A指向设备数据位置，目的是让设备二级指针能够指向设备数据一级指针
    //A 和  dataA 都传到了设备上，但是二者还没有建立对应关系
    for (int i = 0; i < Row; i++) {
        A[i] = d_dataA + Col * i;
        B[i] = d_dataB + Col * i;
        C[i] = d_dataC + Col * i;
    }

    cudaMemcpy(d_A, A, sizeof(int*) * Row, cudaMemcpyHostToDevice);
    cudaMemcpy(d_B, B, sizeof(int*) * Row, cudaMemcpyHostToDevice);
    cudaMemcpy(d_C, C, sizeof(int*) * Row, cudaMemcpyHostToDevice);
    cudaMemcpy(d_dataA, dataA, sizeof(int) * Row * Col, cudaMemcpyHostToDevice);
    cudaMemcpy(d_dataB, dataB, sizeof(int) * Row * Col, cudaMemcpyHostToDevice);
    

    dim3 threadPerBlock(16, 16);
    dim3 blockNumber((Col + threadPerBlock.x - 1)/threadPerBlock.x, (Row +threadPerBlock.y - 1) / threadPerBlock.y );
    addKernel << <blockNumber, threadPerBlock >> > (d_C, d_A, d_B);

    //拷贝计算数据-一级数据指针
    cudaMemcpy(dataC, d_dataC, sizeof(int) * Row * Col, cudaMemcpyDeviceToHost);

    int max_error = 0;
    for(int i=0;i<Row*Col;i++)
    {
        //printf("%d\n", dataC[i]);
        max_error += abs(100-dataC[i]);
    }

    //释放内存
    free(A);
    free(B);
    free(C);
    free(dataA);
    free(dataB);
    free(dataC);

    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);
    cudaFree(d_dataA);
    cudaFree(d_dataB);
    cudaFree(d_dataC);
    printf("max_error is %d\n", max_error);
    gettimeofday( &end, NULL );
    int timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
    printf("total time is %d ms\n", timeuse/1000);
    return 0;
}
