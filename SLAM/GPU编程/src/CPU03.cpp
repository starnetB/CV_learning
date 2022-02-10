#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <math.h>

#define ROWS 1024
#define COLS 1024

using namespace std;

int main()
{
    struct timeval start, end;
    gettimeofday( &start, NULL );
    int *A, **A_ptr, *B, **B_ptr, *C, **C_ptr;
    int total_size = ROWS*COLS*sizeof(int);
    A = (int*)malloc(total_size);
    B = (int*)malloc(total_size);
    C = (int*)malloc(total_size);
    A_ptr = (int**)malloc(ROWS*sizeof(int*));
    B_ptr = (int**)malloc(ROWS*sizeof(int*));
    C_ptr = (int**)malloc(ROWS*sizeof(int*));

    //CPU一维数组初始化
    for(int i=0;i<ROWS*COLS;i++)
    {
        A[i] = 80;
        B[i] = 20;
    }

    for(int i=0;i<ROWS;i++)
    {
        A_ptr[i] = A + COLS*i;
        B_ptr[i] = B + COLS*i;
        C_ptr[i] = C + COLS*i;
    }

    for(int i=0;i<ROWS;i++)
        for(int j=0;j<COLS;j++)
        {
            C_ptr[i][j] = A_ptr[i][j] + B_ptr[i][j];
        }
    //检查结果
    int max_error = 0;
    for(int i=0;i<ROWS*COLS;i++)
    {
        //cout << C[i] << endl;
        max_error += abs(100-C[i]);
    }
    cout << "max_error is " << max_error <<endl;     
    gettimeofday( &end, NULL );
    int timeuse = 1000000 * ( end.tv_sec - start.tv_sec ) + end.tv_usec - start.tv_usec;
    cout << "total time is " << timeuse/1000 << "ms" <<endl;
    return 0;
}

