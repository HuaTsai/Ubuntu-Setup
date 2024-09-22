#include <stdio.h>

__global__ void hello_cuda() {
  printf("Hello World from GPU! %d\n", blockIdx.x * blockDim.x + threadIdx.x);
}

int main() {
  printf("Hello World from CPU!\n");
  hello_cuda<<<2, 2>>>();
  cudaDeviceSynchronize();
}
