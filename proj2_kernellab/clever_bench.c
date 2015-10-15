/*****************************************************************************
 *
 * M1522.000800
 * SYSTEM PROGRAMMING
 * 
 * Lab2. Kernel Lab
 *
 * benchmark.c
 *  - perform as a normal application.
 *  - should be executed through 'profiler'.
 *  - in clever way.
 *
 *  DON'T MODIFY THIS FILE
 *
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SIZE 800

int main(int argc, char argv[]) {
  int i,j,k;

  int x[SIZE][SIZE] = {0,};
  int y[SIZE][SIZE] = {0,};
  int z[SIZE][SIZE] = {0,};
  
  srand(time(NULL));

  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      x[i][j] = rand() % 1000;
      y[i][j] = rand() % 1000;
    }
  }

  for (i = 0; i < SIZE; i++) {
    for (j = 0; j < SIZE; j++) {
      int sum = 0;
      for (k = 0; k < SIZE; k++) {
        sum += x[i][k] * y[j][k];
      }
      z[i][j] = sum;
    }
  }

  printf("clever done\n");

  return 0;
}
