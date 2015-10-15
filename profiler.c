/*****************************************************************************
 *
 * M1522.000800
 * SYSTEM PROGRAMMING
 * 
 * Lab2. Kernel Lab
 *
 * profiler.c
 *  - fork a binary & profile it.
 *
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <wait.h>

#include "chardev.h"

/*****************************************************************************
 * MsrInOut Command Arrays
 */

struct MsrInOut msr_init[] = {
  { MSR_WRITE, PERF_GLOBAL_CTRL, 0x00, 0x00 },  // disable 4 PMCs & 3 FFCs
  { MSR_WRITE, PMC0, 0x00, 0x00 },              // PMC0 initialization
  { MSR_WRITE, PMC1, 0x00, 0x00 },              // PMC1 initialization
  { MSR_WRITE, PMC2, 0x00, 0x00 },              // PMC2 initialization
  { MSR_WRITE, PMC3, 0x00, 0x00 },              // PMC2 initialization
  { MSR_WRITE, FFC0, 0x00, 0x00 },              // PMC2 initialization
  { MSR_WRITE, FFC1, 0x00, 0x00 },              // PMC2 initialization
  { MSR_WRITE, FFC2, 0x00, 0x00 },              // PMC2 initialization
  { MSR_STOP, 0x00, 0x00 }
};

struct MsrInOut msr_set_start[] = {
  /* make msr commands array to mornitor a process
   * YOUR CODE HERE */
  /* { MSR_WRITE, PERF_EVT_SEL0, 0x004101c2, 0x00}, */
  /* { MSR_WRITE, PERF_EVT_SEL1, 0x0041010e, 0x00}, */
  /* { MSR_WRITE, PERF_EVT_SEL2, 0x01c1010e, 0x00}, */
  { MSR_WRITE, PERF_EVT_SEL0, INST_RETIRED | PERF_EVT_SEL_USR |  PERF_EVT_SEL_OS | PERF_EVT_SEL_EN , 0x00},
  { MSR_WRITE, PERF_EVT_SEL1, RESOURCE_STALLS | PERF_EVT_SEL_USR | PERF_EVT_SEL_OS | PERF_EVT_SEL_EN , 0x00},
  { MSR_WRITE, PERF_EVT_SEL2, CPU_CLK_UNHALTED | PERF_EVT_SEL_USR | PERF_EVT_SEL_OS | PERF_EVT_SEL_EN , 0x00},

  { MSR_WRITE, PERF_EVT_SEL3, 0x004101a2, 0x00},
  { MSR_WRITE, FIXED_CTR_CTRL, 0x222, 0x00},
  { MSR_WRITE, PERF_GLOBAL_CTRL, 0x0f, 0x07},
  { MSR_STOP, 0x00, 0x00 }

};

struct MsrInOut msr_stop_read[] = {
  { MSR_WRITE, PERF_GLOBAL_CTRL, 0x00, 0x00 },  // disable 4 PMCs
  { MSR_WRITE, FIXED_CTR_CTRL, 0x00, 0x00 },    // clean up FFC ctrls
  { MSR_READ, PMC0, 0x00 },
  { MSR_READ, PMC1, 0x00 },
  { MSR_READ, PMC2, 0x00 },
  { MSR_READ, PMC3, 0x00 },
  { MSR_READ, FFC0, 0x00 },
  { MSR_READ, FFC1, 0x00 },
  { MSR_READ, FFC2, 0x00 },
  { MSR_STOP, 0x00, 0x00 }
};

struct MsrInOut msr_rdtsc[] = {
  { MSR_RDTSC, 0x00, 0x00 },
  { MSR_STOP, 0x00, 0x00 }
};

/*****************************************************************************
 * Process tree variable
 */

struct PtreeInfo ptree;

/*****************************************************************************
 * Functions
 */

static int load_device() {
  int fd;
  fd = open("/dev/"DEVICE_NAME, 0);
  if (fd < 0) {
    perror("Failed to open /dev/" DEVICE_NAME);
    exit(1);
  }
  return fd;
}

static void close_device(int fd) {
  int e;
  e = close(fd);
  if (e < 0) {
    perror("Failed to close fd");
    exit(1);
  }
}

void print_header(char *name) {
  printf("-----------------------------------------------------------\n");
  printf("TARGET : %s\n", name);
  printf("\n");
}

void print_ptree(void) {
  printf("- Process Tree Information\n\n");
  /* implement a function to show process tree
   * YOUR CODE HERE */

  printf("pid : %d\n", ptree.pid);
  printf("current process : %s\n", ptree.comm_tree[0]);
  int i;
  for(i=0; i<ptree.height; i++) {
    
    /* printf("%dth parent : %s", i+1, ptree.parent_comm[ptree.height-i-1]); */
    printf("%dth parent : %s\n", i+1, ptree.comm_tree[i]);

    printf("\n");
  }
}

void print_profiling(long long nr_inst, long long st_cy, long long cy) {
  double d_nr_inst, d_st_cy, d_cy;

  printf("- Process Mornitoring Information\n\n");
  printf("inst retired :   %20llu\n", nr_inst);
  printf("stalled cycles : %20llu\n", st_cy);
  printf("core cycles :    %20llu\n\n", cy);

  d_nr_inst = (double) nr_inst;
  d_st_cy = (double) st_cy;
  d_cy = (double) cy;

  printf("stall rate :     %20lf %%\n", 100 * d_st_cy/d_cy);
  printf("throughput :     %20lf inst/cycles\n", d_nr_inst/d_cy);
  printf("-----------------------------------------------------------\n");
}

int getPtree(int fd, int pid) {
  /* implement show process tree using ioctl
   * YOUR CODE HERE */
  ioctl(fd, IOCTL_GET_PTREE, &ptree); 
  return -1;
}

int getProfiling(int fd, char* path) {
  /* implement a function to init & start pmu,
   * get profiling result into array.
   * YOUR CODE HERE */

  pid_t pid;
  int status;

  char* argv[] = {path, NULL};

  /* printf("test fun1\n"); */
  /* printf("test fun2\n"); */
  /* printf("test fun3\n"); */
 


  pid = fork();
  if(pid ==0) {
    ioctl(fd, IOCTL_MSR_CMDS, (long long)msr_init);
    ioctl(fd, IOCTL_MSR_CMDS, (long long)msr_set_start);
    execv(path, argv);
  } else {
    wait(&status); 
    ioctl(fd, IOCTL_MSR_CMDS, (long long)msr_stop_read);

    if (WIFEXITED(status) ){
      printf("Child is normally exited\n");
    } else {
      printf("Something error happend\n");
    }
  }
  return -1;
}

int main(int argc, char* argv[]) {
  int pid;
  int fd;

  if (argc <= 1 || argc > 2) {
    printf("Usage : %s {binary}\n", argv[0]);
    return 0;
  }
  
  fd = load_device();
  
  /* implement a routine to fork & execute a given binary and
   * print process tree & profile it.
   * YOUR CODE HERE */
  getPtree(fd, pid);
  print_ptree();

  char path[20];
  strcpy(path, "./");
  strcat(path, argv[1]);

  getProfiling(fd, path);
  unsigned long long inst = msr_stop_read[2].value;
  unsigned long long stall = msr_stop_read[3].value;
  unsigned long long core = msr_stop_read[4].value;
  printf("inst_retired value : 0x%08x\n", msr_set_start[0].eax);
  printf("resource stall value : 0x%08x\n", msr_set_start[1].eax);
  printf("clk_unhalted value : 0x%08x\n", msr_set_start[2].eax);
  print_profiling(inst, stall, core); 


  close_device(fd);
  return 0;
}
