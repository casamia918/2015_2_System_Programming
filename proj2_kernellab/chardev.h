/*****************************************************************************
 *
 * M1522.000800
 * SYSTEM PROGRAMMING
 * 
 * Lab2. Kernel Lab
 *
 * chardev.h
 *  - pmu informations and some struct definitions
 *
*****************************************************************************/

#define MAX_PTREE_SIZE  20
#define MAX_COMM_LENGTH 20

#ifndef _SP_KERNEL_LAB_H
#define _SP_KERNEL_LAB_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define DEVICE_NAME "chardev"
#define MAJOR_NUM 1522
#define MINOR_NUM 0

/* IA-32 Architectural MSRs Address */
#define PERF_GLOBAL_CTRL  0x38f
#define FIXED_CTR_CTRL    0x38d

#define PERF_EVT_SEL0     0x186
#define PERF_EVT_SEL1     0x187
#define PERF_EVT_SEL2     0x188
#define PERF_EVT_SEL3     0x189

#define PMC0              0xc1 // inst_retired
#define PMC1              0xc2 // resource_stalls
#define PMC2              0xc3 // clk_unhalted
#define PMC3              0xc4

#define FFC0              0x309 // instr retired 
#define FFC1              0x30a // core cycles 
#define FFC2              0x30b // ref cycles 

/* PERF_EVT_SEL bit fields */
#define PERF_EVT_SEL_USR  (1 << 16)
#define PERF_EVT_SEL_OS   (1 << 17)
#define PERF_EVT_SEL_EDGE (1 << 18)
#define PERF_EVT_SEL_PC   (1 << 19)
#define PERF_EVT_SEL_INT  (1 << 20)
#define PERF_EVT_SEL_TH   (1 << 21)
#define PERF_EVT_SEL_EN   (1 << 22)
#define PERF_EVT_SEL_INV  (1 << 23)

/* PERF_EVT_SEL event codes [Umask : Event Num] - given for you */
#define INST_RETIRED      0x010e
#define RESOURCE_STALLS   0x1a2
#define CPU_CLK_UNHALTED  0x25c

/* PERF_GLOBAL_CTRL bit fields */
#define PERF_GLOBAL_CTRL_EN_PMC0 (1 << 0) // at [0..31]
#define PERF_GLOBAL_CTRL_EN_PMC1 (1 << 1)
#define PERF_GLOBAL_CTRL_EN_PMC2 (1 << 2)
#define PERF_GLOBAL_CTRL_EN_PMC3 (1 << 3)

#define PERF_GLOBAL_CTRL_EN_FFC0 (1 << 0) // at [32 .. 63]
#define PERF_GLOBAL_CTRL_EN_FFC1 (1 << 1)
#define PERF_GLOBAL_CTRL_EN_FFC2 (1 << 2)

/* FIXED_CTR_CTRL bit fields */
#define FIXED_CTR_CTRL_EN0_OS    (1 << 0)
#define FIXED_CTR_CTRL_EN0_USR   (1 << 1)
#define FIXED_CTR_CTRL_EN0_TH    (1 << 2)
#define FIXED_CTR_CTRL_EN0_PMI   (1 << 3)
#define FIXED_CTR_CTRL_EN1_OS    (1 << 4)
#define FIXED_CTR_CTRL_EN1_USR   (1 << 5)
#define FIXED_CTR_CTRL_EN1_TH    (1 << 6)
#define FIXED_CTR_CTRL_EN1_PMI   (1 << 7)
#define FIXED_CTR_CTRL_EN2_OS    (1 << 8)
#define FIXED_CTR_CTRL_EN2_USR   (1 << 9)
#define FIXED_CTR_CTRL_EN2_TH    (1 << 10)
#define FIXED_CTR_CTRL_EN2_PMI   (1 << 11)

#define MSR_VEC_LIMIT 32

enum MsrOperation {
  MSR_NOP   = 0,
  MSR_READ  = 1,
  MSR_WRITE = 2,
  MSR_STOP  = 3,
  MSR_RDTSC = 4
};

/* reference for making struct PtreeInfo
 * DON'T MODIFY THIS STRUCT */

struct MsrInOut {
  unsigned int op;            // MsrOperation
  unsigned int ecx;           // msr identifier
  union {
    struct {
      unsigned int eax;       // low double word
      unsigned int edx;       // high double word
    };
    unsigned long long value; // quad word
  };
};  

/* declare a struct for communicating with module
 * YOUR CODE HERE */

struct PtreeInfo {
    int pid;
    int height;
    char comm_tree[MAX_PTREE_SIZE][MAX_COMM_LENGTH];
};

/* ioctl options */
#define IOCTL_GET_PTREE _IOWR(MAJOR_NUM, 0, struct PtreeInfo*)
#define IOCTL_MSR_CMDS  _IOWR(MAJOR_NUM, 1, struct MsrInout*) 

#endif /* _SP_KERNEL_LAB_H */
