#ifndef __LC3_H
#define __LC3_H

#include <stdint.h>
#include <stdbool.h>


#define MEMORY_MAX (1 << 16)

enum {
    LC3_STATUS_BIT = (1 << 15)
};

typedef enum {
    LC3_REG_R0 = 0,
    LC3_REG_R1,
    LC3_REG_R2,
    LC3_REG_R3,
    LC3_REG_R4,
    LC3_REG_R5,
    LC3_REG_R6,
    LC3_REG_R7,
    LC3_REG_PC,
    LC3_REG_COND,
    R_COUNT
} lc3_reg;

enum
{
    LC3_OPCODE_BR = 0, /* branch */
    LC3_OPCODE_ADD,    /* add  */
    LC3_OPCODE_LD,     /* load */
    LC3_OPCODE_ST,     /* store */
    LC3_OPCODE_JSR,    /* jump register */
    LC3_OPCODE_AND,    /* bitwise and */
    LC3_OPCODE_LDR,    /* load register */
    LC3_OPCODE_STR,    /* store register */
    LC3_OPCODE_RTI,    /* unused */
    LC3_OPCODE_NOT,    /* bitwise not */
    LC3_OPCODE_LDI,    /* load indirect */
    LC3_OPCODE_STI,    /* store indirect */
    LC3_OPCODE_JMP,    /* jump */
    LC3_OPCODE_RES,    /* reserved (unused) */
    LC3_OPCODE_LEA,    /* load effective address */
    LC3_OPCODE_TRAP    /* execute trap */
};

enum
{
    LC3_CC_POS = 1 << 0, /* P */
    LC3_CC_ZRO = 1 << 1, /* Z */
    LC3_CC_NEG = 1 << 2, /* N */
};

enum 
{
    LC3_TRAP_GETC = 0x20,
    LC3_TRAP_OUT,
    LC3_TRAP_PUTS,
    LC3_TRAP_IN,
    LC3_TRAP_PUTSP,
    LC3_TRAP_HALT
};

enum 
{
    LC3_ADDR_KBSR = 0xFE00,
    LC3_ADDR_KBDR = 0xFE02,
    LC3_ADDR_DSR = 0xFE04,
    LC3_ADDR_DDR = 0xFE06,
    LC3_ADDR_MCR = 0xFFFE
};

typedef uint16_t lc3_addr;
typedef uint16_t lc3_byte;

typedef struct vm_ctx {
    lc3_byte reg[R_COUNT];
    lc3_byte mem[MEMORY_MAX];
    bool running;
} vm_ctx;




void execute(vm_ctx *vm);

#endif