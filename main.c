#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/termios.h>

#define MEMORY_MAX (1 << 16)

enum {
    LC3_STATUS_BIT = (1 << 15)
};

enum {
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,
    R_COND,
    R_COUNT
};

enum
{
    OP_BR = 0, /* branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP    /* execute trap */
};

enum
{
    FL_POS = 1 << 0, /* P */
    FL_ZRO = 1 << 1, /* Z */
    FL_NEG = 1 << 2, /* N */
};

enum 
{
    TRAP_GETC = 0x20,
    TRAP_OUT,
    TRAP_PUTS,
    TRAP_IN,
    TRAP_PUTSP,
    TRAP_HALT
};

enum 
{
    MR_KBSR = 0xFE00,
    MR_KBDR = 0xFE02,
    MR_DSR = 0xFE04,
    MR_DDR = 0xFE06,
    MR_MCR = 0xFFFE
};

typedef struct CPU {
    uint16_t reg[R_COUNT];
    uint16_t mem[MEMORY_MAX];
    uint16_t curr_instr;
    bool running;
} CPU;

static struct termios original_tio;


uint16_t swap16(uint16_t x) {
    return (x >> 8) | (x << 8);
}


uint16_t mem_read(CPU *cpu, uint16_t addr) {
    if (addr == MR_KBSR) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        return select(1, &readfds, NULL, NULL, &timeout) ? LC3_STATUS_BIT : 0;
    }
    else if (addr == MR_KBDR) {
        if (mem_read(cpu, MR_KBSR)) {
            return getchar();
        }
        else {
            return 0;
        }
    }
    else if (addr == MR_DSR) {
        return LC3_STATUS_BIT;
    }
    else if (addr == MR_DDR) {
        return 0;
    }

    return cpu->mem[addr];
}

void mem_write(CPU *cpu, uint16_t addr, uint16_t val) {
    if (addr == MR_KBSR || addr == MR_KBDR || addr == MR_DSR)
        return;
    else if (addr == MR_DDR) {
        putchar(val);
        fflush(stdout);
        return;
    }

    cpu->mem[addr] = val;
}

void read_image(FILE *fp, CPU *cpu) {
    uint16_t origin;
    fread(&origin, sizeof(uint16_t), 1, fp);
    origin = swap16(origin);
    cpu->reg[R_PC] = origin;
    printf(".orig %x\n", cpu->reg[R_PC]);

    uint16_t max_read = MEMORY_MAX - origin;
    uint16_t *load_addr = cpu->mem + origin;
    size_t read = fread(load_addr, sizeof(uint16_t), max_read, fp);

    // Perform swap to little endian
    while (read-- > 0) {
        *load_addr = swap16(*load_addr);
        ++load_addr;
    }
}


uint16_t sextend(uint16_t val, uint16_t n) {
    uint16_t m = 1 << (n - 1);
    val &= ((1 << n) - 1);
    return (val ^ m) - m;
}

void setcc(CPU *cpu, uint16_t result) {
    if (result == 0) {
        cpu->reg[R_COND] = FL_ZRO;
    }
    else if (result >> 15) {
        cpu->reg[R_COND] = FL_NEG;
    }
    else {
        cpu->reg[R_COND] = FL_POS;
    }
}

void add(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t sreg_1 = (cpu->curr_instr >> 6) & 0x7;

    if ((cpu->curr_instr >> 5) & 0x1) {
        uint16_t imm5 = sextend(cpu->curr_instr, 5);
        cpu->reg[dreg] = cpu->reg[sreg_1] + imm5;
    }
    else {
        uint16_t sreg_2 = (cpu->curr_instr & 0x7);
        cpu->reg[dreg] = cpu->reg[sreg_1] + cpu->reg[sreg_2];
    }

    setcc(cpu, cpu->reg[dreg]);
}

void and(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t sreg_1 = (cpu->curr_instr >> 6) & 0x7;

    if ((cpu->curr_instr >> 5) & 0x1) {
        uint16_t imm5 = sextend(cpu->curr_instr, 5);
        cpu->reg[dreg] = cpu->reg[sreg_1] & imm5;
    }
    else {
        uint16_t sreg_2 = (cpu->curr_instr & 0x7);
        cpu->reg[dreg] = cpu->reg[sreg_1] & cpu->reg[sreg_2];
    }

    setcc(cpu, cpu->reg[dreg]);
}

void branch(CPU *cpu) {
    uint16_t cond = cpu->curr_instr >> 9;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    bool test_pos = (cond & 0x1) && (cpu->reg[R_COND] == FL_POS);
    bool test_zro = ((cond >> 1) & 0x1) && (cpu->reg[R_COND] == FL_ZRO);
    bool test_neg = ((cond >> 2) & 0x1) && (cpu->reg[R_COND] == FL_NEG);
    
    if (test_pos || test_zro || test_neg) {
        cpu->reg[R_PC] += offset9;
    }
}

void load(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    cpu->reg[dreg] = mem_read(cpu, cpu->reg[R_PC] + offset9);
    setcc(cpu, cpu->reg[dreg]);
}

void load_indirect(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    cpu->reg[dreg] = mem_read(cpu, mem_read(cpu, cpu->reg[R_PC] + offset9));
    setcc(cpu, cpu->reg[dreg]);
}

void load_boffset(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
    uint16_t offset6 = sextend(cpu->curr_instr, 6);

    cpu->reg[dreg] = mem_read(cpu, cpu->reg[baseR] + offset6);
    setcc(cpu, cpu->reg[dreg]);
}

void not(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t sreg = (cpu->curr_instr >> 6) & 0x7;

    cpu->reg[dreg] = ~cpu->reg[sreg];

    setcc(cpu, cpu->reg[dreg]);
}

void store(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    mem_write(cpu, cpu->reg[R_PC] + offset9, cpu->reg[sreg]);
}

void store_indirect(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    mem_write(cpu, mem_read(cpu, cpu->reg[R_PC] + offset9), cpu->reg[sreg]);
}

void store_boffset(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
    uint16_t offset6 = sextend(cpu->curr_instr, 6);

    mem_write(cpu, cpu->reg[baseR] + offset6, cpu->reg[sreg]);
}

void jump_subroutine(CPU *cpu) {
    cpu->reg[R_R7] = cpu->reg[R_PC];
    
    if ((cpu->curr_instr >> 11) & 0x1) {
        cpu->reg[R_PC] += sextend(cpu->curr_instr, 11);
    }
    else {
        uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
        cpu->reg[R_PC] = cpu->reg[baseR];
    }
}

void jump(CPU *cpu) {
    uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
    cpu->reg[R_PC] = cpu->reg[baseR];
}


void load_effective_addr(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = sextend(cpu->curr_instr, 9);

    cpu->reg[dreg] = cpu->reg[R_PC] + offset9;
    setcc(cpu, cpu->reg[dreg]);
}

void trap_getc(CPU *cpu) {
    cpu->reg[R_R0] = getchar();
    setcc(cpu, cpu->reg[R_R0]);
}

void trap_out(CPU *cpu) {
    putchar((char)cpu->reg[R_R0]);
}

void trap_halt(CPU *cpu) {
    puts("HALT\n");
    fflush(stdout);
    cpu->running = false;
}

void trap_puts(CPU *cpu) {
    uint16_t *c = cpu->mem + cpu->reg[R_R0];

    while (*c) {
        putc(*c, stdout);
        ++c;
    }

    fflush(stdout);
}

void trap_in(CPU *cpu) {
    printf("Enter a character: ");
    char c = getchar();
    putc(c, stdout);
    cpu->reg[R_R0] = (uint16_t)c;
    setcc(cpu, cpu->reg[R_R0]);
}

void trap_putsp(CPU *cpu) {
    uint16_t *c = cpu->mem + cpu->reg[R_R0];

    while (*c) {
        char c1 = *c & 0xFF;
        putc(c1, stdout);
        
        char c2 = (*c >> 8);
        if (c2) putc(c2, stdout);
        
        ++c;
    }

    fflush(stdout);
}

void trap(CPU *cpu) {
    uint16_t trapvec = cpu->curr_instr & 0xFF;

    switch (trapvec) {
        case TRAP_GETC:
            trap_getc(cpu);
            break;
        case TRAP_OUT:
            trap_out(cpu);
            break;
        case TRAP_PUTS:
            trap_puts(cpu);
            break;
        case TRAP_IN:
            trap_in(cpu);
            break;
        case TRAP_PUTSP:
            trap_putsp(cpu);
            break;
        case TRAP_HALT:
            trap_halt(cpu);
            break;
        default:
            break;
    }
}

void print_regs(CPU *cpu) {
    printf("R0: 0x%x\n", cpu->reg[R_R0]);
    printf("R1: 0x%x\n", cpu->reg[R_R1]);
    printf("R2: 0x%x\n", cpu->reg[R_R2]);
    printf("R3: 0x%x\n", cpu->reg[R_R3]);
    printf("R4: 0x%x\n", cpu->reg[R_R4]);
    printf("R5: 0x%x\n", cpu->reg[R_R5]);
    printf("R6: 0x%x\n", cpu->reg[R_R6]);
    printf("R7: 0x%x\n", cpu->reg[R_R7]);
    printf("PC: 0x%x\n", cpu->reg[R_PC]);
    printf("COND: 0x%x\n\n", cpu->reg[R_COND]);
}

void execute(CPU *cpu) {
    cpu->running = true;

    while (cpu->running) {
        // Get next instruction
       // printf("PC: %x\n", cpu->reg[R_PC]);
        cpu->curr_instr = cpu->mem[cpu->reg[R_PC]++];
        uint16_t opcode = cpu->curr_instr >> 12;

        //printf("instruction: %x\n", cpu->curr_instr);
        //print_regs(cpu);

        switch (opcode) {
            case OP_BR:
          //      printf("BR\n");
                branch(cpu);
                break;
            case OP_ADD:
       //         printf("ADD\n");
                add(cpu);
                break;
            case OP_LD:
         //       printf("LOAD\n");
                load(cpu);
                break;     
            case OP_ST:
           //     printf("ST\n");
                store(cpu);
                break;
            case OP_JSR:
         //       printf("JSR\n");
                jump_subroutine(cpu);
                break;
            case OP_AND:
           //     printf("AND\n");
                and(cpu);
                break;   
            case OP_LDR:
             //   printf("LDR\n");
                load_boffset(cpu);
                break;  
            case OP_STR:
            //    printf("STR\n");
                store_boffset(cpu);
                break;    
            case OP_RTI:
                break;    
            case OP_NOT:
              //  printf("NOT\n");
                not(cpu);
                break;   
            case OP_LDI:
               // printf("LDI\n");
                load_indirect(cpu);
                break;   
            case OP_STI:
                //printf("STI\n");
                store_indirect(cpu);
                break;   
            case OP_JMP:
                //printf("JMP\n");
                jump(cpu);
                break;   
            case OP_RES:
                break;  
            case OP_LEA:
               // printf("LEA\n");
                load_effective_addr(cpu);
                break;   
            case OP_TRAP:
              //  printf("TRAP\n");
                trap(cpu);
                break;
            default:
                printf("Error: Invalid opcode. Halting...\n");
                cpu->running = false;
                break;
        }
    }
}

void disable_input_buffering()
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}


void restore_input_buffering()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_signal(int signal)
{
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage ./lc3 [LC3_BINARY_FILE]\n");
        exit(2);
    }

    FILE *f;
    char *filename = argv[1];
    CPU cpu = { 0 };

    f = fopen(filename, "rb");
    if (!f) {
        printf("Error reading file: %s\n", filename);
        exit(EXIT_FAILURE);
    }

    signal(SIGINT, handle_signal);
    disable_input_buffering();

    read_image(f, &cpu);

    execute(&cpu);
    
    restore_input_buffering();
    
    fclose(f);
}


