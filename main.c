#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MEMORY_MAX (1 << 16)

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


typedef struct CPU {
    uint16_t reg[R_COUNT];
    uint16_t mem[MEMORY_MAX];
    uint16_t curr_instr;
    bool running;
} CPU;



uint16_t swap16(uint16_t x) {
    return (x >> 8) | (x << 8);
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

uint16_t sign_extend(uint16_t x, int bit_count) {
    if ((x >> (bit_count - 1)) & 0x1) {
        x |= (0xFFFF << bit_count);
    }

    return x;
}

void update_flags(CPU *cpu, uint16_t result) {
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
        uint16_t imm5 = cpu->curr_instr & 0x1F;
        cpu->reg[dreg] = cpu->reg[sreg_1] + sign_extend(imm5, 5);
    }
    else {
        uint16_t sreg_2 = (cpu->curr_instr & 0x7);
        cpu->reg[dreg] = cpu->reg[sreg_1] + cpu->reg[sreg_2];
    }

    update_flags(cpu, cpu->reg[dreg]);
}

void and(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t sreg_1 = (cpu->curr_instr >> 6) & 0x7;

    if ((cpu->curr_instr >> 5) & 0x1) {
        uint16_t imm5 = cpu->curr_instr & 0x1F;
        cpu->reg[dreg] = cpu->reg[sreg_1] & sign_extend(imm5, 5);
    }
    else {
        uint16_t sreg_2 = (cpu->curr_instr & 0x7);
        cpu->reg[dreg] = cpu->reg[sreg_1] & cpu->reg[sreg_2];
    }

    update_flags(cpu, cpu->reg[dreg]);
}

void branch(CPU *cpu) {
    uint16_t cond = cpu->curr_instr >> 9;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;

    bool test_pos = (cond & 0x1) && (cpu->reg[R_COND] == FL_POS);
    bool test_zro = ((cond >> 1) & 0x1) && (cpu->reg[R_COND] == FL_ZRO);
    bool test_neg = ((cond >> 2) & 0x1) && (cpu->reg[R_COND] == FL_NEG);
    
    if (test_pos || test_zro || test_neg) {
        cpu->reg[R_PC] += sign_extend(offset9, 9);
    }
}

void load(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;
    uint16_t load_addr = cpu->reg[R_PC] + sign_extend(offset9, 9);

    cpu->reg[dreg] = cpu->mem[load_addr];

    update_flags(cpu, cpu->reg[dreg]);
}

void load_indirect(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;
    uint16_t load_addr = cpu->mem[cpu->reg[R_PC] + sign_extend(offset9, 9)];

    cpu->reg[dreg] = cpu->mem[load_addr];

    update_flags(cpu, cpu->reg[dreg]);
}

void not(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t sreg = (cpu->curr_instr >> 6) & 0x7;

    cpu->reg[dreg] = ~cpu->reg[sreg];

    update_flags(cpu, cpu->reg[dreg]);
}

void store(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;
    uint16_t store_addr = cpu->reg[R_PC] + sign_extend(offset9, 9);

    cpu->mem[store_addr] = cpu->reg[sreg];
}

void store_indirect(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;
    uint16_t store_addr = cpu->mem[cpu->reg[R_PC] + sign_extend(offset9, 9)];
    
    cpu->mem[store_addr] = cpu->reg[sreg];
}

void load_boffset(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
    uint16_t offset6 = cpu->curr_instr & 0x3F;

    cpu->reg[dreg] = cpu->mem[cpu->reg[baseR] + sign_extend(offset6, 6)];

    update_flags(cpu, cpu->reg[dreg]);
}

void jump_subroutine(CPU *cpu) {
    cpu->reg[R_R7] = cpu->reg[R_PC];
    
    if ((cpu->curr_instr >> 11) & 0x1) {
        uint16_t offset11 = cpu->curr_instr & 0x7FF;
        cpu->reg[R_PC] += sign_extend(offset11, 11);
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

void store_boffset(CPU *cpu) {
    uint16_t sreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t baseR = (cpu->curr_instr >> 6) & 0x7;
    uint16_t offset6 = cpu->curr_instr & 0x3F;
    uint16_t store_addr = cpu->reg[baseR] + sign_extend(offset6, 6);

    cpu->mem[store_addr] = cpu->reg[sreg];
}

void load_effective_addr(CPU *cpu) {
    uint16_t dreg = (cpu->curr_instr >> 9) & 0x7;
    uint16_t offset9 = cpu->curr_instr & 0x1FF;

    cpu->reg[dreg] = cpu->reg[R_PC] + sign_extend(offset9, 9);

    update_flags(cpu, cpu->reg[dreg]);
}

void trap_getc(CPU *cpu) {
    cpu->reg[R_R0] = getchar();
    update_flags(cpu, cpu->reg[R_R0]);
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
    update_flags(cpu, cpu->reg[R_R0]);
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
        printf("PC: %x\n", cpu->reg[R_PC]);
        cpu->curr_instr = cpu->mem[cpu->reg[R_PC]++];
        uint16_t opcode = cpu->curr_instr >> 12;

        printf("instruction: %x\n", cpu->curr_instr);
        print_regs(cpu);

        switch (opcode) {
            case OP_BR:
                printf("BR\n");
                branch(cpu);
                break;
            case OP_ADD:
                printf("ADD\n");
                add(cpu);
                break;
            case OP_LD:
                printf("LOAD\n");
                load(cpu);
                break;     
            case OP_ST:
                printf("ST\n");
                store(cpu);
                break;
            case OP_JSR:
                printf("JSR\n");
                jump_subroutine(cpu);
                break;
            case OP_AND:
                printf("AND\n");
                and(cpu);
                break;   
            case OP_LDR:
                printf("LDR\n");
                load_boffset(cpu);
                break;  
            case OP_STR:
                printf("STR\n");
                store_boffset(cpu);
                break;    
            case OP_RTI:
                break;    
            case OP_NOT:
                printf("NOT\n");
                not(cpu);
                break;   
            case OP_LDI:
                printf("LDI\n");
                load_indirect(cpu);
                break;   
            case OP_STI:
                printf("STI\n");
                store_indirect(cpu);
                break;   
            case OP_JMP:
                printf("JMP\n");
                jump(cpu);
                break;   
            case OP_RES:
                break;  
            case OP_LEA:
                printf("LEA\n");
                load_effective_addr(cpu);
                break;   
            case OP_TRAP:
                printf("TRAP\n");
                trap(cpu);
                break;
            default:
                printf("Error: Invalid opcode. Halting...\n");
                cpu->running = false;
                break;
        }
    }
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

    read_image(f, &cpu);

    execute(&cpu);

    fclose(f);
}


