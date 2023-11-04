#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

// 65536 memory locations
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


typedef struct CPU {
    uint16_t reg[R_COUNT];
    uint16_t mem[MEMORY_MAX];
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

void add(CPU *cpu, uint16_t instr) {
    uint16_t dreg = (instr >> 9) & 0x3;
    uint16_t sreg_1 = (instr >> 6) & 0x3;

    if ((instr >> 5) & 0x1) {
        uint16_t imm5 = instr & 0x1F;
        cpu->reg[dreg] = cpu->reg[sreg_1] + sign_extend(imm5, 5);
    }
    else {
        uint16_t sreg_2 = (instr & 0x3);
        cpu->reg[dreg] = cpu->reg[sreg_1] + cpu->reg[sreg_2];
    }

    update_flags(cpu, cpu->reg[dreg]);
}

void and(CPU *cpu, uint16_t instr) {
    uint16_t dreg = (instr >> 9) & 0x3;
    uint16_t sreg_1 = (instr >> 6) & 0x3;

    if ((instr >> 5) & 0x1) {
        uint16_t imm5 = instr & 0x1F;
        cpu->reg[dreg] = cpu->reg[sreg_1] & sign_extend(imm5, 5);
    }
    else {
        uint16_t sreg_2 = (instr & 0x3);
        cpu->reg[dreg] = cpu->reg[sreg_1] & cpu->reg[sreg_2];
    }

    update_flags(cpu, cpu->reg[dreg]);
}

void branch(CPU *cpu, uint16_t instr) {
    uint16_t cond = instr >> 9;
    uint16_t offset = instr & 0x1FF;

    bool test_pos = (cond & 0x1) && (cpu->reg[R_COND] == FL_POS);
    bool test_zro = ((cond >> 1) & 0x1) && (cpu->reg[R_COND] == FL_ZRO);
    bool test_neg = ((cond >> 2) & 0x1) && (cpu->reg[R_COND] == FL_NEG);
    
    if (test_pos || test_zro || test_neg) {
        cpu->reg[R_PC] += sign_extend(offset, 9);
    }
}

void load(CPU *cpu, uint16_t instr) {
    uint16_t dreg = (instr >> 9) & 0x3;
    uint16_t offset = instr & 0x1F;
    uint16_t load_addr = cpu->reg[R_PC] + sign_extend(offset, 9);

    cpu->reg[dreg] = cpu->mem[load_addr];

    update_flags(cpu, cpu->reg[dreg]);
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
    int running = 1;

    while (running) {
        // Get next instruction
        printf("PC: %x\n", cpu->reg[R_PC]);
        uint16_t instr = cpu->mem[cpu->reg[R_PC]++];
        uint16_t opcode = instr >> 12;

        printf("instruction: %x\n", instr);
        print_regs(cpu);
        
        if (instr == 0) break;

        switch (opcode) {
            case OP_BR:
                printf("BR\n");
                branch(cpu, instr);
                break;
            case OP_ADD:
                printf("ADD\n");
                add(cpu, instr);
                break;
            case OP_LD:
                load(cpu, instr);
                break;     
            case OP_ST:
                break;     
            case OP_JSR:
                break;
            case OP_AND:
                printf("AND\n");
                and(cpu, instr);
                break;   
            case OP_LDR:
                break;  
            case OP_STR:
                break;    
            case OP_RTI:
                break;    
            case OP_NOT:
                break;   
            case OP_LDI:
                break;   
            case OP_STI:
                break;   
            case OP_JMP:
                break;   
            case OP_RES:
                break;  
            case OP_LEA:
                break;   
            case OP_TRAP:
                break;
            default:
                running = 0;
                break;
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage ./lc3 [LC3_BINARY_FILE]\n");
        exit(EXIT_FAILURE);
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


