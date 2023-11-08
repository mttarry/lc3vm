#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "lc3.h"

lc3_byte mem_read(vm_ctx *vm, lc3_addr addr) {
    if (addr == LC3_ADDR_KBSR) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        return select(1, &readfds, NULL, NULL, &timeout) ? LC3_STATUS_BIT : 0;
    }
    else if (addr == LC3_ADDR_KBDR) {
        if (mem_read(vm, LC3_ADDR_KBSR)) {
            return getchar();
        }
        else {
            return 0;
        }
    }
    else if (addr == LC3_ADDR_DSR) {
        return LC3_STATUS_BIT;
    }
    else if (addr == LC3_ADDR_DDR) {
        return 0;
    }

    return vm->mem[addr];
}

void mem_write(vm_ctx *vm, lc3_addr addr, lc3_byte val) {
    if (addr == LC3_ADDR_KBSR || addr == LC3_ADDR_KBDR || addr == LC3_ADDR_DSR)
        return;
    else if (addr == LC3_ADDR_DDR) {
        putchar(val);
        fflush(stdout);
        return;
    }

    vm->mem[addr] = val;
}


uint16_t sextend(uint16_t val, uint16_t n) {
    uint16_t m = 1 << (n - 1);
    val &= ((1 << n) - 1);
    return (val ^ m) - m;
}

void setcc(vm_ctx *vm, uint16_t result) {
    if (result == 0) { 
        vm->reg[LC3_REG_COND] = LC3_CC_ZRO;
    }
    else if (result >> 15) { 
        vm->reg[LC3_REG_COND] = LC3_CC_NEG;
    }
    else {
        vm->reg[LC3_REG_COND] = LC3_CC_POS;
    }
}


void trap(vm_ctx *vm, lc3_byte instr) {
    switch (instr & 0xFF) {
        case LC3_TRAP_GETC: {
            vm->reg[LC3_REG_R0] = getchar();
            setcc(vm, vm->reg[LC3_REG_R0]);

            break;
        }
        case LC3_TRAP_OUT: {
            putchar((char)vm->reg[LC3_REG_R0]);

            break;
        }
        case LC3_TRAP_PUTS: {
            lc3_byte *c = vm->mem + vm->reg[LC3_REG_R0];

            while (*c) {
                putc(*c, stdout);
                ++c;
            }
            fflush(stdout);

            break;
        }
        case LC3_TRAP_IN: {
            printf("Enter a character: ");
            char c = getchar();
            putc(c, stdout);
            vm->reg[LC3_REG_R0] = (lc3_byte)c;
            setcc(vm, vm->reg[LC3_REG_R0]);

            break;
        }
        case LC3_TRAP_PUTSP: {
            lc3_addr *c = vm->mem + vm->reg[LC3_REG_R0];

            while (*c) {
                char c1 = *c & 0xFF;
                putc(c1, stdout);
                
                char c2 = (*c >> 8);
                if (c2) putc(c2, stdout);
                
                ++c;
            }

            fflush(stdout);
            break;
        }
        case LC3_TRAP_HALT: {
            puts("HALT\n");
            fflush(stdout);
            vm->running = false;

            break;
        }
        default:
            break;
    }
}


void execute(vm_ctx *vm) {
    vm->running = true;

    while (vm->running) {
        lc3_byte instr = vm->mem[vm->reg[LC3_REG_PC]++];

        switch (instr >> 12) {
            case LC3_OPCODE_BR: {
                lc3_byte cc = (instr >> 9) & 0x7;
                lc3_byte curr_cc = vm->reg[LC3_REG_COND] & 0x7;
                lc3_addr offset9 = sextend(instr, 9);
                
                if (cc & curr_cc) {
                    vm->reg[LC3_REG_PC] += offset9;
                }

                break;
            }
            case LC3_OPCODE_ADD: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_reg sreg_1 = (instr >> 6) & 0x7;

                if ((instr >> 5) & 0x1) {
                    lc3_byte imm5 = sextend(instr, 5);
                    vm->reg[dreg] = vm->reg[sreg_1] + imm5;
                }
                else {
                    lc3_reg sreg_2 = (instr & 0x7);
                    vm->reg[dreg] = vm->reg[sreg_1] + vm->reg[sreg_2];
                }

                setcc(vm, vm->reg[dreg]);

                break;
            }
            case LC3_OPCODE_LD: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_addr offset9 = sextend(instr, 9);

                vm->reg[dreg] = mem_read(vm, vm->reg[LC3_REG_PC] + offset9);
                setcc(vm, vm->reg[dreg]);
                
                break;
            }
            case LC3_OPCODE_ST: {
                lc3_reg sreg = (instr >> 9) & 0x7;
                lc3_addr offset9 = sextend(instr, 9);

                mem_write(vm, vm->reg[LC3_REG_PC] + offset9, vm->reg[sreg]);
                
                break;
            }
            case LC3_OPCODE_JSR: {
                vm->reg[LC3_REG_R7] = vm->reg[LC3_REG_PC];
    
                if ((instr >> 11) & 0x1) {
                    vm->reg[LC3_REG_PC] += sextend(instr, 11);
                }
                else {
                    lc3_reg baseR = (instr >> 6) & 0x7;
                    vm->reg[LC3_REG_PC] = vm->reg[baseR];
                }

                break;
            }
            case LC3_OPCODE_AND: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_reg sreg_1 = (instr >> 6) & 0x7;

                if ((instr >> 5) & 0x1) {
                    lc3_byte imm5 = sextend(instr, 5);
                    vm->reg[dreg] = vm->reg[sreg_1] & imm5;
                }
                else {
                    lc3_reg sreg_2 = (instr & 0x7);
                    vm->reg[dreg] = vm->reg[sreg_1] & vm->reg[sreg_2];
                }

                setcc(vm, vm->reg[dreg]);

                break;
            }
            case LC3_OPCODE_LDR: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_reg baseR = (instr >> 6) & 0x7;
                lc3_addr offset6 = sextend(instr, 6);

                vm->reg[dreg] = mem_read(vm, vm->reg[baseR] + offset6);
                setcc(vm, vm->reg[dreg]);
    
                break;
            }
            case LC3_OPCODE_STR: {
                lc3_reg sreg = (instr >> 9) & 0x7;
                lc3_reg baseR = (instr >> 6) & 0x7;
                lc3_addr offset6 = sextend(instr, 6);

                mem_write(vm, vm->reg[baseR] + offset6, vm->reg[sreg]);

                break;
            }
            case LC3_OPCODE_RTI:
                break;    
            case LC3_OPCODE_NOT: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_reg sreg = (instr >> 6) & 0x7;

                vm->reg[dreg] = ~vm->reg[sreg];
                setcc(vm, vm->reg[dreg]);

                break;
            }
            case LC3_OPCODE_LDI: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_addr offset9 = sextend(instr, 9);
        
                vm->reg[dreg] = mem_read(vm, mem_read(vm, vm->reg[LC3_REG_PC] + offset9));
                setcc(vm, vm->reg[dreg]);

                break;
            }
            case LC3_OPCODE_STI: {
                lc3_reg sreg = (instr >> 9) & 0x7;
                lc3_addr offset9 = sextend(instr, 9);

                mem_write(vm, mem_read(vm, vm->reg[LC3_REG_PC] + offset9), vm->reg[sreg]);
                
                break;
            }
            case LC3_OPCODE_JMP: {
                lc3_reg baseR = (instr >> 6) & 0x7;
                vm->reg[LC3_REG_PC] = vm->reg[baseR];
                break;
            }
            case LC3_OPCODE_RES:
                break;  
            case LC3_OPCODE_LEA: {
                lc3_reg dreg = (instr >> 9) & 0x7;
                lc3_addr offset9 = sextend(instr, 9);

                vm->reg[dreg] = vm->reg[LC3_REG_PC] + offset9;
                setcc(vm, vm->reg[dreg]);
                break;
            }
            case LC3_OPCODE_TRAP:
                trap(vm, instr);
                break;
            default:
                printf("Error: Invalid opcode. Halting...\n");
                vm->running = false;
                break;
        }
    }
}
