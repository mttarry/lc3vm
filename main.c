#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/termios.h>

#include "lc3.h"

static struct termios original_tio;

uint16_t swap16(uint16_t x) {
    return (x >> 8) | (x << 8);
}

void read_image(FILE *fp, vm_ctx *vm) {
    uint16_t origin;
    fread(&origin, sizeof(uint16_t), 1, fp);
    origin = swap16(origin);
    vm->reg[LC3_REG_PC] = origin;

    uint16_t max_read = MEMORY_MAX - origin;
    uint16_t *load_addr = vm->mem + origin;
    size_t read = fread(load_addr, sizeof(uint16_t), max_read, fp);

    // Perform swap to little endian
    while (read-- > 0) {
        *load_addr = swap16(*load_addr);
        ++load_addr;
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
    vm_ctx vm = { 0 };

    f = fopen(filename, "rb");
    if (!f) {
        printf("Error reading file: %s\n", filename);
        exit(EXIT_FAILURE);
    }

    signal(SIGINT, handle_signal);
    disable_input_buffering();

    read_image(f, &vm);

    execute(&vm);
    
    restore_input_buffering();
    
    fclose(f);
}


