#ifndef PART2_COMMANDS_H
#define PART2_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    int16_t a;
    int16_t b;
    char kind;
} Cmd_t;

/* GOTO arguments are metres in ASCII and returned as signed millimetres. */
bool ParseVehicleCommand(const char *text, Cmd_t *command);

#endif
