#include "commands.h"
#include <string.h>

static const char *skip_space(const char *text)
{
    while (*text == ' ' || *text == '\t') ++text;
    return text;
}

static bool digit(char c)
{
    return c >= '0' && c <= '9';
}

static const char *read_number(const char *text, unsigned maximum, unsigned decimals, int16_t *out)
{
    const bool negative = *text == '-';
    if (negative || *text == '+') {
        if (decimals == 0U) return NULL;
        ++text;
    }
    unsigned whole = 0U, fraction = 0U, places = 0U;
    bool seen = false;
    for (; digit(*text); ++text) {
        whole = whole * 10U + (unsigned)(*text - '0');
        if (whole > maximum) return NULL;
        seen = true;
    }
    if (*text == '.' && decimals != 0U) {
        for (++text; digit(*text); ++text) {
            if (++places > decimals) return NULL;
            fraction = fraction * 10U + (unsigned)(*text - '0');
            seen = true;
        }
    }
    if (!seen || (whole == maximum && fraction != 0U)) return NULL;
    for (unsigned i = 0U; i < decimals; ++i) whole *= 10U;
    for (; places < decimals; ++places) fraction *= 10U;
    const int32_t value = (int32_t)(whole + fraction);
    *out = (int16_t)(negative ? -value : value);
    return text;
}

bool ParseVehicleCommand(const char *text, Cmd_t *command)
{
    Cmd_t parsed = {0};
    text = skip_space(text);
    if (*text == '\0') return false;
    if ((strncmp(text, "STOP", 4U) == 0 && *skip_space(text + 4) == '\0') ||
        (*text == 'S' && *skip_space(text + 1) == '\0')) {
        parsed.kind = 'S';
    } else if (strncmp(text, "GOTO", 4U) == 0 && skip_space(text + 4) != text + 4) {
        const char *end = read_number(skip_space(text + 4), 3U, 3U, &parsed.a);
        if (end == NULL || skip_space(end) == end) return false;
        end = read_number(skip_space(end), 3U, 3U, &parsed.b);
        if (end == NULL || *skip_space(end) != '\0') return false;
        parsed.kind = 'G';
    } else {
        parsed.kind = *text;
        const bool straight = *text == 'F' || *text == 'B';
        if (!straight && *text != 'L' && *text != 'R') return false;
        const char *end = read_number(skip_space(text + 1), straight ? 3000U : 360U, 0U, &parsed.a);
        if (!end || *skip_space(end) != '\0' || parsed.a == 0) return false;
    }
    *command = parsed;
    return true;
}
