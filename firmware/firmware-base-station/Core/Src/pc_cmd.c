#include "pc_cmd.h"
#include "usb_util.h"
#include "msg_frame.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static int stricmp_simple(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a;
        char cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return (int)ca - (int)cb;
        a++; b++;
    }
    return (int)(*a) - (int)(*b);
}

static bool parse_u16(const char *s, uint16_t *out)
{
    if (!s || !*s) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || v < 0 || v > 65535) return false;
    *out = (uint16_t)v;
    return true;
}

static bool parse_u8(const char *s, uint8_t *out)
{
    uint16_t v;
    if (!parse_u16(s, &v) || v > 255) return false;
    *out = (uint8_t)v;
    return true;
}

bool pc_cmd_poll(pc_cmd_t *out)
{
    if (!out) return false;

    char line[128];
    if (!usb_read_line(line, sizeof(line))) return false;

    // tokenizar
    char *tok[8] = {0};
    int nt = 0;
    char *p = line;
    while (*p && nt < 8) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[nt++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) { *p = 0; p++; }
    }

    if (nt == 0) return false;

    memset(out, 0, sizeof(*out));

    // STATUS
    if (stricmp_simple(tok[0], "STATUS") == 0) {
        out->type = PC_CMD_STATUS;
        return true;
    }

    // REQ <node>
    if (stricmp_simple(tok[0], "REQ") == 0 && nt >= 2) {
        if (!parse_u8(tok[1], &out->node_id)) {
            usb_println("ERR bad node_id");
            return false;
        }
        out->type = PC_CMD_REQ;
        return true;
    }

    // CFG <node> <CONT|LP> <interval_s> [prewarm_s] [extra_s]
    if (stricmp_simple(tok[0], "CFG") == 0 && nt >= 4) {
        if (!parse_u8(tok[1], &out->node_id)) {
            usb_println("ERR bad node_id");
            return false;
        }

        if (stricmp_simple(tok[2], "CONT") == 0) out->mode = CFG_MODE_CONTINUOUS;
        else if (stricmp_simple(tok[2], "LP") == 0 || stricmp_simple(tok[2], "LOW") == 0) out->mode = CFG_MODE_LOW_POWER;
        else {
            usb_println("ERR mode must be CONT or LP");
            return false;
        }

        if (!parse_u16(tok[3], &out->interval_s)) {
            usb_println("ERR bad interval_s");
            return false;
        }

        out->prewarm_s = 60;
        out->extra_s = 120;
        if (nt >= 5) (void)parse_u8(tok[4], &out->prewarm_s);
        if (nt >= 6) (void)parse_u8(tok[5], &out->extra_s);

        out->type = PC_CMD_CFG;
        return true;
    }

    usb_println("ERR unknown cmd. Use: STATUS | REQ <node> | CFG <node> <CONT|LP> <interval_s> [prewarm] [extra]");
    return false;
}
