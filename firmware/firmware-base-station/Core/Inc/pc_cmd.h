#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PC_CMD_NONE = 0,
    PC_CMD_CFG,
    PC_CMD_REQ,
    PC_CMD_STATUS,
} pc_cmd_type_t;

typedef struct {
    pc_cmd_type_t type;
    uint8_t node_id;
    // CFG
    uint8_t mode;          // CFG_MODE_*
    uint16_t interval_s;
    uint8_t prewarm_s;
    uint8_t extra_s;
} pc_cmd_t;

// Llama a usb_read_line() y parsea una línea.
// Devuelve true si obtuvo un comando.
bool pc_cmd_poll(pc_cmd_t *out);

#ifdef __cplusplus
}
#endif
