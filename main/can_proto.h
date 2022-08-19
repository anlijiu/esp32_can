#ifndef CAN_PROTO_H__
#define CAN_PROTO_H__

#include <stdbool.h>
#include <string.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CDCResponse {
  uint32_t length;
  char *response;
} cdc_response;

void init_can();

#ifdef __cplusplus
}
#endif

#endif // CAN_PROTO_H__
