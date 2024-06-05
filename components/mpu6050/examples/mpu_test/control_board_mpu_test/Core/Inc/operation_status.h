#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t status_t;

#define STATUS_OK                      0
#define STATUS_FAIL                    -1

#define STATUS_ERR_NO_MEM              0x101
#define STATUS_ERR_INVALID_ARG         0x102
#define STATUS_ERR_INVALID_STATE       0x103
#define STATUS_ERR_INVALID_SIZE        0x104
#define STATUS_ERR_NOT_FOUND           0x105
#define STATUS_ERR_NOT_SUPPORTED       0x106
#define STATUS_ERR_TIMEOUT             0x107
#define STATUS_ERR_INVALID_RESPONSE    0x108
#define STATUS_ERR_INVALID_CRC         0x109
#define STATUS_ERR_INVALID_VERSION     0x10A


#ifdef __cplusplus
}
#endif