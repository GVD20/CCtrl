#ifndef RM_CRC_H
#define RM_CRC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t rm_crc8(const uint8_t *data, size_t len);
    uint16_t rm_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
