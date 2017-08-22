
#ifndef __BOOT__
#define __BOOT__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    UNKNOWN = 0xFFFFFFFF,
    XA0n = 0x00000001,
    XB02 = 0x010102FF,
    XC01 = 0x020201FF,
    XC02 = 0x020202FF,
}  pcbid_t;

#ifdef __cplusplus
}
#endif

#endif
