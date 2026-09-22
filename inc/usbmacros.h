/****************************************************************************
 * usbmacros.h
 * openacousticdevices.info
 * August 2026
 *****************************************************************************/

#ifndef __USBMACROS_H
#define __USBMACROS_H

/* Mutable string descriptor macro */

#define STATIC_MUTABLE_STRING_DESC(_name, ...)                  \
  SL_PACK_START(1)                                              \
  typedef struct                                                \
  {                                                             \
    uint8_t  len;                                               \
    uint8_t  type;                                              \
    char16_t name[1 + sizeof((char16_t[]){ __VA_ARGS__ }) / 2]; \
  } SL_ATTRIBUTE_PACKED _##_name;                               \
  SL_PACK_END()                                                 \
  SL_ALIGN(4)                                                   \
  SL_PACK_START(1)                                              \
  static _##_name _name SL_ATTRIBUTE_ALIGN(4) =                 \
  {                                                             \
    .len  = sizeof(_##_name) - 2,                               \
    .type = USB_STRING_DESCRIPTOR,                              \
    .name = { __VA_ARGS__ },                                    \
    .name[((sizeof(_##_name) - 2) / 2) - 1] = '\0'              \
  }                                                             \
  SL_PACK_END()
  
#endif /* __USBMACROS_H */
