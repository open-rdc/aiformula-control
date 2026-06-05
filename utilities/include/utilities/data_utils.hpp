#pragma once

#include <cstdint>

/*INT32*/
union Int32_bytes {
    int32_t int_value;
    uint8_t bytes[4];
};
inline int32_t bytes_to_int32(uint8_t *value) {
    Int32_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    tmp.bytes[2]=value[2];
    tmp.bytes[3]=value[3];
    return tmp.int_value;
}
inline void int32_to_bytes(uint8_t *value,int32_t int_value){
    Int32_bytes tmp;
    tmp.int_value=int_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
    value[2]=tmp.bytes[2];
    value[3]=tmp.bytes[3];
}

/*UINT16*/
union UInt16_bytes{
    uint16_t short_value;
    uint8_t bytes[2];
};
inline uint16_t bytes_to_uint16(uint8_t *value) {
    UInt16_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    return tmp.short_value;
}
inline void uint16_to_bytes(uint8_t *value,uint16_t short_value) {
    UInt16_bytes tmp;
    tmp.short_value=short_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
}

/*INT16*/
union Int16_bytes{
    int16_t short_value;
    uint8_t bytes[2];
};
inline int16_t bytes_to_int16(uint8_t *value){
    Int16_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    return tmp.short_value;
}
inline void int16_to_bytes(uint8_t *value,int16_t short_value){
    Int16_bytes tmp;
    tmp.short_value=short_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
}

/*FLOAT*/
union Float_bytes {
    float float_value;
    uint8_t bytes[4];
};
inline float bytes_to_float(uint8_t *value) {
    Float_bytes tmp;
    tmp.bytes[0]=value[0];
    tmp.bytes[1]=value[1];
    tmp.bytes[2]=value[2];
    tmp.bytes[3]=value[3];
    return tmp.float_value;
}
inline void float_to_bytes(uint8_t *value,float float_value) {
    Float_bytes tmp;
    tmp.float_value=float_value;
    value[0]=tmp.bytes[0];
    value[1]=tmp.bytes[1];
    value[2]=tmp.bytes[2];
    value[3]=tmp.bytes[3];
}
