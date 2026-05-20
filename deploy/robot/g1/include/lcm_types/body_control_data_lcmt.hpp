#pragma once

#include <lcm/lcm_coretypes.h>

class body_control_data_lcmt
{
public:
    float q[29];
    float qd[29];
    float root_quat_w[4];
    int64_t timestamp_us;

    inline int encode(void* buf, int offset, int maxlen) const;
    inline int getEncodedSize() const;
    inline int decode(const void* buf, int offset, int maxlen);
    inline static int64_t getHash();
    inline static const char* getTypeName();
    inline int _encodeNoHash(void* buf, int offset, int maxlen) const;
    inline int _getEncodedSizeNoHash() const;
    inline int _decodeNoHash(const void* buf, int offset, int maxlen);
    inline static uint64_t _computeHash(const __lcm_hash_ptr* p);
};

int body_control_data_lcmt::encode(void* buf, int offset, int maxlen) const
{
    int pos = 0;
    int64_t hash = getHash();
    int tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = _encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen;
    return pos + tlen;
}

int body_control_data_lcmt::decode(const void* buf, int offset, int maxlen)
{
    int pos = 0;
    int64_t msg_hash;
    int tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (tlen < 0) return tlen;
    pos += tlen;
    if (msg_hash != getHash()) return -1;
    tlen = _decodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen;
    return pos + tlen;
}

int body_control_data_lcmt::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t body_control_data_lcmt::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(nullptr));
    return hash;
}

const char* body_control_data_lcmt::getTypeName()
{
    return "body_control_data_lcmt";
}

int body_control_data_lcmt::_encodeNoHash(void* buf, int offset, int maxlen) const
{
    int pos = 0;
    int tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &q[0], 29);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &qd[0], 29);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &root_quat_w[0], 4);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &timestamp_us, 1);
    if (tlen < 0) return tlen;
    return pos + tlen;
}

int body_control_data_lcmt::_decodeNoHash(const void* buf, int offset, int maxlen)
{
    int pos = 0;
    int tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &q[0], 29);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &qd[0], 29);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &root_quat_w[0], 4);
    if (tlen < 0) return tlen;
    pos += tlen;
    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &timestamp_us, 1);
    if (tlen < 0) return tlen;
    return pos + tlen;
}

int body_control_data_lcmt::_getEncodedSizeNoHash() const
{
    return __float_encoded_array_size(nullptr, 29) +
           __float_encoded_array_size(nullptr, 29) +
           __float_encoded_array_size(nullptr, 4) +
           __int64_t_encoded_array_size(nullptr, 1);
}

uint64_t body_control_data_lcmt::_computeHash(const __lcm_hash_ptr*)
{
    uint64_t hash = 0x4a2d6c91b37f20d5LL;
    return (hash << 1) + ((hash >> 63) & 1);
}
