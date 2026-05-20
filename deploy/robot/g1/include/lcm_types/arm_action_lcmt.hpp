#pragma once

#include <lcm/lcm_coretypes.h>

class arm_action_lcmt
{
public:
    double act[14];

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

int arm_action_lcmt::encode(void* buf, int offset, int maxlen) const
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

int arm_action_lcmt::decode(const void* buf, int offset, int maxlen)
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

int arm_action_lcmt::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t arm_action_lcmt::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(nullptr));
    return hash;
}

const char* arm_action_lcmt::getTypeName()
{
    return "arm_action_lcmt";
}

int arm_action_lcmt::_encodeNoHash(void* buf, int offset, int maxlen) const
{
    return __double_encode_array(buf, offset, maxlen, &act[0], 14);
}

int arm_action_lcmt::_decodeNoHash(const void* buf, int offset, int maxlen)
{
    return __double_decode_array(buf, offset, maxlen, &act[0], 14);
}

int arm_action_lcmt::_getEncodedSizeNoHash() const
{
    return __double_encoded_array_size(nullptr, 14);
}

uint64_t arm_action_lcmt::_computeHash(const __lcm_hash_ptr*)
{
    uint64_t hash = 0x692f2be95563f8feLL;
    return (hash << 1) + ((hash >> 63) & 1);
}
