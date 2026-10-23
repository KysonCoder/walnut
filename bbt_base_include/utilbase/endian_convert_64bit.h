/**
 * @FilePath: endian_convert_64bit.h
 * @Author: AT
 * @Date: 2026-05-30 15:15:32
 * @LastEditTime: 2026-05-30 15:20:45
 * @Description: data endian change
 * @Copyright (c) 2026 by Balance Beam, All Rights Reserved.
 */
#ifndef _ENDIAN_CONVERT_64BIT_H_
#define _ENDIAN_CONVERT_64BIT_H_


#include <cstdint>

class EndianConvert64
{
public:
    static bool isLittleEndian()
    {
        const uint16_t x = 0x0001;
        return *(reinterpret_cast<const uint8_t*>(&x)) == 0x01;
    }

private:
    static uint64_t maskByBits(uint32_t bits)
    {
        if (bits >= 64)
        {
            return 0xFFFFFFFFFFFFFFFFULL;
        }

        return (1ULL << bits) - 1ULL;
    }

    static bool isSupportedBits(uint32_t bits)
    {
        return bits == 16 ||
               bits == 24 ||
               bits == 32 ||
               bits == 40 ||
               bits == 48 ||
               bits == 56 ||
               bits == 64;
    }

public:
    /*
     * 通用字节交换函数
     *
     * bits 支持：
     * 16 / 24 / 32 / 40 / 48 / 56 / 64
     *
     * 例如：
     * 16bit: 0x1234             -> 0x3412
     * 24bit: 0x123456           -> 0x563412
     * 32bit: 0x12345678         -> 0x78563412
     * 40bit: 0x1122334455       -> 0x5544332211
     * 48bit: 0x112233445566     -> 0x665544332211
     * 56bit: 0x11223344556677   -> 0x77665544332211
     * 64bit: 0x1122334455667788 -> 0x8877665544332211
     */
    static uint64_t swapBytes(uint64_t value, uint32_t bits)
    {
        if (!isSupportedBits(bits))
        {
            return value;
        }

        value &= maskByBits(bits);

        const uint32_t bytes = bits / 8;
        uint64_t result = 0;

        for (uint32_t i = 0; i < bytes; ++i)
        {
            result <<= 8;
            result |= (value & 0xFFULL);
            value >>= 8;
        }

        return result;
    }

public:
    static uint64_t swap16(uint64_t value)
    {
        return swapBytes(value, 16);
    }

    static uint64_t swap24(uint64_t value)
    {
        return swapBytes(value, 24);
    }

    static uint64_t swap32(uint64_t value)
    {
        return swapBytes(value, 32);
    }

    static uint64_t swap40(uint64_t value)
    {
        return swapBytes(value, 40);
    }

    static uint64_t swap48(uint64_t value)
    {
        return swapBytes(value, 48);
    }

    static uint64_t swap56(uint64_t value)
    {
        return swapBytes(value, 56);
    }

    static uint64_t swap64(uint64_t value)
    {
        return swapBytes(value, 64);
    }

public:
    /*
     * 主机格式 -> 大端格式
     *
     * 小端CPU：需要字节交换
     * 大端CPU：不需要交换
     */
    static uint64_t toBigEndian(uint64_t value, uint32_t bits)
    {
        if (!isSupportedBits(bits))
        {
            return value;
        }

        value &= maskByBits(bits);

        if (isLittleEndian())
        {
            return swapBytes(value, bits);
        }

        return value;
    }

    /*
     * 大端格式 -> 主机格式
     *
     * 小端CPU：需要字节交换
     * 大端CPU：不需要交换
     */
    static uint64_t fromBigEndian(uint64_t value, uint32_t bits)
    {
        if (!isSupportedBits(bits))
        {
            return value;
        }

        value &= maskByBits(bits);

        if (isLittleEndian())
        {
            return swapBytes(value, bits);
        }

        return value;
    }

public:
    static uint64_t toBigEndian16(uint64_t value)
    {
        return toBigEndian(value, 16);
    }

    static uint64_t toBigEndian24(uint64_t value)
    {
        return toBigEndian(value, 24);
    }

    static uint64_t toBigEndian32(uint64_t value)
    {
        return toBigEndian(value, 32);
    }

    static uint64_t toBigEndian40(uint64_t value)
    {
        return toBigEndian(value, 40);
    }

    static uint64_t toBigEndian48(uint64_t value)
    {
        return toBigEndian(value, 48);
    }

    static uint64_t toBigEndian56(uint64_t value)
    {
        return toBigEndian(value, 56);
    }

    static uint64_t toBigEndian64(uint64_t value)
    {
        return toBigEndian(value, 64);
    }

public:
    static uint64_t fromBigEndian16(uint64_t value)
    {
        return fromBigEndian(value, 16);
    }

    static uint64_t fromBigEndian24(uint64_t value)
    {
        return fromBigEndian(value, 24);
    }

    static uint64_t fromBigEndian32(uint64_t value)
    {
        return fromBigEndian(value, 32);
    }

    static uint64_t fromBigEndian40(uint64_t value)
    {
        return fromBigEndian(value, 40);
    }

    static uint64_t fromBigEndian48(uint64_t value)
    {
        return fromBigEndian(value, 48);
    }

    static uint64_t fromBigEndian56(uint64_t value)
    {
        return fromBigEndian(value, 56);
    }

    static uint64_t fromBigEndian64(uint64_t value)
    {
        return fromBigEndian(value, 64);
    }
};








#endif // _ENDIAN_CONVERT_64BIT_H_
