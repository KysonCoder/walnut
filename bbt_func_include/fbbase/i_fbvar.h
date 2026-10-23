/**
 * @FilePath: i_fbvar.h
 * @Author: AT
 * @Date: 2025-10-02 16:09:05
 * @LastEditTime: 2025-11-09 17:26:25
 * @Description: The class define all common interface/method for each field
 * bus variants.
 *
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */

#ifndef _I_FBVAR_H_
#define _I_FBVAR_H_
#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <json.h>
#include <string>
#include <stdexcept>
#include <stdint.h>
#include <stddef.h>


#include "fbbase_log.h"

namespace FBBASE {

namespace VAR {

// 定义变量数据类型的枚举
enum class FBVarDataType {
  FBVAR_DATATYPE_INVALID = 0, // 无效数据类型
  FBVAR_DATATYPE_BOOL = 1,    // 布尔类型
  FBVAR_DATATYPE_INT8 = 2,    // 8位有符号整数
  FBVAR_DATATYPE_UINT8 = 3,   // 8位无符号整数
  FBVAR_DATATYPE_INT16 = 4,   // 16位有符号整数
  FBVAR_DATATYPE_UINT16 = 5,  // 16位无符号整数
  FBVAR_DATATYPE_INT32 = 6,   // 32位有符号整数
  FBVAR_DATATYPE_UINT32 = 7,  // 32位无符号整数
  FBVAR_DATATYPE_INT64 = 8,   // 64位有符号整数
  FBVAR_DATATYPE_UINT64 = 9,  // 64位无符号整数
  FBVAR_DATATYPE_FLOAT = 10,  // 单精度浮点数
  FBVAR_DATATYPE_DOUBLE = 11, // 双精度浮点数
  FBVAR_DATATYPE_MAXSIZE = 12 // 最大数据类型数量
};

// 定义现场总线类型的枚举
enum class FBVarBusType {
  FBVAR_BUSTYPE_INVALID = 0,    // 无效总线类型
  FBVAR_BUSTYPE_VIRTUAL = 1,    // 虚拟总线
  FBVAR_BUSTYPE_ETHERCAT = 2,   // EtherCAT总线, Master
  FBVAR_BUSTYPE_CANOPEN = 3,    // CANOpen总线, Master
  FBVAR_BUSTYPE_MODBUSTCP = 4,  // Modbus TCP总线,
  FBVAR_BUSTYPE_MODBUSRTU = 5,  // Modbus RTU总线
  FBVAR_BUSTYPE_ETHERNETIP = 6, // EtherNet/IP总线
  FBVAR_BUSTYPE_PROFINET = 7,   // Profinet总线
  FBVAR_BUSTYPE_MAXSIZE = 8     // 最大总线类型数量
};

// 定义变量值类型，联合体
typedef union _FBVarValue {
  bool bValue;
  char cValue;
  unsigned char ucValue;
  short sValue;
  unsigned short usValue;
  int iValue;
  unsigned int uiValue;
  long long llValue;
  unsigned long long ullValue;
  float fValue;
  double dValue;
  unsigned long long bitArray64;     // 新增：64个bit，bit0 ~ bit63
  char padChars[16];
} FBVarData;

// 定义变量句柄
typedef void *FBVAR_HANDLE;

class IFbvar {
public:
  // 静态工厂方法
  static IFbvar *getBusInstance();

  static FBVarDataType getDataType(const std::string &dataType) {
    std::string lowerType = dataType;
    std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(),
                   ::tolower);

    if (lowerType == "bool")
      return FBVarDataType::FBVAR_DATATYPE_BOOL;
    if (lowerType == "char")
      return FBVarDataType::FBVAR_DATATYPE_INT8;
    if (lowerType == "uchar")
      return FBVarDataType::FBVAR_DATATYPE_UINT8;
    if (lowerType == "short")
      return FBVarDataType::FBVAR_DATATYPE_INT16;
    if (lowerType == "ushort")
      return FBVarDataType::FBVAR_DATATYPE_UINT16;
    if (lowerType == "int")
      return FBVarDataType::FBVAR_DATATYPE_INT32;
    if (lowerType == "uint")
      return FBVarDataType::FBVAR_DATATYPE_UINT32;
    if (lowerType == "longlong")
      return FBVarDataType::FBVAR_DATATYPE_INT64;
    if (lowerType == "ulonglong")
      return FBVarDataType::FBVAR_DATATYPE_UINT64;
    if (lowerType == "float")
      return FBVarDataType::FBVAR_DATATYPE_FLOAT;
    if (lowerType == "double")
      return FBVarDataType::FBVAR_DATATYPE_DOUBLE;

    return FBVarDataType::FBVAR_DATATYPE_INVALID;
  }

  static FBVarBusType getBusType(const std::string &busType) {
    std::string lowerType = busType;
    std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(),
                   ::tolower);

    if (lowerType == "virtual")
      return FBVarBusType::FBVAR_BUSTYPE_VIRTUAL;
    if (lowerType == "ethercat")
      return FBVarBusType::FBVAR_BUSTYPE_ETHERCAT;
    if (lowerType == "canopen")
      return FBVarBusType::FBVAR_BUSTYPE_CANOPEN;
    if (lowerType == "modbustcp")
      return FBVarBusType::FBVAR_BUSTYPE_MODBUSTCP;
    if (lowerType == "modbusrtu")
      return FBVarBusType::FBVAR_BUSTYPE_MODBUSRTU;
    if (lowerType == "ethernetip")
      return FBVarBusType::FBVAR_BUSTYPE_ETHERNETIP;
    if (lowerType == "profinet")
      return FBVarBusType::FBVAR_BUSTYPE_PROFINET;
    return FBVarBusType::FBVAR_BUSTYPE_INVALID;
  }

  static inline uint8_t get_bit(const uint8_t *p, size_t bit) {
    return (p[bit / 8] >> (bit % 8)) & 1u;
  }
  static inline void set_bit(uint8_t *p, size_t bit, uint8_t v) {
    uint8_t mask = uint8_t(1u << (bit % 8));
    uint8_t &b = p[bit / 8];
    b = v ? (uint8_t)(b | mask) : (uint8_t)(b & ~mask);
  }

  inline bool getBit(const FBVarData& data, unsigned int bitIndex)
{
    if (bitIndex >= 64)
        return false;

    return (data.bitArray64 >> bitIndex) & 0x1;
}

inline void setBit(FBVarData& data, unsigned int bitIndex, bool value)
{
    if (bitIndex >= 64)
        return;

    if (value)
        data.bitArray64 |= (uint64_t(1) << bitIndex);
    else
        data.bitArray64 &= ~(uint64_t(1) << bitIndex);
}
  /**
   * 按位拷贝（可重叠，类似 memmove）
   *  dst/dbit:  目标缓冲区与起始位
   *  src/sbit:  源缓冲区与起始位
   *  nbits:     拷贝位数
   *  约定：bit0 是 LSB；支持 src 与 dst 区间重叠
   */
  static void bit_move(uint8_t *dst, size_t dbit, const uint8_t *src,
                       size_t sbit, size_t nbits) {
    if (nbits == 0 || (dst == src && dbit == sbit))
      return;

    // 计算“位地址”用于判断重叠与方向
    uintptr_t s_begin = (uintptr_t)src * 8u + sbit;
    uintptr_t s_end = s_begin + nbits;
    uintptr_t d_begin = (uintptr_t)dst * 8u + dbit;
    uintptr_t d_end = d_begin + nbits;

    if (d_begin < s_begin || d_begin >= s_end) {
      // 前向拷贝
      for (size_t i = 0; i < nbits; ++i) {
        set_bit(dst, dbit + i, get_bit(src, sbit + i));
      }
    } else {
      // 反向拷贝（避免覆盖源位）
      for (size_t i = nbits; i-- > 0;) {
        set_bit(dst, dbit + i, get_bit(src, sbit + i));
      }
    }
  }

static inline uint64_t fb_mask_by_bits(uint32_t bits)
{
    if (bits >= 64)
        return 0xFFFFFFFFFFFFFFFFULL;

    return (1ULL << bits) - 1ULL;
}

static inline uint32_t fb_bytes_by_bits(uint32_t bits)
{
    return (bits + 7U) / 8U;
}

/*
 * 将 value 的低 bits 位，按如下规则转成字节流：
 *
 * 1. value 的低 bits 位是有效 bit 串
 * 2. 从有效 bit 串的高位开始，每 8bit 凑一个字节
 * 3. 最后低位不足 8bit 时，在低位补 0
 *
 * 例如：
 *   value = 0x3E8, bits = 12
 *
 *   有效 bit:
 *     0011 1110 1000
 *
 *   凑字节：
 *     00111110 10000000
 *
 *   输出：
 *     3E 80
 */
static inline int fb_bits_to_bytes_high_first_pad_low_zero(
    uint64_t value,
    uint32_t bits,
    uint8_t out[8],
    uint32_t* outBytes)
{
    uint32_t bytes;
    uint32_t remBits;
    uint32_t fullBytes;
    uint32_t i;

    if (out == NULL || outBytes == NULL)
        return -1;

    if (bits == 0 || bits > 64)
        return -1;

    bytes = fb_bytes_by_bits(bits);
    remBits = bits % 8U;

    value &= fb_mask_by_bits(bits);

    for (i = 0; i < 8U; ++i)
        out[i] = 0;

    if (remBits == 0)
    {
        /*
         * 正好是整字节：
         *
         * value = 0x12345678, bits = 32
         *
         * 输出：
         *   12 34 56 78
         */
        for (i = 0; i < bytes; ++i)
        {
            uint32_t shift = (bytes - 1U - i) * 8U;
            out[i] = (uint8_t)((value >> shift) & 0xFFU);
        }
    }
    else
    {
        /*
         * 不是整字节：
         *
         * value = 0x3E8, bits = 12
         *
         * bits / 8 = 1 个完整字节
         * bits % 8 = 4 个剩余 bit
         *
         * 高位完整字节：
         *   3E
         *
         * 低位剩余 bit：
         *   8
         *
         * 低位补 0：
         *   80
         *
         * 输出：
         *   3E 80
         */
        fullBytes = bits / 8U;

        for (i = 0; i < fullBytes; ++i)
        {
            uint32_t shift = remBits + (fullBytes - 1U - i) * 8U;
            out[i] = (uint8_t)((value >> shift) & 0xFFU);
        }

        out[fullBytes] =
            (uint8_t)((value & fb_mask_by_bits(remBits)) << (8U - remBits));
    }

    *outBytes = bytes;
    return 0;
}

static inline void fb_reverse_bytes(uint8_t buf[8], uint32_t bytes)
{
    uint32_t i;

    for (i = 0; i < bytes / 2U; ++i)
    {
        uint8_t tmp = buf[i];
        buf[i] = buf[bytes - 1U - i];
        buf[bytes - 1U - i] = tmp;
    }
}

/*
 * 核心函数：
 *
 * 先按“高位开始凑字节，低位不足补 0”生成字节流，
 * 再进行大小端字节互换。
 *
 * 例如：
 *   value = 1000 = 0x3E8
 *   bits  = 12
 *
 *   凑字节前：
 *     3E 80
 *
 *   大小端互换后：
 *     80 3E
 */
static inline int fb_endian_swap_bits_to_bytes_pad_low_zero(
    uint64_t value,
    uint32_t bits,
    uint8_t out[8],
    uint32_t* outBytes)
{
    int ret;

    ret = fb_bits_to_bytes_high_first_pad_low_zero(
        value,
        bits,
        out,
        outBytes);

    if (ret != 0)
        return ret;

    fb_reverse_bytes(out, *outBytes);

    return 0;
}

/*
 * 将字节流重新打包成 uint64_t。
 *
 * 例如：
 *   bytes = 80 3E
 *
 * 返回：
 *   0x803E
 */
static inline uint64_t fb_pack_bytes_to_u64(
    const uint8_t buf[8],
    uint32_t bytes)
{
    uint64_t result = 0;
    uint32_t i;

    for (i = 0; i < bytes; ++i)
    {
        result <<= 8;
        result |= buf[i];
    }

    return result;
}

/*
 * 如果希望直接得到 uint64_t 结果，用这个函数。
 */
static inline int fb_endian_swap_bits_to_u64_pad_low_zero(
    uint64_t value,
    uint32_t bits,
    uint64_t* result)
{
    uint8_t buf[8];
    uint32_t bytes;
    int ret;

    if (result == NULL)
        return -1;

    ret = fb_endian_swap_bits_to_bytes_pad_low_zero(
        value,
        bits,
        buf,
        &bytes);

    if (ret != 0)
        return ret;

    *result = fb_pack_bytes_to_u64(buf, bytes);
    return 0;
}





  // 返回新的小写字符串
  static std::string toLower(const std::string &s) {
    std::string out(s); // 拷贝一份再原地改
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) -> char { // 先转成 unsigned char
                     return static_cast<char>(std::tolower(c)); // 再转回 char
                   });
    return out;
  }

  // 递归转换Json对象的所有键为小写
  static Json::Value toLowerJsonKeys(const Json::Value &value) {
    Json::Value result;

    if (value.isObject()) {
      // 处理对象类型
      for (const auto &key : value.getMemberNames()) {
        std::string lowerKey = key;
        std::transform(lowerKey.begin(), lowerKey.end(), lowerKey.begin(),
                       ::tolower);
        result[lowerKey] = toLowerJsonKeys(value[key]);
      }
    } else if (value.isArray()) {
      // 处理数组类型
      for (Json::ArrayIndex i = 0; i < value.size(); ++i) {
        result[i] = toLowerJsonKeys(value[i]);
      }
    } else {
      // 基本类型直接复制
      result = value;
    }
    return result;
  }

  // 添加变量函数, 根据字符串或Json来创建新变量
  virtual bool createVar(const std::string varName, const std::string varConf,
                         FBVAR_HANDLE &busHandle) = 0;
  virtual bool createVar(const std::string varName, const Json::Value varConf,
                         FBVAR_HANDLE &busHandle) = 0;

  // 删除变量
  virtual bool removeVar(const std::string varName) = 0;

  // 获取变量数据类型函数
  virtual bool getVarDatatype(const std::string varName,
                              FBVarDataType &dataType) = 0;

  // 获取变量总线类型函数
  virtual FBVarBusType getVarBustype() = 0;

  // 获取变量句柄函数
  virtual bool getVarHandle(const std::string varName, void *&varHandle) = 0;

  // 判断变量是否存在，不区分大小写
  virtual bool isVarExist(const std::string varName) = 0;

  // 更新变量数据函数(读取最新数据，写入最新数据)
  virtual bool syncVarData2Bus(const std::string varName, bool syncWrite,
                               bool syncRead) = 0;
  virtual bool syncVarData2Bus(const FBVAR_HANDLE varHandle, bool syncWrite,
                               bool syncRead) = 0;

  // 根据变量名称获取变量数据函数
  virtual bool getVarData(const std::string &varName, bool &varValue) = 0;
  virtual bool getVarData(const std::string &varName, int &varValue) = 0;
  virtual bool getVarData(const std::string &varName, float &varValue) = 0;
  // 根据变量句柄获取变量数据函数
  virtual bool getVarData(const FBVAR_HANDLE &varHandle, bool &varValue) = 0;
  virtual bool getVarData(const FBVAR_HANDLE &varHandle, int &varValue) = 0;
  virtual bool getVarData(const FBVAR_HANDLE &varHandle, float &varValue) = 0;
  // 根据变量名称设置变量数据函数
  virtual bool setVarData(const std::string &varName, bool varValue) = 0;
  virtual bool setVarData(const std::string &varName, int varValue) = 0;
  virtual bool setVarData(const std::string &varName, float varValue) = 0;
  // 根据变量句柄设置变量数据函数
  virtual bool setVarData(const FBVAR_HANDLE &varHandle, bool varValue) = 0;
  virtual bool setVarData(const FBVAR_HANDLE &varHandle, int varValue) = 0;
  virtual bool setVarData(const FBVAR_HANDLE &varHandle, float varValue) = 0;

  // 析构函数
  virtual ~IFbvar() = default;

  // 数组操作（如果是数组类型）
  virtual size_t size() = 0;
  virtual bool empty() = 0;
  virtual void clear() = 0;

  // 内存管理
  virtual void addRef() = 0;
  virtual void release() = 0;
  virtual int refCount() = 0;
};

} // namespace VAR

} // namespace FBBASE

#endif // _I_FBVAR_H_