/**
 * @FilePath: fbvar_ethercat.h
 * @Author: AT
 * @Date: 2025-10-02 14:40:23
 * @LastEditTime: 2025-10-18 22:29:47
 * @Description: Declaration class for EtherCAT variables.
 *
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */

#ifndef _FBVAR_ETHERCAT_H_
#define _FBVAR_ETHERCAT_H_

#include "i_fbvar.h"
#include <string>
#include <vector>

namespace FBBASE {

namespace VAR {

typedef struct _EthercatVar {
  std::string varName;                    // 变量名称
  FBBASE::VAR::FBVarBusType varBusType;   // 变量总线类型
  FBBASE::VAR::FBVarDataType varDataType; // 变量数据类型
  bool canWrite;                          // 变量是否可写
  char *pVarDataAddr;                     // 变量地址
  int varBitOffset;                       // 变量位偏移
  int varBitSize;                         // 变量位大小
  IFbvar *pBusInstance;                   // 变量总线句柄
  FBVarData varValue;                     // 变量值
  struct {
    int topPosition; // EtherCAT top position, from 1
    int sdoIndex;
    int sdoSubindex;
    int bitOffset;
    int bitSize;
    bool accessType; // false: read only, true: read/write
    FBBASE::VAR::FBVarDataType
        dataType;          // "bool", "int", "float", "double", "string"
    std::string initValue; // initial data, for read/write 
    bool bigEndian;       // true: big endian, false: little 
    int factorNumer; // factor numerator, for float/double
    int factorDenom; // factor denominator, for float/double
    unsigned long long  bitMap; // bit map for bit access
  } cfg;
  // 不带参数的构造函数
  _EthercatVar() {
    varBusType = FBBASE::VAR::FBVarBusType::FBVAR_BUSTYPE_INVALID;
    varDataType = FBBASE::VAR::FBVarDataType::FBVAR_DATATYPE_INVALID;
    canWrite = false;
    pVarDataAddr = NULL;
    varBitOffset = 0;
    varBitSize = 0;
    pBusInstance = NULL;
    memset(&varValue, 0, sizeof(varValue));
  }
  // 带参数的构造函数
  _EthercatVar(const std::string &name, int topPos, int sdoIdx, int sdoSubIdx,
               int cfgBitOff, int cfgBitSz, bool access,
               FBBASE::VAR::FBVarDataType type, const std::string &initVal, bool bigEndian=false, int factorNumer=1, int factorDenom=1,unsigned long long  bitMap=0)
      : varName(name), cfg{topPos,   sdoIdx, sdoSubIdx, cfgBitOff,
                           cfgBitSz, access, type,      initVal,bigEndian, factorNumer, factorDenom, bitMap} {
    varBusType = FBBASE::VAR::FBVarBusType::FBVAR_BUSTYPE_INVALID;
    varDataType = FBBASE::VAR::FBVarDataType::FBVAR_DATATYPE_INVALID;
    canWrite = false;
    pVarDataAddr = NULL;
    varBitOffset = 0;
    varBitSize = 0;
    pBusInstance = NULL;
    // 初始化系数
    if(0==cfg.factorDenom || 0==cfg.factorNumer){
      cfg.factorDenom = 1;
      cfg.factorNumer = 1;
    }
    FB_I("variant: %s factorDenom=%d factorNumer=%d bitMap=0x%llx\n", name.c_str(), cfg.factorDenom, cfg.factorNumer, cfg.bitMap);
    memset(&varValue, 0, sizeof(varValue));
  }
} ETHERCAT_VAR;

class FbvarEthercat : public IFbvar {
public:
  // 获取单例实例，返回基类指针
  static IFbvar *getInstance() {
    static FbvarEthercat instance;
    return static_cast<IFbvar *>(&instance);
  }

  // 删除拷贝构造和赋值操作
  FbvarEthercat(const FbvarEthercat &) = delete;
  FbvarEthercat &operator=(const FbvarEthercat &) = delete;

  FbvarEthercat() : m_refCount(0) {}
  ~FbvarEthercat() = default;

  // 添加变量函数, 根据字符串或Json来创建新变量
  bool createVar(const std::string varName, const std::string varConf,
                 FBVAR_HANDLE &varHandle) override;
  bool createVar(const std::string varName, const Json::Value varConf,
                 FBVAR_HANDLE &varHandle) override;

  // 删除变量
  bool removeVar(const std::string varName) override;

  // 获取变量数据类型函数
  bool getVarDatatype(const std::string varName,
                      FBVarDataType &dataType) override;

  // 获取变量总线类型函数
  FBVarBusType getVarBustype() override {
    return FBVarBusType::FBVAR_BUSTYPE_ETHERCAT;
  };

  // 获取变量句柄函数
  bool getVarHandle(const std::string varName,
                    FBVAR_HANDLE &varHandle) override;

  // 判断变量是否存在，不区分大小写
  bool isVarExist(const std::string varName) override;

  // 获取变量数据函数
  bool getVarData(const std::string &varName, bool &varValue) override;
  bool getVarData(const std::string &varName, int &varValue) override;
  bool getVarData(const std::string &varName, float &varValue) override;
  bool getVarData(const FBVAR_HANDLE &varHandle, bool &varValue) override;
  bool getVarData(const FBVAR_HANDLE &varHandle, int &varValue) override;
  bool getVarData(const FBVAR_HANDLE &varHandle, float &varValue) override;

  // 设置变量函数
  bool setVarData(const std::string &varName, bool varValue) override;
  bool setVarData(const std::string &varName, int varValue) override;
  bool setVarData(const std::string &varName, float varValue) override;
  bool setVarData(const FBVAR_HANDLE &varHandle, bool varValue) override;
  bool setVarData(const FBVAR_HANDLE &varHandle, int varValue) override;
  bool setVarData(const FBVAR_HANDLE &varHandle, float varValue) override;

  // 更新读写数据
  bool syncVarData2Bus(const std::string varName, bool syncWrite,
                       bool syncRead) override;
  bool syncVarData2Bus(const FBVAR_HANDLE varHandle, bool syncWrite,
                       bool syncRead) override;

  // 数组操作（如果是数组类型）
  size_t size() override;
  bool empty() override;
  void clear() override;

  // 内存管理
  void addRef() override;
  void release() override;
  int refCount() override;

private:
  // 私有成员函数

  // 模板函数：根据数据类型返回相应的值
  template <typename T> bool getTypedValue(const ETHERCAT_VAR *var, T &value) {
    if (!var || !var->pVarDataAddr || 0 >= var->varBitSize) {
      return false;
    }
    switch (var->varDataType) {
    case FBVarDataType::FBVAR_DATATYPE_BOOL:
      value = static_cast<T>(var->varValue.bValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT8:
      value = static_cast<T>(var->varValue.cValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT8:
      value = static_cast<T>(var->varValue.ucValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT16:
      value = static_cast<T>(var->varValue.sValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT16:
      value = static_cast<T>(var->varValue.usValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT32:
      value = static_cast<T>(var->varValue.iValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT32:
      value = static_cast<T>(var->varValue.uiValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT64:
      value = static_cast<T>(var->varValue.llValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT64:
      value = static_cast<T>(var->varValue.ullValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_FLOAT:
      value = static_cast<T>(var->varValue.fValue);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_DOUBLE:
      value = static_cast<T>(var->varValue.dValue);
      return true;

    default:
      return false;
    }
    return false;
  }


  bool  multiplyFactor(const ETHERCAT_VAR *var, const FBVarData fromData, FBVarData &toData) {
    if (!var || 0==var->cfg.factorDenom) {
      return false;
    }
     // No change
    if(var->cfg.factorNumer==var->cfg.factorDenom){
      toData = fromData;
      return true;
    }

    switch (var->varDataType) {

    case FBVarDataType::FBVAR_DATATYPE_INT8:      
      toData.cValue = static_cast<char>((static_cast<int>(fromData.cValue)*var->cfg.factorNumer)/var->cfg.factorDenom);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT8:
      toData.ucValue = static_cast<unsigned char>((static_cast<unsigned int>(fromData.ucValue)*static_cast<unsigned int>(var->cfg.factorNumer))/static_cast<unsigned int>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT16:
      toData.sValue = static_cast<short>((static_cast<int>(fromData.sValue)*var->cfg.factorNumer)/var->cfg.factorDenom);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT16:      
      toData.usValue = static_cast<unsigned short>((static_cast<unsigned int>(fromData.usValue)*static_cast<unsigned int>(var->cfg.factorNumer))/static_cast<unsigned int>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT32:      
      toData.iValue = static_cast<int>((fromData.iValue*var->cfg.factorNumer)/var->cfg.factorDenom);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT32:      
      toData.uiValue = static_cast<unsigned int>((fromData.uiValue*static_cast<unsigned int>(var->cfg.factorNumer))/static_cast<unsigned int>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT64:
      toData.llValue = static_cast<long long>((fromData.llValue*static_cast<long long>(var->cfg.factorNumer))/static_cast<long long>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT64:
      toData.ullValue = static_cast<unsigned long long>((fromData.ullValue*static_cast<unsigned long long>(var->cfg.factorNumer))/static_cast<unsigned long long>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_FLOAT:      
      toData.fValue = static_cast<float>((fromData.fValue*static_cast<float>(var->cfg.factorNumer))/static_cast<float>(var->cfg.factorDenom));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_DOUBLE:      
      toData.dValue = static_cast<double>((fromData.dValue*static_cast<double>(var->cfg.factorNumer))/static_cast<double>(var->cfg.factorDenom));
      return true;

    default:
      return false;
    }
    
  }

  bool  dividFactor(const ETHERCAT_VAR *var, const FBVarData fromData, FBVarData &toData) {
    if (!var || 0==var->cfg.factorNumer) {
      return false;
    }
     // No change
    if(var->cfg.factorNumer==var->cfg.factorDenom){
      toData = fromData;
      return true;
    }

    switch (var->varDataType) {

    case FBVarDataType::FBVAR_DATATYPE_INT8:      
      toData.cValue = static_cast<char>((static_cast<int>(fromData.cValue)*var->cfg.factorDenom)/var->cfg.factorNumer);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT8:
      toData.ucValue = static_cast<unsigned char>((static_cast<unsigned int>(fromData.ucValue)*static_cast<unsigned int>(var->cfg.factorDenom))/static_cast<unsigned int>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT16:
      toData.sValue = static_cast<short>((static_cast<int>(fromData.sValue)*var->cfg.factorDenom)/var->cfg.factorNumer);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT16:      
      toData.usValue = static_cast<unsigned short>((static_cast<unsigned int>(fromData.usValue)*static_cast<unsigned int>(var->cfg.factorDenom))/static_cast<unsigned int>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT32:      
      toData.iValue = static_cast<int>((fromData.iValue*var->cfg.factorDenom)/var->cfg.factorNumer);
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT32:      
      toData.uiValue = static_cast<unsigned int>((fromData.uiValue*static_cast<unsigned int>(var->cfg.factorDenom))/static_cast<unsigned int>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_INT64:
      toData.llValue = static_cast<long long>((fromData.llValue*static_cast<long long>(var->cfg.factorDenom))/static_cast<long long>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_UINT64:
      toData.ullValue = static_cast<unsigned long long>((fromData.ullValue*static_cast<unsigned long long>(var->cfg.factorDenom))/static_cast<unsigned long long>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_FLOAT:      
      toData.fValue = static_cast<float>((fromData.fValue*static_cast<float>(var->cfg.factorDenom))/static_cast<float>(var->cfg.factorNumer));
      return true;

    case FBVarDataType::FBVAR_DATATYPE_DOUBLE:      
      toData.dValue = static_cast<double>((fromData.dValue*static_cast<double>(var->cfg.factorDenom))/static_cast<double>(var->cfg.factorNumer));
      return true;

    default:
      return false;
    }
    
  }

  // 模板函数：设置不同类型的值
  template <typename T> bool setTypedValue(ETHERCAT_VAR *var, const T &value) {
    if (!var || !var->pVarDataAddr || 0 >= var->varBitSize || !var->canWrite) {
      return false;
    }

    if (true != var->canWrite) {
      return false;
    }

    switch (var->varDataType) {
    case FBVarDataType::FBVAR_DATATYPE_BOOL:
      var->varValue.bValue = static_cast<bool>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_INT8:
      var->varValue.cValue = static_cast<char>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_UINT8:
      var->varValue.ucValue = static_cast<unsigned char>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_INT16:
      var->varValue.sValue = static_cast<short>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_UINT16:
      var->varValue.usValue = static_cast<unsigned short>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_INT32:
      var->varValue.iValue = static_cast<int>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_UINT32:
      var->varValue.uiValue = static_cast<unsigned int>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_INT64:
      var->varValue.llValue = static_cast<long long>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_UINT64:
      var->varValue.ullValue = static_cast<unsigned long long>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_FLOAT:
      var->varValue.fValue = static_cast<float>(value);
      break;

    case FBVarDataType::FBVAR_DATATYPE_DOUBLE:
      var->varValue.dValue = static_cast<double>(value);
      break;

    default:
      return false;
    }

    return true;
  }
  // 私有成员变量
  int m_refCount;
  // std::vector<ETHERCAT_VAR *> m_array;
  std::map<std::string, ETHERCAT_VAR *> m_object;
};

} // namespace VAR

} // namespace FBBASE

#endif // _FBVAR_ETHERCAT_H_
