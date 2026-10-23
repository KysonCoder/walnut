/**
 * @FilePath: fbvar_data_manage.h
 * @Author: AT
 * @Date: 2025-10-02 14:47:49
 * @LastEditTime: 2025-11-09 17:29:38
 * @Description: The class for managing all kinds of field bus variants data.
 * @Version: 1.0
 * @LastEditors: Please set LastEditors
 * @Email: at@balancebeam.cn
 * @Website: https://balancebeam.cn
 * @History: <br/>
 * Date					By				Note
 * ------------------------------------------------------
 * 2025-10-02 14:47:49		<AT>		<Create>
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */

#ifndef _FBVAR_DATA_MANAGE_H_
#define _FBVAR_DATA_MANAGE_H_

#include "fbvar_ethercat.h"
#include "fbvar_virtual.h"
#include <cctype>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits> // 添加类型特征支持

namespace FBBASE {

namespace VAR {

class FbvarDataManage {
private:
  struct VarInfo {
    FBVAR_HANDLE handle;
    IFbvar *instance;
    std::string busType;
  };

  std::map<std::string, VarInfo> m_all_objects;
  std::map<std::string, std::shared_ptr<IFbvar>> m_busInstances;
  mutable std::recursive_mutex m_mutex;

  FbvarDataManage() = default;
  ~FbvarDataManage() { cleanup(); };

public:
  static FbvarDataManage *getInstance() {
    static FbvarDataManage instance;
    return &instance;
  }

  FbvarDataManage(const FbvarDataManage &) = delete;
  FbvarDataManage &operator=(const FbvarDataManage &) = delete;

  // 模板数据访问接口
  template <typename T> bool getVarData(const std::string &varName, T &value) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    auto it = m_all_objects.find(varName);
    if (it != m_all_objects.end()) {
      return it->second.instance->getVarData(varName, value);
    }
    return false;
  }

  template <typename T>
  bool setVarData(const std::string &varName, const T &value) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    auto it = m_all_objects.find(varName);
    if (it != m_all_objects.end()) {
      return it->second.instance->setVarData(varName, value);
    }
    return false;
  }

  // 传统的非模板接口（保持向后兼容）
  bool getVarData(const std::string &varName, bool &varValue);
  bool getVarData(const std::string &varName, int &varValue);
  bool getVarData(const std::string &varName, float &varValue);

  bool setVarData(const std::string &varName, bool varValue);
  bool setVarData(const std::string &varName, int varValue);
  bool setVarData(const std::string &varName, float varValue);

  // 初始化
  bool init(const std::string &varCfgFilePath);

  // 变量管理
  bool createVar(const std::string &varName, const std::string &busType,
                 const std::string &varConfig);
  bool createVar(const std::string &varName, const std::string &busType,
                 const Json::Value &varConfig);
  bool removeVar(const std::string &varName);
  bool removeAllVars(void);

  // 更新操作
  bool syncAllVars();
  bool syncVarData2Bus(const std::string &varName);
  bool syncBusVarsData(const std::string &busType);

  // 获取变量信息
  bool printBusVarsInfo(const std::string &busType);

  // 错误处理
  std::string getLastError() const;
  void clearError();

  void cleanup();

private:
  mutable std::string m_lastError; // 存储最后的错误信息

  // 辅助函数
  IFbvar *getBusInstance(const std::string &busType);
  bool validateVarName(const std::string &varName);
  bool loadConfig(const std::string &configFile);
  void setError(const std::string &error) const;
};

} // namespace VAR
} // namespace FBBASE

#endif // _FBVAR_DATA_MANAGE_H_
