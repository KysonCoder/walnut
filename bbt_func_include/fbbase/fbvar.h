/**
 * @FilePath: fbvar.h
 * @Author: AT
 * @Date: 2025-10-02 14:39:42
 * @LastEditTime: 2025-10-27 20:31:43
 * @Description:
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */

#ifndef _FBVAR_H_
#define _FBVAR_H_

#include <string>

// Define variant handle
typedef void *FBVAR_HANDLE;

bool fbvar_init(const std::string cfgFilePath, unsigned int scanCycleUs);
bool fbvar_deinit(void);

bool fbvar_get(const std::string &varName, bool &varValue);
bool fbvar_get(const std::string &varName, int &varValue);
bool fbvar_get(const std::string &varName, float &varValue);

bool fbvar_set(const std::string &varName, bool varValue);
bool fbvar_set(const std::string &varName, int varValue);
bool fbvar_set(const std::string &varName, float varValue);

bool fbvar_createVar(const std::string &varName, const std::string &busType,
                     const std::string varConfig);

bool fbvar_removeVar(const std::string &varName);

bool fbvar_get(const FBVAR_HANDLE varHandle, bool &varValue);
bool fbvar_get(const FBVAR_HANDLE varHandle, int &varValue);
bool fbvar_get(const FBVAR_HANDLE varHandle, float &varValue);

bool fbvar_set(const FBVAR_HANDLE varHandle, bool varValue);
bool fbvar_set(const FBVAR_HANDLE varHandle, int varValue);
bool fbvar_set(const FBVAR_HANDLE varHandle, float varValue);

bool fbvar_get_varhandle(const std::string &varName, FBVAR_HANDLE &varHandle);

bool fbvar_removeVar(const FBVAR_HANDLE varHandle);

bool fbvar_removeAllVars(void);

bool fbvar_data_exchange(const std::string busName);

#endif // _FBVAR_H_
