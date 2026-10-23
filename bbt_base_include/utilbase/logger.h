/**
 * @file logmanager.hpp
 * @brief Define the log message class
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-04-07
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-04-07  0.1      Yonglin Chi       first edition
 */




#ifndef _LOG_MANAGER_H_
#define _LOG_MANAGER_H_
 
//定义不同打印基本的宏
#define Log_D(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_DEBUG,__FILE__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define Log_I(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_INFO,__FILE__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define Log_W(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_WARN,__FILE__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define Log_E(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_ERROR,__FILE__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);
//日志等级
enum ALARMLOG_LEVEL 
{
	LOG_ERROR = 1,
	LOG_WARN = 2,
	LOG_INFO = 3,
	LOG_DEBUG = 4
};
//日志类




class AlarmLog
{
protected:
	AlarmLog();
	~AlarmLog();
public:
	static AlarmLog* GetInstance();
	void  Log_Gen(ALARMLOG_LEVEL loglevel,const char* filename,const char* func,int lineNum,const char* format,...);
	
public:
	int setloglevel;
protected:
	static AlarmLog* _instance;
};
#endif







