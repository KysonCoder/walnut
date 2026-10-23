/*
 * @Author: arthur arthur@bbeamtech.com
 * @Date: 2025-12-13 16:25:00
 * @LastEditors: arthur arthur@bbeamtech.com
 * @LastEditTime: 2025-12-13 17:17:33
 * @FilePath: \fbbase\include\fbbase_log.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @FilePath: fbbase_log.h
 * @Author: AT
 * @Date: 2025-12-13 16:25:00
 * @LastEditTime: 2025-12-13 16:25:07
 * @Description: Define log macros for fbbase.
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
#ifndef _FBBASE_LOG_H_
#define _FBBASE_LOG_H_

#include <logger.h>

#ifndef __FILENAME__
    #ifdef __BASE_FILE__
        #define __FILENAME__ __BASE_FILE__
    #else
        #define __FILENAME__ __FILE__
    #endif
#endif

#define FB_D(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_DEBUG,"",__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define FB_I(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_INFO,"",__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define FB_W(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_WARN,__FILENAME__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);
#define FB_E(format,...) AlarmLog::GetInstance()->Log_Gen(LOG_ERROR,__FILENAME__,__FUNCTION__,__LINE__,format,##__VA_ARGS__);


#endif // _FBBASE_LOG_H_