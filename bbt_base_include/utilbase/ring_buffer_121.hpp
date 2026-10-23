/**
 * @file ring_buffer_121.hpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2024-09-06
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-09-06  0.1      Yonglin Chi       first edition
 * 2025-03-08  0.2      Arthur  WU        Adapting for Windows OS, add control var like semphore and mutex for writing.
 * 2025-03-13  0.3      Arthur  WU        Use C++ atomic, optiimze multiple writing and single reading.
 * 2025-03-16  0.4      Arthur  WU		  Remove retry.
 * 2025-07-12  0.5      Arthur  WU        Fix bug of multiple user reading;
 * 2025-08-27  0.6      Yonglin Chi       Add new functions : ReadEnd() and PopBegin().
 */



/**
 * @file memoryring.h
 * @brief the base class for the ring buffer.
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

#ifndef __RING_BUFFER_121_H__
	#define __RING_BUFFER_121_H__

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <atomic>


#ifdef __linux__
#include <time.h>
#include <unistd.h>
#include "logger.h"
#endif

template <typename T, unsigned int _buffer_length> //!< Define the data number of the ring buf. The actual size is equal to 2^m_BuffSize;
class CRingBuffer
{
protected:
    static constexpr unsigned int sizeMask{_buffer_length-1};   
    T _ring_buffer[_buffer_length];           	//!< Point to the beginning of the ring buffer;

    std::atomic_flag used[_buffer_length];		//!< buffer element used, it means that the element is produced but not consumed.
	volatile bool dataReady[_buffer_length];    //!< Indicate Data writing ready

    std::atomic<unsigned int> m_WriteIndex;     //!< The index for the first availiable writable buffer.
	std::atomic<unsigned int> m_WriteCnt;		//!< Total writing count
    std::atomic<unsigned int> m_ReadCnt;       	//!< The index for the first availiable readable buffer.
	
    std::atomic_flag m_ReadingMutex;            //!< Avoid more than 1 user to


public:



    /**
     * @brief Construct a new CRing1W1R::CRing1W1R object
     * 
     * @param iDataNumber the number of the data which the ring buffer can store at maximum.
     */

    CRingBuffer()
    {
		static_assert(((0<_buffer_length)&&(0==(_buffer_length&(_buffer_length-1)))), "_buffer_length must meet 2^n !");
		
	   Clear();

    }
    /**
     * @brief Destruct a  CRing1W1R::CRing1W1R object
     * 
     * @param 
     */
    virtual ~CRingBuffer()
    {
        ;
    }

    /**
     * @brief Re-initialize writing index, reading index, writing mutex, writing semphore
     * 
     * @param iDataNumber the number of the data which the ring buffer can store at maximum.
     */
    void Init()
    {
        Clear();   
	
    }

    /**
     * @brief Get first ring buffer element at first postion
     * 
     * @param iDataNumber the number of the data which the ring buffer can store at maximum.
     */
    T Head()
    {
        return _ring_buffer[0];
    }

    /**
     * @brief Clear writing index, writing count, reading index, 
     * 
     * @param iDataNumber the number of the data which the ring buffer can store at maximum.
     */
    void Clear()
    {
       	m_WriteIndex = 0;
	    m_ReadCnt = 0;
		m_WriteCnt = 0;
		
		for(unsigned i=0; i<_buffer_length; i++)
		{
			used[i].clear();
			dataReady[i]=false;
		}
        m_ReadingMutex.clear();
    }

        /**
     * @brief Get the data number stored in the ring buffer.
     * 
     * @return int  the number of the stored data 
     */

    unsigned int ReadLength()
    { 
        return (unsigned int)(m_WriteCnt-m_ReadCnt);        
    }


    /**
     * @brief Get the number of the empty space availiable in the ring buffer.
     * 
     * @return int  the number of the empty space
     */
    unsigned int WriteLength()
    {
        int iLength;
        int iCurrentRIndex = m_ReadCnt & sizeMask;
        int iCurrentWIndex = m_WriteCnt & sizeMask;

        if(iCurrentRIndex == iCurrentWIndex)
            iLength = _buffer_length;
        else if(iCurrentRIndex >iCurrentWIndex)
            iLength = iCurrentRIndex - iCurrentWIndex;
        else
            iLength = _buffer_length - iCurrentWIndex + iCurrentRIndex;

        return iLength;   
    }

    /**
     * @brief Check the buffer is full
     * 
     * @return bool  true indicates the buffer is full.
     */
    bool isFull()
    {
        return (ReadLength()>=_buffer_length);        
    }


    /**
     * @brief Get one datum from the ring buffer and delete the datum in the buffer.
     * 
     * @param pDataOut  the memory point to store the read data.
     * @return true if succeeded
     * @return false if failed
     */
    virtual bool GetData(T * data_out)
    {
        while(m_ReadingMutex.test_and_set())
        {
            #ifdef __linux__
					usleep(10);
            #else
                Sleep(1);
            #endif
        }

		if(0!=ReadLength())
		{			
			unsigned int iCurrentRIndex = m_ReadCnt & sizeMask;		
			
			while(true!=dataReady[iCurrentRIndex])
			{
				#ifdef __linux__
					usleep(10);
				#else
					Sleep(1);
				#endif	
			}			
			* data_out = _ring_buffer[iCurrentRIndex];
			dataReady[iCurrentRIndex]=false;
			used[iCurrentRIndex].clear();
			++m_ReadCnt;
            m_ReadingMutex.clear();
			return true;
		}        


        m_ReadingMutex.clear();
        return false;

    }


    /**
     * @brief Get one datum from the end of the ring buffer without deleting the datum in the buffer.
     * 
     * @param data_out 
     * @return true 
     * @return false 
     */
    bool ReadEnd(T * data_out)
    {
		if(0<ReadLength())
		{			
			unsigned int iCurrentRIndex = (m_WriteCnt-1) & sizeMask;		
			* data_out = _ring_buffer[iCurrentRIndex];
			return true;
		}        
        return false;
    }

    /**
     * @brief Delete one datum from the begin of the ring buffer.
     * 
     * @return true 
     * @return false 
     */
    bool PopBegin()
    {
		if(0<ReadLength())
		{			
			unsigned int iCurrentRIndex = m_ReadCnt & sizeMask;		
			dataReady[iCurrentRIndex]=false;
			used[iCurrentRIndex].clear();
			++m_ReadCnt;
			return true;
		}        
        return false;
    }

   
    /**
     * @brief Write one data to the buffer.
     * 
     * @param pDataIn the pointer to the written datum
     * @return true if succeeded
     * @return false if failed
     */
    virtual bool SetData(T data_in) 
    {

		if(true!=isFull())
		{				
			//Get a atomic write index ahead with increase.
			unsigned int iCurrentWIndex = ((++m_WriteIndex)-1) & sizeMask; 
			//Check whether used for the data buffer
			if(!used[iCurrentWIndex].test_and_set())
			{
				_ring_buffer[iCurrentWIndex] = data_in;				
				dataReady[iCurrentWIndex]=true;
				++m_WriteCnt;
				return true;
			}
			else
			{			
				// Return to last write index when full
				--m_WriteIndex;			
			}
		}
					

#ifdef __linux__
		
		Log_E("The buffer is full. m_WriteIndex=%d. m_ReadCnt=%d m_WriteCnt=%d . \n", m_WriteIndex.load(), m_ReadCnt.load(), m_WriteCnt.load());

#else
		//printf("The buffer is full. m_WriteIndex=%d. m_ReadCnt=%d m_WriteCnt=%d . \n", iNowWriteIndex.load(), m_ReadCnt.load(), m_WriteCnt.load());
#endif
        return false;

    }
};
#endif