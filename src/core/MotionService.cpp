//////////////////////////////////////////////////////////////////////////////// 
//
// Name:
//  MotionService Module
//
//  Description:
//
//  This module provide data exchanging interfaces between the external motion control 
//  and internal RT motion control based on the shared memory mechanism. Two shared memories
//  are created for sending real time data to the external applicaiton and receving command
//  data from the external application. 
// 
//  Module Define
//
#define MOTIONSERVICE_CPP
//
//  Include files:
// 
// Unity Header
#ifndef UNITY
#include <windows.h>
#include <iostream>
#include "core/MotionService.h"
#include "core/system_constants.hpp"
#include <cstring>
#include <cstddef> 
#include "RtUK.h"
#include "RtSMR.h"
#endif	// UNITY
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Name:               CONSTANT DEFINITIONS
//////////////////////////////////////////////////////////////////////////////
#define MOT_SERVICE_READ_SEMAPHORE   "MotServiceReadSemaphore"
#define MOT_SERVICE_WRITE_SEMAPHORE  "MotServiceWriteSemaphore"
#define RT_MOTION_DATA_NAME          "RTMotionData"
#define APP_CMD_DATA_NAME            "AppCmdData"

//Default time interval of every exchanging data between RT and the external application is 1ms. 
//The allocated shared memory can hold two seconds data
#define MOTION_SERVICE_MEM_SIZE      SystemConstants::Buffers::SMR_SHARED_MEMORY_SIZE



//////////////////////////////////////////////////////////////////////////////
// Name:                 TYPE DEFINITIONS
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Name:               MODULE VARIABLE DEFINITIONSB
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Name:               FUNCTION PROTOTYPE DEFINITIONS
//////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// MotionService
//
// Function Description:
// Class Constructor
//
///////////////////////////////////////////////////////////////////////////////
MotionService::MotionService(void)
{
    m_RTMotionData = MotServiceMemType();
    m_AppCmdData = MotServiceMemType();
    m_MotServiceDiagInfo = MotionServiceDiagInfoType();
};

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// MotionService
//
// Function Description:
// Class Destructor

//
///////////////////////////////////////////////////////////////////////////////
MotionService::~MotionService(void)
{
    //m_RTMotionData
    if (m_RTMotionData.hReadSemaphore != NULL)
    {
        OsCloseHandle(m_RTMotionData.hReadSemaphore);
    }

    if (m_RTMotionData.hWriteSemaphore != NULL)
    {
        OsCloseHandle(m_RTMotionData.hWriteSemaphore);
    }

    if (m_RTMotionData.hSharedMemory != NULL)
    {
        OsCloseHandle(m_RTMotionData.hSharedMemory);
    }

    //m_AppCmdData
    if (m_AppCmdData.hReadSemaphore != NULL)
    {
        OsCloseHandle(m_AppCmdData.hReadSemaphore);
    }

    if (m_AppCmdData.hWriteSemaphore != NULL)
    {
        OsCloseHandle(m_AppCmdData.hWriteSemaphore);
    }

    if (m_AppCmdData.hSharedMemory != NULL)
    {
        OsCloseHandle(m_AppCmdData.hSharedMemory);
    }
    std::cout << std::endl << "Destructor called " << std::endl;
}


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// InitMotionService
//
// Function Description:
// Initialize resources owned by the module
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::InitMotionService(int* pErrCode)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;

    if (pErrCode == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    result = MotServiceMemInit(&m_RTMotionData, sizeof(RTMotionDataType), MOTION_SERVICE_MEM_SIZE, RT_MOTION_DATA_NAME);
    if (result != MOT_SERVICE_INIT_SUCCESS)
    {
        std::cout << "Motion Data Memory Failed to Init Error Code:" << result << std::endl;
        *pErrCode = result;
    }

    result = MotServiceMemInit(&m_AppCmdData, sizeof(AppCmdDataType), MOTION_SERVICE_MEM_SIZE, APP_CMD_DATA_NAME);
    if (result != MOT_SERVICE_INIT_SUCCESS)
    {
        std::cout << "App Command Memory Failed to Init Error Code:" << result << std::endl;
        *pErrCode = result;
    }
    std::cout << std::endl << "Initialized SMR Buffers" << std::endl;
    return result;

} // InitMotionService

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// AppWriteCmdData
//
// Function Description:
// This function is called by the external applicaiton.
// 
// Return value:
//   None
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::AppWriteCmdData(AppCmdDataType* pMsg, long lWaitTime)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;

    if (pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    result = MotServiceMemWrite(&m_AppCmdData, (void*)pMsg, lWaitTime);

    if (result != MOT_SERVICE_WRITE_SUCCESS)
    {
        m_MotServiceDiagInfo.dwWriteSkipCounts++;
    }

    return result;
}

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// AppReadMotionData
//
// Function Description:
// This function is called by the external applicaiton.
// 
// Return value:
//   None
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::AppReadMotionData(RTMotionDataType* pMsg, long lWaitTime)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;

    if (pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    result = MotServiceMemRead(&m_RTMotionData, pMsg, lWaitTime);

    if (result != MOT_SERVICE_READ_SUCCESS)
    {
        m_MotServiceDiagInfo.dwReadSkipCounts++;
    }

    return result;
} // AppReadMotionData


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// RTReadAppCmdData
//
// Function Description:
// This function is called by the MotionKernel task without blocking.
// Return value:
//   None
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::RTReadAppCmdData(AppCmdDataType* pMsg)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;

    if (pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    result = MotServiceMemRead(&m_AppCmdData, pMsg, NO_WAIT);

    if (result != MOT_SERVICE_READ_SUCCESS)
    {
        m_MotServiceDiagInfo.dwReadSkipCounts++;
    }

    return result;
} // RTReadAppCmdData

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// RTWriteMotionData
//
// Function Description:
// This function is called by the MotionKernel task without blocking.
// 
// Return value:
//   None
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::RTWriteMotionData(RTMotionDataType* pMsg)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;

    if (pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    result = MotServiceMemWrite(&m_RTMotionData, (void*)pMsg, NO_WAIT);

    if (result != MOT_SERVICE_WRITE_SUCCESS)
    {
        m_MotServiceDiagInfo.dwWriteSkipCounts++;
    }

    return result;
}
// RTWriteMotionData


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// ReadMotionServiceDiagInfo
//
// Function Description:
// Read internal diagnostic data.
// 
// Return value:
//   None
//
///////////////////////////////////////////////////////////////////////////////
MotionServiceDiagInfoType MotionService::ReadMotionServiceDiagInfo(void)
{
    return m_MotServiceDiagInfo;
} // ReadMotionServiceDiagInfo

/////////////////////////////////////////////////////////////////////
// Name:
// MotServiceMemWrite
//
// Function Description:
//
// Return value:
// 
///////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::MotServiceMemWrite(MotServiceMemType* pMem, void* pMsg, long lWaitTime)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;
    long  dwWaitingResult = 0, dwPrevious = 0, drPrevious = 0;
    int    iOffset = 0;

    if (pMem == NULL || pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    dwWaitingResult = OsWaitForSingleObject(pMem->hWriteSemaphore, lWaitTime);
    switch (dwWaitingResult)
    {
    case WAIT_OBJECT_0:
        iOffset = (pMem->iDataNum) * (pMem->iMsgSize);
        memcpy((pMem->pDataHead + iOffset), pMsg, pMem->iMsgSize);
        pMem->iDataNum++;
        pMem->iDataNum %= (pMem->iMaxMsgNum);
        //Increase reading message
        OsReleaseSemaphore(pMem->hReadSemaphore, 1, &dwPrevious);
        result = MOT_SERVICE_WRITE_SUCCESS;
        break;

    case WAIT_TIMEOUT:
        result = MOT_SERVICE_TIMEOUT;
        break;

    default:
        break;
    }

    return result;
}

/////////////////////////////////////////////////////////////////////
// Name:
// MotServiceMemRead
//
// Function Description:
//
// Return value:
// 
///////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::MotServiceMemRead(MotServiceMemType* pMem, void* pMsg, long lWaitTime)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;
    long  dwWaitingResult = 0, dwPrevious = 0, drPrevious = 0;
    int    iOffset = 0;

    if (pMem == NULL || pMsg == NULL)
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    dwWaitingResult = OsWaitForSingleObject(pMem->hReadSemaphore, lWaitTime);
    switch (dwWaitingResult)
    {
    case WAIT_OBJECT_0:
        iOffset = (pMem->iDataNum) * (pMem->iMsgSize);
        memcpy(pMsg, (pMem->pDataHead + iOffset), pMem->iMsgSize);
        pMem->iDataNum++;
        pMem->iDataNum %= pMem->iMaxMsgNum;

        //Increase writing message
        OsReleaseSemaphore(pMem->hWriteSemaphore, 1, NULL);
        result = MOT_SERVICE_READ_SUCCESS;
        break;

    case WAIT_TIMEOUT:
        result = MOT_SERVICE_TIMEOUT;
        break;

    default:
        break;
    }

    return result;
}

/////////////////////////////////////////////////////////////////////
// Name:
// MotServiceMemInit
//
// Function Description:
//
// Return value:
// 
///////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::MotServiceMemInit(MotServiceMemType* pMem, int iMsgSize, int iMaxMsgNum, const char* pName)
{
    MotServiceMemType* pData = NULL;
    char  bTemp[MOT_SERVICE_NAME_SIZE] = { 0 };

    if ((pMem == NULL) || (pName == NULL))
    {
        return MOT_SERVICE_BAD_PARAMETER_FAILURE;
    }

    strcpy_s(pMem->MemName, MOT_SERVICE_NAME_SIZE, pName);
    pMem->iMaxMsgNum = iMaxMsgNum;
    pMem->iMsgSize = iMsgSize;

    //Shared memory buffer
#ifdef UNDER_WIN32
    pMem->hSharedMemory = OsOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, pMem->MemName, (void**)&(pMem->pDataHead), (pMem->iMsgSize * pMem->iMaxMsgNum));
#else
    pMem->hSharedMemory = OsOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, pMem->MemName, (void**)&(pMem->pDataHead));
#endif
    if (pMem->hSharedMemory == NULL)
    {
#ifndef RTX64_V4
        pMem->hSharedMemory = OsCreateSharedMemory(PAGE_READWRITE, 0, (pMem->iMsgSize * pMem->iMaxMsgNum), pMem->MemName, (void**)&(pMem->pDataHead));
#else
        pMem->hSharedMemory = OsCreateSharedMemory(SHM_MAP_ALL_ACCESS, 0, (pMem->iMsgSize * pMem->iMaxMsgNum), pMem->MemName, (void**)&(pMem->pDataHead));
#endif
    }

    if (pMem->hSharedMemory == NULL)
    {
        return MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
    }

    //Write Semaphore
    snprintf(&bTemp[0], MOT_SERVICE_NAME_SIZE, "%s_%s", pName, MOT_SERVICE_WRITE_SEMAPHORE);
    pMem->hWriteSemaphore = OsOpenSemaphore(NULL, FALSE, bTemp);
    if (pMem->hWriteSemaphore == NULL)
    {
        pMem->hWriteSemaphore = OsCreateSemaphore(NULL, pMem->iMaxMsgNum, pMem->iMaxMsgNum, bTemp);
    }

    if (pMem->hWriteSemaphore == NULL)
    {
        return MOT_SERVICE_CREATE_OPEN_WRITE_SEMAPHORE_FAILURE;
    }

    //Read Semaphore
    snprintf(&bTemp[0], MOT_SERVICE_NAME_SIZE, "%s_%s", pName, MOT_SERVICE_READ_SEMAPHORE);
    pMem->hReadSemaphore = OsOpenSemaphore(NULL, FALSE, bTemp);
    if (pMem->hReadSemaphore == NULL)
    {
        pMem->hReadSemaphore = OsCreateSemaphore(NULL, 0, pMem->iMaxMsgNum, bTemp);
    }

    if (pMem->hReadSemaphore == NULL)
    {
        return MOT_SERVICE_CREATE_OPEN_READ_SEMAPHORE_FAILURE;
    }

    //Initialize the shared memory 
    memset(pMem->pDataHead, 0x00, (pMem->iMaxMsgNum) * (pMem->iMsgSize));
    pMem->iDataNum = 0;

    return MOT_SERVICE_INIT_SUCCESS;
}

bool MotionService::ClearSemaphores() {
    //std::cout << "Clearing SMR semaphore counts for next experiment..." << std::endl;

    //bool success = true;

    //// Clear RTMotionData semaphores (output buffer - what extraction reads from)
    //// Drain read semaphore to 0 (no data to read)
    //int drainedReads = 0;
    //while (WaitForSingleObject(m_RTMotionData.hReadSemaphore, 0) == WAIT_OBJECT_0) {
    //    drainedReads++;
    //}
    //std::cout << "Drained " << drainedReads << " from RTMotionData read semaphore" << std::endl;

    //// **NEW: Zero out the RTMotionData buffer contents**
    //if (m_RTMotionData.pDataHead) {
    //    size_t bufferSize = m_RTMotionData.iMsgSize * m_RTMotionData.iMaxMsgNum;
    //    memset(m_RTMotionData.pDataHead, 0, bufferSize);
    //    std::cout << "Zeroed RTMotionData buffer (" << bufferSize << " bytes)" << std::endl;
    //}

    //// **NEW: Reset RTMotionData buffer pointer to start**
    //m_RTMotionData.iDataNum = 0;

    //// Reset write semaphore to full capacity
    //LONG prevWriteCount;
    //if (ReleaseSemaphore(m_RTMotionData.hWriteSemaphore, m_RTMotionData.iMaxMsgNum, &prevWriteCount)) {
    //    std::cout << "Reset RTMotionData write semaphore from " << prevWriteCount
    //        << " to " << m_RTMotionData.iMaxMsgNum << std::endl;
    //}
    //else {
    //    success = false;
    //}

    //// Clear AppCmdData semaphores (input buffer - what injection writes to)  
    //// Drain read semaphore to 0 (no commands to read)
    //int drainedCmds = 0;
    //while (WaitForSingleObject(m_AppCmdData.hReadSemaphore, 0) == WAIT_OBJECT_0) {
    //    drainedCmds++;
    //}
    //std::cout << "Drained " << drainedCmds << " from AppCmdData read semaphore" << std::endl;

    //// **NEW: Zero out the AppCmdData buffer contents**
    //if (m_AppCmdData.pDataHead) {
    //    size_t bufferSize = m_AppCmdData.iMsgSize * m_AppCmdData.iMaxMsgNum;
    //    memset(m_AppCmdData.pDataHead, 0, bufferSize);
    //    std::cout << "Zeroed AppCmdData buffer (" << bufferSize << " bytes)" << std::endl;
    //}

    //// **NEW: Reset AppCmdData buffer pointer to start**
    //m_AppCmdData.iDataNum = 0;

    //
    //// Reset write semaphore to full capacity
    //LONG prevCmdWriteCount;
    //if (ReleaseSemaphore(m_AppCmdData.hWriteSemaphore, m_AppCmdData.iMaxMsgNum, &prevCmdWriteCount)) {
    //    std::cout << "Reset AppCmdData write semaphore from " << prevCmdWriteCount
    //        << " to " << m_AppCmdData.iMaxMsgNum << std::endl;
    //}
    //else {
    //    success = false;
    //}

    //std::cout << "Semaphore clearing " << (success ? "completed successfully" : "failed") << std::endl;
    //return success;

    // Drain all read semaphores to 0 - NO OLD DATA SIGNALS
    while (WaitForSingleObject(m_RTMotionData.hReadSemaphore, 0) == WAIT_OBJECT_0) {}
    while (WaitForSingleObject(m_AppCmdData.hReadSemaphore, 0) == WAIT_OBJECT_0) {}

    // Reset write semaphores to full capacity
    while (WaitForSingleObject(m_RTMotionData.hWriteSemaphore, 0) == WAIT_OBJECT_0) {}
    while (WaitForSingleObject(m_AppCmdData.hWriteSemaphore, 0) == WAIT_OBJECT_0) {}
    ReleaseSemaphore(m_RTMotionData.hWriteSemaphore, m_RTMotionData.iMaxMsgNum, nullptr);
    ReleaseSemaphore(m_AppCmdData.hWriteSemaphore, m_AppCmdData.iMaxMsgNum, nullptr);

    // CRITICAL: Reset buffer pointers to start
    m_RTMotionData.iDataNum = 0;
    m_AppCmdData.iDataNum = 0;

    return true;
}