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
#define CONTROL_FLAGS_NAME           "ExperimentControlFlags"

//Default time interval of every exchanging data between RT and the external application is 1ms. 
//The allocated shared memory can hold two seconds data
#define MOTION_SERVICE_MEM_SIZE      8000



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
    controlFlags_ = nullptr;
    controlFlagsMemory_ = nullptr;
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

    if (controlFlagsMemory_ != NULL) {
        OsCloseHandle(controlFlagsMemory_);
    }
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
        *pErrCode = result;
    }

    result = MotServiceMemInit(&m_AppCmdData, sizeof(AppCmdDataType), MOTION_SERVICE_MEM_SIZE, APP_CMD_DATA_NAME);
    if (result != MOT_SERVICE_INIT_SUCCESS)
    {
        *pErrCode = result;
    }
    return result;

    result = initializeControlFlags();
    if (result != MOT_SERVICE_INIT_SUCCESS) {
        *pErrCode = result;
    }
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

bool MotionService::AppCheckInputBufferEmpty() const
{
    if (controlFlags_ == nullptr) {
        return true; // If uninitialized, assume empty to avoid blocking
    }
    return controlFlags_->rt_input_buffer_empty;
}
bool MotionService::AppSetInputBufferFlushRequest(bool request)
{
    if (controlFlags_ == nullptr) {
        return false;
    }
    controlFlags_->app_requests_input_flush = request;
    return true;
}


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
        RTSetInputBufferEmpty(true); // Read times out when empty
    }
    else
    {
        RTSetInputBufferEmpty(false); // Successfully read, so not empty
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

bool MotionService::RTCheckInputFlushRequest() const
{
    if (controlFlags_ == nullptr) {
        return false;
    }
    return controlFlags_->app_requests_input_flush;
}

bool MotionService::RTSetInputBufferEmpty(bool request)
{
    if (controlFlags_ == nullptr) {
        return false;
    }
    controlFlags_->rt_input_buffer_empty = request;
    return true;
}

void MotionService::RTFlushAppCmdBuffer(void)
{
    MOT_SERVICE_RETURN_CODE result = MOT_SERVICE_UNKNOWN;
    int iCounter = 1000;
    do
    {
        AppCmdDataType msg;
        result = MotServiceMemRead(&m_AppCmdData, &msg, NO_WAIT);;
        iCounter--;
    } while (result == MOT_SERVICE_READ_SUCCESS && iCounter > 0);
    RTSetInputBufferEmpty(true); // After flushing, buffer is empty
}


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
    long  dwWaitingResult = 0, dwPrevious = 0;
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
    long  dwWaitingResult = 0, dwPrevious = 0;
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

MOT_SERVICE_RETURN_CODE MotionService::initializeControlFlags() {
    // Try to open existing shared memory first
#ifdef UNDER_WIN32
    controlFlagsMemory_ = OsOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, CONTROL_FLAGS_NAME,
        (void**)&controlFlags_, sizeof(ExperimentControlFlags));
#else
    controlFlagsMemory_ = OsOpenSharedMemory(SHM_MAP_ALL_ACCESS, FALSE, CONTROL_FLAGS_NAME,
        (void**)&controlFlags_);
#endif

    if (controlFlagsMemory_ == NULL) {
        // Create new shared memory if it doesn't exist
#ifndef RTX64_V4
        controlFlagsMemory_ = OsCreateSharedMemory(PAGE_READWRITE, 0, sizeof(ExperimentControlFlags),
            CONTROL_FLAGS_NAME, (void**)&controlFlags_);
#else
        controlFlagsMemory_ = OsCreateSharedMemory(SHM_MAP_ALL_ACCESS, 0, sizeof(ExperimentControlFlags),
            CONTROL_FLAGS_NAME, (void**)&controlFlags_);
#endif
    }

    if (controlFlagsMemory_ == NULL) {
        return MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
    }

    // Initialize the structure 
    if (controlFlags_) {
        controlFlags_->app_requests_input_flush = false;
        controlFlags_->rt_input_buffer_empty = true;
    }

    return MOT_SERVICE_INIT_SUCCESS;
}