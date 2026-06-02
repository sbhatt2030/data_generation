////////////////////////////////////////////////////////////////////////////////
//
// Name:
//  MotionService Module
//
//  Description:
//
//  This module provides data exchanging interfaces between the external motion
//  control application and the internal RT motion control through two named
//  shared-memory regions: one carrying RT motion telemetry to the app, the
//  other carrying command data from the app to RT. Each direction is backed by
//  a lock-free MPMC FIFO (Libraries/RTContainers/MPMCQueue.h) sitting on an
//  SMRBuffer, so the hot path performs no kernel transitions.
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
#include "core/MotionService.h"
#include <cstring>
#include <cstddef>
#endif	// UNITY
//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Name:               CONSTANT DEFINITIONS
//////////////////////////////////////////////////////////////////////////////
#define RT_MOTION_DATA_NAME          "RTMotionData"
#define APP_CMD_DATA_NAME            "AppCmdData"

// Poll cadence (passed to OsSleep) used by the App-side Read/Write APIs when
// the caller supplies a non-zero lWaitTime and the queue is momentarily
// empty/full. Matches the idiom used by RTConnect.cpp.
static constexpr unsigned long APP_POLL_INTERVAL = 100;


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
  : m_RTMotionDataBuffer(RT_MOTION_DATA_NAME),
    m_AppCmdDataBuffer(APP_CMD_DATA_NAME),
    m_RTMotionQueue(&m_RTMotionDataBuffer),
    m_AppCmdQueue(&m_AppCmdDataBuffer),
    m_MotServiceDiagInfo()
{
}

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// ~MotionService
//
// Function Description:
// Class Destructor. The SMRBuffer / MPMCQueue members own and close their
// shared-memory handles, so the destructor body is empty.
//
///////////////////////////////////////////////////////////////////////////////
MotionService::~MotionService(void)
{
}


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// InitMotionService
//
// Function Description:
// Initialize the two MPMC queues used to ferry data between the App and RT.
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::InitMotionService(int* pErrCode)
{
  if (pErrCode == NULL)
  {
    return MOT_SERVICE_BAD_PARAMETER_FAILURE;
  }

  if (m_RTMotionQueue.Init() != MPMCRetCode::SUCCESS)
  {
    *pErrCode = MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
    return MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
  }

  if (m_AppCmdQueue.Init() != MPMCRetCode::SUCCESS)
  {
    *pErrCode = MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
    return MOT_SERVICE_MEM_CREATE_OPEN_FAILURE;
  }

  return MOT_SERVICE_INIT_SUCCESS;
} // InitMotionService

  ///////////////////////////////////////////////////////////////////////////////
  //
  // Name:
  // AppWriteCmdData
  //
  // Function Description:
  // App-side producer for the App -> RT command stream. On BUFFER_FULL and a
  // non-zero wait, polls until the deadline elapses.
  //
  ///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::AppWriteCmdData(AppCmdDataType* pMsg, long lWaitTime)
{
  if (pMsg == NULL)
  {
    return MOT_SERVICE_BAD_PARAMETER_FAILURE;
  }

  const bool bWaitForever = (static_cast<DWORD>(lWaitTime) == WAIT_FOREVER);
  const ULONGLONG deadline = bWaitForever
                               ? 0
                               : GetTickCount64() + static_cast<DWORD>(lWaitTime);

  while (true)
  {
    MPMCRetCode ret = m_AppCmdQueue.Enqueue(*pMsg);
    if (ret == MPMCRetCode::SUCCESS)
    {
      return MOT_SERVICE_WRITE_SUCCESS;
    }
    if (ret != MPMCRetCode::BUFFER_FULL)
    {
      m_MotServiceDiagInfo.dwWriteSkipCounts++;
      return MOT_SERVICE_UNKNOWN_FAILURE;
    }

    if (lWaitTime == NO_WAIT || (!bWaitForever && GetTickCount64() >= deadline))
    {
      m_MotServiceDiagInfo.dwWriteSkipCounts++;
      return MOT_SERVICE_TIMEOUT;
    }

    OsSleep(APP_POLL_INTERVAL);
  }
}

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// AppReadMotionData
//
// Function Description:
// App-side consumer for the RT -> App motion telemetry stream. On
// BUFFER_EMPTY and a non-zero wait, polls until the deadline elapses.
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::AppReadMotionData(RTMotionDataType* pMsg, long lWaitTime)
{
  if (pMsg == NULL)
  {
    return MOT_SERVICE_BAD_PARAMETER_FAILURE;
  }

  const bool bWaitForever = (static_cast<DWORD>(lWaitTime) == WAIT_FOREVER);
  const ULONGLONG deadline = bWaitForever
                               ? 0
                               : GetTickCount64() + static_cast<DWORD>(lWaitTime);

  while (true)
  {
    MPMCRetCode ret = m_RTMotionQueue.Dequeue(*pMsg);
    if (ret == MPMCRetCode::SUCCESS)
    {
      return MOT_SERVICE_READ_SUCCESS;
    }
    if (ret != MPMCRetCode::BUFFER_EMPTY)
    {
      m_MotServiceDiagInfo.dwReadSkipCounts++;
      return MOT_SERVICE_UNKNOWN_FAILURE;
    }

    if (lWaitTime == NO_WAIT || (!bWaitForever && GetTickCount64() >= deadline))
    {
      m_MotServiceDiagInfo.dwReadSkipCounts++;
      return MOT_SERVICE_TIMEOUT;
    }

    OsSleep(APP_POLL_INTERVAL);
  }
} // AppReadMotionData

bool MotionService::AppCheckInputBufferEmpty() const
{
  return m_AppCmdQueue.IsEmpty();
}

// Preserves the original signature for external App-process consumers, but
// now performs the flush directly rather than asking RT to do it. A `false`
// argument is a no-op (there is no longer a latched request flag to clear).
bool MotionService::AppSetInputBufferFlushRequest(bool request)
{
  if (!request)
  {
    return true;
  }
  return m_AppCmdQueue.Flush() == MPMCRetCode::SUCCESS;
}


  ///////////////////////////////////////////////////////////////////////////////
  //
  // Name:
  // RTReadAppCmdData
  //
  // Function Description:
  // RT-side consumer. Single non-blocking Dequeue; lock-free, no kernel
  // transitions.
  //
  ///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::RTReadAppCmdData(AppCmdDataType* pMsg)
{
  if (pMsg == NULL)
  {
    return MOT_SERVICE_BAD_PARAMETER_FAILURE;
  }

  MPMCRetCode ret = m_AppCmdQueue.Dequeue(*pMsg);
  if (ret == MPMCRetCode::SUCCESS)
  {
    return MOT_SERVICE_READ_SUCCESS;
  }

  m_MotServiceDiagInfo.dwReadSkipCounts++;
  if (ret == MPMCRetCode::BUFFER_EMPTY)
  {
    return MOT_SERVICE_TIMEOUT;
  }
  return MOT_SERVICE_UNKNOWN_FAILURE;
} // RTReadAppCmdData

///////////////////////////////////////////////////////////////////////////////
//
// Name:
// RTWriteMotionData
//
// Function Description:
// RT-side producer. Single non-blocking Enqueue.
//
///////////////////////////////////////////////////////////////////////////////
MOT_SERVICE_RETURN_CODE MotionService::RTWriteMotionData(RTMotionDataType* pMsg)
{
  if (pMsg == NULL)
  {
    return MOT_SERVICE_BAD_PARAMETER_FAILURE;
  }

  MPMCRetCode ret = m_RTMotionQueue.Enqueue(*pMsg);
  if (ret == MPMCRetCode::SUCCESS)
  {
    return MOT_SERVICE_WRITE_SUCCESS;
  }

  m_MotServiceDiagInfo.dwWriteSkipCounts++;
  if (ret == MPMCRetCode::BUFFER_FULL)
  {
    return MOT_SERVICE_TIMEOUT;
  }
  return MOT_SERVICE_UNKNOWN_FAILURE;
}
// RTWriteMotionData

unsigned long MotionService::RTAppCmdCount() const
{
  return m_AppCmdQueue.Count();
}


///////////////////////////////////////////////////////////////////////////////
//
// Name:
// ReadMotionServiceDiagInfo
//
// Function Description:
// Read internal diagnostic data.
//
///////////////////////////////////////////////////////////////////////////////
MotionServiceDiagInfoType MotionService::ReadMotionServiceDiagInfo(void)
{
  return m_MotServiceDiagInfo;
} // ReadMotionServiceDiagInfo
