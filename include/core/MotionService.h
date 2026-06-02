////////////////////////////////////////////////////////////////////////////////
//
// MotionService Header
//
// Description:
// Contains the definitions of motion service data structures and APIs
// Simplified for linear axes only (X, Y, Z)
//
// Module Define
//
#pragma once
#ifndef MOTIONSERVICE_H
#define MOTIONSERVICE_H
//
// Include files:
//
// Unity Header
#ifndef UNITY
#include <windows.h>
#include <cstring>
#include "MPMCQueue.h"


constexpr int X_AXIS = 0;
constexpr int Y_AXIS = 1;
constexpr int Z_AXIS = 2;
#endif

#else
#define MOTIONSERVICE_CPP
#endif	// UNITY
//
/////////////////////////////////////////////////////////////////////////////
#ifndef MOTIONSERVICE_CPP      // define scope for include file
#define SCOPE extern
#else
#define SCOPE
#endif

/////////////////////////////////////////////////////////////////////////////
// Name:                   CONSTANT DEFINITIONS
/////////////////////////////////////////////////////////////////////////////
#define MOT_SERVICE_NAME_SIZE  80
#define NO_WAIT                0
#define WAIT_FOREVER           INFINITE
#define WAIT_1_MS              1
#define WAIT_2_MS              2

constexpr int NUM_SHARED_AXES = 3;

/////////////////////////////////////////////////////////////////////////////
// Name:                      TYPE DEFINITIONS
/////////////////////////////////////////////////////////////////////////////

// Enhanced RT Motion Data Structure for CNC Data Collection
typedef struct RTMotionDataType
{
  // Basic motion data (3 linear axes only)
  double dPosition[NUM_SHARED_AXES];

  // Enhanced data for CNC research (NEW)
  double dDeviation[NUM_SHARED_AXES];           // Position deviations applied
  double dVffApplied[NUM_SHARED_AXES];          // VFF signals currently applied
  double dEncoderError[NUM_SHARED_AXES];        // Encoder-based following error
  double dScaleError[NUM_SHARED_AXES];          // Scale-based following error
  unsigned long long ullRTSequence;             // Monotonic RT-side sample counter; lets the App detect dropped / duplicated samples
  int iCurrentLineNumber;                       // Current G-code line number (-1 if no motion)
  bool bHasMotion;

  RTMotionDataType()
  {
    for (int i = 0; i < NUM_SHARED_AXES; i++)
    {
      dPosition[i] = 0.0;
      dDeviation[i] = 0.0;
      dVffApplied[i] = 0.0;
      dEncoderError[i] = 0.0;
      dScaleError[i] = 0.0;
    }
    ullRTSequence     = 0;
    iCurrentLineNumber = -1;
    bHasMotion = false;
  }
} RTMotionDataType;

// Enhanced App Command Data Structure for CNC Data Collection
typedef struct AppCmdDataType
{
  // Basic command data (3 linear axes only)
  double dCmdVelOffset[NUM_SHARED_AXES];        // Feedforward velocity commands
  // Enhanced data for CNC research (NEW)
  double dDeviationCmd[NUM_SHARED_AXES];        // Position deviation commands
  int iTargetLineNumber;                        // Target G-code line number for this command

  AppCmdDataType()
  {
    for (int i = 0; i < NUM_SHARED_AXES; i++)
    {
      dCmdVelOffset[i] = 0.0;
      dDeviationCmd[i] = 0.0;
    }
    iTargetLineNumber = -1;
  }
} AppCmdDataType;

typedef struct MotionServiceDiagInfoType
{
  unsigned int dwWriteSkipCounts;
  unsigned int dwReadSkipCounts;

  MotionServiceDiagInfoType()
  {
    dwWriteSkipCounts = 0;
    dwReadSkipCounts = 0;
  }
}MotionServiceDiagInfoType;

typedef enum MOT_SERVICE_RETURN_CODE
{
  MOT_SERVICE_UNKNOWN,
  MOT_SERVICE_INIT_SUCCESS,
  MOT_SERVICE_BAD_PARAMETER_FAILURE,
  MOT_SERVICE_MEM_CREATE_OPEN_FAILURE,
  MOT_SERVICE_CREATE_OPEN_READ_SEMAPHORE_FAILURE,
  MOT_SERVICE_CREATE_OPEN_WRITE_SEMAPHORE_FAILURE,
  MOT_SERVICE_WRITE_SUCCESS,
  MOT_SERVICE_READ_SUCCESS,
  MOT_SERVICE_TIMEOUT,
  MOT_SERVICE_UNKNOWN_FAILURE,
} MOT_SERVICE_RETURN_CODE, * PMOT_SERVICE_RETURN_CODE;

/////////////////////////////////////////////////////////////////////////////
//                        CLASS DEFINITION
/////////////////////////////////////////////////////////////////////////////
class MotionService
{
public:
  MotionService(void);
  virtual ~MotionService(void);

  //Common APIs
  MOT_SERVICE_RETURN_CODE InitMotionService(int* pErrCode);          //Init internal data memory.
  MotionServiceDiagInfoType ReadMotionServiceDiagInfo(void);

  //APIs called by the application
  MOT_SERVICE_RETURN_CODE AppReadMotionData(RTMotionDataType* pMsg, long lWaitTime); //Read motion data from RT
  MOT_SERVICE_RETURN_CODE AppWriteCmdData(AppCmdDataType* pMsg, long lWaitTime);     //Send command data to RT
  bool AppSetInputBufferFlushRequest(bool request);                                  // App flushes the App->RT command queue
  bool AppCheckInputBufferEmpty() const;                                             // App checks whether the App->RT command queue is empty

  // APIs called by RT.
  MOT_SERVICE_RETURN_CODE RTWriteMotionData(RTMotionDataType* pMsg); //Send motion data from RT.
  MOT_SERVICE_RETURN_CODE RTReadAppCmdData(AppCmdDataType* pMsg);    //Read command data to RT
  unsigned long           RTAppCmdCount() const;                     // Current depth of the App->RT command queue


private:
  // Queue capacities must be powers of 2 (MPMCQueue requirement). 8192 ~= the
  // previous 8000-slot semaphore rings, preserving the ~2 s buffering window at
  // a 1 ms data rate.
  static constexpr unsigned long kRTMotionQueueSize = 8192;
  static constexpr unsigned long kAppCmdQueueSize   = 8192;

  // Order matters: SMRBuffer members must be declared before the queues that
  // bind to them, since C++ initializes members in declaration order.
  SMRBuffer                                          m_RTMotionDataBuffer;  // RT -> App SMR
  SMRBuffer                                          m_AppCmdDataBuffer;    // App -> RT SMR
  MPMCQueue<RTMotionDataType, kRTMotionQueueSize>    m_RTMotionQueue;       // RT -> App
  MPMCQueue<AppCmdDataType,   kAppCmdQueueSize>      m_AppCmdQueue;         // App -> RT

  MotionServiceDiagInfoType m_MotServiceDiagInfo;   //Internal diagnostic data
};

// End of MotionService definitions

#undef SCOPE
//#endif
