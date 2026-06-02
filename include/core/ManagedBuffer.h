#pragma once
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#include <Windows.h>
#include <string>
#include <RTUk.h>

class ManagedBuffer
{
public:
  enum ReturnCode
  {
    UNKNOWN = 0,
    SUCCESS,
    FAILURE,
    ALREADY_INITIALIZED,
  };


  void* m_pBuff;
  bool  m_bIsInitialized;

  ManagedBuffer() noexcept : m_pBuff(nullptr), m_bIsInitialized(false)
  { /* Pass */ }
  virtual ReturnCode Init(unsigned long BufferSize) = 0;
  virtual void Destroy() = 0;
  virtual ~ManagedBuffer() = default;
};

class SMRBuffer: public ManagedBuffer
{
  HANDLE _hSMR;
  std::string _Name;
public:
  SMRBuffer(std::string Name) noexcept : ManagedBuffer(), _hSMR(nullptr), _Name(Name) 
  { /* Pass */ }

  ReturnCode Init(unsigned long BufferSize) final
  {
    ReturnCode ret = ReturnCode::UNKNOWN;
    if(!m_bIsInitialized)
    {
      _hSMR = OsOpenSharedMemory(SMR_ACCESS_READ_WRITE, NULL, (this->_Name + "_SMR").c_str(), (void**)&(this->m_pBuff), BufferSize);
      if(_hSMR == NULL)
      {
        // Create SMR if it does not exist
        _hSMR = OsCreateSharedMemory(SMR_ACCESS_READ_WRITE, 0, BufferSize, (this->_Name + "_SMR").c_str(), (void**)&(this->m_pBuff));
        memset(this->m_pBuff, 0, BufferSize);
        ret = ReturnCode::SUCCESS;
      }
      else
      {
        ret = ReturnCode::ALREADY_INITIALIZED; // If someone else already created the SMR, its their responsibility to initalize it
      }

      if(_hSMR == NULL || this->m_pBuff == nullptr)
      {
        return ReturnCode::FAILURE;
      }
      m_bIsInitialized = true;
    }
    else
    {
      ret = ReturnCode::ALREADY_INITIALIZED;
    }
    return ret;
  }

  void Destroy()
  {
    if(_hSMR != nullptr)
    {
      OsCloseHandle(_hSMR);
    }
    this->m_pBuff = nullptr;
    _hSMR = nullptr;
    m_bIsInitialized = false;
  }

  ~SMRBuffer()
  {
    Destroy();
  }
};

class LocalBuffer: public ManagedBuffer
{
public:
  LocalBuffer() noexcept : ManagedBuffer()
  { /* Pass */ }

  ReturnCode Init(unsigned long BufferSize) final
  {
    if(!m_bIsInitialized)
    {
      this->m_pBuff = new char[BufferSize];
      if(this->m_pBuff == nullptr)
      {
        return ReturnCode::FAILURE;
      }
      memset(this->m_pBuff, 0, BufferSize);
      m_bIsInitialized = true;
      return ReturnCode::SUCCESS;
    }
    return ReturnCode::ALREADY_INITIALIZED;
  }

  void Destroy()
  {
    if(this->m_pBuff != nullptr)
    {
      delete[] this->m_pBuff;
    }
    this->m_pBuff = nullptr;
    m_bIsInitialized = false;
  }

  ~LocalBuffer()
  {
    Destroy();
  }
};