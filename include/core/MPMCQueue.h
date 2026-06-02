#pragma once
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#include <Windows.h>
#include "ManagedBuffer.h"

/// <summary>
/// Defines return codes for function or operation outcomes.
/// </summary>
enum class MPMCRetCode
{
  UNKNOWN = 0,
  SUCCESS,
  FAILURE,
  BUFFER_EMPTY,
  BUFFER_FULL,
  NOT_INITIALIZED,
};

/// <summary>
/// A lock-free MPMC FIFO queue implementation with a fixed-size buffer.
/// Uses atomic operations without any mutexes or semaphores for maximum performance.
/// Note: NumSamples MUST be a power of 2
/// </summary>
/// <typeparam name="SampleType">The type of the individual samples stored in the buffer.</typeparam>
/// <typeparam name="NumSamples">The maximum number of samples the buffer can hold (must be power of 2).</typeparam>
/// <typeparam name="InfoType">The type used for storing sample metadata or information.</typeparam>
template<typename SampleType, unsigned long NumSamples, typename InfoType = void*>
class MPMCQueue
{
  // Compile-time check that NumSamples is a power of 2
  static_assert((NumSamples& (NumSamples - 1)) == 0, "NumSamples must be a power of 2");
  static constexpr unsigned long INDEX_MASK = NumSamples - 1;

  // Compile-time check that SampleType has = operator
  static_assert(std::is_copy_assignable<SampleType>::value, "SampleType must be copy assignable");

  // Compile-time check that InfoType has = operator (if not void*)
  static_assert(std::is_copy_assignable<InfoType>::value || std::is_void<InfoType>::value, "InfoType must be copy assignable");

  // Managed Buffer Object
  ManagedBuffer* m_ManagedBuffer;

  enum SlotStatus : CHAR
  {
    SLOT_EMPTY = 0,
    SLOT_FULL  = 1,
  };

  bool m_bInitialized = false;
  bool m_bInfoTypeIsVoid = std::is_void<InfoType>::value;

  ////////////////////////////////////////////////////////////////////////////
  // Buffer Layout
  ////////////////////////////////////////////////////////////////////////////
  struct
  {
    struct // ControlFlags
    {
      // Use Windows types for interlocked operations
      volatile LONG WriteIndex;
      volatile LONG ReadIndex;

      // Slot status, used to make sure that last operation is complete before performing next 
      volatile CHAR SlotState[NumSamples];
    } ControlFlags;

    InfoType SampleInfo;
    SampleType Buffer[NumSamples];

    void Init()
    {
      ControlFlags.WriteIndex = 0;
      ControlFlags.ReadIndex = 0;
      SampleInfo = InfoType();
      for(unsigned long SlotIndex = 0; SlotIndex < NumSamples; ++SlotIndex)
      {
        ControlFlags.SlotState[SlotIndex] = SLOT_EMPTY;
      }
    }
  }*m_pBuffer = nullptr; // Pointer to the shared buffer structure

public:
  MPMCQueue(ManagedBuffer* Buffer) noexcept : m_ManagedBuffer(Buffer)
  {}

  /// <summary>
  /// Destructor for the MPMCQueue class. Releases resources.
  /// </summary>
  ~MPMCQueue()
  {
    m_bInitialized = false;
    m_ManagedBuffer->Destroy();
    m_pBuffer = nullptr;
  }

  /// <summary>
  /// Initializes all information needed for the queue and the underlying buffer.
  /// No semaphores or mutexes are created - this is a pure lock-free implementation.
  /// </summary>
  /// <returns>A value of MPMCRetCode type indicating the outcome of the initialization process.</returns>
  MPMCRetCode Init()
  {
    ManagedBuffer::ReturnCode Ret = m_ManagedBuffer->Init(sizeof(*m_pBuffer));
    m_pBuffer = static_cast<decltype(m_pBuffer)>(m_ManagedBuffer->m_pBuff);

    // Get typed pointer to buffer
    switch(Ret)
    {
      case ManagedBuffer::ReturnCode::SUCCESS:
      {
        // Initialize the buffer structure manually for the first client
        m_pBuffer->Init();
        break;
      }

      case ManagedBuffer::ReturnCode::ALREADY_INITIALIZED:
      {
        // Nothing to do if already initialized
        break;
      }

      case ManagedBuffer::ReturnCode::FAILURE:
      default:
      {
        m_ManagedBuffer->Destroy();
        m_pBuffer = nullptr;
        return MPMCRetCode::FAILURE;
      }
    }

    m_bInitialized = true;
    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Writes a sample to the buffer using lock-free atomic operations.
  /// Returns immediately with BUFFER_FULL if no space is available.
  /// This implementation is safe for multiple concurrent writers (MP).
  /// </summary>
  /// <param name="Sample">The sample data to be written to the buffer.</param>
  /// <returns>A MPMCRetCode indicating the result of the write operation</returns>
  virtual MPMCRetCode Enqueue(const SampleType& Sample)
  {
    if(!m_bInitialized)
    {
      return MPMCRetCode::NOT_INITIALIZED;
    }

    unsigned long CurrentWrite;
    unsigned long CurrentRead;
    unsigned long SlotIndex;

    // Try to atomically claim a write slot
    do
    {
      // Take snapshot of current state
      MemoryBarrier();
      CurrentWrite = static_cast<unsigned long>(m_pBuffer->ControlFlags.WriteIndex);
      CurrentRead  = static_cast<unsigned long>(m_pBuffer->ControlFlags.ReadIndex);

      // Calculate number of items in queue
      unsigned long ItemsInQueue = CurrentWrite - CurrentRead;

      // Calculate actual buffer slot (wrap around using power-of-2 mask)
      SlotIndex = CurrentWrite & INDEX_MASK;

      MemoryBarrier();
      // Check to make sure there is space in the buffer and the slot has been read
      if(ItemsInQueue >= NumSamples || m_pBuffer->ControlFlags.SlotState[SlotIndex] != SLOT_EMPTY)
      {
        return MPMCRetCode::BUFFER_FULL;
      }

      // Try to atomically increment WriteIndex from CurrentWrite to CurrentWrite+1
      // If another thread changed WriteIndex, this will fail and we retry
      // This ensures that only one thread can claim this slot for writing, and the others will retry and see the updated state
    } while(InterlockedCompareExchange(&m_pBuffer->ControlFlags.WriteIndex,
                                       static_cast<LONG>(CurrentWrite + 1), // This must be unsigned arithmetic to ensure correct wrap-around behavior
                                       static_cast<LONG>(CurrentWrite)) != static_cast<LONG>(CurrentWrite));

    // Write data to buffer
    m_pBuffer->Buffer[SlotIndex] = Sample;
    MemoryBarrier(); // Ensure write is complete before updating slot state
    InterlockedExchange8(&m_pBuffer->ControlFlags.SlotState[SlotIndex], SLOT_FULL); // Update SlotState

    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Reads a sample from the buffer using lock-free atomic operations.
  /// Returns immediately with BUFFER_EMPTY if no data is available.
  /// This implementation is safe for multiple concurrent readers (MC).
  /// </summary>
  /// <param name="Sample">Reference to a SampleType object where the read sample will be stored.</param>
  /// <returns>A MPMCRetCode indicating the result of the read operation</returns>
  virtual MPMCRetCode Dequeue(SampleType& Sample)
  {
    if(!m_bInitialized)
    {
      return MPMCRetCode::NOT_INITIALIZED;
    }

    unsigned long CurrentRead;
    unsigned long CurrentWrite;
    unsigned long SlotIndex;

    // Try to atomically claim a read slot
    do
    {
      MemoryBarrier();
      // Take snapshot of current state
      CurrentRead  = static_cast<unsigned long>(m_pBuffer->ControlFlags.ReadIndex);
      CurrentWrite = static_cast<unsigned long>(m_pBuffer->ControlFlags.WriteIndex);

      // Calculate available items
      unsigned long ItemsAvailable = CurrentWrite - CurrentRead;
      
      // Calculate actual buffer slot (wrap around using power-of-2 mask)
      SlotIndex = CurrentRead & INDEX_MASK;

      MemoryBarrier();
      // Check to make sure there is data in the buffer and the slot has been filled
      if(ItemsAvailable == 0 || m_pBuffer->ControlFlags.SlotState[SlotIndex] != SLOT_FULL)
      {
        return MPMCRetCode::BUFFER_EMPTY;
      }

      // Try to atomically increment ReadIndex from CurrentRead to CurrentRead+1
      // If another thread changed ReadIndex, this will fail and we retry
      // This ensures that only one thread can claim this slot for reading, and the others will retry and see the updated state
    } while(InterlockedCompareExchange(&m_pBuffer->ControlFlags.ReadIndex,
                                       static_cast<LONG>(CurrentRead + 1), // This must be unsigned arithmetic to ensure correct wrap-around behavior
                                       static_cast<LONG>(CurrentRead)) != static_cast<LONG>(CurrentRead));       

    // Now safe to read data from buffer
    Sample = m_pBuffer->Buffer[SlotIndex];
    MemoryBarrier(); // Ensure Read is complete before updating slot state
    InterlockedExchange8(&m_pBuffer->ControlFlags.SlotState[SlotIndex], SLOT_EMPTY);

    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Flushes all elements from the buffer by reading and discarding them until the buffer is empty.
  /// </summary>
  /// <returns>Returns NOT_INITIALIZED if the queue has not been initialised; SUCCESS after the buffer has been completely emptied.</returns>
  MPMCRetCode Flush()
  {
    if(!m_bInitialized)
    {
      return MPMCRetCode::NOT_INITIALIZED;
    }
    SampleType Sample;
    // This could probably be slightly faster by directly manipulating the indices and slot states
    // but this is safer and leverages existing logic
    while(Dequeue(Sample) != MPMCRetCode::BUFFER_EMPTY);
    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Retrieves queue info structure containing sample metadata or information.
  /// </summary>
  /// <param name="Info">A reference to a InfoType object that will be populated with the buffer's sample information.</param>
  /// <returns>A MPMCRetCode indicating the result: NOT_INITIALIZED if the buffer is null, or SUCCESS if the information was successfully retrieved.</returns>
  MPMCRetCode GetInfo(InfoType& Info) const
  {
    if(m_pBuffer == nullptr || m_bInfoTypeIsVoid)
    {
      return MPMCRetCode::NOT_INITIALIZED;
    }
    Info = m_pBuffer->SampleInfo;
    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Sets the queue info structure with new sample metadata or information.
  /// </summary>
  /// <param name="Info">A reference to a InfoType object containing the new sample information.</param>
  /// <returns>A MPMCRetCode indicating the result: NOT_INITIALIZED if the buffer is null, or SUCCESS if the information was successfully set.</returns>
  MPMCRetCode SetInfo(const InfoType& Info)
  {
    if(m_pBuffer == nullptr || m_bInfoTypeIsVoid)
    {
      return MPMCRetCode::NOT_INITIALIZED;
    }
    m_pBuffer->SampleInfo = Info;
    return MPMCRetCode::SUCCESS;
  }

  /// <summary>
  /// Gets the number of samples.
  /// </summary>
  /// <returns>The number of samples.</returns>
  unsigned long Size() const
  {
    return NumSamples;
  }

  /// <summary>
  /// Gets the current number of items in the queue (approximate, as it may change immediately after return)
  /// </summary>
  unsigned long Count() const
  {
    if(m_pBuffer == nullptr)
    {
      return 0;
    }
    unsigned long CurrentWrite = static_cast<unsigned long>(m_pBuffer->ControlFlags.WriteIndex);
    unsigned long CurrentRead = static_cast<unsigned long>(m_pBuffer->ControlFlags.ReadIndex);
    return CurrentWrite - CurrentRead;
  }

  /// <summary>
  /// Gets the available space in the queue (approximate)
  /// </summary>
  unsigned long AvailableSpace() const
  {
    return Size() - Count();
  }

  /// <summary>
  /// Checks whether the buffer is empty.
  /// </summary>
  /// <returns>True if the buffer is empty; false if the buffer is null or not empty.</returns>
  bool IsEmpty() const
  {
    if(m_pBuffer == nullptr)
    {
      return true;
    }
    return (Count() == 0);
  }
  
  /// <summary>
  /// Checks whether the buffer is full.
  /// </summary>
  /// <returns>True if the buffer is full; false if the buffer is null or not full.</returns>
  bool IsFull() const
  {
    if(m_pBuffer == nullptr)
    {
      return false;
    }
    return (Count() >= NumSamples);
  }
};