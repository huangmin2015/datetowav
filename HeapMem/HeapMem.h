/*
 * Copyright (c) 2008-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file       ti/ipc/HeapMem.h
 *
 *  @brief      Multi-processor variable size buffer heap implementation
 *
 *  @note       HeapMem is currently only available for SYS/BIOS.
 *
 *  HeapMem is a heap implementation that manages variable size buffers that
 *  can be used in a multiprocessor system with shared memory. HeapMem
 *  manages a single buffer in shared memory from which blocks of user-
 *  specified length are allocated and freed.
 *
 *  The HeapMem module uses a NameServer instance to
 *  store instance information when an instance is created.  The name supplied
 *  must be unique for all HeapMem instances.
 *
 *  HeapMem_create() initializes the shared memory as needed. Once an
 *  instance is created, HeapMem_open() can be called. The
 *  open is used to gain access to the same HeapMem instance.
 *  Generally an instance is created on one processor and opened on the
 *  other processor(s).
 *
 *  The open returns a HeapMem instance handle like the create,
 *  however the open does not modify the shared memory.
 *
 *  Because HeapMem is a variable-size heap implementation, it is also used
 *  by the SharedRegion module to manage shared memory in each shared
 *  region.  When any shared memory IPC instance is created in a
 *  particular shared region, the HeapMem that manages shared memory in the
 *  shared region allocates shared memory for the instance.
 *
 *  The HeapMem header should be included in an application as follows:
 *  @code
 *  #include <ti/ipc/HeapMem.h>
 *  @endcode
 */

#ifndef ti_ipc_HeapMem__include
#define ti_ipc_HeapMem__include

#if defined (__cplusplus)
extern "C" {
#endif

#include <ti/ipc/GateMP.h>

/* =============================================================================
 *  All success and failure codes for the module
 * =============================================================================
 */

/*!
 *  @brief  The resource is still in use
 */
#define HeapMem_S_BUSY               2

/*!
 *  @brief  The module has been already setup
 */
#define HeapMem_S_ALREADYSETUP       1

/*!
 *  @brief  Operation is successful.
 */
#define HeapMem_S_SUCCESS            0

/*!
 *  @brief  Generic failure.
 */
#define HeapMem_E_FAIL              -1

/*!
 *  @brief  Argument passed to function is invalid.
 */
#define HeapMem_E_INVALIDARG        -2

/*!
 *  @brief  Operation resulted in memory failure.
 */
#define HeapMem_E_MEMORY            -3

/*!
 *  @brief  The specified entity already exists.
 */
#define HeapMem_E_ALREADYEXISTS     -4

/*!
 *  @brief  Unable to find the specified entity.
 */
#define HeapMem_E_NOTFOUND          -5

/*!
 *  @brief  Operation timed out.
 */
#define HeapMem_E_TIMEOUT           -6

/*!
 *  @brief  Module is not initialized.
 */
#define HeapMem_E_INVALIDSTATE      -7

/*!
 *  @brief  A failure occurred in an OS-specific call  */
#define HeapMem_E_OSFAILURE         -8

/*!
 *  @brief  Specified resource is not available  */
#define HeapMem_E_RESOURCE          -9

/*!
 *  @brief  Operation was interrupted. Please restart the operation  */
#define HeapMem_E_RESTART           -10

/* =============================================================================
 *  Structures & Enums
 * =============================================================================
 */

/*!
 *  @brief  HeapMem_Handle type
 */
typedef struct HeapMem_Object *HeapMem_Handle;

/*!
 *  @brief  Structure defining parameters for the HeapMem module.
 *
 *  @sa     HeapMem_create()
 */
typedef struct HeapMem_Params {
    String name;
    /*!< Name of this instance.
     *
     *  The name (if not NULL) must be unique among all HeapMem
     *  instances in the entire system.  When creating a new
     *  heap, it is necessary to supply an instance name.
     *
     *  The name does not have to be persistent.  The supplied string is copied
     *  into persistent memory.
     */

    UInt16 regionId;
    /*!< Shared region ID
     *
     *  The index corresponding to the shared region from which shared memory
     *  will be allocated.
     */

    /*! @cond */
    Ptr sharedAddr;
    /*!< Physical address of the shared memory
     *
     *  This value can be left as 'null' unless it is required to place the
     *  heap at a specific location in shared memory.  If sharedAddr is null,
     *  then shared memory for a new instance will be allocated from the
     *  heap belonging to the region identified by #HeapMem_Params.regionId.
     */
    /*! @endcond */

    SizeT sharedBufSize;
    /*!< Size of shared buffer
     *
     *  This is the size of the buffer to be used in the HeapMem instance.
     *  The actual buffer size in the created instance might actually be less
     *  than the value supplied in 'sharedBufSize' because of alignment
     *  constraints.
     *
     *  It is important to note that the total amount of shared memory required
     *  for a HeapMem instance will be greater than the size supplied here.
     *  Additional space will be consumed by shared instance attributes and
     *  alignment-related padding.
     */

    GateMP_Handle gate;
    /*!< GateMP used for critical region management of the shared memory
     *
     *  Using the default value of NULL will result in use of the GateMP
     *  system gate for context protection.
     */

} HeapMem_Params;

/*!
 *  @brief  Stats structure for HeapMem_getExtendedStats()
 *
 *  @sa     HeapMem_getExtendedStats()
 */
typedef struct HeapMem_ExtendedStats {
    Ptr   buf;
    /*!< Local address of the shared buffer */

    SizeT size;
    /*!< Size of the shared buffer */
} HeapMem_ExtendedStats;

/*!
 *  @brief  Structure defining config parameters for the HeapMem module.
 */
typedef struct HeapMem_Config_tag {
    UInt32      maxNameLen;
    /*!< Maximum length of name */
    UInt32      maxRunTimeEntries;
    /*!< Maximum number of entries */
} HeapMem_Config;

/* =============================================================================
 *  HeapMem g_module-wide Functions
 * =============================================================================
 */
/*!
 *  @brief      Get the default configuration for the HeapMem module.
 *
 *              This function can be called by the application to get their
 *              configuration parameter to HeapMem_setup filled in by
 *              the HeapMem module with the default parameters. If the
 *              user does not wish to make any change in the default parameters,
 *              this API is not required to be called.
 *
 *  @param      cfgParams  Pointer to the HeapMem module configuration
 *                         structure in which the default config is to be
 *                         returned.
 *
 *  @sa         HeapMem_setup
 */
Void HeapMem_getConfig (HeapMem_Config * cfgParams);

/*!
 *  @brief      Setup the HeapMem module.
 *
 *              This function sets up the HeapMem module. This function
 *              must be called before any other instance-level APIs can be
 *              invoked.
 *              Module-level configuration needs to be provided to this
 *              function. If the user wishes to change some specific config
 *              parameters, then HeapMem_getConfig can be called to get
 *              the configuration filled with the default values. After this,
 *              only the required configuration values can be changed. If the
 *              user does not wish to make any change in the default parameters,
 *              the application can simply call HeapMem_setup with NULL
 *              parameters. The default parameters would get automatically used.
 *
 *  @param      cfg   Optional HeapMem module configuration. If provided
 *                    as NULL, default configuration is used.
 *
 *  @sa        HeapMem_destroy
 *             HeapMem_getConfig
 *             NameServer_create
 *             NameServer_delete
 *             GateMutex_delete
 */
Int HeapMem_setup (const HeapMem_Config * config);

/*!
 *  @brief      Close a HeapMem instance
 *
 *  Closing an instance will free local memory consumed by the opened
 *  instance. All opened instances should be closed before the instance
 *  is deleted.
 *
 *  @param[in,out]  handlePtr   Pointer to handle returned from
 *                              HeapMem_open()
 *
 *  @return     HeapMem status:
 *              - #HeapMem_S_SUCCESS: Heap successfully closed
 *
 *  @sa         HeapMem_open()
 */
Int HeapMem_close(HeapMem_Handle *handlePtr);

/*!
 *  @brief      Create a HeapMem instance
 *
 *  @param[in]  params      HeapMem parameters
 *
 *  @return     HeapMem Handle
 *
 *  @sa         HeapMem_delete()
 */
HeapMem_Handle HeapMem_create(const HeapMem_Params *params);

/*!
 *  @brief      Delete a created HeapMem instance
 *
 *  @param[in,out]  handlePtr   Pointer to handle to delete.
 *
 *  @return     HeapMem status:
 *              - #HeapMem_S_SUCCESS: Heap successfully deleted
 *
 *  @sa         HeapMem_create()
 */
Int HeapMem_delete(HeapMem_Handle *handlePtr);

/*!
 *  @brief      Open a created HeapMem instance
 *
 *  Once an instance is created, an open can be performed. The
 *  open is used to gain access to the same HeapMem instance.
 *  Generally an instance is created on one processor and opened on the
 *  other processor.
 *
 *  The open returns a HeapMem instance handle like the create,
 *  however the open does not initialize the shared memory. The supplied
 *  name is used to identify the created instance.
 *
 *  Call #HeapMem_close when the opened instance is not longer needed.
 *
 *  @param[in]  name        Name of created HeapMem instance
 *  @param[out] handlePtr   Pointer to HeapMem handle to be opened
 *
 *  @return     HeapMem status:
 *              - #HeapMem_S_SUCCESS: Heap successfully opened
 *              - #HeapMem_E_NOTFOUND: Heap is not yet ready to be opened.
 *              - #HeapMem_E_FAIL: A general failure has occurred
 *
 *  @sa         HeapMem_close
 */
Int HeapMem_open(String name, HeapMem_Handle *handlePtr);

/*! @cond */
Int HeapMem_openByAddr(Ptr sharedAddr, HeapMem_Handle *handlePtr);

/*! @endcond */

/*!
 *  @brief      Initialize a HeapMem parameters struct
 *
 *  @param[out] params      Pointer to creation parameters
 *
 *  @sa         HeapMem_create()
 */
Void HeapMem_Params_init(HeapMem_Params *params);

/*! @cond */
/*!
 *  @brief      Amount of shared memory required for creation of each instance
 *
 *  @param[in]  params      Pointer to the parameters that will be used in
 *                          the create.
 *
 *  @return     Number of MAUs needed to create the instance.
 */
SizeT HeapMem_sharedMemReq(const HeapMem_Params *params);

/*! @endcond */

/* =============================================================================
 *  HeapMem Per-instance Functions
 * =============================================================================
 */

/*!
 *  @brief      Allocate a block of memory of specified size and alignment
 *
 *  The actual block returned may be larger than requested to satisfy
 *  alignment requirements. NULL is returned if the alloc fails.
 *
 *  HeapMem_alloc will lock the heap using the HeapMem gate
 *  while it traverses the list of free blocks to find a large enough block
 *  for the request.
 *
 *  Guidelines for using large heaps and multiple alloc() calls.
 *      - If possible, allocate larger blocks first. Previous allocations
 *        of small memory blocks can reduce the size of the blocks
 *        available for larger memory allocations.
 *      - Realize that allocation can fail even if the heap contains a
 *        sufficient absolute amount of unallocated space. This is
 *        because the largest free memory block may be smaller than
 *        total amount of unallocated memory.
 *
 *  @param[in]  handle    Handle to previously created/opened instance.
 *  @param[in]  size      Size to be allocated (in MADUs)
 *  @param[in]  align     Alignment for allocation (power of 2)
 *
 *  @sa         HeapMem_free()
 */
Void *HeapMem_alloc(HeapMem_Handle handle, SizeT size, SizeT align);

/*!
 *  @brief      Frees a block of memory.
 *
 *  HeapMem_free() places the memory block specified by addr and size back
 *  into the free pool of the heap specified. The newly freed block is combined
 *  with any adjacent free blocks. The space is then available for future
 *  allocations.
 *
 *  HeapMem_free() will lock the heap using the HeapMem gate if one is
 *  specified or the system GateMP if not.
 *
 *  @param[in]  handle    Handle to previously created/opened instance.
 *  @param[in]  block     Block of memory to be freed.
 *  @param[in]  size      Size to be freed (in MADUs)
 *
 *  @sa         HeapMem_alloc()
 */
Void HeapMem_free(HeapMem_Handle handle, Ptr block, SizeT size);

/*!
 *  @brief      Get extended memory statistics
 *
 *  This function retrieves extended statistics for a HeapMem
 *  instance. Refer to #HeapMem_ExtendedStats for more information
 *  regarding what information is returned.
 *
 *  @param[in]  handle    Handle to previously created/opened instance.
 *  @param[out] stats     ExtendedStats structure
 *
 *  @sa     HeapMem_getStats
 */
Void HeapMem_getExtendedStats(HeapMem_Handle handle,
                                HeapMem_ExtendedStats *stats);

/*!
 *  @brief      Get memory statistics
 *
 *  @param[in]  handle    Handle to previously created/opened instance.
 *  @param[out] stats     Memory statistics structure
 *
 *  @sa     HeapMem_getExtendedStats()
 */
Void HeapMem_getStats(HeapMem_Handle handle, Ptr stats);

/*!
 *  @brief      Restore an instance to it's original created state.
 *
 *  This function restores an instance to
 *  its original created state. Any memory previously allocated from the
 *  heap is no longer valid after this API is called. This function
 *  does not check whether there is allocated memory or not.
 *
 *  @param[in]  handle    Handle to previously created/opened instance.
 */
Void HeapMem_restore(HeapMem_Handle handle);

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* ti_ipc_HeapMem__include */
