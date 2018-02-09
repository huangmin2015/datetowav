/*
 *  @file   HeapMem.c
 *
 *  @brief      Defines HeapMem based memory allocator.
 *
 *  HeapMem is a heap implementation that manages variable size buffers that
 *  can be used in a multiprocessor system with shared memory. HeapMem
 *  manages a single buffer in shared memory from which blocks of user-
 *  specified length are allocated and freed.
 *
 *  The HeapMem module uses a NameServerinstance to
 *  store instance information when an instance is created.  The name supplied
 *  must be unique for all HeapMem instances.
 *
 *  The create initializes the shared memory as needed. Once an
 *  instance is created, an open can be performed. The
 *  open is used to gain access to the same HeapMem instance.
 *  Generally an instance is created on one processor and opened on the
 *  other processor(s).
 *
 *  The open returns a HeapMem instance handle like the create,
 *  however the open does not modify the shared memory.
 *
 *  Because HeapMem is a variable-size heap implementation, it is also used
 *  by Shared Region to manage shared memory in each shared
 *  region.  When any shared memory IPC instance is created in a
 *  particular shared region, the HeapMem that manages shared memory in the
 *  shared region allocates shared memory for the instance.
 *
 *
 *
 *  ============================================================================
 *
 *  Copyright (c) 2008-2017, Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  
 *  *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  Contact information for paper mail:
 *  Texas Instruments
 *  Post Office Box 655303
 *  Dallas, Texas 75265
 *  Contact information: 
 *  http://www-k.ext.ti.com/sc/technical-support/product-information-centers.htm?
 *  DCMP=TIHomeTracking&HQS=Other+OT+home_d_contact
 *  ============================================================================
 *  
 */

/* Standard headers */
#include <assert.h>
#include <string.h>
#include <stdlib.h>

/* IPC headers */
#include <ti/ipc/Std.h>
#include "Std.h"
#include <ti/ipc/MultiProc.h>
#include <SharedRegion.h>
#include <IGateProvider.h>
#include <GateMutex.h>

#include "List.h"
#include "HeapMem.h"
#include "Memory.h"
#include "Cache.h"

#include "_SharedRegion.h"

//#define ENABLE_LOCAL_LOCK

/* Module level headers */
#include <_HeapMem.h>
/* round up the value 'size' to the next 'align' boundary */
#define ROUNDUP(size, align) \
    (UInt32)(((UInt32)(size) + ((UInt32)(align) - 1)) & ~((UInt32)(align) - 1))

#define SYSLINK_BUILD_OPTIMIZE

/* =============================================================================
 * Globals
 * =============================================================================
 */
/*!
 *  @var    HeapMem_nameServer
 *
 *  @brief  Name of the reserved NameServer used for HeapMem.
 */
#define HeapMem_NAMESERVER  "HeapMem"

/*! @brief Macro to make a correct module magic number with refCount */
#define HeapMem_MAKE_MAGICSTAMP(x) ((HeapMem_MODULEID << 12u) | (x))

/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */
/*!
 *  @brief  Structure defining processor related information for the
 *          module.
 */
typedef struct HeapMem_ProcAttrs_tag {
    Bool   creator;   /*!< Creatoror opener */
    UInt16 procId;    /*!< Processor Identifier */
    UInt32 openCount; /*!< How many times it is opened on a processor */
} HeapMem_ProcAttrs;

/* Header */
typedef struct HeapMem_Header_tag {
    SharedRegion_SRPtr next;
    /* SRPtr to next header (Header *)    */
    Bits32             size;
    /* Size of this segment (Memory.size) */
} HeapMem_Header;

/*! Structure of attributes in shared memory */
typedef struct HeapMem_Attrs_tag {
    Bits32              status;
    /* Module status                 */
    SharedRegion_SRPtr  bufPtr;
    /* Memory managed by instance    */
    HeapMem_Header    head;
    /* HeapMem      head */
    SharedRegion_SRPtr  gateMPAddr;
    /* GateMP SRPtr (shm safe)       */
} HeapMem_Attrs;

/*!
 *  Structure for HeapMem module state
 */
typedef struct HeapMem_ModuleObject {
    UInt32                 refCount;
    /*!< Reference count */
    IGateProvider_Handle   localLock;
    /*!< Handle to lock for protecting objList */
    List_Object            objList;
    /*!< List holding created objects */
    HeapMem_Config       cfg;
    /*!< Current config values */
    HeapMem_Config       defaultCfg;
    /*!< Default config values */
    HeapMem_Params       defaultInstParams;
    /*!< Default instance creation parameters */
} HeapMem_ModuleObject;

/*!
 *  @brief  Structure for the Handle for the HeapMem.
 */
typedef struct HeapMem_Obj_tag {
    List_Elem           listElem;
    /* Used for creating a linked list */
    volatile HeapMem_Attrs   * attrs;
    /* Local pointer to attrs        */
    GateMP_Handle       gate;
    /* Gate for critical regions     */
    Ptr                 nsKey;
    /* Used to remove NS entry       */
    Bool                cacheEnabled;
    /* Whether to do cache calls     */
    UInt16              regionId;
    /* SharedRegion index            */
    UInt32              allocSize;
    /* Shared memory allocated       */
    Char              * buf;
    /* Local pointer to buf          */
    UInt32              minAlign;
    /* Minimum alignment required    */
    UInt32              bufSize;
    /* Buffer Size */
    HeapMem_ProcAttrs owner;
    /* Processor related information for owner processor */
    Ptr                 top;
    /* Pointer to the top Object */
    HeapMem_Params    params;
    /* instance creation parameters */
} HeapMem_Obj;

/* =============================================================================
 * Globals
 * =============================================================================
 */
#if !defined(SYSLINK_BUILD_DEBUG)
static
#endif /* if !defined(SYSLINK_BUILD_DEBUG) */
HeapMem_ModuleObject HeapMem_state =
{
    .defaultCfg.maxNameLen            = 32u,
    .defaultCfg.maxRunTimeEntries     = 32u,
    .defaultInstParams.gate           = NULL,
    .defaultInstParams.name           = NULL,
    .defaultInstParams.regionId       = 0u,
    .defaultInstParams.sharedAddr     = NULL,
    .defaultInstParams.sharedBufSize  = 0,
};

/*!
 *  @var    HeapMem_module
 *
 *  @brief  Pointer to the HeapMem module state.
 */
#if !defined(SYSLINK_BUILD_DEBUG)
static
#endif /* if !defined(SYSLINK_BUILD_DEBUG) */
HeapMem_ModuleObject * HeapMem_module = &HeapMem_state;

/* =============================================================================
 * Forward declaration of internal function
 * =============================================================================
 */
/*!
 *  @brief      Creates a new instance of HeapMem module.
 *              This is an internal function as both HeapMem_create
 *              and HeapMem_open use the functionality.
 *
 *  @param      handlePtr  Return value: Handle
 *  @param      params  Instance config-params structure.
 *  @param      createFlag Indicates whether this is a create or open call.
 *
 *  @sa         HeapMem_delete,
 *              ListMP_Params_init
 *              ListMP_sharedMemReq
 *              Gate_enter
 *              Gate_leave
 *              GateMutex_delete
 *              NameServer_addUInt32
 */
Int _HeapMem_create (      HeapMem_Handle * handlePtr,
                       const HeapMem_Params * params,
                             Bool               createFlag);

/* slice and dice the buffer */
Int HeapMem_postInit (HeapMem_Object * handle);

/* =============================================================================
 * APIS
 * =============================================================================
 */
/* Get the default configuration for the HeapMem module. */
Void
HeapMem_getConfig (HeapMem_Config * cfgParams)
{
    GT_1trace (curTrace, GT_ENTER, "HeapMem_getConfig", cfgParams);

    GT_assert (curTrace, (cfgParams != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (cfgParams == NULL) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getConfig",
                             HeapMem_E_INVALIDARG,
                             "Argument of type (HeapMem_Config *) passed "
                             "is null!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
    if ((HeapMem_module->refCount & HeapMem_MAKE_MAGICSTAMP(0)) < 
        HeapMem_MAKE_MAGICSTAMP(1)) {
            memcpy (cfgParams,
                         &HeapMem_module->defaultCfg,
                         sizeof (HeapMem_Config));
        }
        else {
            memcpy (cfgParams,
                         &HeapMem_module->cfg,
                         sizeof (HeapMem_Config));
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_0trace (curTrace, GT_ENTER, "HeapMem_getConfig");
}

/* Setup the HeapMem module. */
Int
HeapMem_setup (const HeapMem_Config * cfg)
{
    Int               status = HeapMem_S_SUCCESS;
    Error_Block       eb;
    HeapMem_Config  tmpCfg;

    GT_1trace (curTrace, GT_ENTER, "HeapMem_setup", cfg);
    Error_init (&eb);

    if (cfg == NULL) {
        HeapMem_getConfig (&tmpCfg);
        cfg = &tmpCfg;
    }

    if (cfg == NULL) {
        HeapMem_getConfig (&tmpCfg);
        cfg = &tmpCfg;
    }

    /* This sets the refCount variable is not initialized, upper 16 bits is
     * written with module Id to ensure correctness of refCount variable.
     */
    if((HeapMem_module->refCount & HeapMem_MAKE_MAGICSTAMP(0))
        != HeapMem_MAKE_MAGICSTAMP(0)) {
        HeapMem_module->refCount = HeapMem_MAKE_MAGICSTAMP(0);
    }
    if ( ++HeapMem_module->refCount
        != HeapMem_MAKE_MAGICSTAMP(1u)) {
        status = HeapMem_S_ALREADYSETUP;
        GT_0trace (curTrace,
                   GT_2CLASS,
                   "HeapMem Module already initialized!");
    }
    else {
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (cfg->maxNameLen == 0) {
            /*! @retval HeapMem_E_INVALIDARG cfg->maxNameLen passed is 0 */
            status = HeapMem_E_INVALIDARG;
            GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "HeapMem_setup",
                                 status,
                                 "cfg->maxNameLen passed is 0!");
        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

                /* Construct the list object */
                List_construct (&HeapMem_module->objList, NULL);
                /* Copy the cfg */
                memcpy ((Ptr) &HeapMem_module->cfg,
                             (Ptr) cfg,
                             sizeof (HeapMem_Config));
#ifdef ENABLE_LOCAL_LOCK
                /* Create a lock for protecting list object */
                HeapMem_module->localLock = (IGateProvider_Handle)
                               GateMutex_create ((GateMutex_Params*)NULL, &eb);
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (HeapMem_module->localLock == NULL) {
                    /*! @retval HeapMem_E_FAIL Failed to create the localLock*/
                    status = HeapMem_E_FAIL;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "HeapMem_setup",
                                         HeapMem_E_FAIL,
                                         "Failed to create the localLock!");
                    HeapMem_destroy ();
                }
            }
        }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
    }

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_setup", status);

    /*! @retval HeapMem_S_SUCCESS Operation successful */
    return status;
}

/* Function to destroy the HeapMem module. */
Int
HeapMem_destroy (void)
{
    Int           status = HeapMem_S_SUCCESS;
    List_Elem  *  elem;

    GT_0trace (curTrace, GT_ENTER, "HeapMem_destroy");

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        /*! @retval HeapMem_E_INVALIDSTATE Module was not initialized */
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_destroy",
                             status,
                             "Module was not initialized!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        if (   --HeapMem_module->refCount
            == HeapMem_MAKE_MAGICSTAMP(0)) {
            /* Temporarily increment refCount here. */
           HeapMem_module->refCount = 
                        HeapMem_MAKE_MAGICSTAMP(1);
            /* Check if any HeapMem instances have not been deleted so far.
             * If not, delete them.
             */
            List_traverse (elem, (List_Handle) &HeapMem_module->objList) {
                if (   (   ((HeapMem_Obj *) elem)->owner.procId
                    == MultiProc_self ())) {
                    status = HeapMem_delete ((HeapMem_Handle*)
                                             &(((HeapMem_Obj *) elem)->top));
                }
                else {
                    status = HeapMem_close ((HeapMem_Handle *)
                                              &(((HeapMem_Obj *) elem)->top));
                }
            }

            /* Decrease the refCount */
            HeapMem_module->refCount =
                        HeapMem_MAKE_MAGICSTAMP(0);

            /* Destruct the list object */
            List_destruct (&HeapMem_module->objList);
#ifdef ENABLE_LOCAL_LOCK
            /* Delete the list lock */
            if (HeapMem_module->localLock != NULL) {
                status = GateMutex_delete ((GateMutex_Handle *)
                                           &HeapMem_module->localLock);
                GT_assert (curTrace, (status >= 0));
                if (status < 0) {
                    /* Override the status to return a HeapMem status code. */
                    status = HeapMem_E_FAIL;
                }
            }
#endif
            memset (&HeapMem_module->cfg, 0, sizeof (HeapMem_Config));
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_destroy", status);

    /*! @retval HeapMem_S_SUCCESS Operation successful */
    return status;
}

/* Initialize this config-params structure with supplier-specified
 * defaults before instance creation.
 */
Void
HeapMem_Params_init (HeapMem_Params * params)
{
    GT_1trace (curTrace, GT_ENTER, "HeapMem_Params_init", params);

    GT_assert (curTrace, (params != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_Params_init",
                             HeapMem_E_INVALIDSTATE,
                             "Module was not initialized!");
    }
    else if (params == NULL) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_Params_init",
                             HeapMem_E_INVALIDARG,
                             "Argument of type (HeapMem_Params *) is "
                             "NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        memcpy (params,
                     &(HeapMem_module->defaultInstParams),
                     sizeof (HeapMem_Params));
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_Params_init");
}

/*!
 *  @brief      Creates a new instance of HeapMem module.
 *
 *  @param      params  Instance config-params structure.
 *
 *  @sa         HeapMem_delete,
 *              ListMP_Params_init
 *              ListMP_sharedMemReq
 *              Gate_enter
 *              Gate_leave
 *              NameServer_addUInt32
 */
HeapMem_Handle
HeapMem_create (const HeapMem_Params * params)
{
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    Int                status = HeapMem_S_SUCCESS;
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
    HeapMem_Object * handle = NULL;
    HeapMem_Params   sparams;

    GT_1trace (curTrace, GT_ENTER, "HeapMem_create", params);

    GT_assert (curTrace, (params != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        /*! @retval NULL if Module was not initialized */
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_create",
                             status,
                             "Module was not initialized!");
    }
    else if (params == NULL) {
        /*! @retval NULL if params pointer passed is NULL */
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_create",
                             status,
                             "params pointer passed is NULL!");
    }
    else if (params->sharedBufSize == 0) {
        /*! @retval HeapMem_E_INVALIDARG if Shared Buffer size cannot be 0
         */
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_create",
                             status,
                             "Shared buffer size is 0!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        memcpy (&sparams,
                     (Ptr)params,
                     sizeof (HeapMem_Params));
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        status =
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        _HeapMem_create ((HeapMem_Handle *) &handle, &sparams, TRUE);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (status < 0) {
            GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_create",
                             status,
                             "_HeapMem_create failed in HeapMem_create!");
        }
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_create", handle);

    /*! @retval Valid-Handle Operation successful*/
    return (HeapMem_Handle) handle;
}

/* Deletes an instance of HeapMem module. */
Int
HeapMem_delete (HeapMem_Handle * handlePtr)
{
    Int                status = HeapMem_S_SUCCESS;
    HeapMem_Object * handle = NULL;
    HeapMem_Obj    * obj    = NULL;
    HeapMem_Params * params = NULL;
    HeapMem_Handle * regionHeap = NULL;
#ifdef ENABLE_GATEMP
    IArg               key    = 0;
#endif
#ifdef ENABLE_LOCAL_LOCK
    IArg               key1   = 0;
#endif

    GT_1trace (curTrace, GT_ENTER, "HeapMem_delete", handlePtr);

    GT_assert (curTrace, (handlePtr != NULL));
    GT_assert (curTrace, ((handlePtr != NULL) && (*handlePtr != NULL)));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_delete",
                             status,
                             "Module was not initialized!");
    }
    else if (handlePtr == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_delete",
                             status,
                             "The parameter handlePtr i.e. pointer to handle "
                             "passed is NULL!");
    }
    else if (*handlePtr == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_delete",
                             status,
                             "Handle passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        handle = (HeapMem_Object *) (*handlePtr);

        obj    = (HeapMem_Obj *) handle->obj;

        if (obj != NULL) {
#if !defined(SYSLINK_BUILD_OPTIMIZE)
            /* Check if we have created the HeapMem or not */
            if (obj->owner.procId != MultiProc_self ()) {
                status = HeapMem_E_INVALIDSTATE;
                GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "HeapMem_delete",
                                 status,
                                 "Instance was not created on this processor!");
             }
             else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
#ifdef ENABLE_GATEMP
                /* Take the local lock */
               key = GateMP_enter (obj->gate);
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (obj->owner.openCount > 1) {
                    status = HeapMem_E_INVALIDSTATE;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "HeapMem_delete",
                                         status,
                                         "Unmatched open/close calls!");
#ifdef ENABLE_GATEMP
                    /* Release the global lock */
                    GateMP_leave (obj->gate, key);
#endif
                }
                else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
#ifdef ENABLE_LOCAL_LOCK
                    /* Remove from  the local list */
                    key1 = IGateProvider_enter (HeapMem_module->localLock);
#endif
                    List_remove ((List_Handle) &HeapMem_module->objList,
                                 &obj->listElem);
#ifdef ENABLE_LOCAL_LOCK
                    IGateProvider_leave (HeapMem_module->localLock, key1);
#endif

                    params = (HeapMem_Params *) &obj->params;

                    if (EXPECT_TRUE(params->name != NULL)) {

                        Memory_free (NULL,
                                     obj->params.name,
                                     strlen(obj->params.name) + 1u);
                    }

                    /* Set status to 'not created' */
                    if (obj->attrs != NULL) {
                        obj->attrs->status = 0;
                        if (obj->cacheEnabled) {
                            Cache_wbInv ((Ptr) obj->attrs,
                                         sizeof (HeapMem_Attrs),
                                         Cache_Type_ALL,
                                         TRUE);
                        }
                    }

#ifdef ENABLE_GATEMP
                    /* Release the shared lock */
                   GateMP_leave (obj->gate, key);
#endif

                    /* If necessary, free shared memory  if memory is internally
                     * allocated
                     */
                    regionHeap = SharedRegion_getHeap (obj->regionId);

                    if (   (regionHeap != NULL)
                        && (obj->params.sharedAddr == NULL)
                        && (obj->attrs != NULL)){
                        Memory_free ((IHeap_Handle) regionHeap,
                                     (Ptr) obj->attrs,
                                     obj->allocSize);
                    }

                    /* Now free the handle */
                    Memory_free (NULL, obj, sizeof (HeapMem_Obj));
                    Memory_free (NULL, handle, sizeof (HeapMem_Object));
                    *handlePtr = NULL;
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                }
            }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        }
        else {
            Memory_free (NULL, handle, sizeof (HeapMem_Object));
            *handlePtr = NULL;
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_delete", status);

    return status;
}

/*  Opens a created instance of HeapMem module. */
Int
HeapMem_open (String             name,
                HeapMem_Handle * handlePtr)
{
    Int                  status = HeapMem_S_SUCCESS;
    SharedRegion_SRPtr   sharedShmBase = SharedRegion_INVALIDSRPTR;
    Ptr                  sharedAddr    = NULL;
#ifdef ENABLE_LOCAL_LOCK
    IArg                 key    = 0;
#endif
    Bool                 doneFlag = FALSE;
    List_Elem      *     elem     = NULL;

    GT_2trace (curTrace, GT_ENTER, "HeapMem_open", name, handlePtr);

    GT_assert (curTrace, (name != NULL));
    GT_assert (curTrace, (handlePtr != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_open",
                             status,
                             "Module was not initialized!");
    }
    else if (name == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_open",
                             status,
                             "name passed is NULL!");
    }
    else if (handlePtr == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_open",
                             status,
                             "Pointer to handle instance passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* First check in the local list */
        List_traverse (elem, (List_Handle) &HeapMem_module->objList) {
            if (((HeapMem_Obj *) elem)->params.name != NULL) {
                if (strcmp (   ((HeapMem_Obj *)elem)->params.name, name)
                                == 0) {
#ifdef ENABLE_LOCAL_LOCK
                    key = IGateProvider_enter (HeapMem_module->localLock);
#endif
                    /* Check if we have created the HeapMem or not */
                    if (   ((HeapMem_Obj *) elem)->owner.procId
                        == MultiProc_self ()) {
                        ((HeapMem_Obj *) elem)->owner.openCount++;
                    }

                    *handlePtr = (((HeapMem_Obj *) elem)->top);
#ifdef ENABLE_LOCAL_LOCK
                    IGateProvider_leave (HeapMem_module->localLock, key);
#endif
                    doneFlag = TRUE;
                    break;
                }
            }
        }

        if (EXPECT_TRUE (doneFlag == FALSE)) {
#if !defined(SYSLINK_BUILD_OPTIMIZE)
            else if (EXPECT_FALSE (status < 0)) {
                /* Override the status to return a HeapMem status code. */
                status = HeapMem_E_FAIL;
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "HeapMem_open",
                                     status,
                                     "Failure in NameServer_get!");
            }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            {
                /* Convert from shared region pointer to local address*/
                sharedAddr = SharedRegion_getPtr (sharedShmBase);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (EXPECT_FALSE (sharedAddr == NULL)) {
                    status = HeapMem_E_FAIL;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "HeapMem_open",
                                         status,
                                         "Invalid pointer received from"
                                         "NameServer!");
                }
                else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    status = HeapMem_openByAddr (sharedAddr, handlePtr);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                    if (EXPECT_FALSE (status < 0)) {
                        GT_setFailureReason (curTrace,
                                             GT_4CLASS,
                                             "HeapMem_open",
                                             status,
                                            "Failed to open HeapMem handle!");
                    }
                }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            }
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_open", status);

    return status;
}

/* Closes previously opened instance of HeapMem module. */
Int
HeapMem_close (HeapMem_Handle * handlePtr)
{
    Int                status = HeapMem_S_SUCCESS;
    HeapMem_Object * handle = NULL;
    HeapMem_Obj    * obj    = NULL;
#ifdef ENABLE_LOCAL_LOCK
    IArg               key    = 0;
#endif
    GT_1trace (curTrace, GT_ENTER, "HeapMem_close", handlePtr);

    GT_assert (curTrace, (handlePtr != NULL));
    GT_assert (curTrace, ((handlePtr != NULL) && (*handlePtr != NULL)));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_close",
                             status,
                             "Module was not initialized!");
    }
    else if (handlePtr == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_close",
                             status,
                             "The parameter handlePtr i.e. pointer to handle "
                             "passed is NULL!");
    }
    else if (*handlePtr == NULL) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_close",
                             status,
                             "*handlePtr passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        handle = (HeapMem_Object *) (*handlePtr);
        obj    = (HeapMem_Obj *) handle->obj;
        if (obj != NULL) {
#ifdef ENABLE_LOCAL_LOCK
            key = IGateProvider_enter (HeapMem_module->localLock);
#endif
            if (obj->owner.procId == MultiProc_self ()) {
                obj->owner.openCount--;
            }

            /* Check if HeapMem is opened on same processor and this is the
             * last closure.
             */
            if (    (obj->owner.creator   == FALSE)
                &&  (obj->owner.openCount == 0)) {
                List_remove ((List_Handle) &HeapMem_module->objList,
                             &obj->listElem);

                if (obj->gate != NULL) {
#ifdef ENABLE_GATEMP
                    /* Close the  instance gate*/
                    GateMP_close (&obj->gate);
#endif
                }

                /* Now free the handle */
                Memory_free (NULL, obj, sizeof (HeapMem_Obj));
                obj = NULL;
                Memory_free (NULL, handle, sizeof (HeapMem_Object));
                *handlePtr = NULL;
            }

#ifdef ENABLE_LOCAL_LOCK
            IGateProvider_leave (HeapMem_module->localLock, key);
#endif
        }
        else {
            Memory_free (NULL, handle, sizeof (HeapMem_Object));
            *handlePtr = NULL;
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_close", status);

    return status;
}

/*
 * NOTE:
 * Embedded within the code for HeapMem_alloc and HeapMem_free are comments
 * that can be used to match a shared memory reference with its required
 * cache call.  This is done because the code for alloc and free is complex.
 * These two-character comments indicate
 * 1) The type of cache operation that is being performed {A, B}
 *    A = Cache_inv
 *    B = Cache_wbInv
 * 2) A numerical id of the specific cache call that is performed.
 *    1, 2, 3
 *    For example, the comment 'A2' indicates that the corresponding cache call
 *    is a Cache_inv operation identified by the number '2'
 */

/*
 *  ======== HeapMem_alloc ========
 *  HeapMem is implemented such that all of the memory and blocks it works
 *  with have an alignment that is a multiple of the minimum alignment and have
 *  a size which is a multiple of the minAlign. Maintaining this requirement
 *  throughout the implementation ensures that there are never any odd
 *  alignments or odd block sizes to deal with.
 *
 *  Specifically:
 *  The buffer managed by HeapMem:
 *    1. Is aligned on a multiple of obj->minAlign
 *    2. Has an adjusted size that is a multiple of obj->minAlign
 *  All blocks on the freelist:
 *    1. Are aligned on a multiple of obj->minAlign
 *    2. Have a size that is a multiple of obj->minAlign
 *  All allocated blocks:
 *    1. Are aligned on a multiple of obj->minAlign
 *    2. Have a size that is a multiple of obj->minAlign
 *
 */

/* Allocs a block */
Void *
HeapMem_alloc (HeapMem_Handle handle,
                 UInt32           reqSize,
                 UInt32           reqAlign)
{
    HeapMem_Obj    * obj    = NULL;
    Char             * allocAddr = NULL;
#ifdef ENABLE_GATEMP
    IArg             key    = 0;
#endif
    HeapMem_Header * prevHeader;
    HeapMem_Header * newHeader;
    HeapMem_Header * curHeader;
    UInt32           curSize;
    UInt32           adjSize;
    UInt32           remainSize; /* free memory after allocated memory */
    UInt32           adjAlign;
    UInt32           offset;

    GT_3trace (curTrace, GT_ENTER, "HeapMem_alloc",
               handle, reqSize, reqAlign);

    GT_assert (curTrace, (handle != NULL));
    GT_assert (curTrace, (reqSize != 0u));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_alloc",
                             HeapMem_E_INVALIDSTATE,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handle == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_alloc",
                             HeapMem_E_INVALIDARG,
                             "Invalid NULL handle pointer specified!");
    }
    else if (EXPECT_FALSE ((reqAlign & (reqAlign - 1)) != 0)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_alloc",
                             HeapMem_E_MEMORY,
                             "Requested reqAlign is not a power of 2!");
    }
    else if (EXPECT_FALSE (reqSize == 0)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_alloc",
                             HeapMem_E_INVALIDARG,
                             "Requested block size is zero!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        obj = (HeapMem_Obj *) ((HeapMem_Object *) handle)->obj;
        GT_assert (curTrace, (obj != NULL));

        adjSize = (UInt32) reqSize;

        /* Make size requested a multiple of obj->minAlign */
        if ((offset = (adjSize & (obj->minAlign - 1))) != 0) {
            adjSize = adjSize + (obj->minAlign - offset);
        }

        /*
         *  Make sure the alignment is at least as large as obj->minAlign
         *  Note: adjAlign must be a power of 2 (by function constraint) and
         *  obj->minAlign is also a power of 2,
         */
        adjAlign = reqAlign;
        if (adjAlign == 0) {
            adjAlign =  obj->minAlign;
        }

        if (adjAlign & (obj->minAlign - 1)) {
            /* adjAlign is less than obj->minAlign */
            adjAlign = obj->minAlign;
        }

        /* No need to Cache_inv Attrs- 'head' should be constant */
        prevHeader = (HeapMem_Header *) &obj->attrs->head;

#ifdef ENABLE_GATEMP
        key = GateMP_enter (obj->gate);
#endif

        /*
         *  The block will be allocated from curHeader. Maintain a pointer to
         *  prevHeader so prevHeader->next can be updated after the alloc.
         */
        if (EXPECT_FALSE (obj->cacheEnabled)) {
            Cache_inv (prevHeader,
                       sizeof (HeapMem_Header),
                       Cache_Type_ALL,
                       TRUE); /* A1 */
        }
        curHeader = (HeapMem_Header *) SharedRegion_getPtr (prevHeader->next);
        /* A1 */

        /* Loop over the free list. */
        while (curHeader != NULL) {
            /* Invalidate curHeader */
            if (EXPECT_FALSE (obj->cacheEnabled)) {
                Cache_inv (curHeader,
                           sizeof (HeapMem_Header),
                           Cache_Type_ALL,
                           TRUE); /* A2 */
            }

            curSize = curHeader->size;

            /*
             *  Determine the offset from the beginning to make sure
             *  the alignment request is honored.
             */
            offset = (UInt32)curHeader & (adjAlign - 1);
            if (offset) {
                offset = adjAlign - offset;
            }

            /* Internal Assert that offset is a multiple of obj->minAlign */
            if (((offset & (obj->minAlign - 1)) != 0)) {
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "HeapMem_alloc",
                                     HeapMem_E_FAIL,
                                     "offset is not a multiple of"
                                     " obj->minAlign!");
            }
            else {
                /* big enough? */
                if (curSize >= (adjSize + offset)) {
                    /* Set the pointer that will be returned. Alloc from front */
                    allocAddr = (Char *) ((UInt32) curHeader + offset);

                    /*
                     *  Determine the remaining memory after the allocated block.
                     *  Note: this cannot be negative because of above comparison.
                     */
                    remainSize = curSize - adjSize - offset;

                    /* Internal Assert that remainSize is a multiple of
                     * obj->minAlign
                     */
                    if (((remainSize & (obj->minAlign - 1)) != 0)) {
                        allocAddr = NULL;
                        GT_setFailureReason (curTrace,
                                             GT_4CLASS,
                                             "HeapMem_alloc",
                                             HeapMem_E_FAIL,
                                             "remainSize is not a multiple of"
                                             " obj->minAlign!");
                        break;
                    }
                    else {
                        /*
                         *  If there is memory at the beginning (due to alignment
                         *  requirements), maintain it in the list.
                         *
                         *  offset and remainSize must be multiples of
                         *  sizeof(HeapMem_Header). Therefore the address of the newHeader
                         *  below must be a multiple of the sizeof(HeapMem_Header), thus
                         *  maintaining the requirement.
                         */
                        if (offset) {
                            /* Adjust the curHeader size accordingly */
                            curHeader->size = offset; /* B2 */
                            /* Cache wb at end of this if block */

                            /*
                             *  If there is remaining memory, add into the free list.
                             *  Note: no need to coalesce and we have HeapMem locked so
                             *        it is safe.
                             */
                            if (remainSize) {
                                newHeader = (HeapMem_Header *)
                                            ((UInt32) allocAddr + adjSize);

                                /* curHeader has been inv at top of 'while' loop */
                                newHeader->next = curHeader->next;  /* B1 */
                                newHeader->size = remainSize;       /* B1 */
                                if (EXPECT_FALSE (obj->cacheEnabled)) {
                                    /* Writing back curHeader will cache-wait */
                                    Cache_wbInv (newHeader,
                                                 sizeof (HeapMem_Header),
                                                 Cache_Type_ALL,
                                                 FALSE); /* B1 */
                                }

                                curHeader->next = SharedRegion_getSRPtr
                                                                (newHeader,
                                                                 obj->regionId);
                                GT_assert (curTrace,
                                           (    curHeader->next
                                            !=  SharedRegion_INVALIDSRPTR));
                            }

                            /* Write back (and invalidate) newHeader and curHeader */
                            if (EXPECT_FALSE (obj->cacheEnabled)) {
                                /* B2 */
                                Cache_wbInv (curHeader,
                                             sizeof (HeapMem_Header),
                                             Cache_Type_ALL,
                                             TRUE);
                            }
                        }
                        else {
                            /*
                             *  If there is any remaining, link it in,
                             *  else point to the next free block.
                             *  Note: no need to coalesce and we have HeapMem locked so
                             *        it is safe.
                             */
                            if (remainSize) {
                                newHeader = (HeapMem_Header *)
                                            ((UInt32) allocAddr + adjSize);

                                newHeader->next  = curHeader->next; /* A2, B3  */
                                newHeader->size  = remainSize;      /* B3      */

                                if (EXPECT_FALSE (obj->cacheEnabled)) {
                                    /* Writing back prevHeader will cache-wait */
                                    Cache_wbInv (newHeader,
                                                 sizeof (HeapMem_Header),
                                                 Cache_Type_ALL,
                                                 FALSE); /* B3 */
                                }

                                /* B4 */
                                prevHeader->next = SharedRegion_getSRPtr (newHeader,
                                                                          obj->regionId);
                            }
                            else {
                                /* curHeader has been inv at top of 'while' loop */
                                prevHeader->next = curHeader->next; /* A2, B4 */
                            }

                            if (EXPECT_FALSE (obj->cacheEnabled)) {
                                /* B4 */
                                Cache_wbInv (prevHeader,
                                             sizeof(HeapMem_Header),
                                             Cache_Type_ALL,
                                             TRUE);
                            }
                        }
                    }

                    /* Success, return the allocated memory */
                    break;

                }
                else {
                    prevHeader = curHeader;
                    curHeader = SharedRegion_getPtr (curHeader->next);
                }
            }
        }

#ifdef ENABLE_GATEMP
        GateMP_leave (obj->gate, key);
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_alloc", allocAddr);

    return allocAddr;
}

/* Frees a block */
Void
HeapMem_free (HeapMem_Handle   handle,
                Ptr                addr,
                UInt32             size)
{
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    Int                status = HeapMem_S_SUCCESS;
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
    HeapMem_Obj    * obj    = NULL;
#ifdef ENABLE_GATEMP
    IArg               key    = 0;
#endif
    HeapMem_Header * curHeader = NULL;
    HeapMem_Header * newHeader = NULL;
    HeapMem_Header * nextHeader= NULL;
    SizeT              offset;

    GT_3trace (curTrace, GT_ENTER, "HeapMem_free", handle, addr, size);

    GT_assert (curTrace, (handle != NULL));
    GT_assert (curTrace, (addr != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_free",
                             status,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handle == NULL)) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_free",
                             status,
                             "Invalid NULL handle pointer specified!");
    }
    else if (EXPECT_FALSE (addr == NULL)) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_free",
                             status,
                             "Invalid NULL addr pointer specified!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        obj = (HeapMem_Obj *) ((HeapMem_Object *) handle)->obj;
        GT_assert (curTrace, (obj != NULL));
        GT_assert (curTrace, ((UInt32)addr % obj->minAlign == 0));

        /*
         * obj->attrs never changes, doesn't need Gate protection
         * and Cache invalidate
         */
        curHeader = (HeapMem_Header *) &(obj->attrs->head);

        /* Restore size to actual allocated size */
        offset = size & (obj->minAlign - 1);
        if (offset != 0) {
            size += obj->minAlign - offset;
        }

#ifdef ENABLE_GATEMP
        key = GateMP_enter (obj->gate);
#endif

        newHeader = (HeapMem_Header *) addr;

        if (EXPECT_FALSE (obj->cacheEnabled)) {
            /* A1 */
            Cache_inv (curHeader,
                       sizeof (HeapMem_Header),
                       Cache_Type_ALL,
                       TRUE);
        }
        nextHeader = SharedRegion_getPtr (curHeader->next);

#if !defined(SYSLINK_BUILD_OPTIMIZE)
        /* Make sure the entire buffer is in the range of the heap. */
        if (EXPECT_FALSE (!(    ((SizeT) newHeader >= (SizeT) obj->buf)
                            && (   (SizeT) newHeader + size
                                <= (SizeT) obj->buf + obj->bufSize)))) {
            status = HeapMem_E_FAIL;
            GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_free",
                             status,
                             "Entire buffer is not in the range of the heap!");
        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            /* Go down freelist and find right place for buf */
            while ((nextHeader != NULL) && (nextHeader < newHeader)) {
                if (EXPECT_FALSE (obj->cacheEnabled)) {
                    Cache_inv (nextHeader,
                               sizeof(HeapMem_Header),
                               Cache_Type_ALL,
                               TRUE); /* A2 */
                }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
                /* Make sure the addr is not in this free block */
                if (EXPECT_FALSE (    (SizeT) newHeader
                                  < ((SizeT) nextHeader + nextHeader->size))) {
                    /* A2 */
                    status = HeapMem_E_INVALIDSTATE;
                    GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "HeapMem_free",
                                     status,
                                     "Address is in this free block");
                    break;
                }
                else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    curHeader = nextHeader;
                    /* A2 */
                    nextHeader = SharedRegion_getPtr (nextHeader->next);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
            if (EXPECT_TRUE (status >= 0)) {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                /* B2 */
                newHeader->next = SharedRegion_getSRPtr (nextHeader,
                                                         obj->regionId);
                newHeader->size = size;

                /* B1, A1 */
                curHeader->next = SharedRegion_getSRPtr (newHeader,
                                                         obj->regionId);

                /* Join contiguous free blocks */
                if (nextHeader != NULL) {
                    /*
                     *  Verify the free size is not overlapping. Not all cases
                     *  are detectable, but it is worth a shot. Note: only do
                     *  this assert if nextHeader is non-NULL.
                     */
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                    if (EXPECT_FALSE (  ((SizeT) newHeader + size)
                                      > (SizeT) nextHeader)) {
                        /* A2 */
                        status = HeapMem_E_INVALIDSTATE;
                        GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "HeapMem_free",
                                         status,
                                         "Free size is overlapping");
                    }
                    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                        /* Join with upper block */
                        if (((UInt32) newHeader + size) == (UInt32)nextHeader) {
                            if (EXPECT_FALSE (obj->cacheEnabled)) {
                                Cache_inv (nextHeader,
                                           sizeof(HeapMem_Header),
                                           Cache_Type_ALL,
                                           TRUE);
                            }
                            newHeader->next = nextHeader->next; /* A2, B2 */
                            newHeader->size += nextHeader->size; /* A2, B2 */

                            /* Correct size for following Cache_wbInv. Needed
                             * due to another cache line fill caused by
                             * reading nextHeader->next
                             */
                            size += obj->minAlign;

                            /* Don't Cache_wbInv, this will be done later */
                        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (EXPECT_TRUE (status >= 0)) {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    /*
                     *  Join with lower block. Make sure to check to see if not the
                     *  first block. No need to invalidate attrs since head shouldn't change.
                     */
                    if (   (curHeader != &obj->attrs->head)
                        && (    ((UInt32) curHeader + curHeader->size)
                            ==  (UInt32) newHeader)) {
                        /*
                         * Don't Cache_inv newHeader since newHeader has
                         * data that hasn't been written back yet (B2)
                         */
                        curHeader->next = newHeader->next; /* B1, B2 */
                        curHeader->size += newHeader->size; /* B1, B2 */
                    }

                    if (EXPECT_FALSE (obj->cacheEnabled)) {
                        Cache_wbInv (curHeader,
                                     sizeof (HeapMem_Header),
                                     Cache_Type_ALL,
                                     FALSE); /* B1 */
                        /*
                         *  Invalidate entire buffer being freed to ensure that
                         *  stale cache data in block isn't evicted later.
                         */
                        Cache_wbInv (newHeader,
                                     size,
                                     Cache_Type_ALL,
                                     TRUE);  /* B2 */
                    }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                }
            }
        }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
#ifdef ENABLE_GATEMP
        GateMP_leave (obj->gate, key);
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_free");
}

/* Get memory statistics */
Void
HeapMem_getStats (HeapMem_Handle  handle,
                    Ptr               stats)
{
    HeapMem_Obj    * obj    = NULL;
    HeapMem_Header * curHeader = NULL;
    Memory_Stats     * memStats  = NULL;
#ifdef ENABLE_GATEMP
    IArg               key       = 0;
#endif

    GT_2trace (curTrace, GT_ENTER, "HeapMem_getStats", handle, stats);

    GT_assert (curTrace, (handle != NULL));
    GT_assert (curTrace, (stats != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getStats",
                             HeapMem_E_INVALIDARG,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handle == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getStats",
                             HeapMem_E_INVALIDARG,
                             "handle passed is null!");
    }
    else if (EXPECT_FALSE (stats == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getStats",
                             HeapMem_E_INVALIDARG,
                             "Invalid NULL stats pointer specified!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        obj = (HeapMem_Obj *) ((HeapMem_Object *) handle)->obj;
        GT_assert (curTrace, (obj != NULL));

        memStats = (Memory_Stats *) stats;

        memStats->totalSize         = obj->bufSize;
        memStats->totalFreeSize     = 0;  /* determined later */
        memStats->largestFreeSize   = 0;  /* determined later */

#ifdef ENABLE_GATEMP
        key = GateMP_enter(obj->gate);
#endif

        /* Invalidate curHeader */
        if (EXPECT_FALSE (obj->cacheEnabled)) {
            Cache_inv ((Ptr) &(obj->attrs->head),
                       sizeof (HeapMem_Header),
                       Cache_Type_ALL,
                       TRUE);
        }

        curHeader = SharedRegion_getPtr ((SharedRegion_SRPtr)
                                                    obj->attrs->head.next);

        while (curHeader != NULL) {
            /* Invalidate curHeader */
            if (EXPECT_FALSE (obj->cacheEnabled)) {
                Cache_inv (curHeader,
                           sizeof (HeapMem_Header),
                           Cache_Type_ALL,
                           TRUE);
            }

            memStats->totalFreeSize += curHeader->size;
            if (memStats->largestFreeSize < curHeader->size) {
                memStats->largestFreeSize = curHeader->size;
            }
            /* This condition is required to avoid
             * assertions during call to SharedRegion_getPtr  because at the end
             * of the calculation curHeader->next  will become
             * SharedRegion_INVALIDSRPTR.
             */
            if (curHeader->next != SharedRegion_INVALIDSRPTR) {
                curHeader = SharedRegion_getPtr ((SharedRegion_SRPtr)
                                                               curHeader->next);
            }
            else {
                curHeader = NULL;
            }
        }

#ifdef ENABLE_GATEMP
        GateMP_leave(obj->gate, key);
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_getStats");
}

/* Indicate whether the heap may block during an alloc or free call */
Bool
HeapMem_isBlocking (HeapMem_Handle handle)
{
    Bool isBlocking = FALSE;

    GT_1trace (curTrace, GT_ENTER, "Heap_isBlocking", handle);

    GT_assert (curTrace, (handle != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (handle == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_isBlocking",
                             HeapMem_E_INVALIDARG,
                             "handle passed is null!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* TBD: Figure out how to determine whether the gate is blocking */
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_isBlocking", isBlocking);

    return isBlocking;
}

/* Get extended statistics */
Void
HeapMem_getExtendedStats (HeapMem_Handle           handle,
                            HeapMem_ExtendedStats  * stats)
{
    HeapMem_Obj * obj = NULL;

    GT_2trace (curTrace, GT_ENTER, "HeapMem_getExtendedStats", handle, stats);

    GT_assert (curTrace, (handle != NULL));
    GT_assert (curTrace, (stats != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getExtendedStats",
                             HeapMem_E_INVALIDSTATE,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handle == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getExtendedStats",
                             HeapMem_E_INVALIDARG,
                             "Invalid NULL handle pointer specified!");
    }
    else if (EXPECT_FALSE (stats == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getExtendedStats",
                             HeapMem_E_INVALIDARG,
                             "Invalid NULL stats pointer specified!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        obj = (HeapMem_Obj *) ((HeapMem_Object *) handle)->obj;
        GT_assert (curTrace, (obj != NULL));

        stats->buf   = obj->buf;
        stats->size  = obj->bufSize;
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_getExtendedStats");
}

/* Returns the shared memory size requirement for a single instance. */
SizeT
HeapMem_sharedMemReq (const HeapMem_Params * params)
{
    SizeT   memReq = 0;
    UInt32  minAlign;
    UInt16  regionId;

    GT_1trace (curTrace, GT_ENTER, "HeapMem_sharedMemReq", params);

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (params == NULL)) {
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_sharedMemReq",
                             HeapMem_E_INVALIDARG,
                             "Params pointer passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        if (params->sharedAddr == NULL) {
            regionId = params->regionId;
        }
        else {
            regionId = SharedRegion_getId(params->sharedAddr);
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (regionId == SharedRegion_INVALIDREGIONID)  {
            GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "HeapMem_sharedMemReq",
                                 HeapMem_E_FAIL,
                                 "params->sharedAddr is not in a"
                                 " valid SharedRegion!");
        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            minAlign = sizeof (HeapMem_Header);
            if (SharedRegion_getCacheLineSize(regionId) > minAlign) {
                minAlign = SharedRegion_getCacheLineSize (regionId);
            }

            /* Add size of HeapMem Attrs */
            memReq = ROUNDUP (sizeof(HeapMem_Attrs), minAlign);

            /* Add the buffer size */
            memReq += params->sharedBufSize;

            /* Make sure the size is a multiple of minAlign (round down) */
            memReq = (memReq / minAlign) * minAlign;
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        }
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_sharedMemReq", memReq);

    return memReq;
}

/*!
 *  @brief      Returns the HeapMem kernel object pointer.
 *
 *  @param      handle  Handle to previousely created/opened instance.
 *
 */
Void *
HeapMem_getKnlHandle (HeapMem_Handle handle)
{
    GT_1trace (curTrace, GT_ENTER, "HeapMem_getKnlHandle", handle);

    GT_assert (curTrace, (handle != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        /*! @retval NULL Module was not initialized */
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getKnlHandle",
                             HeapMem_E_INVALIDSTATE,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handle == NULL)) {
        /*! @retval NULL handle passed is NULL */
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_getKnlHandle",
                             HeapMem_E_INVALIDARG,
                             "handle passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* Nothing to be done for kernel-side implementation. */
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_getKnlHandle", handle);

    /*! @retval Kernel-Object-handle Operation successfully completed. */
    return (handle);
}

/* HeapMem open by address */
Int
HeapMem_openByAddr (Ptr                sharedAddr,
                      HeapMem_Handle * handlePtr)
{
    Int               status   = HeapMem_S_SUCCESS;
    Bool              doneFlag = FALSE;
    HeapMem_Attrs * attrs    = NULL;
    List_Elem *       elem     = NULL;
#ifdef ENABLE_LOCAL_LOCK
    IArg              key;
#endif
    UInt16            id;
    HeapMem_Params  params;

    GT_2trace (curTrace, GT_ENTER, "HeapMem_openByAddr",
               sharedAddr, handlePtr);

    GT_assert (curTrace, (handlePtr != NULL));
    GT_assert (curTrace, (sharedAddr != NULL));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (EXPECT_FALSE (   Atomic_cmpmask_and_lt (&(HeapMem_module->refCount),
                                                HeapMem_MAKE_MAGICSTAMP(0),
                                                HeapMem_MAKE_MAGICSTAMP(1))
                      == TRUE)) {
        status = HeapMem_E_INVALIDSTATE;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_openByAddr",
                             status,
                             "Module was not initialized!");
    }
    else if (EXPECT_FALSE (handlePtr == NULL)) {
        status = HeapMem_E_INVALIDARG;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "HeapMem_openByAddr",
                             status,
                             "handlePtr pointer passed is NULL!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* First check in the local list */
        List_traverse (elem, (List_Handle) &HeapMem_module->objList) {
            if (((HeapMem_Obj *) elem)->params.sharedAddr == sharedAddr) {
#ifdef ENABLE_LOCAL_LOCK
                key = IGateProvider_enter (HeapMem_module->localLock);
#endif
                if (   ((HeapMem_Obj *)elem)->owner.procId
                    == MultiProc_self ()) {
                    ((HeapMem_Obj *)elem)->owner.openCount++;
                }
#ifdef ENABLE_LOCAL_LOCK
                IGateProvider_leave (HeapMem_module->localLock, key);
#endif
                *handlePtr = (((HeapMem_Obj *)elem)->top);
                doneFlag = TRUE;
                break;
            }
        }

        /* If not already existing locally, create object locally for open. */
        if (EXPECT_FALSE (doneFlag == FALSE)) {
            HeapMem_Params_init(&params);
            params.sharedAddr = sharedAddr;
            attrs = (HeapMem_Attrs *) sharedAddr;
            id = SharedRegion_getId (sharedAddr);

            if (EXPECT_FALSE (SharedRegion_isCacheEnabled (id))) {
                Cache_inv (attrs,
                           sizeof (HeapMem_Attrs),
                           Cache_Type_ALL,
                           TRUE);
            }

            if (EXPECT_FALSE (attrs->status != HeapMem_CREATED)) {
                *handlePtr = NULL;
                status = HeapMem_E_NOTFOUND;
                /* Don't set failure reason since this is an expected
                 * run-time failure.
                 */
                GT_1trace (curTrace,
                           GT_3CLASS,
                           "HeapMem Instance is not created yet at the "
                           "provided shared addr.\n"
                           "    sharedAddr [0x%x]!",
                           sharedAddr);
            }
            else {
                status = _HeapMem_create (handlePtr, &params, FALSE);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (EXPECT_FALSE (status < 0)) {
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "HeapMem_openByAddr",
                                         status,
                                         "_HeapMem_create failed!");
                }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            }
        }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_openByAddr", status);

    return (status);
}

/* =============================================================================
 * Internal function
 * =============================================================================
 */
GateMP_Handle HeapMem_getGate(HeapMem_Handle handle)
{
    HeapMem_Object *  object;
    HeapMem_Obj    *  obj;

    object = (HeapMem_Object *)handle;
    obj = (HeapMem_Obj *)(object->obj);
    return(obj->gate);
}

/* Creates a new instance of HeapMem module.
 * This is an internal function as both HeapMem_create
 * and HeapMem_open use the functionality.
 */
Int
_HeapMem_create (      HeapMem_Handle *  handlePtr,
                   const HeapMem_Params *  params,
                         Bool                createFlag)
{
    Int                 status = HeapMem_S_SUCCESS;
    HeapMem_Obj    *  obj    = NULL;
    HeapMem_Object *  handle = NULL;
#ifdef ENABLE_GATEMP
    GateMP_Handle       gateHandle = NULL;
    Ptr                 localAddr = NULL;
    SharedRegion_SRPtr  sharedShmBase;
#endif
#ifdef ENABLE_LOCAL_LOCK
    IArg                key;
#endif

    GT_3trace (curTrace, GT_ENTER, "_HeapMem_create",
               handlePtr, params, createFlag);

    GT_assert (curTrace, (handlePtr != NULL));
    GT_assert (curTrace, (params != NULL));

    /* No need for parameter checks, since this is an internal function. */

    /* Create the generic handle */
    handle = (HeapMem_Object *) Memory_calloc (NULL,
                                                 sizeof (HeapMem_Object),
                                                 0u,
                                                 NULL);
    *handlePtr = (HeapMem_Handle) handle;

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (handle == NULL) {
        /*! @retval HeapMem_E_MEMORY Memory allocation failed for pointer of
         *          type HeapMem_Object
         */
        status = HeapMem_E_MEMORY;
        GT_setFailureReason (curTrace,
                             GT_4CLASS,
                             "_HeapMem_create",
                             status,
                             "Memory allocation failed for pointer"
                             " of type HeapMem_Object!");
    }
    else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* Create the Heap Buf specific handle */
        obj = (HeapMem_Obj *) Memory_calloc (NULL,
                                               sizeof (HeapMem_Obj),
                                               0u,
                                               NULL);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (obj == NULL) {
            /*! @retval HeapMem_E_MEMORY Memory allocation failed for pointer
             *          of type HeapMem_Obj
             */
            status = HeapMem_E_MEMORY;
            GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "_HeapMem_create",
                                 status,
                                 "Memory allocation failed for pointer"
                                 " of type HeapMem_Obj");
        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            handle->obj = (HeapMem_Obj *) obj ;
            handle->alloc = (IHeap_allocFxn) &HeapMem_alloc;
            handle->free  = (IHeap_freeFxn) &HeapMem_free;
            handle->getStats     = (IHeap_getStatsFxn) &HeapMem_getStats;
            handle->getKnlHandle = (IHeap_getKnlHandleFxn)
                                                    &HeapMem_getKnlHandle;
            handle->isBlocking   = (IHeap_isBlockingFxn) &HeapMem_isBlocking;

            obj->nsKey     = NULL;
            obj->allocSize = 0;

            /* Put in the local list */
#ifdef ENABLE_LOCAL_LOCK
            key = IGateProvider_enter (HeapMem_module->localLock);
#endif
            List_elemClear (&obj->listElem);
            List_put ((List_Handle) &HeapMem_module->objList, &obj->listElem);
#ifdef ENABLE_LOCAL_LOCK
            IGateProvider_leave (HeapMem_module->localLock, key);
#endif

            if (createFlag == FALSE) {
                obj->owner.creator = FALSE;
                obj->owner.openCount = 0u;
                obj->owner.procId = MultiProc_INVALIDID;
                obj->top = handle;

                obj->attrs = (HeapMem_Attrs *) params->sharedAddr;

                /* No need to Cache_inv- already done in openByAddr() */
                obj->buf = (Char *) SharedRegion_getPtr ((SharedRegion_SRPtr)
                                                            obj->attrs->bufPtr);
                obj->bufSize      = obj->attrs->head.size;
                obj->regionId     = SharedRegion_getId (obj->buf);
                GT_assert (curTrace,
                           (obj->regionId != SharedRegion_INVALIDSRPTR));
                obj->cacheEnabled = SharedRegion_isCacheEnabled (obj->regionId);

                /* Set minAlign */
                obj->minAlign = sizeof (HeapMem_Header); /* 64 bits = 8 bytes */
                if (    SharedRegion_getCacheLineSize (obj->regionId)
                    >   obj->minAlign) {
                    obj->minAlign = SharedRegion_getCacheLineSize (
                                                                obj->regionId);
                }

#ifdef ENABLE_GATEMP
                localAddr = SharedRegion_getPtr (obj->attrs->gateMPAddr);
                status = GateMP_openByAddr (localAddr, &gateHandle);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (status < 0) {
                    /* Override the status with a HeapMem status code. */
                    status = HeapMem_E_FAIL;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "_HeapMem_create",
                                         status,
                                         "Failed to open GateMP!");
                }
                else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    obj->gate = gateHandle;
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
#endif
            }
            else {
                /* Creating the HeapMem ... */
                obj->owner.creator = TRUE;
                obj->owner.openCount = 1u;
                obj->owner.procId = MultiProc_self ();
                obj->top = handle;

                /* Creating the gate */
                if (params->gate != NULL) {
                    obj->gate = params->gate;
                }
#ifdef ENABLE_GATEMP
                else {
                    /* If no gate specified, get the default system gate */
                    obj->gate = GateMP_getDefaultRemote ();
                }
#endif
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                if (obj->gate == NULL) {
                    status = HeapMem_E_FAIL;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "_HeapMem_create",
                                         status,
                                         "GateMP is NULL!");
                }
                else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    obj->bufSize = params->sharedBufSize;

                    if (params->sharedAddr == NULL) {
                        /* Creating using a shared region ID */
                        /* It is allowed to have NULL name for an anonymous, not
                         * to be opened by name, heap.
                         */
                        obj->attrs = NULL; /* Will be alloced in postInit */
                        obj->regionId = params->regionId;
                    }
                    else {
                        /* Creating using sharedAddr */
                        obj->regionId = SharedRegion_getId (params->sharedAddr);

                        /* Assert that the buffer is in a valid shared
                         * region
                         */
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                        if (obj->regionId == SharedRegion_INVALIDREGIONID) {
                            status = HeapMem_E_FAIL;
                            /*! @retval HeapMem_E_FAIL params->sharedAddr is
                                              not in a valid SharedRegion. */
                            GT_setFailureReason (curTrace,
                                                 GT_4CLASS,
                                                 "_HeapMem_create",
                                                 status,
                                                 "params->sharedAddr is not in"
                                                 " a valid SharedRegion!");
                        }
                        else if (  ((UInt32) params->sharedAddr
                                 % SharedRegion_getCacheLineSize (obj->regionId)
                                 != 0)) {
                            status = HeapMem_E_FAIL;
                            /*! @retval HeapMem_E_FAIL params->sharedAddr does
                                        not meet cache alignment constraints */
                            GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "_ListMP_create",
                                         status,
                                         "params->sharedAddr does not"
                                         " meet cache alignment constraints!");
                        }
                        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                            /* obj->buf will get alignment-adjusted in
                             * postInit
                             */
                            obj->buf = (Ptr) (  (UInt32) params->sharedAddr
                                              + sizeof (HeapMem_Attrs));
                            obj->attrs = (HeapMem_Attrs *) params->sharedAddr;
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                        }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
                    if (status >= 0) {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                        obj->cacheEnabled = SharedRegion_isCacheEnabled (
                                                                obj->regionId);

                        /* Set minAlign */
                        obj->minAlign = sizeof (HeapMem_Header);
                        if (    SharedRegion_getCacheLineSize(obj->regionId)
                            >   obj->minAlign) {
                            obj->minAlign = SharedRegion_getCacheLineSize (
                                                                obj->regionId);
                        }

                        status = HeapMem_postInit ((HeapMem_Object *)
                                                         handle);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                        if (EXPECT_FALSE (status < 0)) {
                            GT_setFailureReason (curTrace,
                                                 GT_4CLASS,
                                                 "_HeapMem_create",
                                                 status,
                                                 "HeapMem_postInit failed!");
                        }
                    }
                }

                if (status >= 0) {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    /* Populate the params member */
                    memcpy ((Ptr) &obj->params,
                                 (Ptr) params,
                                 sizeof (HeapMem_Params));

                    /* Copy the name */
                    if (params->name != NULL) {
                        obj->params.name = (String) Memory_alloc (NULL,
                                            (strlen (params->name)+ 1u),
                                            0,
                                            NULL);

#if !defined(SYSLINK_BUILD_OPTIMIZE)
                        if (obj->params.name == NULL) {
                            /*! @retval HeapMem_E_MEMORY Memory allocation
                             *          failed for name
                             */
                            status = HeapMem_E_MEMORY;
                            GT_setFailureReason (curTrace,
                                                 GT_4CLASS,
                                                 "_HeapMem_create",
                                                 status,
                                                 "Memory allocation "
                                                 "failed for name!");
                        }
                        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                            strncpy (obj->params.name,
                                         params->name,
                                         strlen (params->name) + 1u);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
                        }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                    }
                }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
            }
        }
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (status < 0) {
        if (createFlag == TRUE) {
            HeapMem_delete ((HeapMem_Handle *) handlePtr);
        }
        else {
            HeapMem_close ((HeapMem_Handle *) handlePtr);
        }
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "_HeapMem_create", status);

    /*! @retval HeapMem_S_SUCCESS Operation successful*/
    return status;
}

/*
 *  Slice and dice the buffer up into the correct size blocks and
 *  add to the freelist.
 */
Int
HeapMem_postInit (HeapMem_Object * handle)
{
    Int              status     = HeapMem_S_SUCCESS;
    HeapMem_Obj *  obj        = NULL;
    HeapMem_Handle regionHeap = NULL;
    HeapMem_Params params;

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_postInit");

    GT_assert (curTrace, (handle != NULL));

    /* No need for parameter checks, since this is an internal function. */

    obj = handle->obj;

    if (obj->attrs == NULL) {
        /* Need to allocate from the heap */
        HeapMem_Params_init (&params);
        params.regionId      = obj->regionId;
        params.sharedBufSize = obj->bufSize;
        obj->allocSize = HeapMem_sharedMemReq (&params);
        regionHeap = SharedRegion_getHeap (obj->regionId);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (regionHeap == NULL) {
            status = HeapMem_E_FAIL;
            GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "HeapMem_postInit",
                                 status,
                                 "Shared Region heap is null!");

        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            obj->attrs = Memory_alloc ((IHeap_Handle) regionHeap,
                                       obj->allocSize,
                                       obj->minAlign,
                                       NULL);
#if !defined(SYSLINK_BUILD_OPTIMIZE)
            if (obj->attrs == NULL) {
                status = HeapMem_E_MEMORY;
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "HeapMem_postInit",
                                     status,
                                     "Failed to allocate shared memory!");
            }
            else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
                obj->buf = (Ptr)((UInt32)obj->attrs + sizeof(HeapMem_Attrs));
#if !defined(SYSLINK_BUILD_OPTIMIZE)
            }
        }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
    }

#if !defined(SYSLINK_BUILD_OPTIMIZE)
    if (status >= 0) {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
        /* Round obj->buf up by obj->minAlign */
        obj->buf = (Ptr) ROUNDUP ((obj->buf), (obj->minAlign));

#if !defined(SYSLINK_BUILD_OPTIMIZE)
        if (EXPECT_FALSE (  obj->bufSize
                          < SharedRegion_getCacheLineSize (obj->regionId))) {
            status = HeapMem_E_FAIL;
            GT_setFailureReason (curTrace,
                                 GT_4CLASS,
                                 "HeapMem_postInit",
                                 status,
                                 "Buffer is not large enough!");
        }
        else {
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */
            /* Make sure the size is a multiple of obj->minAlign */
            obj->bufSize = (obj->bufSize / obj->minAlign) * obj->minAlign;
#ifdef ENABLE_GATEMP
            obj->attrs->gateMPAddr = GateMP_getSharedAddr (obj->gate);
#endif
            obj->attrs->bufPtr     = SharedRegion_getSRPtr (obj->buf,
                                                            obj->regionId);

            /* Store computed obj->bufSize in shared mem */
            obj->attrs->head.size = obj->bufSize;

            /* Place the initial header */
            HeapMem_restore ((HeapMem_Handle) handle);

            /* Last thing, set the status */
            obj->attrs->status = HeapMem_CREATED;

            if (EXPECT_FALSE (obj->cacheEnabled)) {
                Cache_wbInv ((Ptr) obj->attrs,
                             sizeof (HeapMem_Attrs),
                             Cache_Type_ALL,
                             TRUE);
            }
#if !defined(SYSLINK_BUILD_OPTIMIZE)
        }
    }
#endif /* if !defined(SYSLINK_BUILD_OPTIMIZE) */

    GT_1trace (curTrace, GT_LEAVE, "HeapMem_postInit", status);

    return status;
}

/* Restore an instance to it's original created state. */
Void
HeapMem_restore (HeapMem_Handle handle)
{
    HeapMem_Header * begHeader = NULL;
    HeapMem_Obj    * obj       = NULL;

    GT_1trace (curTrace, GT_ENTER, "HeapMem_restore", handle);

    obj = ((HeapMem_Object *) handle)->obj;
    GT_assert (curTrace, (obj != NULL));

    /*
     *  Fill in the top of the memory block
     *  next: pointer will be NULL (end of the list)
     *  size: size of this block
     *  NOTE: no need to Cache_inv because obj->attrs->bufPtr should be const
     */
    begHeader = (HeapMem_Header *) obj->buf;
    begHeader->next = (SharedRegion_SRPtr) SharedRegion_INVALIDSRPTR;
    begHeader->size = obj->bufSize;

    obj->attrs->head.next = obj->attrs->bufPtr;
    if (EXPECT_FALSE (obj->cacheEnabled)) {
        Cache_wbInv ((Ptr) &(obj->attrs->head),
                     sizeof (HeapMem_Header),
                     Cache_Type_ALL,
                     FALSE);
        Cache_wbInv (begHeader,
                     sizeof (HeapMem_Header),
                     Cache_Type_ALL,
                     TRUE);
    }

    GT_0trace (curTrace, GT_LEAVE, "HeapMem_restore");
}
