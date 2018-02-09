/*
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
 */

/**
 *  @file   SharedRegion.c
 *
 *  @brief  Shared Region Manager
 */

/* Standard headers */
#include <assert.h>
#include <string.h>
#include <stdlib.h>

/* IPC headers */
#include <ti/ipc/Std.h>
#include "Std.h"
#include <ti/ipc/MultiProc.h>
#include <IGateProvider.h>
#include <GateMutex.h>

#include "Trace.h"
#include "Cache.h"
#include "Memory.h"

#include "_SharedRegion.h"

//#define ENABLE_LOCAL_LOCK

#define ROUNDUP(size, align) \
    (UInt32)(((UInt32)(size) + ((UInt32)(align) - 1)) & ~((UInt32)(align) - 1))

/* =============================================================================
 * Macros
 * =============================================================================
 */
/* Macro to make a correct module magic number with refCount */
#define SharedRegion_MAKE_MAGICSTAMP(x) ((SharedRegion_MODULEID << 16u) | (x))

/* =============================================================================
 * Structure & Enums
 * =============================================================================
 */

/*!
 *  @brief  Module state object.
 */
typedef struct SharedRegion_ModuleObject_tag {
    UInt32                    refCount;
    /*!< Reference count */
#ifdef ENABLE_LOCAL_LOCK
    IGateProvider_Handle      localLock;
    /*!< Handle to a gate instance */
#endif
    SharedRegion_Region     * regions;
    /*!< Pointer to the regions */
    SharedRegion_Config       cfg;
    /*!< Current config values */
    SharedRegion_Config       defCfg;
    /*!< Default config values */
    UInt32                    numOffsetBits;
    /*!< number of bits for the offset for a SRPtr. This value is calculated */
    UInt32                    offsetMask;
    /*!< offset bitmask using for generating a SRPtr */
    bool                      isStarted;
    /*!< indicates whether shared region start API is called */
    UInt32                    *regionRefCount;
    /* To keep track of how many start/stop region API is  called for a region*/
} SharedRegion_ModuleObject;

/* =============================================================================
 * Global
 * =============================================================================
 */
/*!
 *  @brief  Shared region state object variable with default settings
 */
static
SharedRegion_ModuleObject SharedRegion_state = {
    .numOffsetBits        = 0,
    .regions              = NULL,
#ifdef ENABLE_LOCAL_LOCK
    .localLock            = NULL,
#endif
    .regionRefCount       = NULL,
    .offsetMask           = 0,
    .defCfg.numEntries    = 4u,
    .defCfg.translate     = TRUE,
    .defCfg.cacheLineSize = 128u
};

/*!
 *  @var    SharedRegion_module
 *
 *  @brief  Pointer to the SharedRegion module state.
 */
static
SharedRegion_ModuleObject * SharedRegion_module = &SharedRegion_state;

/*!
 *  @brief  SharedRegion_CREATED flag in shared memory
 */
const UInt32 SharedRegion_CREATED = 0x08111963;

/*!
 *  @brief  SharedRegion_VERSION attribute in shared memory
 */
const UInt32 SharedRegion_VERSION = 1;

/* =============================================================================
 * Forward declarations of internal functions
 * =============================================================================
 */
/*!
 *  @brief      Checks to make sure overlap does not exists.
 *              Return error if overlap found.
 *
 *  @param      base      Base of Shared Region
 *  @param      len       Length of Shared Region
 *
 *  @sa         None
 */
static Int SharedRegion_checkOverlap (Ptr base, UInt32 len);

/*
 *  @brief      Return the number of offsetBits bits
 *
 *  @param      None
 *
 *  @sa         None
 */
static UInt32 SharedRegion_getNumOffsetBits (Void);

/* =============================================================================
 * APIs
 * =============================================================================
 */
/* Function to get the configuration */
Void
SharedRegion_getConfig (SharedRegion_Config * config)
{
    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getConfig", config);

    GT_assert (curTrace, (config != NULL));

    if ((SharedRegion_module->refCount & SharedRegion_MAKE_MAGICSTAMP(0)) < 
        SharedRegion_MAKE_MAGICSTAMP(1)) {
        memcpy ((Ptr) config,
                     (Ptr) &SharedRegion_module->defCfg,
                     sizeof (SharedRegion_Config));
    }
    else {
        memcpy ((Ptr) config,
                     (Ptr) &SharedRegion_module->cfg,
                     sizeof (SharedRegion_Config));
    }

    GT_0trace (curTrace, GT_LEAVE, "SharedRegion_getConfig");
}

/* Function to setup the SharedRegion module. */
Int
SharedRegion_setup (SharedRegion_Config * cfg)
{
    Int                 status = SharedRegion_S_SUCCESS;
    UInt32         eb;
    SharedRegion_Config tmpCfg;
    UInt32              i;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_setup", cfg);
    eb=0;

    if (cfg == NULL) {
        SharedRegion_getConfig (&tmpCfg);
        cfg = &tmpCfg;
    }

    if (cfg == NULL) {
        SharedRegion_getConfig (&tmpCfg);
        cfg = &tmpCfg;
    }

    /* This sets the refCount variable is not initialized, upper 16 bits is
     * written with module Id to ensure correctness of refCount variable.
     */
    if((SharedRegion_module->refCount & SharedRegion_MAKE_MAGICSTAMP(0))
        != SharedRegion_MAKE_MAGICSTAMP(0)) {
        SharedRegion_module->refCount = SharedRegion_MAKE_MAGICSTAMP(0);
    }
    if (++SharedRegion_module->refCount 
        != SharedRegion_MAKE_MAGICSTAMP(1u)) {
        status = SharedRegion_S_ALREADYSETUP;
        GT_0trace (curTrace,
                   GT_2CLASS,
                   "SharedRegion Module already initialized!");
    }
    else {
        /* copy the user provided values into the state object */
        memcpy ((Ptr) &SharedRegion_module->cfg,
                     (Ptr) cfg,
                     sizeof (SharedRegion_Config));

        /* In a single processor system, we should never translate.
         * but on HlOS side  where there are user space virtual space
         * we should set translate = TRUE.
         */

        SharedRegion_module->cfg.translate = TRUE;

        /* Allocate memory for the regions */
        SharedRegion_module->regions = (SharedRegion_Region *)
                                            malloc(sizeof(SharedRegion_Region)
                                                   *SharedRegion_module->cfg.numEntries);

            SharedRegion_module->regionRefCount = malloc (sizeof (UInt32)
                                                          * SharedRegion_module->cfg.numEntries);
            GT_assert(curTrace,(SharedRegion_module->regionRefCount != NULL));

                for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
                    SharedRegion_module->regions[i].entry.base = NULL;
                    SharedRegion_module->regions[i].entry.len = 0;
                    SharedRegion_module->regions[i].entry.ownerProcId = 0;
                    SharedRegion_module->regions[i].entry.isValid = FALSE;
                    SharedRegion_module->regions[i].entry.cacheEnable = FALSE;
                    SharedRegion_module->regions[i].entry.cacheLineSize =
                                            SharedRegion_module->cfg.cacheLineSize;
                    SharedRegion_module->regions[i].entry.createHeap   = FALSE;
                    SharedRegion_module->regions[i].reservedSize = 0;
#ifdef ENABLE_HEAPMP
                    SharedRegion_module->regions[i].heap = NULL;
#endif
                    SharedRegion_module->regions[i].entry.name = NULL;
                }

                /* set the defaults for region 0  */
#ifdef ENABLE_HEAPMP
                SharedRegion_module->regions[0].entry.createHeap  = TRUE;
#endif
                SharedRegion_module->regions[0].entry.ownerProcId = MultiProc_self();

                SharedRegion_module->numOffsetBits = SharedRegion_getNumOffsetBits ();
                SharedRegion_module->offsetMask =
                                        (1 << SharedRegion_module->numOffsetBits) - 1;
                SharedRegion_module->isStarted = FALSE;
#ifdef ENABLE_LOCAL_LOCK
                /* Create a lock for protecting list object */
                SharedRegion_module->localLock = (IGateProvider_Handle)
                               GateMutex_create ((GateMutex_Params*)NULL, &eb);
#endif
    }

    if (status < 0) {
        SharedRegion_destroy ();
    }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_setup", status);

    /*! @retval SharedRegion_SUCCESS operation was successful */
    return status;
}

/* Function to destroy the SharedRegion module. */
Int
SharedRegion_destroy (Void)
{
    Int status = SharedRegion_S_SUCCESS;
#ifdef ENABLE_LOCAL_LOCK
    IArg key;
#endif
    GT_0trace (curTrace, GT_ENTER, "SharedRegion_destroy");

        if (  --SharedRegion_module->refCount
            == SharedRegion_MAKE_MAGICSTAMP(0)) {
#ifdef ENABLE_LOCAL_LOCK
            /* Enter the gate */
            key = IGateProvider_enter (SharedRegion_module->localLock);
#endif
            if (SharedRegion_module->regionRefCount != NULL) {
                free (SharedRegion_module->regionRefCount);
                SharedRegion_module->regionRefCount = NULL;
            }

            if (SharedRegion_module->regions != NULL) {
                free (SharedRegion_module->regions);
                SharedRegion_module->regions = NULL;
            }

            memset (&SharedRegion_module->cfg,
                        0,
                        sizeof (SharedRegion_Config));

            SharedRegion_module->numOffsetBits = 0;
            SharedRegion_module->offsetMask    = 0;
#ifdef ENABLE_LOCAL_LOCK
            /* Leave the gate */
            IGateProvider_leave (SharedRegion_module->localLock, key);
            /* Delete the local lock */
            if (SharedRegion_module->localLock != NULL) {
                status = GateMutex_delete ((GateMutex_Handle *)
                                           &SharedRegion_module->localLock);
                GT_assert (curTrace, (status >= 0));
                if (status < 0) {
                    status = SharedRegion_E_FAIL;
                }
            }
#endif
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_destroy", status);

    return status;
}

/* API to know whether shared region is started or not.Returns true if shared
 * region is already started othere wise returns false.
 */
Int
_SharedRegion_isStarted(Void)
{
    return(SharedRegion_module->isStarted) ;
}

/* Creates a heap by owner of region for each SharedRegion.
 * Function is called by Ipc_start(). Requires that SharedRegion 0
 * be valid before calling start().
 */
Int
SharedRegion_start (Void)
{
    Int                   status  = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region  = NULL;
#ifdef ENABLE_HEAPMP
    Ptr                   sharedAddr = NULL;
    HeapMemMP_Handle      heapHandle = NULL;
    HeapMemMP_Params      params;
#endif
    Int                   i;

    GT_0trace (curTrace, GT_ENTER, "SharedRegion_start");

        /*
         *  Loop through shared regions. If an owner of a region is specified
         *  and createHeap has been specified for the SharedRegion, then
         *  the owner creates a HeapMemMP and the other processors open it.
         */
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions[i]);
            if (region->entry.isValid) {
#ifdef ENABLE_HEAPMP
                if ((region->entry.ownerProcId == MultiProc_self())
                    && (region->entry.createHeap)
                    && (region->heap == NULL)) {
                    /* get the next free address in each region */
                    sharedAddr = (Ptr)((UInt32)region->entry.base
                                                 + region->reservedSize);

                    /*  Create the HeapMemMP in the region. */
                    HeapMemMP_Params_init(&params);
                    params.sharedAddr = sharedAddr;
                    params.sharedBufSize = region->entry.len - region->reservedSize;

                    /* Adjust to account for the size of HeapMemMP_Attrs */
                    params.sharedBufSize -= (HeapMemMP_sharedMemReq(&params)
                                         - params.sharedBufSize);
                    heapHandle = HeapMemMP_create(&params);

                        /* put heap handle into SharedRegion Module state */
                        region->heap = heapHandle;
                }
#endif
                SharedRegion_module->regionRefCount[i] += 1;
            }
        }

        /* Set flag indiacting shared region has been started */
        SharedRegion_module->isStarted = TRUE;

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_start", status);

    return status;
}

/* Function to stop the SharedRegion module.  */
Int
SharedRegion_stop (Void)
{
    Int                   status    = SharedRegion_S_SUCCESS;
#ifdef ENABLE_HEAPMP
    Int                   tmpStatus = SharedRegion_S_SUCCESS;
#endif
    SharedRegion_Region * region  = NULL;
    Int                   i;

    GT_0trace (curTrace, GT_ENTER, "SharedRegion_stop");

        /*
         *  Loop through shared regions. If an owner of a region is specified
         *  and createHeap has been specified for the SharedRegion, then
         *  the other processors close it and the owner deletes the HeapMemMP.
         */
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions[i]);
            if (region->entry.isValid) {
#ifdef ENABLE_HEAPMP
                if ((region->entry.ownerProcId == MultiProc_self ())
                    && (region->entry.createHeap)
                    && (region->heap != NULL)) {
                    /* Delete heap */
                    tmpStatus = HeapMemMP_delete ((HeapMemMP_Handle *)
                                                        &(region->heap));
                    if ((tmpStatus < 0) && (status >= 0)) {
                        status = SharedRegion_E_FAIL;
                    }
                }
#endif
                memset (region, 0, sizeof (SharedRegion_Region));
                SharedRegion_entryInit (&(region->entry));

                SharedRegion_module->regionRefCount[i] -= 1;
            }
        }

        /* set the defaults for region 0  */
        memset (&(SharedRegion_module->regions[0]),
                    0,
                    sizeof (SharedRegion_Region));
        SharedRegion_entryInit (&(SharedRegion_module->regions[0].entry));
        SharedRegion_module->regions[0].entry.createHeap  = TRUE;
        SharedRegion_module->regions[0].entry.ownerProcId = MultiProc_self();
        /* Set flag indicating shared region has been started */
        SharedRegion_module->isStarted = FALSE;

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_stop", status);

    return status;
}

/* Opens a heap, for non-owner processors, for each SharedRegion. */
Int
SharedRegion_attach (UInt16 remoteProcId)
{
    Int                   status     = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region     = NULL;
#ifdef ENABLE_HEAPMP
    Ptr                   sharedAddr = NULL;
#endif
    Int                   i;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_attach", remoteProcId);

    GT_assert (curTrace, (remoteProcId < MultiProc_MAXPROCESSORS));

        /*
         *  Loop through the regions and open the heap if not owner
         */
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions [i]);
            if (region->entry.isValid) {
#ifdef ENABLE_HEAPMP
                if ((region->entry.ownerProcId != MultiProc_self ()) &&
                    (region->entry.ownerProcId != MultiProc_INVALIDID) &&
                    (region->entry.createHeap) &&
                    (region->heap == NULL)) {
                    /* SharedAddr should match creator's for each region */
                    sharedAddr = (Ptr) ((UInt32) region->entry.base +
                                                    region->reservedSize);

                    /* Heap should already be created so open by address */
                    status = HeapMemMP_openByAddr (sharedAddr,
                                        (HeapMemMP_Handle *) &(region->heap));
                    if (status < 0) {
                        status = SharedRegion_E_FAIL;

                        /* Break the loop on failure. */
                        break;
                    }
                }
#endif
                SharedRegion_module->regionRefCount[i] ++;
			}
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_attach", status);

    return status;
}

/* Closes a heap, for non-owner processors, for each SharedRegion. */
Int
SharedRegion_detach (UInt16 remoteProcId)
{
    Int                   status    = SharedRegion_S_SUCCESS;
#ifdef ENABLE_HEAPMP
    Int                   tmpStatus = SharedRegion_S_SUCCESS;
#endif
    SharedRegion_Region * region    = NULL;
    UInt16                i;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_detach", remoteProcId);

    GT_assert (curTrace, (remoteProcId < MultiProc_MAXPROCESSORS));

        /*
         *  Loop through the regions and open the heap if not owner
         */
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions [i]);
            if (region->entry.isValid) {
#ifdef ENABLE_HEAPMP
                if ((region->entry.ownerProcId != MultiProc_self ()) &&
                    (region->entry.ownerProcId != MultiProc_INVALIDID) &&
                    (region->entry.createHeap) && (region->heap != NULL)) {
	                /* Close the heap */
	                tmpStatus = HeapMemMP_close ((HeapMemMP_Handle *)
	                    &(region->heap));
	                if ((tmpStatus < 0) && (status >= 0)) {
	                    status = SharedRegion_E_FAIL;

	                }
	            }
#endif
	            SharedRegion_module->regionRefCount[i]--;
            }
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_detach", status);

    return status;
}

/* Returns the address pointer associated with the shared region pointer. */
Ptr
SharedRegion_getPtr (SharedRegion_SRPtr srPtr)
{
    SharedRegion_Region * region    = NULL;
    Ptr                   returnPtr = NULL;
    UInt16                regionId;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getPtr", srPtr);

    /* srPtr can be invalid. */

    if (srPtr != SharedRegion_INVALIDSRPTR) {
        if (SharedRegion_module->cfg.translate == FALSE) {
            returnPtr = (Ptr) srPtr;
        }
        else {
            regionId = (UInt32) (srPtr >> SharedRegion_module->numOffsetBits);

                region = &(SharedRegion_module->regions [regionId]);

                returnPtr = (Ptr)(  (srPtr & SharedRegion_module->offsetMask)
                                  + (UInt32) region->entry.base);

        }
    }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getPtr", returnPtr);

    return returnPtr;
}

/* Returns the address pointer associated with the shared region pointer. */
SharedRegion_SRPtr
SharedRegion_getSRPtr (Ptr addr, UInt16 id)
{
    SharedRegion_Region * region = NULL;
    SharedRegion_SRPtr    retPtr = SharedRegion_INVALIDSRPTR;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_getSRPtr", addr, id);

    /* addr can be NULL. */

    /* Return invalid for NULL addr */
    if (addr != NULL) {
        GT_assert (curTrace, (id != SharedRegion_INVALIDREGIONID));
        GT_assert (curTrace,
                   (    (id != SharedRegion_INVALIDREGIONID)
                    &&  (id < SharedRegion_module->cfg.numEntries)));

            /* if no shared region configured, set SRPtr to addr */
            if (SharedRegion_module->cfg.translate == FALSE) {
                retPtr = (SharedRegion_SRPtr) addr;
            }
            else {
                region = &(SharedRegion_module->regions[id]);

                /*
                 *  Note: The very last byte on the very last id cannot be
                 *        mapped because SharedRegion_INVALIDSRPTR which is ~0
                 *        denotes an error. Since pointers should be word
                 *        aligned, we don't expect this to be a problem.
                 *
                 *        ie: numEntries = 4,
                 *            id = 3, base = 0x00000000, len = 0x40000000
                 *            ==> address 0x3fffffff would be invalid because
                 *                the SRPtr for this address is 0xffffffff
                 */
                if (    ((UInt32) addr >= (UInt32) region->entry.base)
                    &&  (  (UInt32) addr
                         < ((UInt32) region->entry.base + region->entry.len))) {
                    retPtr = (SharedRegion_SRPtr)
                              (  (id << SharedRegion_module->numOffsetBits)
                               | ((UInt32) addr - (UInt32) region->entry.base));
                }
                else {
                    retPtr = SharedRegion_INVALIDSRPTR;
                    GT_setFailureReason (curTrace,
                                         GT_4CLASS,
                                         "SharedRegion_getSRPtr",
                                         SharedRegion_E_INVALIDARG,
                                         "Provided addr is not in correct range"
                                         " for the specified id!");
                }
            }

    }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getSRPtr", retPtr);

    return retPtr;
}

/* Whether address translation is enabled */
Bool
SharedRegion_translateEnabled (Void)
{
    GT_0trace (curTrace, GT_ENTER, "SharedRegion_translateEnabled");
    GT_1trace (curTrace,
               GT_LEAVE,
               "SharedRegion_translateEnabled",
               SharedRegion_module->cfg.translate);

    return (SharedRegion_module->cfg.translate);
}

/* Gets the number of regions */
UInt16
SharedRegion_getNumRegions (Void)
{
    GT_0trace (curTrace, GT_ENTER, "SharedRegion_getNumRegions");
    GT_1trace (curTrace,
               GT_LEAVE,
               "SharedRegion_getNumRegions",
               SharedRegion_module->cfg.numEntries);
    return (SharedRegion_module->cfg.numEntries);
}

/* Sets the table information entry in the table. */
Int
SharedRegion_setEntry (UInt16                id,
                       SharedRegion_Entry  * entry)
{
    Int                   status  = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region  = NULL;
#ifdef ENABLE_HEAPMP
    Ptr                   sharedAddr    = NULL;
#endif
#ifdef ENABLE_HEAPMP
    HeapMemMP_Handle      heapHandle    = NULL;
    HeapMemMP_Handle   *  heapHandlePtr = NULL;
    HeapMemMP_Params      params;
#endif
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_setEntry", id, entry);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));
    GT_assert (curTrace, (entry != NULL));

        region = &(SharedRegion_module->regions [id]);

        /* Make sure region does not overlap existing ones */
        status = SharedRegion_checkOverlap (region->entry.base,
                                            region->entry.len);

            if (region->entry.isValid) {
                /* region already exist */
                SharedRegion_module->regionRefCount[id] += 1;
            }
            else {
#ifdef ENABLE_LOCAL_LOCK
                /* needs to be thread safe */
                key = IGateProvider_enter (SharedRegion_module->localLock);
#endif
                /* set specified region id to entry values */
                memcpy ((Ptr) &(region->entry),
                             (Ptr) entry,
                             sizeof (SharedRegion_Entry));
#ifdef ENABLE_LOCAL_LOCK
                /* leave gate */
                IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

#ifdef ENABLE_HEAPMP
                if (entry->ownerProcId == MultiProc_self ()) {
                    if ((entry->createHeap) && (region->heap == NULL)) {
                        /* get current Ptr (reserve memory with size of 0) */
                        sharedAddr = SharedRegion_reserveMemory (id, 0);
                        HeapMemMP_Params_init (&params);
                        params.sharedAddr    = sharedAddr;
                        params.sharedBufSize =   region->entry.len
                                              -  region->reservedSize;

                        /*
                         *  Calculate size of HeapMemMP_Attrs and adjust
                         *  sharedBufSize. Size of HeapMemMP_Attrs =
                         *  HeapMemMP_sharedMemReq(&params) - params.sharedBufSize
                         */
                        params.sharedBufSize -= (  HeapMemMP_sharedMemReq (&params)
                                                 - params.sharedBufSize);

                        heapHandle = HeapMemMP_create (&params);
                        if (heapHandle == NULL) {
                            region->entry.isValid = FALSE;
                            status = SharedRegion_E_MEMORY;

                        }
                        else {
                            region->heap = heapHandle;
                        }
                    }
                }
                else {
                    if ((entry->createHeap) && (region->heap == NULL)) {
                        /* sharedAddr should match creator's for each region */
                        sharedAddr = (Ptr)(  (UInt32) region->entry.base
                                           + region->reservedSize);

                        /* set the pointer to a heap handle */
                        heapHandlePtr = (HeapMemMP_Handle *) &(region->heap);

                        /* open the heap by address */
                        status = HeapMemMP_openByAddr (sharedAddr, heapHandlePtr);
                        if (status < 0) {
                            region->entry.isValid = FALSE;
                            status = SharedRegion_E_FAIL;

                        }
                    }
                }
#endif
                if (region->entry.isValid) {
                    SharedRegion_module->regionRefCount[id] ++;
                }
            }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_setEntry", status);

    return (status);
}

/* Clears the region in the table. */
Int
SharedRegion_clearEntry (UInt16 id)
{
    Int                   status     = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region     = NULL;
#ifdef ENABLE_HEAPMP
    HeapMemMP_Handle      heapmemPtr = NULL;
    UInt16                myId;
    UInt16                ownerProcId;
#endif
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_clearEntry", id);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));
    /* Need to make sure not trying to clear Region 0 */

#ifdef ENABLE_HEAPMP
        myId = MultiProc_self ();
#endif

#ifdef ENABLE_LOCAL_LOCK
        /* Needs to be thread safe */
        key = IGateProvider_enter (SharedRegion_module->localLock);
#endif

        region = &(SharedRegion_module->regions [id]);

#ifdef ENABLE_HEAPMP
        /* Store these fields to local variables */
        ownerProcId = region->entry.ownerProcId;
        heapmemPtr  = region->heap;
#endif
        if (region->entry.isValid) {
            SharedRegion_module->regionRefCount[id] -= 1;
        }

        if (SharedRegion_module->regionRefCount[id] == 0) {
           /* Assert if region is 0.
            * Region 0 should always be cleared through SharedRegion_stop API
            */
            GT_assert (curTrace, (id != 0));
            /* Clear region to their defaults */
            region->entry.isValid       = FALSE;
            region->entry.base          = NULL;
            region->entry.len           = 0u;
            region->entry.ownerProcId   = 0u;
            region->entry.cacheEnable   = FALSE;
            region->entry.cacheLineSize = SharedRegion_module->cfg.cacheLineSize;
            region->entry.createHeap    = TRUE;
            region->entry.name          = NULL;
            region->reservedSize        = 0u;
#ifdef ENABLE_HEAPMP
            region->heap                = NULL;
#endif

#ifdef ENABLE_LOCAL_LOCK
            IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

#ifdef ENABLE_HEAPMP
            /* Delete or close previous created heap outside the gate */
            if (heapmemPtr != NULL) {
                if (ownerProcId == myId) {
                    status = HeapMemMP_delete ((HeapMemMP_Handle *) &heapmemPtr);
                    if (status < 0) {
                        status = SharedRegion_E_FAIL;

                    }
                }
                else if (ownerProcId != MultiProc_INVALIDID) {
                    status = HeapMemMP_close ((HeapMemMP_Handle *) &heapmemPtr);
                    if (status < 0) {
                        status = SharedRegion_E_FAIL;

                    }
                }
            }
#endif
#ifdef ENABLE_LOCAL_LOCK
            /* Needs to be thread safe */
            key = IGateProvider_enter (SharedRegion_module->localLock);
#endif
        }
#ifdef ENABLE_LOCAL_LOCK
        IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_clearEntry", status);

    return status;
}

/* Clears the reserve memory for each region in the table. */
Void
SharedRegion_clearReservedMemory (Void)
{
    SharedRegion_Region * region = NULL;
    Int                   i;

    GT_0trace (curTrace, GT_ENTER, "SharedRegion_clearReservedMemory");

    /*
     *  Loop through shared regions. If an owner of a region is specified,
     *  the owner zeros out the reserved memory in each region.
     */
    for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
        region = &(SharedRegion_module->regions [i]);
        if (   (region->entry.isValid)
            && (region->entry.ownerProcId == MultiProc_self ())) {
            /* Clear reserved memory, if any */
            if (region->reservedSize != 0) {
                memset (region->entry.base, 0, region->reservedSize);

                /* Writeback invalidate the cache if enabled in region */
                if (region->entry.cacheEnable) {
                    Cache_wbInv (region->entry.base,
                                 region->reservedSize,
                                 Cache_Type_ALL,
                                 TRUE);
                }
            }
        }
    }

    GT_0trace (curTrace, GT_LEAVE, "SharedRegion_clearReservedMemory");
}

/* Initializes the entry fields */
Void
SharedRegion_entryInit (SharedRegion_Entry * entry)
{
    GT_1trace (curTrace, GT_ENTER, "SharedRegion_entryInit", entry);

    GT_assert (curTrace, (entry != NULL));

        /* init the entry to default values */
        entry->base          = NULL;
        entry->len           = 0u;
        entry->ownerProcId   = 0u;
        entry->cacheEnable   = TRUE;
        entry->cacheLineSize = SharedRegion_module->cfg.cacheLineSize;
        entry->createHeap    = TRUE;
        entry->name          = NULL;
        entry->isValid       = FALSE;

    GT_0trace (curTrace, GT_LEAVE, "SharedRegion_entryInit");
}

/* Returns Heap Handle of associated id */
Ptr
SharedRegion_getHeap (UInt16 id)
{
#ifdef ENABLE_HEAPMP
    HeapMemMP_Handle heap = NULL;
#endif

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getHeap", id);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));

        /*
         *  If translate == TRUE or translate == FALSE
         *  and 'id' is not INVALIDREGIONID, then assert id is valid.
         *  Return the heap associated with the region id.
         *
         *  If those conditions are not met, the id is from
         *  an addres in local memory so return NULL.
         */
        if (    (SharedRegion_module->cfg.translate)
            ||  (   (SharedRegion_module->cfg.translate == FALSE)
                 && (id != SharedRegion_INVALIDREGIONID))) {
            if (id >= SharedRegion_module->cfg.numEntries) {
                /* Need to make sure id is smaller than numEntries */
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "SharedRegion_getHeap",
                                     SharedRegion_E_INVALIDARG,
                                     "id cannot be larger than numEntries!");
            }
            else {
#ifdef ENABLE_HEAPMP
                heap = SharedRegion_module->regions [id].heap;
#endif
            }
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getHeap", heap);

#ifdef ENABLE_HEAPMP
    return (Ptr) heap;
#else
    return NULL;
#endif
}

/* Returns the id of shared region in which the pointer resides.
 * Returns 0 if SharedRegion_module->cfg.translate set to TRUE.
 * It returns SharedRegion_INVALIDREGIONID if no entry is found.
 */
UInt16
SharedRegion_getId (Ptr addr)
{
    SharedRegion_Region * region   = NULL;
    UInt16                regionId = SharedRegion_INVALIDREGIONID;
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif
    UInt16                i;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getId", addr);

    /* addr can be NULL. */

    /* Return invalid for NULL addr */
    if (addr != NULL) {
#ifdef ENABLE_LOCAL_LOCK
        key = IGateProvider_enter (SharedRegion_module->localLock);
#endif

        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions [i]);
            if (   (region->entry.isValid)
                && (addr >= region->entry.base)
                && (  addr
                    < (Ptr)((UInt32) region->entry.base + region->entry.len))) {
                regionId = i;
                break;
            }
        }

#ifdef ENABLE_LOCAL_LOCK
        IGateProvider_leave (SharedRegion_module->localLock, key);
#endif
    }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getId", regionId);

    return (regionId);
}

/* Returns the id of shared region that matches name.
 * Returns SharedRegion_INVALIDREGIONID if no region is found.
 */
UInt16
SharedRegion_getIdByName (String name)
{
    SharedRegion_Region * region   = NULL;
    UInt16                regionId = SharedRegion_INVALIDREGIONID;
    UInt16                i;
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getIdByName", name);

    GT_assert (curTrace, (name != NULL));

#ifdef ENABLE_LOCAL_LOCK
        /* Needs to be thread safe */
        key = IGateProvider_enter (SharedRegion_module->localLock);
#endif

        /* loop through entries to find matching name */
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &(SharedRegion_module->regions [i]);

            if (region->entry.isValid) {
                if (strcmp (region->entry.name, name) == 0) {
                    regionId = i;
                    break;
                }
            }
        }

#ifdef ENABLE_LOCAL_LOCK
        /* leave the gate */
        IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getIdByName", regionId);

    return (regionId);
}

/* Gets the entry information for the specified region id */
Int
SharedRegion_getEntry (UInt16               id,
                       SharedRegion_Entry * entry)
{
    Int                   status = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region = NULL;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_getEntry", id, entry);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));
    GT_assert (curTrace, (entry != NULL));

        region = &(SharedRegion_module->regions [id]);

        memcpy ((Ptr) entry,
                     (Ptr) &(region->entry),
                     sizeof (SharedRegion_Entry));

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getEntry", status);

    return status;
}

/* Get cache line size */
SizeT
SharedRegion_getCacheLineSize (UInt16 id)
{
    SizeT cacheLineSize = 0u;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getCacheLineSize", id);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));

        /*
         *  If translate == TRUE or translate == FALSE
         *  and 'id' is not INVALIDREGIONID, then assert id is valid.
         *  Return the heap associated with the region id.
         *
         *  If those conditions are not met, the id is from
         *  an addres in local memory so return NULL.
         */
        if (    (SharedRegion_module->cfg.translate)
            ||  (   (SharedRegion_module->cfg.translate == FALSE)
                 && (id != SharedRegion_INVALIDREGIONID))) {
            if (id >= SharedRegion_module->cfg.numEntries) {
                /* Need to make sure id is smaller than numEntries */
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "SharedRegion_getCacheLineSize",
                                     SharedRegion_E_INVALIDARG,
                                     "id cannot be larger than numEntries!");
            }
            else {
                cacheLineSize =
                        SharedRegion_module->regions [id].entry.cacheLineSize;
            }
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getCacheLineSize",
               cacheLineSize);

    return cacheLineSize;
}

/* Is cache enabled */
Bool
SharedRegion_isCacheEnabled (UInt16 id)
{
    Bool cacheEnable = FALSE;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_isCacheEnabled", id);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));

        /*
         *  If translate == TRUE or translate == FALSE
         *  and 'id' is not INVALIDREGIONID, then assert id is valid.
         *  Return the heap associated with the region id.
         *
         *  If those conditions are not met, the id is from
         *  an addres in local memory so return NULL.
         */
        if (    (SharedRegion_module->cfg.translate)
            ||  (   (SharedRegion_module->cfg.translate == FALSE)
                 && (id != SharedRegion_INVALIDREGIONID))) {
            if (id >= SharedRegion_module->cfg.numEntries) {
                /* Need to make sure id is smaller than numEntries */
                GT_setFailureReason (curTrace,
                                     GT_4CLASS,
                                     "SharedRegion_isCacheEnabled",
                                     SharedRegion_E_INVALIDARG,
                                     "id cannot be larger than numEntries!");
            }
            else {
                cacheEnable =
                            SharedRegion_module->regions[id].entry.cacheEnable;
            }
        }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_isCacheEnabled", cacheEnable);

    return (cacheEnable);
}

/* Returns invalid SRPtr value. */
SharedRegion_SRPtr SharedRegion_invalidSRPtr (Void)
{
    return (SharedRegion_INVALIDSRPTR);
}

/* =============================================================================
 *  Internal Functions
 * =============================================================================
 */
/* Reserves the specified amount of memory from the specified region id. */
Ptr
SharedRegion_reserveMemory (UInt16 id, SizeT size)
{
    Ptr                   retPtr = NULL;
    SharedRegion_Region * region = NULL;
    UInt32                minAlign;
    SizeT                 newSize;
    SizeT                 curSize;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_reserveMemory", id, size);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));

    minAlign = Memory_getMaxDefaultTypeAlign ();

    if (SharedRegion_getCacheLineSize (id) > minAlign) {
        minAlign = SharedRegion_getCacheLineSize (id);
    }

    region = &(SharedRegion_module->regions [id]);

    /* Set the current size to the reservedSize */
    curSize = region->reservedSize;

    /* No need to round here since curSize is already aligned */
    retPtr = (Ptr)((UInt32) region->entry.base + curSize);

    /*  Round the new size to the min alignment since */
    newSize = ROUNDUP (size, minAlign);

    /* Add the new size to current size */
    region->reservedSize = curSize + newSize;

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_reserveMemory", retPtr);

    return (retPtr);
}

/* Reserves the specified amount of memory from the specified region id. */
Void
SharedRegion_unreserveMemory (UInt16 id, SizeT size)
{
    SharedRegion_Region * region = NULL;
    UInt32                minAlign;
    SizeT                 newSize;
    SizeT                 curSize;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_unreserveMemory", id, size);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));

    minAlign = Memory_getMaxDefaultTypeAlign ();

    if (SharedRegion_getCacheLineSize (id) > minAlign) {
        minAlign = SharedRegion_getCacheLineSize (id);
    }

    region = &(SharedRegion_module->regions [id]);

    /* Set the current size to the unreservedSize */
    curSize = region->reservedSize;

    /*  Round the new size to the min alignment since */
    newSize = ROUNDUP (size, minAlign);

    /* Add the new size to current size */
    region->reservedSize = curSize - newSize;

    GT_0trace (curTrace, GT_LEAVE, "SharedRegion_unreserveMemory");

    return ;
}

/* Return the number of offsetBits bits */
static UInt32 SharedRegion_getNumOffsetBits (Void)
{
    UInt32    numEntries = SharedRegion_module->cfg.numEntries;
    UInt32    indexBits  = 0;
    UInt32    numOffsetBits = 0;

    GT_0trace (curTrace, GT_ENTER, "SharedRegion_getNumOffsetBits");

    if (numEntries == 0) {
        indexBits = 0;
    }
    else if (numEntries == 1) {
        indexBits = 1;
    }
    else {
        numEntries = numEntries - 1;

        /* determine the number of bits for the index */
        while (numEntries) {
            indexBits++;
            numEntries = numEntries >> 1;
        }
    }

    numOffsetBits = 32 - indexBits;

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getNumOffsetBits",
            numOffsetBits);

    return (numOffsetBits);
}

/* Checks to make sure overlap does not exists. */
static Int SharedRegion_checkOverlap(Ptr base, SizeT len)
{
    Int                   status = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region = NULL;
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key    = 0;
#endif
    UInt32                i;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_checkOverlap", base, len);

#ifdef ENABLE_LOCAL_LOCK
    key = IGateProvider_enter (SharedRegion_module->localLock);
#endif
    /* check whether new region overlaps existing ones */
    for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
        region = &(SharedRegion_module->regions [i]);
        if (region->entry.isValid) {
            if ((base == region->entry.base)&&(len == region->entry.len)) {
                /* OK.Return sucess as user is going to create the same
                 * shared region from different process
                 */
                break;
            }
            else if (base >= region->entry.base) {
                if (   base
                        <  (Ptr)(   (UInt32) region->entry.base
                                +   region->entry.len)) {
                    status = SharedRegion_E_FAIL;
                    GT_setFailureReason (curTrace,
                            GT_4CLASS,
                            "SharedRegion_checkOverlap",
                            status,
                            "Specified region falls within another"
                            " region!");
                    /* Break on failure. */
                    break;
                }
            }
            else {
                if ((Ptr) ((UInt32) base + len) > region->entry.base) {
                    status = SharedRegion_E_FAIL;
                    GT_setFailureReason (curTrace,
                            GT_4CLASS,
                            "SharedRegion_checkOverlap",
                            status,
                            "Specified region spans across"
                            " multiple regions!");
                    /* Break on failure. */
                    break;
                }
            }
        }
    }

#ifdef ENABLE_LOCAL_LOCK
    IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_checkOverlap", status);

    return (status);
}

/* Return the region info */
Void
SharedRegion_getRegionInfo (UInt16                i,
                            SharedRegion_Region * region)
{
    SharedRegion_Region * regions = NULL;

    GT_2trace (curTrace, GT_ENTER, "SharedRegion_getRegionInfo", i, region);

    GT_assert (curTrace, (i < SharedRegion_module->cfg.numEntries));
    GT_assert (curTrace, (region != NULL));

        regions = &(SharedRegion_module->regions[i]);

        memcpy ((Ptr) region,
                     (Ptr) regions,
                     sizeof (SharedRegion_Region));

    GT_0trace (curTrace, GT_LEAVE, "SharedRegion_getRegionInfo");
}

/* Sets the table information entry in the table (doesn't create heap). */
Int
_SharedRegion_setEntry (UInt16                id,
                        SharedRegion_Entry  * entry)
{
    Int                   status  = SharedRegion_S_SUCCESS;
    SharedRegion_Region * region  = NULL;
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif

    GT_2trace (curTrace, GT_ENTER, "_SharedRegion_setEntry", id, entry);

    GT_assert (curTrace, (id < SharedRegion_module->cfg.numEntries));
    GT_assert (curTrace, (entry != NULL));

        region = &(SharedRegion_module->regions [id]);

        /* Make sure region does not overlap existing ones */
        status = SharedRegion_checkOverlap (region->entry.base,
                                            region->entry.len);
#ifdef ENABLE_LOCAL_LOCK
            /* needs to be thread safe */
            key = IGateProvider_enter (SharedRegion_module->localLock);
#endif

            /* set specified region id to entry values */
            memcpy ((Ptr) &(region->entry),
                         (Ptr) entry,
                         sizeof (SharedRegion_Entry));

#ifdef ENABLE_LOCAL_LOCK
            /* leave gate */
            IGateProvider_leave (SharedRegion_module->localLock, key);
#endif

    GT_1trace (curTrace, GT_LEAVE, "_SharedRegion_setEntry", status);

    return (status);
}

/* Gets region ID when given a physical address */
UInt16 _SharedRegion_getIdPhys(Ptr physAddr)
{
    SharedRegion_Region * region   = NULL;
    UInt16                regionId = SharedRegion_INVALIDREGIONID;
#ifdef ENABLE_LOCAL_LOCK
    IArg                  key;
#endif
    UInt16                i;
    Ptr                   base;

    GT_1trace (curTrace, GT_ENTER, "SharedRegion_getIdPhys", physAddr);

    /* physAddr can be NULL. */

    /* Return invalid for NULL physAddr */
    if (physAddr != NULL) {
#ifdef ENABLE_LOCAL_LOCK
        key = IGateProvider_enter (SharedRegion_module->localLock);
#endif
        for (i = 0; i < SharedRegion_module->cfg.numEntries; i++) {
            region = &SharedRegion_module->regions[i];
            if (region->entry.isValid) {
#ifdef ENABLE_HEAPMP
                if (region->entry.createHeap) {
                    base = MemoryOS_translate(region->entry.base,
                                              Memory_XltFlags_Virt2Phys);
                }
                else
#endif
		{
                    base = region->entry.base;
                }
                if ((physAddr >= base) &&
                    (physAddr < (Ptr)((UInt32)base + region->entry.len))) {

                    regionId = i;
                    break;
                }
            }
        }

#ifdef ENABLE_LOCAL_LOCK
        IGateProvider_leave (SharedRegion_module->localLock, key);
#endif
    }

    GT_1trace (curTrace, GT_LEAVE, "SharedRegion_getIdPhys", regionId);

    return (regionId);
}

/*
 *  ======== SharedRegion_getEntryPtr ========
 */
SharedRegion_Entry *SharedRegion_getEntryPtr(UInt16 id)
{
    SharedRegion_Region *region;

    region = &(SharedRegion_module->regions[id]);
    return ((SharedRegion_Entry *)(&region->entry));
}
