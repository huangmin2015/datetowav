/** 
 *  @file   _HeapMem.h
 *
 *  @brief      Defines HeapMem based memory allocator.
 *
 *
 */
/* 
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



#ifndef HeapMem_H_0x4C56
#define HeapMem_H_0x4C56


/* Osal & Utils headers */
#include "IHeap.h"


#if defined (__cplusplus)
extern "C" {
#endif


/*!
 *  @def    HeapMem_MODULEID
 *  @brief  Unique module ID.
 */
#define HeapMem_MODULEID        (0x4AD8)

/*!
 *  @var    HeapMem_CREATED
 *
 *  @brief  HeapMemMp tag used in the attrs->status field
 */
#define HeapMem_CREATED     0x07041976

/*
 *  @brief  Object for the HeapMem Handle
 */
#define HeapMem_Object IHeap_Object


/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */

/* =============================================================================
 *  APIs
 * =============================================================================
 */

/*!
 *  @brief      Function to destroy the HeapMem module.
 *
 *  @param      None
 *
 *  @sa         HeapMem_setup
 *              HeapMem_getConfig
 *              NameServer_create
 *              NameServer_delete
 *              GateMutex_delete
 */
Int HeapMem_destroy (void);

/*!
 *  @brief      Returns the HeapMem kernel object pointer.
 *
 *  @param      handle  Handle to previousely created/opened instance.
 *
 */
Ptr HeapMem_getKnlHandle (HeapMem_Handle hpHandle);

/*!
 *  @brief      Returns a HeapMem user object pointer.
 *
 *  @param      handle  Handle to kernel handle for which user handle is to be
 *                      provided.
 *
 */
Ptr HeapMem_getUsrHandle (HeapMem_Handle hpHandle);


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* HeapMem_H_0x4C56 */

