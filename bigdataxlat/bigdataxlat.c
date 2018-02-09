/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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
#include <_bigdataxlat.h>

Int bigDataXlatetoGlobalAndSync(UInt16  regionId,
    bigDataLocalDesc_t *localDesc,
    bigDataSharedDesc_t *SharedDesc) {

    if (localDesc->localPtr == NULL)
        return -1;

    /* Cache write back data buffer */
    if (SharedRegion_isCacheEnabled(regionId)) {
        Cache_wb(localDesc->localPtr, localDesc->size, Cache_Type_ALL, TRUE);
    }

    /* Translate Address to Shared address */
    SharedDesc->sharedPtr = SharedRegion_getSRPtr(localDesc->localPtr, regionId);
    SharedDesc->size = localDesc->size;
    return(0);
}

Int bigDataXlatetoLocalAndSync(UInt16  regionId,
    bigDataSharedDesc_t *SharedDesc,
    bigDataLocalDesc_t *localDesc) {

    if (SharedDesc->sharedPtr == 0)
        return -1;

    /* Translate to local address */
    localDesc->localPtr = SharedRegion_getPtr(SharedDesc->sharedPtr);
    localDesc->size = SharedDesc->size;

    /* If shared region is configured for cache enabled do cache operation */
    if (SharedRegion_isCacheEnabled(regionId)) {
        Cache_inv(localDesc->localPtr,
            localDesc->size, Cache_Type_ALL, TRUE);
    }

    return(0);
}
