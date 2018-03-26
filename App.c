/*
 * Copyright (c) 2013-2017, Texas Instruments Incorporated
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

/*
 *  ======== App.c ========
 *
 */

/* host header files */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>


#include <errno.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <ti/cmem.h>

/* package header files */
#include <ti/ipc/Std.h>
#include "Std.h"
#include <ti/ipc/MessageQ.h>
#include <string.h>
//#include <bigdataxlat.h>
#include "HeapMem.h"
#include "SharedRegion.h"
#include "MemoryDefs.h"

/* local header files */
#include "AppCommon.h"
#include "App.h"
#include "Wav.h"

/* Application specific defines */
#define BIG_DATA_POOL_SIZE 0x1000000
#define WAV_FILE    "/tmp/test.wav"

/* round up the value 'size' to the next 'align' boundary */
#define ROUNDUP(size, align) \
    (UInt32)(((UInt32)(size) + ((UInt32)(align) - 1)) & ~((UInt32)(align) - 1))
#define DEBUG 1
/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;
} App_Module;

/* private data */
static App_Module g_module;
static HeapMem_Handle g_sr1Heap = NULL;
static void *g_sharedRegionAllocPtr=NULL;
static CMEM_AllocParams g_cmemAttrs;
static bigDataLocalDesc_t g_bigDataLocalDesc = {0};

/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    printf("--> App_create:\n");

    /* setting default values */
    g_module.hostQue = NULL;
    g_module.slaveQue = MessageQ_INVALIDMESSAGEQ;
    g_module.heapId = App_MsgHeapId;
    g_module.msgSize = sizeof(App_Msg);

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);

    g_module.hostQue = MessageQ_create(App_HostMsgQueName, &msgqParams);

    if (g_module.hostQue == NULL) {
        printf("App_create: Failed creating MessageQ\n");
        status = -1;
        goto leave;
    }

    /* open the remote message queue */
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(remoteProcId));

    do {
        status = MessageQ_open(msgqName, &g_module.slaveQue);
        sleep(1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        printf("App_create: Failed opening MessageQ\n");
        goto leave;
    }

    printf("App_create: Host is ready\n");

leave:
    printf("<-- App_create:\n");
    return(status);
}

/*
 *  ======== App_delete ========
 */
Int App_delete(Void)
{
    Int         status;

    printf("--> App_delete:\n");

    /* close remote resources */
    status = MessageQ_close(&g_module.slaveQue);

    if (status < 0) {
        goto leave;
    }

    /* delete the host message queue */
    status = MessageQ_delete(&g_module.hostQue);

    if (status < 0) {
        goto leave;
    }

leave:
    printf("<-- App_delete:\n");
    return(status);
}


Int init_CMEM(void **sharedRegionAllocPtr, CMEM_AllocParams *cmemAttrs)
{
    Int status;
    Int pool_id = -1;

    printf("start init cmem\n");
    do {
        /* CMEM: contiguous memory manager for HLOS */
        /* initialised here */
        status = CMEM_init();
        if (status < 0) {
            printf("CMEM_init failed\n");
            break;
        }
        printf("CMEM_init success\n");

        pool_id = CMEM_getPool(BIG_DATA_POOL_SIZE);
        if (pool_id < 0) {
            printf("CMEM_getPool failed\n");
            break;
        }
        printf("CMEM_getPool success\n");

        cmemAttrs->type = CMEM_HEAP;
        cmemAttrs->flags =  CMEM_CACHED;
        cmemAttrs->alignment = 0;
        *sharedRegionAllocPtr = CMEM_allocPool(pool_id, cmemAttrs);
        if (*sharedRegionAllocPtr == NULL) {
            printf("CMEM_allocPool failed\n");
            status = -1;
            break;
        }
        printf("CMEM_allocPool success: Allocated buffer %p\n", *sharedRegionAllocPtr);

    } while(0);

    if (status < 0 || pool_id < 0) {

        if (*sharedRegionAllocPtr) {
            printf("free share region ptr\n");
            /* free the message */
            CMEM_free(sharedRegionAllocPtr, cmemAttrs);
        }
        status = -1;
    }

    return status;
}

Int create_shared_region(SharedRegion_Entry **pSrEntry, UInt16 regionId, void *sharedRegionAllocPtr)
{
    Int status;

    SharedRegion_Config sharedRegionConfig;
    SharedRegion_Entry srEntry;

    do {
        /* Create shared region */
        SharedRegion_getConfig (&sharedRegionConfig);
        status = SharedRegion_setup (&sharedRegionConfig);
        if (status < 0) {
            printf("SharedRegion_setup failed\n");
            break;
        }
        printf("SharedRegion_setup success\n");

        /* Configure srEntry */
        srEntry.base = sharedRegionAllocPtr;
        srEntry.len = BIG_DATA_POOL_SIZE;
        srEntry.ownerProcId = MultiProc_self();
        /* Make sure this is enabled if using Cached memory */
        srEntry.isValid = TRUE;
        srEntry.cacheEnable = TRUE;
        srEntry.createHeap = FALSE;
        srEntry.cacheLineSize = 128;
        srEntry.name = "SR1";

        status = SharedRegion_setEntry (regionId, &srEntry);

        *pSrEntry = SharedRegion_getEntryPtr(regionId);
        printf("App_taskFxn: SR_1, base 0x%x, len=%x\n", (UInt32)(*pSrEntry)->base, (*pSrEntry)->len);

    } while(0);

    return status;
}

Int create_HeapMem(HeapMem_Handle *sr1Heap, SharedRegion_Entry *pSrEntry)
{
    Int status;
    HeapMem_Config HeapMemConfig;
    HeapMem_Params heapMem_params;
    Memory_Stats stats;
    HeapMem_ExtendedStats extStats;

    do {

        /* Setup HeapMem module */
        HeapMem_getConfig (&HeapMemConfig);
        status = HeapMem_setup (&HeapMemConfig);
        if (status < 0) {
            printf("HeapMem_setup failed\n");
            break;
        }
        printf("HeapMem_setup success\n");

        /* Create HeapMP at run-time:
            This heap is intended to be used for big data ipc */
        HeapMem_Params_init(&heapMem_params);
        heapMem_params.name = "sr1HeapMem";
        heapMem_params.sharedAddr = pSrEntry->base;
        heapMem_params.sharedBufSize = ROUNDUP(pSrEntry->len, pSrEntry->cacheLineSize);
        heapMem_params.gate = NULL;
        *sr1Heap = HeapMem_create(&heapMem_params);
        if ( !(*sr1Heap) ) {
            printf("sr1Heap creation failed\n");
            status = -1;
            break;
        }
        printf("HeapMem_create success\n");
        HeapMem_getStats((HeapMem_Handle)(*sr1Heap), &stats);
        printf("App_taskFxn: SR_1 heap, totalSize=%d,totalFreeSize=%d,largestFreeSize=%d\n", stats.totalSize, stats.totalFreeSize, stats.largestFreeSize);
        HeapMem_getExtendedStats((HeapMem_Handle)(*sr1Heap), &extStats);
        printf("App_taskFxn: SR_1 heap, buf=0x%p,size=%d\n", extStats.buf, extStats.size);

        /* fill process pipeline */

        printf("App_exec: send App_CMD_SHARED_REGION_INIT\n");
    } while(0);

    return status;
}

Int Led_Ctrl(Int c)
{
     char cmdstr[40];
     system("i2cset -y -f 0 0x3c 0 1");
     system("i2cset -y -f 0 0x3c 0x4a 0");
     sprintf(cmdstr,"i2cset -y -f 0 0x3c 0x%02x 0x33\n",c*3+1);
     system(cmdstr);
     sprintf(cmdstr,"i2cset -y -f 0 0x3c 0x%02x 0x01\n",0x25+c*3+1);
     system(cmdstr);
     system("i2cset -y -f 0 0x3c 0x25 0");
}
Int Led_Blank(void)
{
     system(" i2cset -y -f 0 0x3c 0x4f 0");
}


Int transfer_init()
{
    Int status = -1;
    SharedRegion_Entry *pSrEntry;
    UInt16 regionId = 1;
    UInt32 cnt = 0;
    int i;
    printf("--> App_exec:\n");
	
    g_sr1Heap = NULL;
    App_Msg *msg = NULL;


    do {

        status =  init_CMEM(&g_sharedRegionAllocPtr, &g_cmemAttrs);
        if (status < 0) {
            break;
        }


        status = create_shared_region(&pSrEntry, regionId, g_sharedRegionAllocPtr);
        if (status < 0) {
            break;
        }

        status = create_HeapMem(&g_sr1Heap, pSrEntry);
        if (status < 0) {
            break;
        }

		//sleep(1);
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(g_module.heapId, g_module.msgSize);

        if (msg == NULL) {
            status = -1;
            break;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(g_module.hostQue, (MessageQ_Msg)msg);
        /* fill in message payload for Shared region init*/
        msg->cmd = App_CMD_SHARED_REGION_INIT;
        msg->id = 0; //used to count all the message
        msg->regionId = regionId;
        /* Passing the local shared memory address to the remote */
        /* Actually this can be any allocated buffer for the used for the heap */
        msg->u.sharedRegionInitCfg.base = (UInt64)CMEM_getPhys(pSrEntry->base);
        //printf("Shared memory phys Addr %llx\n", msg->u.sharedRegionInitCfg.base);
        if (!msg->u.sharedRegionInitCfg.base) {
            printf("CMEM_getPhys failed\n");
        }
        msg->u.sharedRegionInitCfg.size = (UInt64)(pSrEntry->len);

        /* send message */
        MessageQ_put(g_module.slaveQue, (MessageQ_Msg)msg);
        /* wait for return message */

        /* wait for return message */
        status = MessageQ_get(g_module.hostQue, (MessageQ_Msg *)&msg,
                              (UInt)MessageQ_FOREVER);
        if (status < 0) {
            break;
        }

        /* extract message payload */

        if(msg->cmd == App_CMD_SHARED_REGION_INIT){
           // printf(" DSP SrINIT ok!\n");
        }

        //printf("App_exec: msg is: %d cnt:%d\n", msg->id,cnt++);
	printf("c:%d\n",cnt++);
        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);


    } while(0);

    return status;
}

Int transfer_recv_data(unsigned short **data, Int *size)
{
    Int status = -1;
    UInt16 regionId = 1;
    Int retVal;
	Int i;
    UInt32 *bigDataLocalPtr;
    //UInt32 errorCount=0;
    bigDataSharedDesc_t _SharedDesc;  //msg->u.bigDataSharedDesc)
	    UInt32 cnt = 0;
    App_Msg *msg = NULL;

    do {

        /* Allocate buffer from HeapMem */
        bigDataLocalPtr = (UInt32 *)(HeapMem_alloc(g_sr1Heap,
                                                   BIGDATA_BUF_SIZE,
                                                   BIGDATA_BUF_ALIGN));

        if (!bigDataLocalPtr) {
            status = -1;
            printf("HeapMem_alloc failed\n");
            break;
        }



        /* Populate the Local descriptor */
        g_bigDataLocalDesc.localPtr = (Ptr)bigDataLocalPtr;
        g_bigDataLocalDesc.size = BIGDATA_BUF_SIZE;
        retVal = bigDataXlatetoGlobalAndSync(regionId,
                                             &g_bigDataLocalDesc,
                                             &_SharedDesc);
printf("file:%s ,line:%d \n",__FILE__,__LINE__);
        if (retVal) {
            status = -1;
            printf("bigDataXlatetoGlobalAndSync failed\n");
            break;
        }
		while(1){
            //printf("start rcv data\n");
			//usleep(5000);
			cnt++;
			/* allocate message */
        	msg = (App_Msg *)MessageQ_alloc(g_module.heapId, g_module.msgSize);
        	if (msg == NULL) {
            	status = -1;
            	break;
        	}
        	/* set the return address in the message header */
        	MessageQ_setReplyQueue(g_module.hostQue, (MessageQ_Msg)msg);


        	msg->regionId = regionId;
        	msg->cmd = App_CMD_BIGDATA; 
            msg->id = 1; //used to count all the message
            msg->u.bigDataSharedDesc.sharedPtr=_SharedDesc.sharedPtr;
			msg->u.bigDataSharedDesc.size=_SharedDesc.size;
            //for(i=0;i<BIGDATA_BUF_SIZE/4;i++)
			for(i=0;i<8;i++)
				*((Int *)(g_bigDataLocalDesc.localPtr)+i)=i+3+cnt;
        	/* send message */
        	MessageQ_put(g_module.slaveQue, (MessageQ_Msg)msg);

        	/* wait for return message */
       		 status = MessageQ_get(g_module.hostQue,
                              (MessageQ_Msg *)&msg,
                              (UInt)MessageQ_FOREVER);
        	if (status < 0) {
            	break;
        	}

        	/* extract message payload */
        	if (msg->cmd == App_CMD_BIGDATA) {

            	retVal = bigDataXlatetoLocalAndSync(msg->regionId,
                                                &msg->u.bigDataSharedDesc,
                                                &g_bigDataLocalDesc);
            	if (retVal) {
              	  status = -1;
              	  break;
            	}


            	(*data) = (unsigned short *) g_bigDataLocalDesc.localPtr;
            	(*size) = g_bigDataLocalDesc.size;
				
            	/* Free big data buffer */
            	//printf("App_exec: message received %d\n", msg->id);
           		//printf("c:%d size:%d s:%d ", cnt,g_bigDataLocalDesc.size, status);
				printf("c:%d", cnt);
				//for(i=0;i<2;i++)
				//printf("0x%x ",*((Int *)(g_bigDataLocalDesc.localPtr)+i));
				printf("\n");
            	/* free the message */
            	MessageQ_free((MessageQ_Msg)msg);
            	msg = NULL;
        	}
		}
    } while(0);

    return status;
}



Int transfer_clean()
{
    if (g_sr1Heap) {
        printf("clean the sr1Heap\n");
        sleep(1);
        HeapMem_free(g_sr1Heap, g_bigDataLocalDesc.localPtr, g_bigDataLocalDesc.size);
        HeapMem_delete(&g_sr1Heap);
        g_sr1Heap = NULL;
    }

    if (g_sharedRegionAllocPtr) {
        printf(" outside free share region ptr\n");
        sleep(1);

        /* free the message */
        CMEM_free(g_sharedRegionAllocPtr, &g_cmemAttrs);
        g_sharedRegionAllocPtr = NULL;
    }

    /* Print error count if non-zero */

}

Int Transfer_send(UInt32 cmd, App_Msg *msg)
{
    /* set the return address in the message header */
    MessageQ_setReplyQueue(g_module.hostQue, (MessageQ_Msg)msg);

    /* fill in message payload */
    msg->cmd = cmd;

    /* send message */
    MessageQ_put(g_module.slaveQue, (MessageQ_Msg)msg);

}



Int Transfer_recv(UInt32 cmd, App_Msg *msg, void *data)
{
    Int status = -1;

    do {
        /* wait for return message */
        status = MessageQ_get(g_module.hostQue, (MessageQ_Msg *)&msg,
                              MessageQ_FOREVER);

        if (status < 0) {
            break;
        }

        switch (cmd) {
            case App_CMD_NOP:
                status = 0;
                printf("success reply\n");
                break;
            case App_CMD_SHUTDOWN:
                status = 0;
                printf("success to shutdown\n");
                break;
            case App_CMD_START:
                status = 0;
                printf("success to start\n");
                break;

            case App_CMD_STOP:
                status = 0;
                printf("success to stop\n");
                break;

            default:
                break;
        }

    } while(0);

    return status;
}

Int Transfer_shutdown()
{
    App_Msg *msg;
    Int status;

    do {
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(g_module.heapId, g_module.msgSize);
        if (msg == NULL) {
            status = -1;
            break;
        }
        status = Transfer_send(App_CMD_SHUTDOWN, msg);
        if (status < 0) {
            printf("transfer send fail\n");
            break;
        }

        Transfer_recv(App_CMD_SHUTDOWN, msg, NULL);
        if (status < 0) {
            printf("transfer receive fail\n");
            break;
        }


    } while(0);
    return status;

}

Int Transfer_start()
{
    App_Msg *msg;
    Int status;

    do {
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(g_module.heapId, g_module.msgSize);
        if (msg == NULL) {
            status = -1;
            break;
        }
        status = Transfer_send(App_CMD_START, msg);
        if (status < 0) {
            printf("transfer start fail\n");
            break;
        }

        Transfer_recv(App_CMD_START, msg, NULL);
        if (status < 0) {
            printf("transfer start receive fail\n");
            break;
        }


    } while(0);
    return status;

}

Int Transfer_stop()
{
    App_Msg *msg;
    Int status;

    do {
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(g_module.heapId, g_module.msgSize);
        if (msg == NULL) {
            status = -1;
            break;
        }
        status = Transfer_send(App_CMD_STOP, msg);
        if (status < 0) {
            printf("transfer stop fail\n");
            break;
        }

        Transfer_recv(App_CMD_STOP, msg, NULL);
        if (status < 0) {
            printf("transfer stop fail\n");
            break;
        }


    } while(0);
    return status;

}


Int App_exec(Void)
{
    Int status = -1;
    unsigned short *data;
    Int size;
    WAVE_HEAD head = {0};

    int j;

    do {
		 printf("transfer_init!!!\n");
        status = transfer_init();
        if (status < 0) {
            break;
        }
		printf("Transfer_start!!!\n");
        status = Transfer_start();
        if (status < 0) {
            break;
        }

		while(1){

			 printf("enter recv data\n");
        	status = transfer_recv_data(&data, &size);
        	if (status < 0 ) {
            	break;
        	}
			sleep(1);
		}
		
        //将数据保存到wav格式的文件
        wav_init(&head, 16, 2, 44100);

        for (j = 0;  j < size / sizeof(unsigned short); j += 4) {
            wav_write(&head, WAV_FILE, data[j]);
        }

        wave_finished(&head);

        status = Transfer_stop();

        if (status < 0) {
            break;
        }

    } while(0);

    transfer_clean();
    return status;
}
