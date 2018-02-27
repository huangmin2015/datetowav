#
#  Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  *  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  *  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  *  Neither the name of Texas Instruments Incorporated nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
#  ======== Makefile ========
#

BIGDATA_DIR = ./bigdataxlat
SHARE_REGION_DIR = ./SharedRegion
UTILS_DIR = ./utils
HEAP_MEM_DIR = ./HeapMem

#  ======== toolchain macros ========
TOOLCHAIN_PREFIX := ${ROOT_PATH}/usr/local/oecore-x86_64/sysroots/x86_64-angstromsdk-linux/usr/bin/arm-angstrom-linux-gnueabi/arm-angstrom-linux-gnueabi-
SDK_LINUX_INSTALL := /home/alan/am57xx-evm-04.02.00.09



PLATFORM=

srcs = ${wildcard ./*.c} 
local_lib_srcs += ${wildcard $(BIGDATA_DIR)/*.c}
local_lib_srcs += ${wildcard $(SHARE_REGION_DIR)/*.c}
local_lib_srcs += ${wildcard $(UTILS_DIR)/*.c}
local_lib_srcs += ${wildcard $(HEAP_MEM_DIR)/*.c}

objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A,$(srcs)))
local_lib_objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A, ${notdir $(local_lib_srcs)}))



all:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	#$(MAKE) PROFILE=debug app_host
	$(MAKE) PROFILE=release app_host

help:
	@$(ECHO) "make                  # build executables"
	@$(ECHO) "make clean            # clean everything"

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/$(PLATFORM)/release/app_host $(EXEC_DIR)/release

clean::
	$(RMDIR) bin

test:
	@echo $(CFLAGS)

#
#  ======== rules ========
#

app_host:  bin/$(PLATFORM)/$(PROFILE)/app_host

bin/$(PLATFORM)/$(PROFILE)/app_host: $(objs) $(local_lib_objs)
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)
bin/$(PROFILE)/obj/%.ov7A: $(BIGDATA_DIR)/%.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^

bin/$(PROFILE)/obj/%.ov7A: $(HEAP_MEM_DIR)/%.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^

bin/$(PROFILE)/obj/%.ov7A: $(SHARE_REGION_DIR)/%.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^

bin/$(PROFILE)/obj/%.ov7A: $(UTILS_DIR)/%.c 
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^

bin/$(PROFILE)/obj/%.ov7A: %.c 
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^


#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
	$(error must specify EXEC_DIR)
endif
endif




CC = $(TOOLCHAIN_PREFIX)gcc
AR = $(TOOLCHAIN_PREFIX)ar
LD = $(TOOLCHAIN_PREFIX)gcc
CFLAGS =
CPPFLAGS =
LDFLAGS = -L$(SDK_LINUX_INSTALL)/linux-devkit/sysroots/armv7ahf-neon-linux-gnueabi/usr/lib/
        

CFLAGS += -c -MD -MF $@.dep

ARFLAGS = cr

CPPFLAGS += -D_REENTRANT

CFLAGS +=  -ffloat-store -fPIC -Wunused -pthread -Dfar= $(CCPROFILE_$(PROFILE)) \
    -I. -I.. 

#include 的目录
CFLAGS += -I$(SDK_LINUX_INSTALL)/linux-devkit/sysroots/armv7ahf-neon-linux-gnueabi/usr/include
CFLAGS += -I$(HEAP_MEM_DIR) -I$(SHARE_REGION_DIR) -I$(BIGDATA_DIR)  -I$(UTILS_DIR)  
LDFLAGS += $(LDPROFILE_$(PROFILE)) -Wall -Wl,-Map=$@.map

LDLIBS = -lpthread -lc -lrt -lticmem -ltitransportrpmsg \
         -ltiipc -ltiipcutils


CCPROFILE_debug = -ggdb -D DEBUG
CCPROFILE_release = -O3 -D NDEBUG

LDPROFILE_debug = -ggdb
LDPROFILE_release = -O3

#  ======== standard macros ========
 	# use these on Linux
CP      = cp
ECHO    = echo
MKDIR   = mkdir -p
RM      = rm -f
RMDIR   = rm -rf

MAKECMDGOALS=

#======== create output directories ========
ifneq (,$(PROFILE))

ifeq ( , $(wildcard bin/$(PROFILE)/obj))
$(shell $(MKDIR) -p bin/$(PROFILE)/obj)
endif

ifeq ("debug", $(PROFILE))
CFLAGS += -D DEBUG
endif

endif

