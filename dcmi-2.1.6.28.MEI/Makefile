################################################################################
#
#
# Part of Intel(R) Data Center Host Interface (DCMI-HI) Linux driver 
#
# Copyright (c) 2010 Intel Corp.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions, and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    substantially similar to the "NO WARRANTY" disclaimer below
#    ("Disclaimer") and any redistribution must be conditioned upon
#    including a substantially similar Disclaimer requirement for further
#    binary redistribution.
# 3. Neither the names of the above-listed copyright holders nor the names
#    of any contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGES.
#
################################################################################

###########################################################################
# Driver files

# core driver files
CFILES = dcmi_init.c io_dcmi.c dcmi_protocol.c dcmi_transport.c dcmi_main.c
HFILES = dcmi.h io_dcmi.h dcmi_protocol.h dcmi_transport.h ver.h kcompat.h
ifeq (,$(BUILD_KERNEL))
BUILD_KERNEL = $(shell uname -r)
endif

###########################################################################
# Environment tests

# Kernel Search Path
# All the places we look for kernel source
KSP :=  /lib/modules/$(BUILD_KERNEL)/source \
        /usr/src/kernel-headers-$(BUILD_KERNEL) \
        /usr/src/linux-headers-$(BUILD_KERNEL) \
        /usr/src/kernel-source-$(BUILD_KERNEL) \
        /usr/src/linux-$(BUILD_KERNEL) \
        /usr/src/linux-source-$(BUILD_KERNEL) \
        /usr/src/kernel-headers-$(shell echo $(BUILD_KERNEL) | sed 's/-.*//') \
        /usr/src/linux-headers-$(shell echo $(BUILD_KERNEL) | sed 's/-.*//') \
        /usr/src/kernel-source-$(shell echo $(BUILD_KERNEL) | sed 's/-.*//') \
        /usr/src/linux-$(shell echo $(BUILD_KERNEL) | sed 's/-.*//') \
        /usr/src/linux-source-$(shell echo $(BUILD_KERNEL) | sed 's/-.*//') \
        /usr/src/kernel-headers-$(shell echo $(BUILD_KERNEL) | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/linux-headers-$(shell echo $(BUILD_KERNEL) | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/kernel-source-$(shell echo $(BUILD_KERNEL) | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/linux-$(shell echo $(BUILD_KERNEL) | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/linux-source-$(shell echo $(BUILD_KERNEL) | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/linux

# prune the list down to only values that exist
# and have an include/linux sub-directory
test_dir = $(shell [ -e $(dir)/include/linux ] && echo $(dir))
KSP := $(foreach dir, $(KSP), $(test_dir))

# we will use this first valid entry in the search path
ifeq (,$(KSRC))
  KSRC := $(firstword $(KSP))
endif
ifeq (,$(KSRC))
  $(error Linux kernel source not found)
endif

# Point to a build area for this kernel in order to do
# module compiling.  If not found settle for a source area.
ifneq (,$(wildcard /lib/modules/$(BUILD_KERNEL)/build))
  KOBJ := /lib/modules/$(BUILD_KERNEL)/build
  CONFIG_FILE := $(KOBJ)/.config
  ifeq (,$(wildcard $(CONFIG_FILE)))
    $(error Linux kernel build not configured - missing .config)
  endif
else
  KOBJ := $(KSRC)
  ifneq (,$(wildcard $(KSRC)/include/linux/config.h))
    CONFIG_FILE := $(KSRC)/include/linux/config.h
  else
    ifneq (,$(wildcard $(KSRC)/include/linux/autoconf.h))
      CONFIG_FILE := $(KSRC)/include/linux/autoconf.h
    endif
  endif
  ifeq (,$(wildcard $(CONFIG_FILE)))
    $(error Linux kernel source not configured - missing config.h and autoconf.h)
  endif
endif

# pick a compiler
ifneq (,$(findstring egcs-2.91.66, $(shell cat /proc/version)))
  CC := kgcc gcc cc
else
  CC := gcc cc
endif
test_cc = $(shell $(cc) --version > /dev/null 2>&1 && echo $(cc))
CC := $(foreach cc, $(CC), $(test_cc))
CC := $(firstword $(CC))
ifeq (,$(CC))
  $(error Compiler not found)
endif

# we need to know what platform the driver is being built on
# some additional features are only built on Intel platforms
ARCH := $(shell uname -m | sed 's/i.86/i386/')
ifeq ($(ARCH),alpha)
  EXTRA_CFLAGS += -ffixed-8 -mno-fp-regs
endif
ifeq ($(ARCH),x86_64)
  EXTRA_CFLAGS += -mcmodel=kernel -mno-red-zone
endif
ifeq ($(ARCH),ppc)
  EXTRA_CFLAGS += -msoft-float
endif
ifeq ($(ARCH),ppc64)
  EXTRA_CFLAGS += -m64 -msoft-float
  LDFLAGS += -melf64ppc
endif

# standard flags for module builds
EXTRA_CFLAGS += -DLINUX -D__KERNEL__ -DMODULE -O2 -pipe -Wall
ifneq (,$(BUILD_VERSION))
EXTRA_CFLAGS += -DBUILD_VERSION_STR=\"$(BUILD_VERSION)\"
endif
EXTRA_CFLAGS += -I$(KSRC)/include -I.
EXTRA_CFLAGS += $(shell [ -f $(KSRC)/include/linux/modversions.h ] && \
            echo "-DMODVERSIONS -DEXPORT_SYMTAB \
                  -include $(KSRC)/include/linux/modversions.h")

EXTRA_CFLAGS += $(CFLAGS_EXTRA)

RHC := $(KSRC)/include/linux/rhconfig.h
ifneq (,$(wildcard $(RHC)))
  # 7.3 typo in rhconfig.h
  ifneq (,$(shell $(CC) $(CFLAGS) -E -dM $(RHC) | grep __module__bigmem))
	CFLAGS += -D__module_bigmem
  endif
endif

# get the kernel version - we use this to find the correct install path
KVER := $(shell uname -r)

KKVER := $(shell echo $(KVER) | \
         awk '{ if ($$0 ~ /2\.[4-9]\./) print "1"; else print "0"}')
#ifeq ($(KKVER), 0)
#  $(error *** Aborting the build. \
#          *** This driver is not supported on kernel versions older than 2.4.0)
#endif

# set the install path
INSTDIR := /lib/modules/$(KVER)/kernel/drivers/char

# determine SMP setting of build area
ifeq ($(KOBJ),$(KSRC))
  SMP := $(shell $(CC) $(CFLAGS) -E -dM $(CONFIG_FILE) | \
           grep "CONFIG_SMP " | awk '{ print $$3 }')
  ifneq ($(SMP),1)
    SMP := 0
  endif
else
  SMP := $(shell grep 'CONFIG_SMP=y' $(CONFIG_FILE) > \
           /dev/null 2>&1 && echo 1 || echo 0)
endif

# compare SMP setting for running kernel and build area
ifneq ($(SMP),$(shell uname -a | grep SMP > /dev/null 2>&1 && echo 1 || echo 0))
  $(warning ***)
  ifeq ($(SMP),1)
    $(warning *** Warning: kernel source configuration (SMP))
    $(warning *** does not match running kernel (UP))
  else
    $(warning *** Warning: kernel source configuration (UP))
    $(warning *** does not match running kernel (SMP))
  endif
  $(warning *** Continuing with build,)
  $(warning *** resulting driver may not be what you want)
  $(warning ***)
endif

ifeq ($(SMP),1)
  EXTRA_CFLAGS += -D__SMP__
endif


###########################################################################
# 2.4.x & 2.6.x Specific rules

K_VERSION:=$(shell uname -r | cut -c1-3 | sed 's/2\.[56]/2\.6/')

#ifeq ($(K_VERSION), 2.6)

# Makefile for 2.6.x kernel
TARGET = dcmi.ko

ifneq ($(PATCHLEVEL),)
EXTRA_CFLAGS += $(CFLAGS_EXTRA)
obj-m += dcmi.o
dcmi-objs := $(CFILES:.c=.o)
else
default: 
ifeq ($(KOBJ),$(KSRC))
	make -C $(KSRC) M=$(shell pwd) modules
else
	make -C $(KSRC) O=$(KOBJ) M=$(shell pwd) modules
endif
endif

#endif # ifeq ($(K_VERSION),2.6)

install: default
	# remove all old versions of the driver
	find $(INSTALL_MOD_PATH)/lib/modules/$(KVER) -name $(TARGET) -exec rm -f {} \; || true
	find $(INSTALL_MOD_PATH)/lib/modules/$(KVER) -name $(TARGET).gz -exec rm -f {} \; || true
	if [ -e /etc/init.d/dcmi ] ; then \
	    rm -f /etc/init.d/dcmi ; \
	fi
	install -D -m 644 $(TARGET) $(INSTALL_MOD_PATH)$(INSTDIR)/$(TARGET)
ifeq (,$(INSTALL_MOD_PATH))
	/sbin/depmod -a || true
else
  ifeq ($(DEPVER),1 )
	/sbin/depmod -r $(INSTALL_MOD_PATH) -a || true
  else
	/sbin/depmod -b $(INSTALL_MOD_PATH) -a -n > /dev/null || true
  endif
endif

.PHONY: clean install uninstall test

clean:
	rm -rf $(TARGET) $(TARGET:.ko=.o) $(TARGET:.ko=.mod.o) $(TARGET:.ko=.mod.c) $(CFILES:.c=.o) .*cmd .tmp_versions Module.symvers Modules.symvers 
test:
	@echo $(KVER)

uninstall:
	#remove dcmi.ko
	if [ -e $(INSTDIR)/$(TARGET) ]; then \
	    rm -f ${INSTDIR}/$(TARGET) ; \
	fi
