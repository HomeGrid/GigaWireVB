###############################################################################
#
#
#  <legal_notice>
#  * BSD License 2.0
#  *
#  * Copyright (c) 2021, MaxLinear, Inc.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions are met:
#  * 1. Redistributions of source code must retain the above copyright notice, 
#  *    this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright notice, 
#  *    this list of conditions and the following disclaimer in the documentation 
#  *    and/or other materials provided with the distribution.
#  * 3. Neither the name of the copyright holder nor the names of its contributors 
#  *    may be used to endorse or promote products derived from this software 
#  *    without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
#  * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  * OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
#  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT \(INCLUDING NEGLIGENCE OR OTHERWISE\) 
#  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#  * POSSIBILITY OF SUCH DAMAGE.
#  </legal_notice>
#
#
###############################################################################

MAKE         := make

################################################################################
# Compiler configuration
################################################################################
INCLUDES      :=
WARNINGS      :=
SPECIAL       :=
TARGET_LIST   := x86 MIPS ARMV7b ARM64 ARMV7
MACROS        := -D_X86_=0 -D_MIPS_=1 -D_ARMV7_=2 -D_ARMV7b_=3 -D_ARM64_=4 -D_WITH_SYSLOG_=1

ifeq ($(COMPILER),ARMV7)
  ifeq ($(ARMV7_CROSS), )
    $(error No compiler variable path defined)
  endif
  ifeq ($(ARMV7_SYSROOT), )
    $(error No compiler include path defined)
  endif
  CC            := $(ARMV7_CROSS)gcc
  AR            := $(ARMV7_CROSS)ar
  INCLUDES      += -I $(ARMV7_SYSROOT)/include
  MACROS        += -D_CONFIG_LOG_=1 -D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__ -D_CONFIG_TARGET_=_ARMV7_
  SPECIAL       += -static 
#  SPECIAL       += -O2 -fno-strict-aliasing  
  SPECIAL       += -O0 -g -fno-strict-aliasing  
else ifeq ($(COMPILER),ARMV7b)
  ifeq ($(ARMV7b_CROSS), )
    $(error No compiler variable path defined)
  endif
  ifeq ($(ARMV7b_SYSROOT), )
    $(error No compiler include path defined)
  endif
  CC            :=  $(ARMV7b_CROSS)gcc
  AR            :=  $(ARMV7b_CROSS)ar
  INCLUDES      += -I $(ARMV7b_SYSROOT)/include
  MACROS        += -D_CONFIG_LOG_=1 -D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__ -D_CONFIG_TARGET_=_ARMV7b_
  SPECIAL       += -static 
  SPECIAL       += -O2 -fno-strict-aliasing  
#  SPECIAL       += -O0 -g -fno-strict-aliasing  
else ifeq ($(COMPILER),ARM64)
  ifeq ($(ARM64_CROSS), )
    $(error No compiler variable path defined)
  endif
  ifeq ($(ARM64_SYSROOT), )
    $(error No compiler include path defined)
  endif
  CC            :=  $(ARM64_CROSS)gcc
  AR            :=  $(ARM64_CROSS)ar
  INCLUDES      += -I $(ARM64_SYSROOT)/include
  MACROS        += -D_CONFIG_LOG_=1 -D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__ -D_CONFIG_TARGET_=_ARM64_
  SPECIAL       += -static 
  SPECIAL       += -O2 -fno-strict-aliasing  
#  SPECIAL       += -O0 -g -fno-strict-aliasing    
else ifeq ($(COMPILER),MIPS)
  ifeq ($(MIPS_CROSS), )
    $(error No compiler variable path defined)
  endif
  ifeq ($(MIPS_SYSROOT), )
    $(error No compiler include path defined)
  endif
  CC            := $(MIPS_CROSS)gcc
  AR            := $(MIPS_CROSS)ar
  INCLUDES      += -I $(MIPS_SYSROOT)/include
  MACROS        += -D_REENTRANT -D__BYTE_ORDER__=__ORDER_BIG_ENDIAN__ -D_CONFIG_TARGET_=_MIPS_ -D_CONFIG_LOG_=1
  WARNINGS      += -Wno-pointer-sign
  SPECIAL       += -O2 -fno-strict-aliasing  -std=gnu99 -static    
# SPECIAL       += -O0 -fno-strict-aliasing  -std=gnu99 -static -g   
else ifeq ($(COMPILER),x86)
  CC            := gcc
  AR            := ar
  #CC            := clang
  MACROS        += -D_CONFIG_LOG_=1 -D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__ -D_CONFIG_TARGET_=_X86_
  #SPECIAL       += -m32
  
  ifeq ($(VECTORBOOST_VALGRIND),yes)
    SPECIAL       += -O0 -g -fno-strict-aliasing
  else
    SPECIAL       += -static 
    ifeq ($(VECTORBOOST_DEBUG),yes)
      SPECIAL     += -O0 -g -fno-strict-aliasing
    else
      SPECIAL     += -O2 -fno-strict-aliasing
    endif
  endif
    
  SYS_INCLUDES  := $(addprefix -I ,$(shell echo | $(CC) -Wp,-v -x c - -fsyntax-only 2>&1 | grep -e "^ "))
endif

ifeq ($(VECTORBOOST_MALLOC_DEBUG),yes)
  #MACROS        += -D_MEMORY_DEBUG_=1 -D_USE_MALLOC_MUTEX_=1 -D_VALGRIND_=0
  #SPECIAL       += -Wl,--wrap,malloc,--wrap,calloc,--wrap,free,--wrap,realloc,--wrap,strdup,--wrap,strndup
  MACROS        += -D_MEMORY_DEBUG_=1 -D_USE_MALLOC_MUTEX_=0 -D_VALGRIND_=0
else ifeq ($(VECTORBOOST_VALGRIND),yes)
  MACROS        += -D_MEMORY_DEBUG_=0 -D_USE_MALLOC_MUTEX_=0 -D_VALGRIND_=1
else
  MACROS        += -D_MEMORY_DEBUG_=0 -D_USE_MALLOC_MUTEX_=0 -D_VALGRIND_=0
endif

# Export all the just defined variables
TOP_LEVEL_MAKEFILE := 1
export



################################################################################
# Makefile rules
################################################################################

all: vector_boost_driver vector_boost_engine

.PHONY: vector_boost_driver
vector_boost_driver:
ifeq ($(filter $(COMPILER), $(TARGET_LIST)),)
	$(error No compiler defined)
endif
	@echo ""
	@echo ">> Compiling vector boost driver..."
	$(MAKE) -C driver bin/vector_boost_driver
	@echo ""
	@echo ">> Done! vector boost driver generated"


.PHONY: vector_boost_engine
vector_boost_engine:
ifeq ($(filter $(COMPILER), $(TARGET_LIST)),)
	$(error No compiler defined)
endif
	@echo ""
	@echo ">> Compiling vector boost engine..."
	$(MAKE) -C engine bin/vector_boost_engine
	@echo ""
	@echo ">> Done! vector boost engine generated"

.PHONY: distclean
distclean: clean
	@rm -rf release


.PHONY: clean
clean:
	@$(MAKE) -C driver clean
	@$(MAKE) -C engine clean
	@$(MAKE) -C common/ezxml clean


.PHONY: install
install:
	@$(MAKE) -C driver install
	@$(MAKE) -C engine install


.PHONY: static-analysis
scan:
	@$(MAKE) -C driver static-analysis
	@$(MAKE) -C engine static-analysis


.PHONY: release
release:
	rm -rf release
	$(MAKE) clean
	mkdir release
	@echo ""
	@echo ">> Preparing X86 release package..."
	COMPILER=x86 $(MAKE)  all
	COMPILER=x86 $(MAKE) -C driver INSTALL_PATH=`pwd`/release/x86 install
	COMPILER=x86 $(MAKE) -C engine INSTALL_PATH=`pwd`/release/x86 install
	COMPILER=x86 $(MAKE) -C driver clean
	COMPILER=x86 $(MAKE) -C engine clean
	@echo ""
	@echo ">> Preparing ARM64 release package..."
	COMPILER=ARM64 $(MAKE)  all
	COMPILER=ARM64 $(MAKE) -C driver INSTALL_PATH=`pwd`/release/arm64 install
	COMPILER=ARM64 $(MAKE) -C engine INSTALL_PATH=`pwd`/release/arm64 install
	COMPILER=ARM64 $(MAKE) -C driver clean
	COMPILER=ARM64 $(MAKE) -C engine clean
	#@echo ""
	#@echo ">> Preparing MIPS release package..."
	#COMPILER=MIPS $(MAKE) all
	#COMPILER=MIPS $(MAKE) -C driver INSTALL_PATH=`pwd`/release/mips install
	#COMPILER=MIPS $(MAKE) -C engine INSTALL_PATH=`pwd`/release/mips install
	#COMPILER=MIPS $(MAKE) -C driver clean
	#COMPILER=MIPS $(MAKE) -C engine clean
	#@echo ""
	#@echo ">> Preparing ARMv7 release package..."
	#COMPILER=ARMV7 $(MAKE) all
	#COMPILER=ARMV7 $(MAKE) -C driver INSTALL_PATH=`pwd`/release/armv7 install
	#COMPILER=ARMV7 $(MAKE) -C engine INSTALL_PATH=`pwd`/release/armv7 install
	#COMPILER=ARMV7 $(MAKE) -C driver clean
	#COMPILER=ARMV7 $(MAKE) -C engine clean
	@echo ""
	@echo ">> Preparing ARMV7b release package..."
	COMPILER=ARMV7b $(MAKE) all
	COMPILER=ARMV7b $(MAKE) -C driver INSTALL_PATH=`pwd`/release/armv7b install
	COMPILER=ARMV7b $(MAKE) -C engine INSTALL_PATH=`pwd`/release/armv7b install
	COMPILER=ARMV7b $(MAKE) -C driver clean
	COMPILER=ARMV7b $(MAKE) -C engine clean
	@echo ""
	@echo ">> Preparing src release package..."
	mkdir release/vectorBoost_src
	cp -r * release/vectorBoost_src || true;
	cd release; tar czvf vectorBoost_src.tgz vectorBoost_src > /dev/null
	cd release; rm -rf vectorBoost_src 
	@echo ""
	@echo ">> Done! Binaries and source code installed in 'release' folder"

ALL_INDENT_FILES = $(shell find . -name "*.[ch]")

.PHONY: cppcheck
cppcheck:	
ifeq ($(FOLDER),)
	@echo "Processing all files..."
	@rm -f `find . -name *.dump`;
	cppcheck --dump --enable=warning,style,performance,portability --inconclusive --inline-suppr --suppress=memleakOnRealloc:* $(ALL_INDENT_FILES) 1>/dev/null
	@rm -f `find . -name *.dump`;
else
	@echo "Processing files in folder $(FOLDER)..."
	@rm -f `find . -name *.dump`;
	cppcheck --dump --enable=warning,style,performance,portability --inconclusive --inline-suppr --suppress=memleakOnRealloc:* $(FOLDER) 1>/dev/null
	@rm -f `find . -name *.dump`;
endif

