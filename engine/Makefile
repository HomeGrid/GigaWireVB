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

ifneq ($(TOP_LEVEL_MAKEFILE),1)
	$(error Do not call this Makefile directly! Use ../Makefile instead)
endif



################################################################################
# Source code files
################################################################################

SEARCH_PATH  := src ../common

SRC          := $(shell find $(SEARCH_PATH) -name *.c)
OBJECTS      := $(addprefix bin/,$(notdir $(SRC:.c=.o)))

# Process dependency information
-include $(OBJECTS:%.o=%.d)



################################################################################
# Compiler independent flags
################################################################################

INCLUDES     += $(addprefix -I ,$(shell for x in `find $(SEARCH_PATH) -name "*.h"`; do dirname $$x; done | sort | uniq))
MACROS       += -D_USE_SYSLOG_ -D_USE_SYSLOG_ENABLED_BY_DEFAULT_
WARNINGS     += -Wall -Werror
SPECIAL      += -MD -MP

CFLAGS       := $(INCLUDES) $(WARNINGS) $(MACROS) $(SPECIAL)
LFLAGS       := -pthread -lrt -lm



################################################################################
# Makefile rules
################################################################################

vpath %.c $(shell for x in `find $(SEARCH_PATH) -name *.c`; do dirname $$x; done | sort | uniq)


.PHONY: all
all: bin/vector_boost

bin/vector_boost_engine: $(OBJECTS) vb_engine.ini 
	@printf ">COMPILE %-50s: " $@; echo "$(CC) $(CFLAGS) -o $@ $(OBJECTS) $(LFLAGS)"
	mkdir -p bin
	@$(CC) $(CFLAGS) $(CFLAGCOMPILER) -o $@ $(OBJECTS) $(LFLAGS)
	@cp vb_engine.ini ./bin

$(OBJECTS): bin/%.o : %.c
	mkdir -p bin
	$(CC) $(CFLAGS) $(CFLAGCOMPILER) -c $< -o $@  

.PHONY: clean
clean:
	@rm -rf bin

.PHONY: install
install:
ifdef INSTALL_PATH
	@mkdir -p $(INSTALL_PATH)
	@echo Copying bin/vectorboost_engine to $(INSTALL_PATH)
	cp bin/vector_boost_engine $(INSTALL_PATH)
	cp bin/*.ini $(INSTALL_PATH)
else
	@echo Error: INSTALL_PATH not defined
endif



################################################################################
# Static analysis
################################################################################

.PHONY: static-analysis
scan: clean
	rm -rf scan_report
	scan-build -o scan_report $(MAKE) all




