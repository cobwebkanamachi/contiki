###############################################################################
#
# MODULE:   config_ba2.mk
#
# DESCRIPTION: Compiler definitions for the BA2 architecture
# This file should only contain architecture specific options.
###############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139]. 
# You, and any third parties must reproduce the copyright and warranty notice
# and any other legend of ownership on each copy or partial copy of the 
# software.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright NXP B.V. 2012. All rights reserved
#
###############################################################################

# Set toolchain path to use ba2 version of compiler
TOOLCHAIN_PATH = ba-elf-ba2
CROSS_COMPILE  = ba-elf

# BA2 archtecture compiler flags
CFLAGS += -msibcall -mno-multi -mno-setcc
CFLAGS += -mno-carry -mno-subb -mno-sext
CFLAGS += -mno-hard-div -mhard-mul -mbranch-cost=3 -msimple-mul
CFLAGS += -march=ba2 -mredzone-size=4
CFLAGS +=  -ffixed-r16 -ffixed-r17 -ffixed-r18 -ffixed-r19
CFLAGS +=  -ffixed-r20 -ffixed-r21 -ffixed-r22 -ffixed-r23
CFLAGS +=  -ffixed-r24 -ffixed-r25 -ffixed-r26 -ffixed-r27
CFLAGS +=  -ffixed-r28 -ffixed-r29 -ffixed-r30 -ffixed-r31

# omit frame pointer by default
CFLAGS += -fomit-frame-pointer

# default to optimise for size
CFLAGS += -Os

# Output each function and static data in their own sections
CFLAGS += -fdata-sections -ffunction-sections

# Debug Support

# Define seperate debug for SW and HW debug
ifeq ($(DEBUG), SW)
CFLAGS  := $(subst -Os,,$(CFLAGS))
CFLAGS  += -g -O0 -DGDB
CFLAGS  += -DSWDEBUG

BIN_SUFFIX ?= _swdbg
$(info Building SW debug version ...)
endif


ifeq ($(DEBUG), HW)
CFLAGS  := $(subst -Os,,$(CFLAGS))
CFLAGS  += -g -O0 -DGDB
HARDWARE_DEBUG_ENABLED=1
endif

ifeq ($(DEBUG), HW_SIZEOPT)
CFLAGS  += -g  -DGDB
$(info Size optimisation enabled with HW debug ...)
HARDWARE_DEBUG_ENABLED=1
endif

ifeq ($(HARDWARE_DEBUG_ENABLED), 1)
# Set DEBUG_PORT to UART0 or UART1 dependant on connection to serial port on board
CFLAGS += -D$(DEBUG_PORT)_DEBUG
CFLAGS  += -DHWDEBUG
BIN_SUFFIX ?= _hwdbg
$(info Building HW debug version ...)
endif


###############################################################################
