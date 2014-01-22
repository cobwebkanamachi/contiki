##############################################################################
#
# MODULE:   config_JN516x.mk
#
# DESCRIPTION:Configuration make include file for JN516x
#
############################################################################
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
############################################################################

# Ensure chip and family are set
override JENNIC_CHIP = UNDEFINED
override JENNIC_CHIP_FAMILY = JN516x

# Define numeric value for chip type
CFLAGS += -DJN516x=5160
CFLAGS += -DJENNIC_CHIP_NAME=_JN516x

# Base dir for selected chip
CHIP_BASE_DIR = $(SDK_BASE_DIR)/Chip/$(JENNIC_CHIP_FAMILY)

# include definitions for the BA2 architecture
include $(SDK_BASE_DIR)/Chip/Common/Build/config_ba2.mk

# Chip specific C flags
# watchdog is enabled by default on chip so allow disable if reqd
CFLAGS += -DWATCHDOG_ENABLED

# Feature set:
#   JENNIC_HW_xx is silicon feature and so fixed
#   JENNIC_SW_xx is build option and can be changed
#
#   JENNIC_HW_BBC_RXINCCA:    BBC RX during CCA capability
#   JENNIC_HW_BBC_DMA:        BBC frame buffer DMA mechanism
#   JENNIC_HW_BBC_ISA:        BBC inline security engine 
#   JENNIC_SW_EXTERNAL_FLASH: whether external flash is supported
#   JN516X_DMA_UART_BACKWARDS_COMPATIBLE_API: API behaves like the older versions with no visible UART DMA 
#
CFLAGS += -DJENNIC_HW_BBC_RXINCCA=1
CFLAGS += -DJENNIC_HW_BBC_DMA=1
CFLAGS += -DJENNIC_HW_BBC_ISA=0
#CFLAGS += -DJENNIC_SW_EXTERNAL_FLASH=0
CFLAGS += -DJN516X_DMA_UART_BACKWARDS_COMPATIBLE_API=1

# no link stage for device family
override LDFLAGS = UNDEFINED

