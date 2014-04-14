###############################################################################
#
# MODULE:   extra_flags.mk
#
# DESCRIPTION: Build configuration for JN514x family device software.
#
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

# All Flags cause conditional build of software
MODCFLAGS       += -DSTANDALONE -DEMBEDDED -DLEAN_N_MEAN -DRND_256_ONLY
MODCFLAGS       += -DCHIP_RELEASE_514xT01
MODCFLAGS       += -DUART0
#MODCFLAGS      += -DCHIP_RELEASE_CACTI
#MODCFLAGS      += -DCHIP_RELEASE_LEOPARD
MODCFLAGS       += -DSINGLE_CONTEXT -DZED_BUILD
MODCFLAGS       += -DPHY_CALLBACK
MODCFLAGS       += -DPLME_SAP
MODCFLAGS       += -DUSER_VSR_HANDLER 
MODCFLAGS       += -DAHI_RELEASE
##MODCFLAGS       += -DROM_PATCH
MODCFLAGS       += -DOPTIMISE_REG_STRUCT
MODCFLAGS       += -DHOUSEKEEPING
MODCFLAGS       += -DHOST_PROCESSOR_BIG_ENDIAN


EXTRACFLAGS  += -DENABLE_OPT_IN_MAKE
#EXTRACFLAGS += -DENABLE_OPT_FFD
#EXTRACFLAGS += -DENABLE_OPT_MLME_GTS_COORD
#EXTRACFLAGS += -DENABLE_OPT_MLME_GTS_DEVICE
#EXTRACFLAGS += -DENABLE_OPT_SECURITY
EXTRACFLAGS += -DPIB_DIRECT_ACCESS

ifndef REDUCED_ZIGBEE_MAC_BUILD
ifndef DISABLE_MAC_REGULAR_BEACONS
EXTRACFLAGS += -DENABLE_OPT_REGULAR_BEACONS
EXTRACFLAGS += -DENABLE_LP_BEACON_TRACK
endif
endif
ifndef REDUCED_ZIGBEE_MAC_BUILD
EXTRACFLAGS += -DENABLE_OPT_SECURITY
EXTRACFLAGS += -DENABLE_OPT_MLME_ENABLE_DISASSOCIATE
endif
EXTRACFLAGS += -DENABLE_OPT_ORPHAN_SCAN
EXTRACFLAGS += -DENABLE_OPT_COORD
EXTRACFLAGS += -DENABLE_OPT_DEVICE
#EXTRACFLAGS += -DENABLE_OPT_MLME_GTS_COORD
#EXTRACFLAGS += -DENABLE_OPT_MLME_GTS_DEVICE
EXTRACFLAGS += -DENABLE_OPT_MCPS_PURGE
EXTRACFLAGS += -DENABLE_OPT_MLME_ASSOCIATE
EXTRACFLAGS += -DENABLE_OPT_PROMISCUOUS
EXTRACFLAGS += -DENABLE_OPT_ED_SCAN
EXTRACFLAGS += -DENABLE_OPT_ACTIVE_SCAN
EXTRACFLAGS += -DENABLE_OPT_RXENABLE
EXTRACFLAGS += -DBUILD_802_15_4B

#EXTRACFLAGS += -DNO_NESTED_INTERRUPTS
ifeq ($(BUILD), ROM)
EXTRACFLAGS += -DROM_BUILD
EXTRACFLAGS += -URAM_BUILD
endif
EXTRACFLAGS += -DENABLE_OPT_LICENCE_SUPPORT
EXTRACFLAGS += -DENABLE_TXCALLBACK
#EXTRACFLAGS += -DTOF_ENABLED
EXTRACFLAGS += -DCOMPILER_FAMILY_BA2
#EXTRACFLAGS += -DJENNET_HDR_CRC

EXTRACFLAGS += -DENABLE_OASWDL

#Set global debug. Effects all libraries
ifeq ($(DEBUG_GLOBAL), 1)
EXTRACFLAGS  += -DUART0_DEBUG
$(info Building debug version for all libs ...)
endif

ifeq ($(BUILD), ROM)
# This is being used as a quick fix to build the JN5148T01 ROM without
# individual function and data sections. They all get included anyway, 
# and we want to rename them once the component libraries are built so 
# we can manage them better (location in ROM, resetting of variable
# spaces)
EXTRACFLAGS += -fno-data-sections -fno-function-sections
endif

ifeq ($(JENNIC_CHIP), JN5161_HDK)
# Following is essential to ensure no switch jump tables are created, as they
# would be inline on ba2 and thus potentially in the unreadable ROM area of 
# the HDK build
EXTRACFLAGS += -fno-jump-tables
endif

ASFLAGS     = $(CFLAGS)
