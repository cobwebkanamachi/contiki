/****************************************************************************
 *
 * MODULE:             JIP: MIB definition macros
 *
 * DESCRIPTION:
 * This file redefines several macros while repeatedly including the header
 * file given by MIB_HEADER. This allows it to construct several structures
 * for the MIB and an initialiser from one common definition.
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139]. 
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the 
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "jip_data.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifndef MIB_HEADER
#error MIB_HEADER must be defined before including jip_define_mib.h
#else

/****************************************************************************\
 * Define some shorter versions of the JIP enumeration names to allow more
 * readable MIB definitions.
\****************************************************************************/

#define TRAP        E_JIP_ACCESS_TYPE_TRAP
#define READ        E_JIP_ACCESS_TYPE_READ
#define WRITE       E_JIP_ACCESS_TYPE_WRITE
#define READ_WRITE  E_JIP_ACCESS_TYPE_READ | E_JIP_ACCESS_TYPE_WRITE

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************\
 * Only do anything if START_DEFINE_MIB isn't defined. This avoids an include
 * loop when MIB_HEADER includes this file.
\****************************************************************************/

#ifndef DEFINE_MIB_INCLUDED
#define DEFINE_MIB_INCLUDED

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * Define structure with elements named after IDs. This will cause a compile
 * error if more than one variable has the same ID.
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) typedef struct {
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)  int  ID_##ID;
#define END_DEFINE_MIB(NAME) } _ts##NAME##Mib_TestIDs;

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * Define the MIB definition structure.
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) typedef struct { tsJIP_MibDef sMib;
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)  tsJIP_VarDef  sVar##NAME;
#define END_DEFINE_MIB(NAME) } ts##NAME##Mib_Def;

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * Define the MIB instance structure.
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) typedef struct { tsJIP_MibInst sMib;
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)  tsJIP_##TYPE  sVar##NAME;
#define END_DEFINE_MIB(NAME) } PACK ts##NAME##Mib_Inst;

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * If DECLARE_MIB, declare and initialise the MIB definition.
\****************************************************************************/
#ifdef DECLARE_MIB

#define START_DEFINE_MIB(ID, NAME) static const ts##NAME##Mib_Def s##NAME##MibDef = { { ID, (sizeof(ts##NAME##Mib_Def) - sizeof(tsJIP_MibDef)) / sizeof(tsJIP_VarDef) },
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)  { (DISPLAY) ? DISPLAY : #NAME, ID, FLAGS, E_JIP_VAR_TYPE_##TYPE, ACCESS, JIP_CACHE_##CACHE, E_JIP_SECURITY_##SECURITY },
#define END_DEFINE_MIB(NAME) };

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

#else

/****************************************************************************\
 * Otherwise declare the MIB definition as external.
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) extern ts##NAME##Mib_Def s##NAME##MibDef;
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)
#define END_DEFINE_MIB(NAME)

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

#endif

/****************************************************************************\
 * Declare handles for the variables
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) typedef struct {
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY) uint32 h##NAME;
#define END_DEFINE_MIB(NAME) } th##NAME##_Handles;

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * Declare handles for the variables
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME) static const th##NAME##_Handles s##NAME##Handles = {
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY) ID,
#define END_DEFINE_MIB(NAME) };

#include MIB_HEADER

#undef START_DEFINE_MIB
#undef DEFINE_VAR
#undef END_DEFINE_MIB

/****************************************************************************\
 * Finally, define the macros as blank to avoid any further effects.
\****************************************************************************/
#define START_DEFINE_MIB(ID, NAME)
#define DEFINE_VAR(ID, TYPE, NAME, DISPLAY, FLAGS, ACCESS, CACHE, SECURITY)
#define END_DEFINE_MIB(NAME)

#undef MIB_HEADER
#undef DEFINE_MIB_INCLUDED
#endif
#endif

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

