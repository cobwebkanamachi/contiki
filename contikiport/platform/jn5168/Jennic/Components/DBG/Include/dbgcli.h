/*****************************************************************************
 *
 * MODULE:             Debug Cli
 *
 * DESCRIPTION:        Legacy support for dbgcli interface
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

#ifndef DEBUGCLI_H
#define DEBUGCLI_H

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "jendefs.h"
#define DEBUG_LEVEL 4

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* This should be defined if you wish to have debug print and print log in the
 * debug cli build undefining it will make the DBG_PRINT and PRINT_LOG macros
 * benign
 */

#ifdef DEBUGPRINT

/* GASHHH : Nasty - needs to be properly sorted out */

/****************************************************************/

#ifdef LINUX

#ifdef OUTPUT_VERBOSE
extern void pause_a_bit(void);
#define PAUSE_A_BIT pause_a_bit()
#else
#define PAUSE_A_BIT
#endif
#define DEBUG_PRINT_BASE(args)          (printk args)
#define PRINT_LOG(u8LogLevel, args)

#endif /* LINUX */

/****************************************************************/

#ifdef ECOS

#define PAUSE_A_BIT

/* Yeuch - really cheesy */
#define DEBUG_PRINT_BASE(args)          { extern bool_t g_bInISR; if (!g_bInISR) vDBG_DebugPrint args; }
#define PRINT_LOG(u8LogLevel, args)

#endif /* ECOS */

/****************************************************************/

#if defined(_WIN32) || defined(WIN32)

#include "dbg.h"

#undef DBG_vPrintf
#ifdef DBG_VERBOSE
#define DBG_vPrintf(FORMAT, ARGS...)                                \
    do {                                                            \
        const char *pccFormat = NULL;                               \
        if(1) { pccFormat = (FORMAT); }                             \
        else       { (void)(FORMAT); }                              \
        dbg_vPrintfImpl(__FILE__, __LINE__, pccFormat, ## ARGS);    \
    } while(0)
#else
#define DBG_vPrintf(FORMAT, ARGS...)                                \
    do {                                                            \
        const char *pccFormat = NULL;                               \
        if(1) { pccFormat = (FORMAT); }                             \
        else       { (void)(FORMAT); }                              \
        dbg_vPrintfImpl("",0, pccFormat, ## ARGS);                  \
    } while(0)
#endif

#define PAUSE_A_BIT
#define DEBUG_PRINT_BASE(args)          DBG_vPrintf args
//#define DEBUG_PRINT_BASE(args)
#define PRINT_LOG(u8LogLevel, args)

#endif /* defined(_WIN32) || defined(WIN32) */

/****************************************************************/

#ifdef _VXWORKS

#define PAUSE_A_BIT
#define DEBUG_PRINT_BASE(args)          printf##args
#define PRINT_LOG(u8LogLevel, args)

#endif /* _VXWORKS */

/****************************************************************/

#ifdef M68K

#define PAUSE_A_BIT
#define DEBUG_PRINT_BASE(args)          printf##args; \
                                        fflush(stdout)
#define PRINT_LOG(u8LogLevel, args)

#endif /* M68K */

/****************************************************************/

#else /* !DEBUGPRINT */

#define PAUSE_A_BIT
#define DEBUG_PRINT_BASE(args)
#define PRINT_LOG(u8LogLevel, args)

#endif /* !DEBUGPRINT */

/* This should be defined if you wish to have CLI print in the debug cli build
 * undefining it will make the CLI_PRINT macro benign
 */

#ifdef CLIPRINT
#define CLI_PRINT(args)                 (vDBG_CLIprint args)
#else
#define CLI_PRINT(args)
#endif

/* This should be defined if you wish to have User print in the debug cli build
 * undefining it will make the USER_PRINT macro benign
 */

#ifdef USERPRINT
#define USER_PRINT(args)                (vDBG_UserPrint args)
#else
#define USER_PRINT(args)
#endif

/* TRACE MACRO */
/* This should be defined if you wish to have Trace in the debug cli build
 * undefining it will make the TRACE macro benign
 */

#ifdef USERTRACE
#define TRACE(args)             (vDBG_Trace args)
#else
//#define TRACE(args)
#endif

/* Debug CLI version of assert */

/* #undef assert */
#ifndef DEBUGASSERT
#define DBG_ASSERT(cond)
#else
#define DBG_ASSERT(cond) \
   ((cond)           \
     ? (void)  0     \
     : vDBG_assert(#cond,__FILE__,__LINE__))
#endif


#if DEBUG_LEVEL == 1
/* No info or warning or error */
#define DEBUG_PRINT_FATAL(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_ERROR(args)
#define DEBUG_PRINT_WARN(args)
#define DEBUG_PRINT_INFO(args)
#define DEBUG_PRINT(args)
#elif DEBUG_LEVEL == 2
/* No info or warning */
#define DEBUG_PRINT_FATAL(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_ERROR(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_WARN(args)
#define DEBUG_PRINT_INFO(args)
#define DEBUG_PRINT(args)
#elif DEBUG_LEVEL == 3
/* No info */
#define DEBUG_PRINT_FATAL(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_ERROR(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_WARN(args)      DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_INFO(args)
#define DEBUG_PRINT(args)
#elif DEBUG_LEVEL == 4
/* All four levels */
#define DEBUG_PRINT_FATAL(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_ERROR(args)     DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_WARN(args)      DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT_INFO(args)      DEBUG_PRINT_BASE(args)
#define DEBUG_PRINT(args)           DEBUG_PRINT_BASE(args)
#else
/* All off by default */
#define DEBUG_PRINT_FATAL(args)
#define DEBUG_PRINT_ERROR(args)
#define DEBUG_PRINT_WARN(args)
#define DEBUG_PRINT_INFO(args)
#define DEBUG_PRINT(args)
#endif

/* Additional debug print macros
   _V is for verbose, only printed when OUTPUT_VERBOSE is defined
   _P causes the PAUSE_A_BIT macro to be appended                 */
#ifdef OUTPUT_VERBOSE
#define DEBUG_PRINT_V(args)         DEBUG_PRINT_INFO(args)
#define DEBUG_PRINT_ERROR_V_P(args) DEBUG_PRINT_ERROR(args); \
                                    PAUSE_A_BIT
#define DEBUG_PRINT_V_P(args)       DEBUG_PRINT_V(args); \
                                    PAUSE_A_BIT
#else
#define DEBUG_PRINT_V(args)
#define DEBUG_PRINT_ERROR_V_P(args)
#define DEBUG_PRINT_V_P(args)
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef long ( * lpfDBG_printHandlerFunction )(char *dbgString);
typedef void ( * vpfCLI_cmdHandlerFunction )(int iArgc, char *pArgv[]);
typedef long ( * lpfDBG_inputCharHandlerFunction )(char *dbgString);

typedef long ( * lpfDBG_traceHandlerFunction )(char *dbgString);

/****************************************************************************/
/***                        CLI Table Structure                           ***/
/****************************************************************************/

typedef struct CLI_TABLE_ENTRY_TAG {
    char    *psCommandName;
    vpfCLI_cmdHandlerFunction vpfCLIhandlerFunction;
    int     iArgc;
    char    *psCommandDescription;
    char    *psCommandParameters;
} CLI_TABLE_ENTRY;

typedef enum {
    DP_SUCCESS = 0,
    DP_FAILURE,
    DP_IGNORED,
    /*this must always be last */
    NUMBER_OF_DP_RETURN_CODES
} DP_STATUS;

/****************************************************************************/
/***        User Structure used to Configure the Debug CLI module         ***/
/****************************************************************************/

typedef struct debugcliConfigTag {
    /* debug config */
    lpfDBG_printHandlerFunction lpfDebugPrintFunction;
    lpfDBG_printHandlerFunction lpfCLIPrintFunction;
    lpfDBG_printHandlerFunction lpfUserPrintFunction;
    lpfDBG_printHandlerFunction lpfEmergencyPrintFunction;
    uint32 u32PrintBufferSize;
    uint32 u32NumberOfPrintBuffers;
    uint8 u8MasterPrintLevel;
    /* cli config */
    lpfDBG_inputCharHandlerFunction lpfCharHandlerFunction;
    /* cli table dimensions */
    uint32 u32commandNameSize;
    uint32 u32commandDescSize;
    uint32 u32commandParamListSize;
    /* trace buffer config */
    lpfDBG_traceHandlerFunction lpfTraceHandlerFunction;
    uint32 u32TraceBufferSize;
    uint32 u32NumberOfTraceBuffers;
    /* task priorities */
    uint32 u32DebugTaskPriority;
    uint32 u32CliTaskPriority;
    uint32 u32TraceTaskPriority;
} tDebugcliConfig;

/****************************************************************************/
/***           User Structure used to Configure the Debug module          ***/
/****************************************************************************/

typedef struct debugConfigTag {
    /* debug config */
    lpfDBG_printHandlerFunction lpfDebugPrintFunction;
    lpfDBG_printHandlerFunction lpfCLIPrintFunction;
    lpfDBG_printHandlerFunction lpfUserPrintFunction;
    lpfDBG_printHandlerFunction lpfEmergencyPrintFunction;
    uint32 u32PrintBufferSize;
    uint32 u32NumberOfPrintBuffers;
    uint8 u8MasterPrintLevel;
    uint32 u32DebugTaskPriority;
} tDebugConfig;

/****************************************************************************/
/***           User Structure used to Configure the Trace module          ***/
/****************************************************************************/

typedef struct traceConfigTag {
    /* trace config */
    lpfDBG_traceHandlerFunction lpfTraceHandlerFunction;
    lpfDBG_printHandlerFunction lpfEmergencyPrintFunction;
    uint32 u32TraceBufferSize;
    uint32 u32NumberOfTraceBuffers;
    uint32 u32TraceTaskPriority;
} tTraceConfig;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

PUBLIC void vDBG_CreateDebugCliManager(tDebugcliConfig *ptUserDebugcliConfig);
PUBLIC void vDBG_DeleteDebugCliManager(void);

/****************************************************************************/
/***        Debug (Print) Manager                                         ***/
/****************************************************************************/

/* the debug module can be created and used as a standalone module PROVIDING
 * the CLI module is not being used. If the CLI module is being used, this
 * module is created (and deleted) by the CLI.
 */

PUBLIC void vDBG_CreateDebugManager(tDebugConfig *ptDebugConfig);

PUBLIC void vDBG_DeleteDebugManager(void);

/****************************************************************************/
/***        Platform Print Functions                                      ***/
/****************************************************************************/

PUBLIC void vDBG_DebugPrint(const char *fmt, ...);
PUBLIC void vDBG_CLIprint(const char *fmt, ...);
PUBLIC void vDBG_UserPrint(const char *fmt, ...);
PUBLIC void vDBG_PrintLog(uint8 u8LogLevel, const char *fmt, ...);

PUBLIC void vCLI_handleUserCLIcommand(char * psCommandString);

PUBLIC void vDBG_SimpleDebugFatal(int iDebugFatalParameter);

/****************************************************************************/
/***        CLI Table Registration Functions                              ***/
/****************************************************************************/

PUBLIC bool boCLI_RegisterCLItable(const char * const sTableLabel,
                                   const char * const sTableDecscription,
                                   CLI_TABLE_ENTRY *paCLIcommandTable,
                                   uint32 u32CLItableEntries,
                                   uint32 *pu32TableId);

PUBLIC bool boCLI_RegisterCLIsubTable(const char * const sTableLabel,
                                      CLI_TABLE_ENTRY *paCLIcommandTable,
                                      uint32 u32CLItableEntries,
                                      uint32 u32ParentTableId,
                                      uint32 *pu32TableId);

PUBLIC bool boCLI_setWorkingTable(uint32 u32workingTableId);

/****************************************************************************/
/***                        Trace Buffer Functions                        ***/
/****************************************************************************/

/* the debug module can be created and used as a standalone module PROVIDING
 * the CLI module is not being used. If the CLI module is being used, this
 * module is created (and deleted) by the CLI. The advantage of using Trace
 * function via the CLI interface being, it allows the user to access all
 * trace functions dynamically.
 */

PUBLIC void vDBG_CreateTraceManager(tTraceConfig *ptTraceConfig);

PUBLIC void vDBG_DeleteTraceManager(void);

PUBLIC void vDBG_Trace(const char *fmt, ...);

PUBLIC void vDBG_ResetTraceBuffer(void);

PUBLIC void vDBG_ExamineTraceBuffer(void);

PUBLIC void vDBG_TraceEnable(void);

PUBLIC void vDBG_TraceDisable(void);

/****************************************************************************/
/***                            Michellaneous                             ***/
/****************************************************************************/

void vDBG_assert(char *cond, char *fname, uint32 u32Lineno);

#ifdef __cplusplus
}
#endif
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif  /* _DEBUGCLI_H */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

/* End of file $RCSfile: dbgcli.h,v $ *******************************************/

/****************************************************************************
 *
 * CHANGE HISTORY:
 *
 * $Log: dbgcli.h,v $
 * Revision 1.1.1.1  2005/07/01 09:30:44  rcc
 * Initial release
 *
 * Revision 1.3  2004/05/21 14:55:24  cjg
 * Added macros for m68k build
 *
 * Revision 1.2  2003/10/09 07:41:56  cjg
 * Added simple console print facility
 *
 * Revision 1.1.1.1  2003/06/20 10:55:40  rcc
 * initial checkin
 *
 *
 ****************************************************************************/
