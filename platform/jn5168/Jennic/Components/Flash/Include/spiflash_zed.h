/****************************************************************************
 *
 * MODULE:             SPI
 *
 * COMPONENT:          $RCSfile: spiflash_zed.h,v $
 *
 * VERSION:            $Name: JN5148_0035_ROM_CP0 $
 *
 * REVISION:           $Revision: 1.9 $
 *
 * DATED:              $Date: 2008/08/19 17:08:53 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             rcc
 *
 * DESCRIPTION:
 * SPI Flash routines for ZED device - header
 *
 * CHANGE HISTORY:     At end of file
 *
 * LAST MODIFIED BY:   $Author: dclar $
 *                     $Modtime$
 *
 *
 ****************************************************************************
 *
 *  (c) Copyright 2004, Jennic Limited
 *
 ****************************************************************************/

#ifndef  _spiflash_zed_h_
#define  _spiflash_zed_h_

#if defined __cplusplus
extern "C" {
#endif

#include <jendefs.h>
#include <AppHardwareApi.h>

/************************/
/**** MACROS/DEFINES ****/
/************************/

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/


#define WIP_CLEAR_WAIT 600000




/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

//PUBLIC tSPIflashFncTable SPIFlashFncTab_ST_M25P10A;
//PUBLIC tSPIflashFncTable SPIFlashFncTab_Atmel_AT25F512;
//PUBLIC tSPIflashFncTable SPIFlashFncTab_SST_25VF010;

extern tSPIflashFncTable *pSPIflashFncTable;

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

PUBLIC bool_t   bZSPIWaitWIPClear(void);
PUBLIC uint8    u8SelectFlashChipType(uint8 *pau8Incoming);
PUBLIC uint16   u16ZSPIflashRDID(void);
PUBLIC uint32   u32ZSPIflashRDJEDECID(void);

#if defined __cplusplus
}
#endif

#endif  /* _spiflash_zed_h_ */

/* End of file $RCSfile: spiflash_zed.h,v $ *******************************************/

/****************************************************************************
 *
 * CHANGE HISTORY:
 *
 * $Log: spiflash_zed.h,v $
 * Revision 1.9  2008/08/19 17:08:53  dclar
 * added read JEDEC ID Command. Added support for STM25P40. Lengthend timeout to support 4MB flash
 *
 * Revision 1.8  2007/11/29 14:22:41  dclar
 * changed WIP_CLEAR_WAIT define
 *
 * Revision 1.7  2007/01/30 18:01:42  dclar
 * moved declaration of tSPIflashFncTable to AppHardwareApi.h
 *
 * Revision 1.6  2006/08/04 11:01:25  jthom
 * Renamed u8ZSPIflashRDID(void) to u16ZSPIflashRDID(void)
 *
 * Revision 1.5  2006/08/01 09:51:09  jthom
 * Renamed u8SelectFlashChip to u8SelectFlashChipType to avoid confusion between setting the SPI chip select and selecting the type of flash chip connected to the SPI bus
 *
 * Revision 1.4  2006/07/21 13:52:14  jthom
 * Added support for flash chips: Atmel AT25F512, SST 25VF010
 *     Introduced tSPIflashFncTable - a table of falsh access function pointers enabling easy support for multiple flash chips
 *     Flash access should now be done via the pSPIflashFncTable pointer set to a set of functions matching the attached flash chip
 * Added CRA authentication: If flash has been secured (EFuse) the Flash programmer must login before the flash can be accessed
 *
 * Revision 1.3  2006/02/20 13:16:11  jthom
 * Added vZSPIflashEWRSR() and u16ZSPIflashRDID()
 *
 * Revision 1.2  2005/03/24 11:51:46  rcc
 * Removed WRDI, added SE
 *
 * Revision 1.1  2004/10/15 14:36:06  rcc
 * Bootstrap flash programmer
 *
 *
 ****************************************************************************/
