/*****************************************************************************
 *
 * MODULE:             ZPS_ROM.ld
 *
 * DESCRIPTION:        Linker command file for ZBPro component patches
 *                     
 *****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd. 2009 All rights reserved
 *
 ****************************************************************************/

MEMORY
{
        rom    : ORIGIN = 0x00000000, LENGTH = 0x20000
}


INCLUDE libOSPatch_JN5148.ld
INCLUDE libPDUMPatch_JN5148.ld
INCLUDE libPWRMPatch_JN5148.ld
INCLUDE libDBGPatch_JN5148.ld
INCLUDE libZPSMACPatch_JN5148.ld
INCLUDE libZPSAPLPatch_JN5148.ld
INCLUDE libZPSNWKPatch_JN5148.ld
