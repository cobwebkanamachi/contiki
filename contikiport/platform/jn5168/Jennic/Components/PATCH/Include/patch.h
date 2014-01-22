/*****************************************************************************
 *
 * MODULE:      PATCH
 *
 * DESCRIPTION: Patching support for modules
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

#ifndef PATCH_H_
#define PATCH_H_

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifdef PATCH_ENABLE
#define PATCH_POINT_PUBLIC_OVERLAYS(_type_, _fn_,_ovly_) PUBLIC _type_ rom_##_fn_
#define PATCH_POINT_MODULE_OVERLAYS(_type_, _fn_,_ovly_) MODULE _type_ rom_##_fn_
#define PATCH_POINT_PRIVATE_OVERLAYS(_type_, _fn_,_ovly_) PUBLIC _type_ rom_##_fn_
#define PATCH_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ rom_##_fn_
#define PATCH_POINT_MODULE(_type_, _fn_) MODULE _type_ rom_##_fn_
#define PATCH_POINT_PRIVATE(_type_, _fn_) PUBLIC _type_ rom_##_fn_
#define PATCH_INDIRECT_PRIVATE(_type_, _fn_) PUBLIC _type_ _fn_
#define NORMAL_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ _fn_
#define NORMAL_POINT_MODULE(_type_, _fn_) MODULE _type_ _fn_
#define NORMAL_POINT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#define NORMAL_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#else
#if !(defined JENNIC_CHIP_FAMILY_JN513x) && !(defined JENNIC_CHIP_FAMILY_JN514x)
#define PATCH_POINT_PUBLIC_OVERLAYS(_type_, _fn_, _ovly_) PUBLIC _type_ _fn_
#define PATCH_POINT_MODULE_OVERLAYS(_type_, _fn_, _ovly_) MODULE _type_ _fn_
#define PATCH_POINT_PRIVATE_OVERLAYS(_type_, _fn_, _ovly_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define PATCH_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ _fn_
#define PATCH_POINT_MODULE(_type_, _fn_) MODULE _type_ _fn_
#define PATCH_POINT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define PATCH_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define NORMAL_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ _fn_
#define NORMAL_POINT_MODULE(_type_, _fn_) MODULE _type_ _fn_
#define NORMAL_POINT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#define NORMAL_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#else /* Can use overlays on JN513x and JN514x */
#ifdef JENNIC_CHIP_JN5148
#define PATCH_POINT_PUBLIC_OVERLAYS(_type_, _fn_, _ovly_) PUBLIC _type_ OVERLAY_FUNCTION(_ovly_,_fn_)
#define PATCH_POINT_MODULE_OVERLAYS(_type_, _fn_, _ovly_) MODULE _type_ OVERLAY_FUNCTION(_ovly_,_fn_)
#define PATCH_POINT_PRIVATE_OVERLAYS(_type_, _fn_, _ovly_) PRIVATE _type_ OVERLAY_FUNCTION(_ovly_,_fn_)
#define PATCH_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ _fn_
#define PATCH_POINT_MODULE(_type_, _fn_) MODULE _type_ _fn_
#define PATCH_POINT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#define PATCH_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#define NORMAL_POINT_PUBLIC(_type_, _fn_) PUBLIC _type_ _fn_
#define NORMAL_POINT_MODULE(_type_, _fn_) MODULE _type_ _fn_
#define NORMAL_POINT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#define NORMAL_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE _type_ _fn_
#else
#define PATCH_POINT_PUBLIC_OVERLAYS(_type_, _fn_, _ovly_) PUBLIC _type_ OVERLAY_FUNCTION(_ovly,_fn_)
#define PATCH_POINT_MODULE_OVERLAYS(_type_, _fn_, _ovly_) MODULE _type_ OVERLAY_FUNCTION(_ovly_,_fn_)
#define PATCH_POINT_PRIVATE_OVERLAYS(_type_, _fn_, _ovly_) PRIVATE _type_ OVERLAY_FUNCTION(_ovly_,_fn_)
#define PATCH_POINT_PUBLIC(_type_, _fn_) PUBLIC __attribute__ ((noinline)) _type_ _fn_
#define PATCH_POINT_MODULE(_type_, _fn_) MODULE __attribute__ ((noinline)) _type_ _fn_
#define PATCH_POINT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define PATCH_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define NORMAL_POINT_PUBLIC(_type_, _fn_) PUBLIC __attribute__ ((noinline)) _type_ _fn_
#define NORMAL_POINT_MODULE(_type_, _fn_) MODULE __attribute__ ((noinline)) _type_ _fn_
#define NORMAL_POINT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#define NORMAL_INDIRECT_PRIVATE(_type_, _fn_) PRIVATE __attribute__ ((noinline)) _type_ _fn_
#endif
#endif
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Inlined Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /* DBG_PATCH_H_ */
