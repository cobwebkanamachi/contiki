/*
 *  linux/include/asm-or32/abi.h
 *
 *  or32 version
 *    author(s): Gyorgy Jeney (nog@bsemi.com)
 *
 *  For more information about BA processors, licensing and
 *  design services you may contact Beyond Semiconductor at
 *  sales@bsemi.com or visit website http://www.bsemi.com.
 *
 *  changes:
 *  01. 08. 2006: Gyorgy Jeney (nog@bsemi.com)
 *    initial abi definition file
 *
 */

#if __GNUC__ >= 4

# define REG_RETVAL_ASM		r3
# define REG_RETVAL_STR		"r2"
# define REG_RETVAL_PT		1
# define REG_RETVAL		3

# define REG_FP_ASM		r10
# define REG_FP_STR		"r10"
# define REG_FP_PT		8
# define REG_FP			10

#else

# define REG_RETVAL_ASM		r11
# define REG_RETVAL_STR		"r11"
# define REG_RETVAL_PT		9
# define REG_RETVAL		11

# define REG_FP_ASM		r2
# define REG_FP_STR		"r2"
# define REG_FP_PT		0
# define REG_FP			2

#endif
