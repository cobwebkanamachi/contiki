/*
 * Automatically generated C config: don't edit
 */
#if !defined __FEATURES_H && !defined __need_uClibc_config_h
#error Never include <bits/uClibc_config.h> directly; use <features.h> instead
#endif

/*
 * Version Number
 */
#define __UCLIBC_MAJOR__ 0
#define __UCLIBC_MINOR__ 9
#define __UCLIBC_SUBLEVEL__ 28
#undef __TARGET_alpha__
#undef __TARGET_arm__
#undef __TARGET_bfin__
#undef __TARGET_cris__
#undef __TARGET_e1__
#undef __TARGET_frv__
#undef __TARGET_h8300__
#undef __TARGET_i386__
#undef __TARGET_i960__
#undef __TARGET_m68k__
#undef __TARGET_microblaze__
#undef __TARGET_mips__
#undef __TARGET_nios__
#undef __TARGET_nios2__
#define __TARGET_ba__ 1
#undef __TARGET_powerpc__
#undef __TARGET_sh__
#undef __TARGET_sh64__
#undef __TARGET_sparc__
#undef __TARGET_v850__
#undef __TARGET_x86_64__

/*
 * Target Architecture Features and Options
 */
#define __HAVE_ELF__ 1
#define __TARGET_ARCH__ "ba"
#define __ARCH_SUPPORTS_BIG_ENDIAN__ 1
#define __ARCH_HAS_C_SYMBOL_PREFIX__ 1
#define __CROSS__ "ba-elf-"
#define __CONFIG_OR32__ 1
#undef __ARCH_LITTLE_ENDIAN__
#define __ARCH_BIG_ENDIAN__ 1
#undef __ARCH_HAS_NO_MMU__
#define __ARCH_HAS_MMU__ 1
#define __UCLIBC_HAS_FLOATS__ 1
#undef __HAS_FPU__
#define __UCLIBC_HAS_SOFT_FLOAT__ 1
#undef __DO_C99_MATH__
#define __KERNEL_SOURCE__ "/home/bats/src/ba/linux-2.6.19-ba-r8352/"
#define __C_SYMBOL_PREFIX__ "_"
#define __HAVE_DOT_CONFIG__ 1

/*
 * General Library Settings
 */
#define __HAVE_NO_PIC__ 1
#define __HAVE_NO_SHARED__ 1
#define __ARCH_HAS_NO_LDSO__ 1
#undef __DL_FINI_CRT_COMPAT__
#undef __UCLIBC_CTOR_DTOR__
#undef __HAS_NO_THREADS__
#undef __UCLIBC_HAS_THREADS__
#define __UCLIBC_HAS_LFS__ 1
#define __UCLIBC_STATIC_LDCONFIG__ 1
#undef __MALLOC__
#undef __MALLOC_SIMPLE__
#define __MALLOC_STANDARD__ 1
#define __MALLOC_GLIBC_COMPAT__ 1
#undef __UCLIBC_DYNAMIC_ATEXIT__
#define __HAS_SHADOW__ 1
#define __UNIX98PTY_ONLY__ 1
#define __ASSUME_DEVPTS__ 1
#undef __UCLIBC_HAS_TM_EXTENSIONS__
#define __UCLIBC_HAS_TZ_CACHING__ 1
#define __UCLIBC_HAS_TZ_FILE__ 1
#define __UCLIBC_HAS_TZ_FILE_READ_MANY__ 1
#define __UCLIBC_TZ_FILE_PATH__ "/etc/TZ"

/*
 * Networking Support
 */
#undef __UCLIBC_HAS_IPV6__
#define __UCLIBC_HAS_RPC__ 1
#undef __UCLIBC_HAS_FULL_RPC__

/*
 * String and Stdio Support
 */
#define __UCLIBC_HAS_STRING_GENERIC_OPT__ 1
#define __UCLIBC_HAS_STRING_ARCH_OPT__ 1
#undef __UCLIBC_HAS_CTYPE_TABLES__
#undef __UCLIBC_HAS_WCHAR__
#undef __UCLIBC_HAS_LOCALE__
#define __USE_OLD_VFPRINTF__ 1
#undef __UCLIBC_HAS_SCANF_GLIBC_A_FLAG__
#undef __UCLIBC_HAS_STDIO_BUFSIZ_NONE__
#define __UCLIBC_HAS_STDIO_BUFSIZ_256__ 1
#undef __UCLIBC_HAS_STDIO_BUFSIZ_512__
#undef __UCLIBC_HAS_STDIO_BUFSIZ_1024__
#undef __UCLIBC_HAS_STDIO_BUFSIZ_2048__
#undef __UCLIBC_HAS_STDIO_BUFSIZ_4096__
#undef __UCLIBC_HAS_STDIO_BUFSIZ_8192__
#define __UCLIBC_HAS_STDIO_BUILTIN_BUFFER_NONE__ 1
#undef __UCLIBC_HAS_STDIO_BUILTIN_BUFFER_4__
#undef __UCLIBC_HAS_STDIO_BUILTIN_BUFFER_8__
#undef __UCLIBC_HAS_STDIO_SHUTDOWN_ON_ABORT__
#define __UCLIBC_HAS_STDIO_GETC_MACRO__ 1
#define __UCLIBC_HAS_STDIO_PUTC_MACRO__ 1
#define __UCLIBC_HAS_STDIO_AUTO_RW_TRANSITION__ 1
#undef __UCLIBC_HAS_FOPEN_LARGEFILE_MODE__
#define __UCLIBC_HAS_FOPEN_EXCLUSIVE_MODE__ 1
#define __UCLIBC_HAS_GLIBC_CUSTOM_STREAMS__ 1
#define __UCLIBC_HAS_PRINTF_M_SPEC__ 1
#define __UCLIBC_HAS_ERRNO_MESSAGES__ 1
#undef __UCLIBC_HAS_SYS_ERRLIST__
#define __UCLIBC_HAS_SIGNUM_MESSAGES__ 1
#define __UCLIBC_HAS_SYS_SIGLIST__ 1
#define __UCLIBC_HAS_GNU_GETOPT__ 1

/*
 * Big and Tall
 */
#define __UCLIBC_HAS_REGEX__ 1
#undef __UCLIBC_HAS_WORDEXP__
#undef __UCLIBC_HAS_FTW__
#define __UCLIBC_HAS_GLOB__ 1

/*
 * Library Installation Options
 */
#define __RUNTIME_PREFIX__ "/opt/ba-elf/"
#define __DEVEL_PREFIX__ "/opt/ba-elf/usr/"

/*
 * uClibc security related options
 */
#undef __UCLIBC_SECURITY__

/*
 * uClibc development/debugging options
 */
#define __CROSS_COMPILER_PREFIX__ "ba-elf-"
#undef __DODEBUG__
#undef __DOASSERTS__
#define __WARNINGS__ "-Wall -maj -mno-cmov -Os"
#undef __UCLIBC_MJN3_ONLY__
