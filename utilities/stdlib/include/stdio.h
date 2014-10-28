
/*HEADER**********************************************************************
*
* Copyright 2013 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This is a standard stdio header implemented by MQX
*
*
*END************************************************************************/
#ifndef __stdio_h__
#define __stdio_h__

#include <stdarg.h>
#include "std_prv.h"

/*
 *                        MACRO DECLARATIONS
 */

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Macros suitable for use as third argument to the setvbuf function */
#define IO_FBF      (0)     // IO fully buffered.
#define IO_LBF      (1)     // IO line buffered.
#define IO_NBF      (2)     // IO no buffered.

#define BUFSIZ      (512)   // Size of stdio.h buffers.

#define EOF         (-1)    // End of file value.

///check folowed 3 values
#define FOPEN_MAX   (16)     // Number of streams which the implementation guarantees can be open simultaneously.
#define FILENAME_MAX (128)   // Maximum size in bytes of the longest filename string that the implementation guarantees can be opened.
#define L_tmp_nam   (128)    // Maximum size of character array to hold tmpnam() output

#define SEEK_CUR    (1)     // Seek relative to current position.
#define SEEK_END    (2)     // Seek relative to end of file.
#define SEEK_SET    (0)     // Seek relative to start-of-file.

#define TMP_MAX     (32)    // Minimum number of unique filenames generated by tmpnam().

///define _files in c file!!!
#define stdin       _files[0]   // Standard input stream.
#define stdout      _files[1]   // Standard output stream.
#define stderr      _files[2]   // Standard error output stream.
#define fileno(file_ptr) ((file_ptr)->_FD)

/*--------------------------------------------------------------------------*/

/*
 *                            DATATYPE DECLARATIONS
 */

 /*!
 * \brief This structure defines file structure
 */
typedef   struct
{
    /*! \brief */
    unsigned int _MODE;
    /*! \brief */
    int _FD;

    /*! \brief Pointer to start of stream buffer. */
    unsigned char *_BUF;
    /*! \brief Pointer  to one beyond last byte in buffer. */
//    unsigned char *_BEND;
    /*! \brief  _Next points to next character to read or write. */
//    unsigned char *_NEXT;
    /*! \brief  _Rend points to one beyond last byte that can be read. */
//    unsigned char *_REND;
    /*! \brief _Rsave holds _Rend if characters have been pushed back. */
//    unsigned char *_RSAVE;
    /*! \brief  _Wend points to one beyond last byte that can be written. */
//    unsigned char *_WEND;
    /*! \brief  _Rback points to last pushed back character in _Back. If it has value
       one beyond the _Back array no pushed back chars exists. */
//    unsigned char *_RBACK;
    /*! \brief  One character buffer if no other buffer is available*/
    unsigned char _CBUF;
    /*! \brief  _WRback points to last pushed back wchar_t in _WBack. If it has value
       one beyond the _WBack array no pushed back wchar_ts exists. */
    /*! \brief */
//    unsigned char _BACK[2];
    /*! \brief */
//    unsigned char _NBACK;

    /*! \brief */
//    char *_TMPNAM;
} FILE;


 /*!
 * \brief
 */
typedef   struct
{
    unsigned long _OFF;
} fpos_t;


/* Unsigned type of `sizeof'  */

#ifndef __size_t__	/* BeOS */
#ifndef __SIZE_T__	/* Cray Unicos/Mk */
#ifndef _SIZE_T	/* in case <sys/types.h> has defined it. */
#ifndef _SYS_SIZE_T_H
#ifndef _T_SIZE_
#ifndef _T_SIZE
#ifndef __SIZE_T
#ifndef _SIZE_T_
#ifndef _BSD_SIZE_T_
#ifndef _SIZE_T_DEFINED_
#ifndef _SIZE_T_DEFINED
#ifndef _BSD_SIZE_T_DEFINED_	/* Darwin */
#ifndef _SIZE_T_DECLARED	/* FreeBSD 5 */
#ifndef ___int_size_t_h
#ifndef _GCC_SIZE_T
#ifndef _SIZET_
#ifndef __size_t    /* UV4 */

#define __size_t__	/* BeOS */
#define __SIZE_T__	/* Cray Unicos/Mk */
#define _SIZE_T
#define _SYS_SIZE_T_H
#define _T_SIZE_
#define _T_SIZE
#define __SIZE_T
#define _SIZE_T_
#define _BSD_SIZE_T_
#define _SIZE_T_DEFINED_
#define _SIZE_T_DEFINED
#define _BSD_SIZE_T_DEFINED_	/* Darwin */
#define _SIZE_T_DECLARED	/* FreeBSD 5 */
#define ___int_size_t_h
#define _GCC_SIZE_T
#define _SIZET_
#define __size_t

typedef unsigned int size_t;

#endif /* __size_t */
#endif /* _SIZET_ */
#endif /* _GCC_SIZE_T */
#endif /* ___int_size_t_h */
#endif /* _SIZE_T_DECLARED */
#endif /* _BSD_SIZE_T_DEFINED_ */
#endif /* _SIZE_T_DEFINED */
#endif /* _SIZE_T_DEFINED_ */
#endif /* _BSD_SIZE_T_ */
#endif /* _SIZE_T_ */
#endif /* __SIZE_T */
#endif /* _T_SIZE */
#endif /* _T_SIZE_ */
#endif /* _SYS_SIZE_T_H */
#endif /* _SIZE_T */
#endif /* __SIZE_T__ */
#endif /* __size_t__ */

extern FILE *_files[];
extern unsigned int _files_cnt;


/*--------------------------------------------------------------------------*/

/*
 *                      FUNCTION PROTOTYPES
 */

void     clearerr(FILE *);
int      fclose(FILE *);
int      feof(FILE *);
int      ferror(FILE *);
int      fflush(FILE *);
int      fgetc(FILE *);
///fgetline?? remove
int      fgetpos(FILE *, fpos_t *);
char    *fgets(char *, int, FILE *);
FILE    *fopen(const char *, const char *);
FILE    *fdopen(int , const char *);
int      fprintf(FILE *, const char *, ...);
int      fputc(int, FILE *);
int      fputs(const char *, FILE *);
size_t   fread(void *, size_t, size_t, FILE *);
//FILE    *freopen(const char *, const char *, FILE *);
int      fscanf(FILE *, const char *, ...);
int      fseek(FILE *, long int, int);
int      fsetpos(FILE *, const fpos_t *);
//fstatus??
long int ftell(FILE *);
size_t   fwrite(const void *, size_t, size_t, FILE *);
int      getc(FILE *);
int      getchar(void);
char    *gets(char *);
///ioctl??
//void     perror(const char *);
int      printf(const char *, ...);
int      putc(int, FILE *);
int      putchar(int);
int      puts(const char *);
//int      remove(const char *);
//int      rename(const char *, const char *);
void     rewind(FILE *);
int      scanf(const char *, ...);
//void     setbuf(FILE *, char *);
//int      setvbuf(FILE *, char *, int, size_t);
int      snprintf(char *, size_t, const char *, ...);
int      sprintf(char *, const char *, ...);
int      sscanf(const char *, const char *, ...);
//FILE    *tmpfile(void);
//char    *tmpnam(char *);
///fungetc??
int      ungetc(int, FILE *);
int      vfprintf(FILE *, const char *, va_list);
int      vprintf(const char *, va_list);
int      vsnprintf(char *, size_t, const char *,va_list);
int      vsprintf(char *, const char *, va_list);
/*--------------------------------------------------------------------------*/

#endif
