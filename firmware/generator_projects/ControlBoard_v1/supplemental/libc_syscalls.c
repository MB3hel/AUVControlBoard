// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <device.h> /* for ARM CMSIS __BKPT() */

#ifdef __cplusplus
extern "C" {
#endif

/* Harmony specific
 * We implement only the syscalls we want over the stubs provided by libpic32c
 */
    
extern int _end;
    
extern void _exit(int status);
extern caddr_t _sbrk(int incr);

extern void _exit(int status)
{
    /* Software breakpoint */
#ifdef DEBUG
//    asm("bkpt #0");
    __BKPT(0);
#endif

    /* halt CPU */
    while (1)
    {
    }
}

/**
 * \brief Replacement of C library of _sbrk
 */
extern caddr_t _sbrk(int incr)
{
	static unsigned char *heap = NULL;
	unsigned char *       prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&_end;
	}
	prev_heap = heap;

	heap += incr;

	return (caddr_t)prev_heap;
}


int _close(int file) {
	return -1;
}

int _lseek(int file, int ptr, int dir) {
	return 0;
}

int _write (int file, char * ptr, int len) {
	return -1;
}

int _read (int file, char * ptr, int len) {
	return -1;
}

int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;

  return 0;
}

int _isatty(int file) {
  return 1;
}


#ifdef __cplusplus
}
#endif
