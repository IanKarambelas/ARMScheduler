#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <reent.h>
#include <unistd.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/uart.h"

/* Forward prototypes.  */
int     _system       _PARAMS ((const char *));
int     _rename       _PARAMS ((const char *, const char *));
int     _isatty       _PARAMS ((int));
clock_t _times        _PARAMS ((struct tms *));
int     _gettimeofday _PARAMS ((struct timeval *, struct timezone *));
void    _raise        _PARAMS ((void));
int     _unlink       _PARAMS ((void));
int     _link         _PARAMS ((void));
int     _stat         _PARAMS ((const char *, struct stat *));
int     _fstat        _PARAMS ((int, struct stat *));
caddr_t _sbrk         _PARAMS ((int));
int     _getpid       _PARAMS ((int));
int     _kill         _PARAMS ((int, int));
void    _exit         _PARAMS ((int));
int     _close        _PARAMS ((int));
int     _open         _PARAMS ((const char *, int, ...));
int     _write        _PARAMS ((int, char *, int));
int     _lseek        _PARAMS ((int, int, int));
int     _read        _PARAMS ((int, char *, int));

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

/* Return number of characters read, no more than 'len' */
int _read (int file, char *ptr, int len)
{
  return 0;
}

int _lseek (int file, int ptr, int dir)
{
  return 0;
}

/* Return number of characters written */
int _write (int file, char *ptr, int len) 
{
  unsigned i;

  for (i=0; i < len; i++) {
    UARTCharPut(UART0_BASE, *ptr++);
  }
  return len;
}

int _open (const char *path, int flags, ...)
{
  return 0;
}

int _close (int file)
{
  return 0;
}

void _exit (int n)
{
  (void) n;  // Parameter not used
  while (1);
}

int _kill (int n, int m)
{
  (void) n; (void) m;
  return 0;
}

int _getpid (int n)
{
  (void) n;
  return 1;
}

char * heap_end;
caddr_t _sbrk (int incr)
{
  extern char   end asm ("end");    /* Defined by the linker.  */
  char *        prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;
  
  prev_heap_end = heap_end;
  
  if (heap_end + incr > stack_ptr) {
      errno = ENOMEM;
      return (caddr_t) -1;
  }
  
  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

int _fstat (int file, struct stat * st)
{
  return 0;
}

int _stat (const char *fname, struct stat *st)
{
    return -1;
}

int _link (void)
{
  return -1;
}

int _unlink (void)
{
  return -1;
}

void _raise (void)
{
  return;
}

int _gettimeofday (struct timeval * tp, struct timezone * tzp)
{
  return 0;
}

/* Return a clock that ticks at 100Hz.  */
clock_t _times (struct tms * tp)
{
  return 0;
};


int _isatty (int fd)
{
  return 1;
  fd = fd;
}

int _system (const char *s)
{
  if (s == NULL) return 0;
  errno = ENOSYS;
  return -1;
}

int _rename (const char * oldpath, const char * newpath)
{
  errno = ENOSYS;
  return -1;
}
