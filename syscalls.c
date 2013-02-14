// Thumb2 Newlib Toolchain example project
// Written by Elias Önal <EliasOenal@gmail.com>, released as public domain.

#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#undef errno
extern int errno;


int _write(int file, char *ptr, int len);


void* _sbrk(int incr)
{
	extern char __end__; // Defined in linkerscript, end of .data and .bss

	static char *heap_end;
	char *prev_heap_end;
	extern char _estack;
	extern char __StackLimit;

	if (heap_end == NULL) {
		heap_end = &__end__;
	}
	prev_heap_end = heap_end;
	if((((uint32_t)heap_end) + ((uint32_t)incr))
			> ((uint32_t)&__StackLimit)) // stack protection
	{
		errno = ENOMEM;
		return (void*) -1;
	}

	heap_end += incr;
	return (void*) prev_heap_end;
}

int _getpid()
{
	return 1;
}


int _kill(int pid, int sig)
{
	errno=EINVAL;
	return(-1);
}

void _exit(int i)
{
	while(1);
}

int _close(int file)
{
	return -1;
}

int _open(const char *name, int flags, int mode){
	return -1;
}


#include <sys/stat.h>
int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _read(int file, char *ptr, int len)
{
	return 0;
}

int _write(int file, char *ptr, int len)
{
	int count = 0;

	return count;
}

#include <sys/time.h>
#include <time.h>
#include <unistd.h>

int _gettimeofday(struct timeval *tv, void *tz) {
	errno = ENOSYS;
	return -1;
}



