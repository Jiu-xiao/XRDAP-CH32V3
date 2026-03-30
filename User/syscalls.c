#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

extern char _heap_start; // Heap region start from linker script.
extern char _heap_end;   // Heap region end from linker script.

caddr_t _sbrk(int incr)
{
  static char* heap_curr = NULL;
  char* prev_heap_curr;
  char* heap_limit = &_heap_end;

  if (heap_curr == NULL)
  {
    heap_curr = &_heap_start;
  }

  prev_heap_curr = heap_curr;

  if (incr > 0 && (heap_curr + incr) > heap_limit)
  {
    errno = ENOMEM;
    return (caddr_t)-1;
  }

  heap_curr += incr;
  return (caddr_t)prev_heap_curr;
}

int _close(int fd)
{
  (void)fd;
  errno = ENOSYS;
  return -1;
}

int _fstat(int fd, struct stat* st)
{
  (void)fd;
  if (st != NULL)
  {
    st->st_mode = S_IFCHR;
  }
  return 0;
}

int _isatty(int fd)
{
  (void)fd;
  return 1;
}

int _lseek(int fd, int ptr, int dir)
{
  (void)fd;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int fd, void* ptr, size_t len)
{
  (void)fd;
  (void)ptr;
  (void)len;
  errno = ENOSYS;
  return -1;
}

int _write(int fd, const void* ptr, size_t len)
{
  (void)fd;
  (void)ptr;
  (void)len;
  return (int)len;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = ENOSYS;
  return -1;
}

int _getpid(void) { return 1; }

int _getentropy(void* buffer, size_t length)
{
  (void)buffer;
  (void)length;
  errno = ENOSYS;
  return -1;
}
