/********************************************************************************
 * @file   retarget_gcc.c
 *
 * @brief  Dummy implementation of system calls to satisfy linker requirements.
 ********************************************************************************/

int _close(int file) { return -1; }
int _lseek(int file, int ptr, int dir) { return -1; }
int _read(int file, char *ptr, int len) { return -1; }
int _write(int file, char *ptr, int len) { return -1; }