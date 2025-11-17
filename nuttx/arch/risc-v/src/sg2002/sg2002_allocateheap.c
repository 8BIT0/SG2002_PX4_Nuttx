#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <arch/irq.h>
#include "riscv_internal.h"
#include "chip.h"

#define HEAP_SIZE 1024*128

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* Return the heap settings */

  // memset(g_idle_topstack, 0, HEAP_SIZE);

  *heap_start = (void *)g_idle_topstack;
  *heap_size = HEAP_SIZE;
}

void up_addregion(void)
{
}


