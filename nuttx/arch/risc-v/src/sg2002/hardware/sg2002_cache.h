#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_CACHE_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_CACHE_H

void sg2002_flush_dcache_range(uintptr_t addr, size_t size);
void sg2002_clean_dcache_range(uintptr_t addr, size_t size);
void sg2002_inv_dcache_range(uintptr_t addr, size_t size);
void sg2002_enable_dcache(void);
void sg2002_disable_dcache(void);

#endif
