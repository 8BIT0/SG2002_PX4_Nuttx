#ifndef __SG2002_SPI_H
#define __SG2002_SPI_H

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

struct spi_dev_s *sg2002_spibus_initialize(int port);

int sg2002_spibus_uninitialize(struct spi_dev_s *dev);

#endif
