#pragma once
#ifndef __SG2002_IIC_H
#define __SG2002_IIC_H

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip.h"
#include "hardware/sg2002_i2c.h"

struct i2c_master_s *sg2002_i2cbus_initialize(int port);

int sg2002_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif

