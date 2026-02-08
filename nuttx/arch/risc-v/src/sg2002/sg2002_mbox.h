#ifndef __SG2002_MBOX_H
#define __SG2002_MBOX_H

#include <nuttx/config.h>

#include "chip.h"
#include <nuttx/mbox/mbox.h>
#include "hardware/sg2002_mbox.h"

struct mbox_dev_s *sg2002_mailbox_initialize(void);

int sg2002_mailbox_uninitialize(struct mbox_dev_s *dev);

#endif

