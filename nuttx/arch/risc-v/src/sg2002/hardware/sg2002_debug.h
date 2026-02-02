#pragma once
#ifndef __SG2002_DEBUG_H
#define __SG2002_DEBUG_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>

void sg2002_trace_dirout(char *fmt, ...);
uint32_t sg2002_create_stream(uint32_t stream_size);
bool sg2002_delete_stream(uint32_t *stream);
bool sg2002_reset_stream(uint32_t stream);
void sg2002_stream_write(uint32_t stream, char *fmt, ...);
void sg2002_stream_dump(uint32_t stream);

#endif
