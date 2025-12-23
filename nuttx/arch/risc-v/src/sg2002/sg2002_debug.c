#include "hardware/sg2002_debug.h"

#define TRACE_BUF_SIZE      2048
#define TRACE_HEADER_SIZE   1024

#define TRACE_HEADER        "[ 8B!T0 trace ] "

static uint8_t trace_print_buff[TRACE_BUF_SIZE];
static uint8_t header[TRACE_HEADER_SIZE];

void sg2002_trace(char *fmt, ...) {
    va_list args;
    uint32_t len = 0;
    
    memset(header, '\0', TRACE_HEADER_SIZE);
    memset(trace_print_buff, '\0', TRACE_BUF_SIZE);

    va_start(args, fmt);
    strcpy(header, TRACE_HEADER);
    strcat(header, fmt);
    len = vsprintf(trace_print_buff, header, args);
    va_end(args);

    if (len < 0)
        return;

    for (uint32_t i = 0; i < len; i ++)
        up_putc(trace_print_buff[i]);
}

