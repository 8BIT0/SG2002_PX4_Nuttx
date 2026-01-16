#include "hardware/sg2002_debug.h"

#define TRACE_BUF_SIZE      2048
#define TRACE_HEADER_SIZE   1024

#define TRACE_HEADER        "[ 8B!T0 trace ] "

static char trace_print_buff[TRACE_BUF_SIZE];
static char header[TRACE_HEADER_SIZE];

void sg2002_trace(char *fmt, ...) {
    va_list args;
    irqstate_t flags;
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
    
    flags = enter_critical_section();
    for (uint32_t i = 0; i < len; i ++)
        up_putc(trace_print_buff[i]);
    leave_critical_section(flags);
}

