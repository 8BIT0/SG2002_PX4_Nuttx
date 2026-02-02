#include "hardware/sg2002_debug.h"

#define TRACE_BUF_SIZE      2048
#define TRACE_HEADER_SIZE   1024

#define TRACE_HEADER        "[ 8B!T0 trace ] "

static char trace_print_buff[TRACE_BUF_SIZE];
static char header[TRACE_HEADER_SIZE];

typedef struct {
    uint32_t total_size;

    uint8_t *p_buf_s;
    uint8_t *p_buf_e;
    uint8_t *p_seek;
} SG2002_DebugStream_TypeDef;

void sg2002_trace_dirout(char *fmt, ...) {
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

uint32_t sg2002_create_stream(uint32_t stream_size) {
    SG2002_DebugStream_TypeDef *p_stream = NULL;

    p_stream = malloc(sizeof(SG2002_DebugStream_TypeDef));
    if (p_stream == NULL)
        return 0;

    memset(p_stream, 0, sizeof(SG2002_DebugStream_TypeDef));
    p_stream->p_buf_s = malloc(stream_size);
    if (p_stream->p_buf_s == NULL) {
        free(p_stream);
        return 0;
    }

    memset(p_stream->p_buf_s, 0, stream_size);

    p_stream->total_size = stream_size;
    p_stream->p_seek = p_stream->p_buf_s;
    p_stream->p_buf_e = p_stream->p_buf_s + stream_size;

    return ((uint32_t)(uintptr_t)p_stream);
}

bool sg2002_delete_stream(uint32_t *stream) {
    SG2002_DebugStream_TypeDef *p_stream = (SG2002_DebugStream_TypeDef *)((uintptr_t)*stream);

    if ((stream == NULL) || \
        (p_stream == NULL) || \
        (p_stream->total_size == 0) || \
        (p_stream->p_buf_s == NULL))
        return false;

    memset(p_stream->p_buf_s, 0, p_stream->total_size);
    free(p_stream->p_buf_s);
    p_stream->p_buf_s = NULL;

    p_stream->total_size = 0;
    p_stream->p_buf_e = NULL;
    p_stream->p_seek = NULL;

    *stream = 0;

    return true;
}

bool sg2002_reset_stream(uint32_t stream) {
    SG2002_DebugStream_TypeDef *p_stream = (SG2002_DebugStream_TypeDef *)(uintptr_t)stream;

    if ((p_stream == NULL) || \
        (p_stream->total_size == 0) || \
        (p_stream->p_buf_s == NULL) || \
        (p_stream->p_buf_e == NULL) || \
        (((uint32_t)(uintptr_t)(p_stream->p_buf_e) - (uint32_t)(uintptr_t)(p_stream->p_buf_s)) != p_stream->total_size))
        return false;

    memset(p_stream->p_buf_s, 0, p_stream->total_size);
    p_stream->p_buf_e = p_stream->p_buf_s + p_stream->total_size;
    p_stream->p_seek = p_stream->p_buf_s;

    return true;
}

void sg2002_stream_write(uint32_t stream, char *fmt, ...) {
    va_list args;
    SG2002_DebugStream_TypeDef *p_stream = (SG2002_DebugStream_TypeDef *)(uintptr_t)stream;
    irqstate_t flags;

    if ((p_stream == NULL) || \
        (p_stream->total_size == 0) || \
        (p_stream->p_buf_s == NULL) || \
        (p_stream->p_buf_e == NULL) || \
        (((uint32_t)(uintptr_t)(p_stream->p_buf_e) - (uint32_t)(uintptr_t)(p_stream->p_buf_s)) != p_stream->total_size) || \
        (p_stream->p_seek == NULL) || \
        (p_stream->p_seek >= p_stream->p_buf_e))
        return;

    flags = enter_critical_section();
    va_start(args, fmt);
    uint32_t len = vsprintf((char *)p_stream->p_seek, fmt, args);
    va_end(args);

    if ((p_stream->p_seek + len) < p_stream->p_buf_e) {
        p_stream->p_seek += len;
    }
    leave_critical_section(flags);
}

/* can not call dump in isr process function 
 * reset automaticaly after dump stream
 */
void sg2002_stream_dump(uint32_t stream) {
    SG2002_DebugStream_TypeDef *p_stream = (SG2002_DebugStream_TypeDef *)(uintptr_t)stream;

    if ((p_stream == NULL) || (p_stream->p_buf_s == NULL))
        return;
    
    printf("%s", p_stream->p_buf_s);
    sg2002_reset_stream(stream);
}