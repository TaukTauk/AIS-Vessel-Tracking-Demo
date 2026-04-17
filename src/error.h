#ifndef AIS_ERROR_H
#define AIS_ERROR_H

#include <stdarg.h>

typedef enum {
    AIS_OK               = 0,
    AIS_ERR_IO           = 1,
    AIS_ERR_CHECKSUM     = 2,
    AIS_ERR_MALFORMED    = 3,
    AIS_ERR_UNSUPPORTED  = 4,
    AIS_ERR_TRUNCATED    = 5,
    AIS_ERR_TABLE_FULL   = 6,
    AIS_ERR_NULL         = 7,
    AIS_FRAG_PENDING     = 8,   /* first fragment stored; awaiting second */
} AisStatus;

typedef struct {
    int sentences_read;
    int sentences_ok;
    int err_checksum;
    int err_malformed;
    int err_unsupported;
    int err_truncated;
    int err_table_full;
    int warn_sentinel;
    int warn_quality;

    AisStatus   last_status;
    char        last_msg[128];
    int         last_line;
} AisErrorCtx;

const char *ais_status_str(AisStatus s);
void        ais_error_init(AisErrorCtx *ctx);
void        ais_error_log(AisErrorCtx *ctx, AisStatus s,
                          int line_num, const char *fmt, ...);
void        ais_error_summary(const AisErrorCtx *ctx);

#endif /* AIS_ERROR_H */
