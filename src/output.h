#ifndef OUTPUT_H
#define OUTPUT_H

#include "vessel.h"
#include "error.h"

AisStatus output_terminal(const VesselTable *t);
AisStatus output_html(const VesselTable *t, const char *path);

#endif /* OUTPUT_H */
