#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>     /* <<— add this */
#include <stdbool.h>

typedef struct CSVLogger {
    FILE *fp;
} CSVLogger;

bool csv_init(CSVLogger *logger, const char *path);
void csv_log(CSVLogger *logger,
             double t,
             double ax, double ay, double az,
             double vx, double vy, double vz,
             double dx, double dy, double dz,
             double rx, double ry, double rz);
void csv_close(CSVLogger *logger);

extern CSVLogger logger;

#endif /* LOGGER_H */
