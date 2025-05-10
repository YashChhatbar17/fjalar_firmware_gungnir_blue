#include "logger.h"
#include <stdio.h>
#include <sys/stat.h>

/* Return true if file exists on disk */
static bool file_exists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

bool csv_init(CSVLogger *logger, const char *path) {
    // always truncate (delete old contents)
    logger->fp = fopen(path, "w");
    if (!logger->fp) {
        return false;
    }

    fprintf(logger->fp,
        "time,"
        "acceleration_x,acceleration_y,acceleration_z,"
        "velocity_x,velocity_y,velocity_z,"
        "distance_x,distance_y,distance_z,"
        "rotation_x,rotation_y,rotation_z\n"
    );
    fflush(logger->fp);
    return true;
}

void csv_log(CSVLogger *logger,
             double    t,
             double    ax, double ay, double az,
             double    vx, double vy, double vz,
             double    dx, double dy, double dz,
             double    rx, double ry, double rz)
{
    fprintf(logger->fp,
        "%.6f,"    /* time */
        "%.6f,%.6f,%.6f,"  /* accel */
        "%.6f,%.6f,%.6f,"  /* vel   */
        "%.6f,%.6f,%.6f,"  /* dist  */
        "%.6f,%.6f,%.6f\n",/* rot   */
        t,
        ax, ay, az,
        vx, vy, vz,
        dx, dy, dz,
        rx, ry, rz
    );
    fflush(logger->fp);
}

void csv_close(CSVLogger *logger) {
    if (logger->fp) {
        fclose(logger->fp);
        logger->fp = NULL;
    }
}
