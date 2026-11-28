// Radial classifier validated by the offline RS16 benchmark.
#include <Python.h>
#include <cmath>

static void classify_sector(const double *xyz, const double *radii,
                            int begin, int end, unsigned char *obstacle,
                            double width, double global_limit, double local_limit,
                            double split_distance, double split_height, double wheelbase) {
    double ground_r = 0., ground_z = 0., ground_slope = 0.;
    double sum_r = 0., sum_z = 0., count = 0.;
    double last_x = 0., last_y = 0., last_z = 0.;
    bool last_ground = false;

    for (int i = begin; i < end; ++i) {
        const double x = xyz[3 * i], y = xyz[3 * i + 1], z = xyz[3 * i + 2];
        const double radius = radii[i];
        if (i == begin) {
            ground_r = x > wheelbase ? wheelbase : 0.;
            last_x = ground_r;
        }
        const double dx = x - last_x, dy = y - last_y, dz = z - last_z;
        const double distance = sqrt(dx * dx + dy * dy + dz * dz);
        const bool close = distance < radius * width + split_distance;
        double delta_z = z - ground_z, delta_r = radius - ground_r;
        if (close && count > 0.) {
            delta_z = z - sum_z / count;
            delta_r = radius - sum_r / count;
        }

        const bool above_global = radius > 0. && z / radius > global_limit;
        const bool follow = !above_global && last_ground && close && fabs(delta_z) < split_height;
        const bool object = above_global || (!follow && atan2(delta_z, delta_r) - ground_slope > local_limit);
        obstacle[i] = object;
        if (!object) {
            if (!follow) {
                sum_r = sum_z = count = 0.;
            }
            ground_r = radius;
            ground_z = z;
            sum_r += radius;
            sum_z += z;
            count += 1.;
            ground_slope = atan2(sum_z / count, sum_r / count);
        }
        last_ground = !object;
        last_x = x;
        last_y = y;
        last_z = z;
    }
}

extern "C" void radial(const double *xyz, const double *radii, const int *offsets,
                       int sectors, unsigned char *obstacle, double width, double global_limit,
                       double local_limit, double split_distance, double split_height, double wheelbase) {
    for (int sector = 0; sector < sectors; ++sector) {
        classify_sector(xyz, radii, offsets[sector], offsets[sector + 1], obstacle,
                        width, global_limit, local_limit, split_distance, split_height, wheelbase);
    }
}

static PyModuleDef module = {PyModuleDef_HEAD_INIT, "_radial_ground", nullptr, -1, nullptr};

PyMODINIT_FUNC PyInit__radial_ground() {
    return PyModule_Create(&module);
}
