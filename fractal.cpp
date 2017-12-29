/*
 * fractal.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: n69136
 */

#include <complex>

#include "ppm.h"

size_t calculate(double x, double y) {
    std::complex<double> c(x, y);
    std::complex<double> z(0, 0);
    size_t iterations(0);
    while (z.real() * z.real() + z.imag() * z.imag() < 4.0) {
        if ((++iterations) >= 10000) {
            return 0;
        }
        z = z * z + c;
    }
    return iterations;
}

int fractal() {
    image::ppm image(1200, 800);

    double x(-2.0), y(1.0);
    double dx(2 * x / image.width), dy(2 * y / image.height);
    for (uint32_t ix = 0; ix < image.width; ix++) {
        y = 1.0;
        for (uint32_t iy = 0; iy < image.height; iy++) {
            size_t iterations = calculate(x, y);
            image.set(ix, iy, image::color(iterations % 255, iterations % 255, iterations % 255));
            x -= dx;
            y -= dy;
        }
    }

    image.write("fractal.ppm");
    return 0;
}
