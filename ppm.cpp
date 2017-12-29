/*
 * ppm.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: n69136
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <exception>
#include <vector>
#include <algorithm>

#include "ppm.h"

namespace image {

    ppm::ppm() :
            nr_lines(0), nr_columns(0), r(), g(), b(), height(0), width(0), max_col_val(255), size(0) {
    }

    ppm::ppm(const std::string &fname) :
            nr_lines(0), nr_columns(0), r(), g(), b(), height(0), width(0), max_col_val(255), size(0) {
        read(fname);
    }

    ppm::ppm(const uint32_t _width, const uint32_t _height) :
            nr_lines(_height), nr_columns(_width), r(), g(), b(), height(_height), width(_width), max_col_val(255), size(_width * _height) {
        r.resize(size);
        g.resize(size);
        b.resize(size);
        std::fill(r.begin(), r.end(), 0);
        std::fill(g.begin(), g.end(), 0);
        std::fill(b.begin(), b.end(), 0);
    }

    void ppm::read(const std::string& fname) {
        std::ifstream in(fname.c_str(), std::ios::in | std::ios::binary);
        if (in) {
            std::string line;
            std::getline(in, line);
            if (line != "P6") {
                std::cerr << "invalid file format" << std::endl;
                return;
            }
            do {
                std::getline(in, line);
            } while (line[0] == '#');
            {
                std::stringstream istr(line);
                try {
                    istr >> width >> height;
                    nr_lines = height;
                    nr_columns = width;
                } catch (std::exception& e) {
                    std::cerr << "invalid header. " << e.what() << std::endl;
                }
            }
            std::getline(in, line);
            {
                std::stringstream istr(line);
                try {
                    istr >> max_col_val;
                } catch (std::exception& e) {
                    std::cerr << "invalid header. " << e.what() << std::endl;
                }
            }
            size = width * height;
            r.resize(size);
            g.resize(size);
            b.resize(size);
            char temp;
            for (size_t idx = 0; idx < size; idx++) {
                in.read(&temp, 1);
                r[idx] = temp;
                in.read(&temp, 1);
                g[idx] = temp;
                in.read(&temp, 1);
                b[idx] = temp;
            }
        } else {
            std::cerr << "unable to open " << fname << std::endl;
        }
    }

    void ppm::write(const std::string& fname) {
        std::ofstream out(fname.c_str(), std::ios::out | std::ios::binary);
        if (out) {
            out << "P6" << std::endl << width << " " << height << std::endl << max_col_val << std::endl;
            char temp;
            for (size_t idx = 0; idx < size; idx++) {
                temp = r[idx];
                out.write(&temp, 1);
                temp = g[idx];
                out.write(&temp, 1);
                temp = b[idx];
                out.write(&temp, 1);
            }
        } else {
            std::cerr << "unable to open " << fname << std::endl;
        }
    }

    void ppm::write3(const std::string& fname) {
        std::ofstream out(fname.c_str(), std::ios::out | std::ios::binary);
        if (out) {
            out << "P3" << std::endl << width << " " << height << std::endl << max_col_val << std::endl;
            for (size_t idx = 0; idx < size; idx++) {
                out << " " << uint32_t(r[idx]) << " " << uint32_t(g[idx]) << " " << uint32_t(b[idx]);
                if (idx != 0 && idx % width == 0) {
                    out << std::endl;
                } else {
                    out << " ";
                }
            }
        } else {
            std::cerr << "unable to open " << fname << std::endl;
        }
    }

    void ppm::set(uint32_t x, uint32_t y, const color & col) {
        if (x < width && y < height) {
            uint32_t pos = x + y * width;
            r[pos] = col.r;
            g[pos] = col.g;
            b[pos] = col.b;
        }
    }

    color ppm::get (uint32_t x, uint32_t y) {
        if (x < width && y < height) {
            uint32_t pos = x + y * width;
            return color(r[pos], g[pos], b[pos]);
        }
        return color(0, 0, 0);
    }

} /* namespace image */
