/*
 * ppm.h
 *
 *  Created on: Aug 28, 2017
 *      Author: n69136
 */

#ifndef PPM_H_
#define PPM_H_

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>

namespace image {

    class color {
    public:
        uint8_t r;
        uint8_t g;
        uint8_t b;

        color (uint8_t r_ = 0, uint8_t g_ = 0, uint8_t b_ = 0) : r(r_), g(g_), b(b_) {}

        friend std::ostream& operator << (std::ostream& out, const color& col) {
            out << "rgb(" << uint16_t(col.r) << "," << uint16_t(col.g) << "," << uint16_t(col.b) << ")";
            return out;
        }
    };

    class ppm {
        //info about the PPM file (height and width)
        uint32_t nr_lines;
        uint32_t nr_columns;
    public:
        //arrays for storing the R,G,B values
        std::vector<uint8_t> r;
        std::vector<uint8_t> g;
        std::vector<uint8_t> b;
        //
        uint32_t height;
        uint32_t width;
        uint32_t max_col_val;

        //total number of elements (pixels)
        size_t size;

        ppm();
        //create a PPM object and fill it with data stored in fname
        ppm(const std::string &fname);
        //create an "epmty" PPM image with a given width and height;the R,G,B arrays are filled with zeros
        ppm(const uint32_t _width, const uint32_t _height);
        //read the PPM image from fname
        void read(const std::string &fname);
        //write the PPM image in fname
        void write(const std::string &fname);
        void write3(const std::string &fname);

        void set(uint32_t x, uint32_t y, const color & col);

        color get(uint32_t x, uint32_t y);
    };

} /* namespace image */

#endif /* PPM_H_ */
