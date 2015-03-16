// Copyright (c) 2015 Oliver Sengpiel
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef YUVINTERPRETER_HPP
#define YUVINTERPRETER_HPP

#define int_p_NULL nullptr

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <iostream>
#include <string>

#include "ImageTypes.hpp"

namespace gar
{

enum class PixelClass : unsigned char
{
    UNKNOWN = 0x00,
    CARPET = 0x01,
    TEAM1 = 0x02,
    TEAM2 = 0x04,
    WHITE = 0x08,
    OBSTACLE = 0x10
};

/**
 * @brief Holds information about pixel classification
 *
 * The YUVInterpreter stores a lookup matrix for the classification of image
 * pixels in the color space YUV. The properties of the pixel are represented as
 * bit fields. The individual flags and files which represent them can be added
 * individually.
 */

class YUVInterpreter
{
public:
    unsigned char getClassification(int y, int cb, int cr);

    template <typename... Args>
    void addImage(const std::string & path, Args... args)
    {
        boost::gil::gray8_image_t image;
        boost::gil::png_read_image(path.c_str(), image);

        addPixelClass(image, args...);
    }

    template <typename... Args>
    void addPixelClass(boost::gil::gray8_image_t & img, PixelClass pixelclass,
                       Args... args)
    {
        addPixelClass(img, pixelclass);
        addPixelClass(img, args...);
    }

    void addPixelClass(boost::gil::gray8_image_t & img,
                       PixelClass pixelclass);

    unsigned char operator()(YUV pixel)
    {
        return getClassification(pixel.y, pixel.cb, pixel.cr);
    }

private:
    unsigned char cb_y[256][256] = {};
    unsigned char cr_y[256][256] = {};
    unsigned char cr_cb[256][256] = {};
};

unsigned char YUVInterpreter::getClassification(int y, int cb, int cr)
{
    return (cb_y[cb][y] & cr_y[cr][y] & cr_cb[cr][cb]);
}

void YUVInterpreter::addPixelClass(boost::gil::gray8_image_t & img,
                                   PixelClass pixelclass)
{
    auto view = boost::gil::view(img);
    auto black = boost::gil::gray8_pixel_t(0);
    for (int y = 0; y < 256; ++y)
    {
        for (int x = 0; x < 256; ++x)
        {
            if (view(x, y) != black)
                cb_y[x][y] |= static_cast<unsigned char>(pixelclass);
        }
    }
    for (int y = 0; y < 256; ++y)
    {
        for (int x = 0; x < 256; ++x)
        {
            if (view(x + 256, y) != black)
                cr_y[x][y] |= static_cast<unsigned char>(pixelclass);
        }
    }
    for (int y = 0; y < 256; ++y)
    {
        for (int x = 0; x < 256; ++x)
        {
            if (view(x + 512, y) != black)
                cr_cb[x][y] |= static_cast<unsigned char>(pixelclass);
        }
    }
}

} // Namespace gar

#endif
