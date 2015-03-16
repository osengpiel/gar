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

#ifndef GAR_IMAGE_HPP
#define GAR_IMAGE_HPP

#include <cassert>
#include <iterator>
#include <algorithm>

#include "ImageTypes.hpp"

namespace gar
{

/**
 * @brief Copies sequences of elements with gaps between them.
 *
 * This algorihm copies sequences of 'copy' elements. Between two copied
 * sequences there is always a skipped sequence of 'skip' elements.
 *
 * @param first Start of range.
 * @param last End of range.
 * @param copy Length of the copied sequences.
 * @param skip Length of the skipped sequences.
 * @param result Output.
 *
 * @return Iterator pointing just beyond the values written to result.
 */
template <int copy, int skip, typename _InputIterator, typename _OutputIterator>
_OutputIterator copy_skip( _InputIterator first, _InputIterator last,
                           _OutputIterator result )
{
    static_assert( copy > 0, "copy must be positive" );
    static_assert( skip > 0, "skip must be positive" );

    if ( first == last )
    {
        return result;
    }

    while ( first + copy <= last )
    {
        result = std::copy( first, first + copy, result );
        first += copy + skip;
    }
    return result;
}

template <typename _II, typename _OI>
void _YUV_copy_contiguous_pair_sequence( _II & current, int pairs,
                                         _OI & result )
{
    for ( int i = 0; i < pairs; ++i )
    {
        *result = YUV{( *current ).left.y, ( *current ).left.cb,
                      ( *current ).left.cr};

        *++result = YUV{( *current ).right.y, ( *current ).right.cb,
                        ( *current ).right.cr};
        ++result;
        ++current;
    }
}

template <bool, bool> struct _YUV_copy;

template <> struct _YUV_copy<false, true>
{
    template <typename _II, typename _OI>
    static bool YUV_copy( _II & first, bool odd, int copy, int skip,
                          _OI & result )
    {
        auto current = first;
        auto last = copy / 2;

        if ( odd )
        {
            --last;

            *result = YUV{( *current ).right.y, ( *current ).right.cb,
                          ( *current ).right.cr};
            ++result;
            ++current;
        }

        _YUV_copy_contiguous_pair_sequence( current, last, result );

        if ( odd )
        {
            *result = YUV{( *current ).left.y, ( *current ).left.cb,
                          ( *current ).left.cr};
        }

        if ( odd )
            first += ( skip + copy ) / 2 + 1;
        else
            first += ( skip + copy ) / 2;

        return !odd;
    }
};

template <> struct _YUV_copy<true, false>
{
    template <typename _II, typename _OI>
    static bool YUV_copy( _II & first, bool odd, int copy, int skip,
                          _OI & result )
    {
        auto current = first;

        if ( odd )
        {
            *result++ = YUV{( *current ).right.y, ( *current ).right.cb,
                            ( *current ).right.cr};
            ++current;
        }

        _YUV_copy_contiguous_pair_sequence( current, copy / 2, result );

        if ( !odd )
        {
            *result++ = YUV{( *current ).left.y, ( *current ).left.cb,
                            ( *current ).left.cr};
        }

        if ( odd )
            first += ( skip + copy ) / 2 + 1;
        else
            first += ( skip + copy ) / 2;

        return !odd;
    }
};

template <> struct _YUV_copy<true, true>
{
    template <typename _II, typename _OI>
    static bool YUV_copy( _II & first, bool, int copy, int skip, _OI & result )
    {
        auto current = first;

        _YUV_copy_contiguous_pair_sequence( current, copy / 2, result );

        *result++ = YUV{( *current ).left.y, ( *current ).left.cb,
                        ( *current ).left.cr};

        first += ( skip + copy ) / 2;

        return false;
    }
};

template <> struct _YUV_copy<false, false>
{
    template <typename _II, typename _OI>
    static bool YUV_copy( _II & first, bool, int copy, int skip, _OI & result )
    {
        auto current = first;

        _YUV_copy_contiguous_pair_sequence( current, copy / 2, result );

        *result++ = YUV{( *current ).left.y, ( *current ).left.cb,
                        ( *current ).left.cr};
        first += ( skip + copy ) / 2;

        return false;
    }
};

/**
 * @brief Does the copy_skip operation on a YUYV image
 *
 * A YUYV image is more difficult to handle than a regular image, because the
 * information for two adjacent pixels is mixed. The output of this algorithm is
 * a YUV picture with individual pixels.
 *
 * @param first Start of range.
 * @param last End of range.
 * @param copy Length of the copied sequences.
 * @param skip Length of the skipped sequences.
 * @param result Output.
 *
 * @return Iterator pointing just beyond the values written to result.
 */
template <int copy, int skip, typename _OutputIterator>
_OutputIterator copy_skip( YUYV * first, YUYV * last, _OutputIterator result )
{
    static_assert( copy > 0, "copy must be positive" );
    static_assert( skip > 0, "skip must be positive" );

    if ( first == last )
    {
        return result;
    }

    auto current = first;
    bool odd = false;

    while ( current + copy <= last )
    {
        odd = _YUV_copy<copy % 2, skip % 2>::YUV_copy( current, odd, copy, skip,
                                                       result );
    }
    return result;
}

/**
 * @brief Extract elements which lie on a grid.
 *
 * Given a container which represents a two dimensional data set this algorithm
 * copies the elements to another container skipping a certain number of
 * elements while traversing vertically and skipping whole lines.
 *
 * @param first Start of range.
 * @param last End of range.
 * @param result Output.
 * @param line_size The number of elements in one line.
 * @param h_skip The number of skipped elements between two copied ones.
 * @param v_skip The number of skipped lines after a non-skipped one.
 *
 * @return Iterator pointing just beyond the values written to result.
 */
template <int h_skip, int v_skip, typename _InputIterator,
          typename _OutputIterator>
_OutputIterator scanline( _InputIterator first, _InputIterator last,
                          int line_size, _OutputIterator result )
{
    static_assert( h_skip >= 0, "h_skip cannot be negative" );
    static_assert( v_skip >= 0, "v_skip cannot be negative" );

    assert( line_size > 0 );

    if ( first == last )
        return result;

    while ( first + line_size <= last )
    {
        result = copy_skip<1, h_skip>( first, first + line_size, result );
        first += ( v_skip + 1 ) * line_size;
    }

    return result;
}

} // Namespace gar

#endif
