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

#ifndef GAR_ANALYZE_HPP
#define GAR_ANALYZE_HPP

#include <iterator>
#include <algorithm>

namespace gar
{

/**
 * @brief Finds horizontal edges to sequences of the given value
 *
 * Scans a given range of elements and returns the positions of the edges to
 * contiguous sequences of the value.
 *
 * @param first The first element of the sequence
 * @param last The last element of the sequence
 * @param result The destination of the result
 * @param value The desired elements of the sequence
 *
 * @return Iterator pointing to just beyond the values written to result.
 */
template <typename _InputIterator, typename _OutputIterator, typename T>
_OutputIterator findEdges(_InputIterator first, _InputIterator last,
                          _OutputIterator result, T value)
{
    if (first == last)
    {
        return result;
    }

    auto current = first;
    while (++current != last - 1)
    {
        if (((*(current - 1) & value) ^ (*(current + 1) & value)) &&
            (*current & value))
        {
            *result = std::distance(first, current);
        }
        ++result;
    }

    return result;
}

// This is a struct which holds the information about the found circle
template <typename T> struct Circle
{
    std::pair<T, T> position;
    T radius;
};

/**
 * @brief Calculates a circle trough the three points
 *
 * Takes three points which lie on a circle and returns the radius and midpoint
 * of the circle.
 *
 * @param first The first point
 * @param second The second point
 * @param third The third point
 *
 * @return A struct which holds the position and the radius
 */
template <typename T>
Circle<T> getCircleFromThreePoints(std::pair<T, T> first,
                                   std::pair<T, T> second,
                                   std::pair<T, T> third)
{
    double m_a =
        double(second.second - first.second) / (second.first - first.first);
    double m_b =
        double(third.second - second.second) / (third.first - second.first);

    double center_x = (m_a * m_b * (third.second - first.second) +
                       m_a * (second.first + third.first) -
                       m_b * (first.first + second.first)) /
                      (2 * (m_a - m_b));
    double center_y;
    if (m_a != 0)
    {
        center_y = ((double(first.second + second.second) / 2) -
                    (center_x - double(first.first + second.first) / 2) / m_a);
    }
    else if (m_b != 0)
    {
        center_y = ((double(second.second + third.second) / 2) -
                    (center_x - double(second.first + third.first) / 2) / m_b);
    }
    else
    {
        return Circle<T>{std::make_pair(0, 0), 0};
    }

    double radius =
        sqrt(pow(first.first - center_x, 2) + pow(first.second - center_y, 2));

    return Circle<T>{std::make_pair(T(center_x), T(center_y)), T(radius)};
}

/**
 * @brief Clusters the lines into distinct lines
 *
 * This algorihm selects only those points from the given lines, which have two
 * or less neighbors. contiguous sequences of points are then assigned to the
 * same cluster. The neighborhood function provides the neighbors of the points.
 *
 * @param first First element of the sequence
 * @param last Last element of the sequence
 * @param result Output
 * @param neighborFunction Determines the neighbors of the elements
 *
 * @return Iterator that points just beyond the values written to the result
 */

template <typename _InputIterator, typename _OutputIterator, typename _UnaryOp>
_OutputIterator clusterLineSegments(_InputIterator first, _InputIterator last,
                                    _OutputIterator result,
                                    _UnaryOp neighborFunction)
{
    if (first == last)
    {
        return result;
    }

    auto marked = std::vector<typename _InputIterator::value_type>{};
    auto stack = std::vector<typename _InputIterator::value_type>{};

    while (first != last)
    {
        if (std::find(marked.begin(), marked.end(), *first) == marked.end())
        {
            stack.push_back(*first);

            auto cluster = std::vector<typename _InputIterator::value_type>{};

            while (!stack.empty())
            {

                auto current = stack.back();
                stack.pop_back();
                marked.push_back(current);

                auto neighbors = neighborFunction(current);

                if (neighbors.size() <= 2)
                {
                    cluster.push_back(current);

                    for (auto neighbor : neighbors)
                    {
                        if (std::find(marked.begin(), marked.end(), neighbor) ==
                            marked.end())
                        {
                            stack.push_back(neighbor);
                        }
                    }
                }
            }
            if (!cluster.empty())
            {
                *(result++) = cluster;
            }
        }
        ++first;
    }

    return result;
}

} // Namespace gar

#endif
