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

/**
 * @file Analyze.hpp
 * @brief Functions to analyze image data.
 */

#ifndef GAR_ANALYZE_HPP
#define GAR_ANALYZE_HPP

#include <iterator>
#include <algorithm>
#include <random>
#include <cmath>

#include <iostream>

namespace gar
{

/**
 * @brief Finds horizontal edges to sequences of the given value.
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

    auto current = first + 1;
    for (; current != last - 1; ++current)
    {
        if (((*(current - 1) & value) ^ (*(current + 1) & value)) &&
            (*current & value))
        {
            *result = std::distance(first, current);
            ++result;
        }
    }

    return result;
}

template <typename _InputIterator, typename _OutputIterator, typename T>
_OutputIterator findEdgesInImage(_InputIterator first, _InputIterator last,
                          _OutputIterator result,int line_size, T value)
{
    assert(std::distance(first,last) % line_size == 0);

    if(first == last)
    {
	return result;
    }

    int currentLine = 0;
    for(; first != last; first += line_size, ++currentLine)
    {
	auto pointsOnLine = std::vector<int>{};
	findEdges(first, first+line_size, std::back_inserter(pointsOnLine), value);
	for(auto point : pointsOnLine)
	{
	    *(result++) = std::make_pair(point,currentLine);
	}
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
Circle<T> getCircleFromThreePoints(std::pair<T, T> one,
                                   std::pair<T, T> two,
                                   std::pair<T, T> three)
{
    assert(one != two);
    assert(one != three);
    assert(two != three);

    auto m_a = double(two.second - one.second) / (two.first - one.first);
    auto m_b = double(three.second - two.second) / (three.first - two.first);

    auto result = Circle<T>{std::make_pair(0, 0), 0};

    if (m_a != m_b && (std::isfinite(m_a) && std::isfinite(m_b)))
    {
        result.position.first =
            (m_a * m_b * (three.second - one.second) +
             m_a * (two.first + three.first) - m_b * (one.first + two.first)) /
            (2 * (m_a - m_b));

        if (m_a != 0.0)
        {
            result.position.second =
                ((double(one.second + two.second) / 2) -
                 (result.position.first - double(one.first + two.first) / 2) /
                     m_a);
        }
        else if (m_b != 0.0)
        {
            result.position.second =
                ((double(two.second + three.second) / 2) -
                 (result.position.first - double(two.first + three.first) / 2) /
                     m_b);
        }
        result.radius = sqrt(pow(one.first - result.position.first, 2) +
                             pow(one.second - result.position.second, 2));
    }

    return result;
}

template <typename T>
int _pointDistance(std::pair<T,T> left, std::pair<T,T> right)
{
    return sqrt(pow(left.first - right.first, 2) +
                pow(left.second - right.second, 2));
}

template <typename T>
Circle<T> getCircleFromLineSegment(std::vector<std::pair<T, T>> & points)
{
    using Pair = std::pair<T, T>;

    auto result = Circle<T>{};

    if (points.size() < 8)
    {
        return result;
    }

    std::sort(points.begin(), points.end(), [](Pair left, Pair right)
              {
                  return left.second < right.second;
              });

    auto pointDistance = (points.size()-4)/4;

    auto first = points.begin();
    auto second = first + pointDistance;
    auto third = second + pointDistance;

    auto counter = 0;
    for (; third != points.end(); ++first, ++second, ++third )
    {
        auto currentCircle = getCircleFromThreePoints(*first, *second, *third);

	if(currentCircle.radius > 0)
	{
	    result.position.first += currentCircle.position.first;
	    result.position.second += currentCircle.position.second;
	    result.radius += currentCircle.radius;

	    ++counter;
	}
    }

    if(counter != 0)
    {
        result.position.first /= counter;
        result.position.second /= counter;
        result.radius /= counter;
    }

    return result;
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

    for (;first != last; ++first)
    {
        if (std::none_of(marked.begin(), marked.end(), 
		    [&](decltype(*first) other) { return other == *first; }))
        {
            stack.push_back(*first);

            auto cluster = std::vector<typename _InputIterator::value_type>{};

	    // DFS for distict lines
            while (!stack.empty())
            {
                auto current = stack.back();
                stack.pop_back();
                marked.push_back(current);

                auto neighbors = neighborFunction(current);

                if (neighbors.size() <= 2) // A line segment has 2 neighbors
                {
                    cluster.push_back(current);

                    for (auto neighbor : neighbors)
                    {
                        if (std::none_of(marked.begin(), marked.end(),
                                         [&](decltype(neighbor) other)
                                         { return other == neighbor; }))
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
    }

    return result;
}

} // Namespace gar

#endif
