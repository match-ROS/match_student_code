#ifndef BOOST_VORONOI_HPP
#define BOOST_VORONOI_HPP

#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/polygon/voronoi.hpp>
#include <boost/foreach.hpp>

using boost::polygon::high;
using boost::polygon::low;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;

static const std::size_t EXTERNAL_COLOR = 1;

/*
struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};
*/

struct Segment
{
    cv::Point p0;
    cv::Point p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2)
    {
    }
};

namespace boost
{
namespace polygon
{

template <>
struct geometry_concept<cv::Point2i>
{
    typedef point_concept type;
};

template <>
struct point_traits<cv::Point2i>
{
    typedef int coordinate_type;

    static inline coordinate_type get(const cv::Point2i& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

template <>
struct geometry_concept<Segment>
{
    typedef segment_concept type;
};

template <>
struct segment_traits<Segment>
{
    typedef int coordinate_type;
    typedef cv::Point2i point_type;

    static inline point_type get(const Segment& segment, direction_1d dir)
    {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};
}  // namespace polygon
}  // namespace boost

namespace voronoi_generation
{
/** @brief uses boost library to create voronoi map
 *
 * @param img_binary grayscale image with obstacles in white (255) and free space in black (0)
 * @param voronoi_img output voronoi image with voronoi map in white, everything else black
 * @param debug_output if debug prints and images should be enabled
 */
bool create_boost_voronoi(const cv::Mat& img_binary, cv::Mat& voronoi_img, bool debug_output);

} // namespace voronoi_generation

#endif /* BOOST_VORONOI_HPP */