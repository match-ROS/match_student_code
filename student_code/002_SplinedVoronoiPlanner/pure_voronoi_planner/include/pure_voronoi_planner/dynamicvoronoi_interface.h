#ifndef DYNAMICVORONOI_INTERFACE_HPP
#define DYNAMICVORONOI_INTERFACE_HPP

#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <pure_voronoi_planner/dynamicvoronoi/dynamicvoronoi.h>

namespace voronoi_generation
{

cv::Mat create_dynamicvoronoi(const cv::Mat& img, int free_cell_threshold);

} // namespace voronoi_generation

#endif /* DYNAMICVORONOI_INTERFACE_HPP */