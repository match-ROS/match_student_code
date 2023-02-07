#include <pure_voronoi_planner/boost_voronoi.h>
#include <pure_voronoi_planner/dynamicvoronoi_interface.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <chrono>
#include <fstream>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Need 1 argument" << std::endl;
        return -1;
    }
    std::cout << "===============" << std::endl;
    std::string base_filepath = ros::package::getPath("pure_voronoi_planner") + "/../data/voronoi/";
    // load image
    std::cout << "Load map from " << argv[1] << std::endl;
    double start_resolution = 0.05;
    double distance_to_obstacle = 0.6;
    cv::Mat img_gray = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    // all unknowns as obstacle
    cv::Mat img_binary;
    cv::threshold(img_gray, img_binary, 220, 255, cv::THRESH_BINARY);
    // cv::imwrite("/home/soenke/Bilder/voronoi_comparison/input_img_binary.png", img_binary);
    cv::imwrite(base_filepath + "input_img_binary.png", img_binary);
    // get costmap by distance transform
    std::cout << "Calc costmap from map based on distance transform" << std::endl;
    cv::Mat costmap_img;
    cv::distanceTransform(img_binary, costmap_img, cv::DIST_L2, 3, CV_8UC1);
    // cv::imwrite("/home/rosmatch/Bilder/voronoi_comparison/distance_img.png", costmap_img);
    cv::threshold(costmap_img, costmap_img, distance_to_obstacle / start_resolution, 255, cv::THRESH_BINARY_INV);
    costmap_img.convertTo(costmap_img, CV_8UC1);
    // cv::imwrite("/home/soenke/Bilder/voronoi_comparison/costmap_img.png", costmap_img);
    cv::imwrite(base_filepath + "costmap_img.png", costmap_img);

    double max_img_dimension = img_gray.cols > img_gray.rows ? img_gray.cols : img_gray.rows;
    std::vector<double> desired_dimensions = {4000.0, 3000.0, 2000.0, 1000.0};
    std::vector<double> times_dynamicvoronoi;
    std::vector<double> times_boostvoronoi;

    int downsampling_idx = 0;
    // std::string base_filepath = "/home/soenke/Bilder/voronoi_comparison/";
    for (auto desired_dimension : desired_dimensions)
    {
        std::cout << "===============" << std::endl;
        // downsampling costmap
        double factor = desired_dimension / max_img_dimension;
        std::cout << "Dimension: " << desired_dimension << std::endl;
        cv::Mat costmap_img_downsampled;
        if (factor < 1.0)
        {
            cv::resize(costmap_img, costmap_img_downsampled, cv::Size(), factor, factor, cv::INTER_AREA);
        }
        else
        {
            cv::resize(costmap_img, costmap_img_downsampled, cv::Size(), factor, factor, cv::INTER_CUBIC);
        }
        std::string downsample_filename = base_filepath + "costmap_img_downsampled_" + std::to_string(downsampling_idx) + ".png";
        cv::imwrite(downsample_filename, costmap_img_downsampled);
        // create dynamicvoronoi
        int free_cell_threshold = 100;
        std::chrono::steady_clock::time_point begin_dynamicvoronoi = std::chrono::steady_clock::now();
        cv::Mat out_dynamicvoronoi = voronoi_generation::create_dynamicvoronoi(~costmap_img_downsampled, free_cell_threshold);
        std::chrono::steady_clock::time_point end_dynamicvoronoi = std::chrono::steady_clock::now();
        double secs_dynamicvoronoi = (std::chrono::duration_cast<std::chrono::microseconds>(end_dynamicvoronoi - begin_dynamicvoronoi).count()) / 1000000.0;
        times_dynamicvoronoi.push_back(secs_dynamicvoronoi);
        std::cout << "Time taken for dynamicvoronoi generation: " << secs_dynamicvoronoi << std::endl;
        // adapt output of dynamicvoronoi to match output of boostvoronoi with obstacles 0, free space 127 and voronoi 255
        cv::Mat out_dynamicvoronoi_corrected(~costmap_img_downsampled / 2 + out_dynamicvoronoi);
        std::string dynamicvoronoi_filename = base_filepath + "dynamicvoronoi_" + std::to_string(downsampling_idx) + ".png";
        cv::imwrite(dynamicvoronoi_filename, out_dynamicvoronoi_corrected);
        // create boostvoronoi
        cv::Mat out_boostvoronoi;
        std::chrono::steady_clock::time_point begin_boostvoronoi = std::chrono::steady_clock::now();
        bool success = voronoi_generation::create_boost_voronoi(costmap_img_downsampled, out_boostvoronoi, false);
        std::chrono::steady_clock::time_point end_boostvoronoi = std::chrono::steady_clock::now();
        double secs_boostvoronoi = (std::chrono::duration_cast<std::chrono::microseconds>(end_boostvoronoi - begin_boostvoronoi).count()) / 1000000.0;
        times_boostvoronoi.push_back(secs_boostvoronoi);
        std::cout << "Time taken for boost voronoi generation: " << secs_boostvoronoi << std::endl;
        // overlay with freespace
        cv::Mat dist_img_narrow;
        cv::distanceTransform(~costmap_img_downsampled, dist_img_narrow, cv::DIST_L2, 3, CV_8UC1);
        // threshold with 4 times formation radius
        cv::threshold(dist_img_narrow, dist_img_narrow, factor * 80, 255, cv::THRESH_BINARY);
        dist_img_narrow.convertTo(dist_img_narrow, CV_8UC1);
        // bitwise or with voronoi image to combine both
        cv::bitwise_or(out_boostvoronoi, dist_img_narrow, out_boostvoronoi);

        std::string boostvoronoi_filename = base_filepath + "boostvoronoi_" + std::to_string(downsampling_idx) + ".png";
        cv::imwrite(boostvoronoi_filename, out_boostvoronoi);
        downsampling_idx++;
    }
    // save results
    std::ofstream out_file(base_filepath + "results.txt");
    if (out_file.is_open())
    {
        out_file << "dimension" << ", " << "dynamicvoronoi" << ", " << "boostvoronoi" << "\n";
        for (int idx = 0; idx < times_dynamicvoronoi.size(); idx++)
        {
            
            out_file << desired_dimensions.at(idx) << ", " << times_dynamicvoronoi.at(idx) << ", " << times_boostvoronoi.at(idx) << "\n";
        }
        out_file.close();
    }
    else
    {
        std::cerr << "Could not open file: " << base_filepath + "results.txt" << std::endl;
        return false;
    }
    std::cout << "===============" << std::endl;
}