#include <pure_voronoi_planner/dynamicvoronoi_interface.h>

namespace voronoi_generation
{

cv::Mat create_dynamicvoronoi(const cv::Mat& img_gray, int free_cell_threshold)
{
    cv::Mat drawing(img_gray);
    /*
    cv::Mat canny_output;
    cv::Canny(img_gray, canny_output, 100, 200);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat(canny_output.size(), CV_8UC1, cv::Scalar(255));
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(0);
        cv::drawContours(drawing, contours, (int)i, color, 1, cv::LINE_4, hierarchy, 0);
    }
    */
    // cv::imshow("Contours", drawing);
    // cv::imwrite("map_contours.png", drawing);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    bool** grid_map_for_dynamicvoronoi;
    DynamicVoronoi voronoi;
    grid_map_for_dynamicvoronoi = new bool*[drawing.rows];
    for (int x = 0; x < (drawing.rows); x++)
    {
        grid_map_for_dynamicvoronoi[x] = new bool[drawing.cols];
    }
    for (int y = drawing.cols - 1; y >= 0; y--)
    {
        for (int x = 0; x < drawing.rows; x++)
        {
            // TODO: only get differences and update voronoi accordingly
            if (y == 0 || y == drawing.cols - 1 || x == 0 || x == drawing.rows - 1)
            {
                grid_map_for_dynamicvoronoi[x][y] = true;
            }
            else
            {
                grid_map_for_dynamicvoronoi[x][y] = drawing.at<uchar>(x, y) <= free_cell_threshold;
            }
        }
    }
    voronoi.initializeMap(drawing.rows, drawing.cols, grid_map_for_dynamicvoronoi);
    // std::cout << "VoronoiGlobalPlanner voronoi instance initialized!" << std::endl;
    // std::cout << "With map size " << drawing.rows * drawing.cols << std::endl;
    voronoi.update();  // update distance map and Voronoi diagram
    // std::cout << "VoronoiGlobalPlanner voronoi instance updated!" << std::endl;
    voronoi.prune();  // prune the Voronoi
    // std::cout << "VoronoiGlobalPlanner voronoi instance pruned!" << std::endl;


    cv::Mat out_voronoi(drawing.rows, drawing.cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < drawing.rows; i++)
    {
        for (int j = 0; j < drawing.cols; j++)
        {
            if (voronoi.isVoronoi(i, j))
            {
                out_voronoi.at<uchar>(i, j) = 255;
            }
        }
    }

    // cv::imwrite("voronoi_out.png", out_voronoi);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double secs = (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0;
    // std::cout << "Time taken for dynamicvoronoi generation: " << secs << std::endl;
    // std::cout << "===============" << std::endl;

    // for (int x = 0; x < drawing.rows; x++)
    //     delete[] grid_map_for_dynamicvoronoi[x];
    // delete[] grid_map_for_dynamicvoronoi;
    // std::cout << "Cleanup done" << std::endl;
    // std::cout << "===============" << std::endl;
    return out_voronoi;
}
} // namspace voronoi_generation
