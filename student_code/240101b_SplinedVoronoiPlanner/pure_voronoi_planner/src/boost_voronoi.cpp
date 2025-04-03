#include <pure_voronoi_planner/boost_voronoi.h>

namespace voronoi_generation
{

bool is_surrounding_occupied(const cv::Mat& input_img, cv::Point2i pixel, int search_size)
{
    /*
    if (pixel.x < search_size || pixel.x > input_img.rows - search_size - 1 || pixel.y < search_size ||
        pixel.y > input_img.cols - search_size - 1)
    {
        return true;
    }
    int top_left_x = pixel.x - search_size;
    int top_left_y = pixel.y - search_size;
    // int bot_right_x = cv::min(input_img.rows - 1, pixel.x + search_size);
    // int bot_right_y = cv::min(input_img.cols - 1, pixel.y + search_size);
    // int width = bot_right_x - top_left_x;
    // int height = bot_right_y - top_left_y;
    cv::Rect2i roi(top_left_x, top_left_y, search_size, search_size);
    cv::Mat img_roi = input_img(roi);
    if (cv::countNonZero(img_roi) > 0)
    {
        return true;
    }
    return false;
    */
    for (int xoff = -search_size; xoff <= search_size; ++xoff)
    {
        for (int yoff = -search_size; yoff <= search_size; ++yoff)
        {
            cv::Point2i surr_pixel = pixel + cv::Point2i(xoff, yoff);
            // Check if the pixel is within a map
            if (surr_pixel.x < 0 || surr_pixel.x >= input_img.cols || surr_pixel.y < 0 ||
                surr_pixel.y >= input_img.rows)
            {
                continue;
            }
            if (input_img.at<uchar>(surr_pixel) != 0)
            {
                return true;
            }
        }
    }
    return false;
}

void draw_edges(const voronoi_diagram<double>& vd, const cv::Mat& input_img, cv::Mat& output_img, const std::vector<cv::Point2i>& points)
{
    cv::RNG rng(12345);;
    bool internal_edges_only = true;
    // cv::Mat img_out(input_img);
    output_img = cv::Mat(input_img.size(), CV_8UC1, cv::Scalar(127));
    cv::Mat all_lines_img = cv::Mat(input_img.size(), CV_8UC1, cv::Scalar(0));
    int num_edges = vd.edges().size();
    int count_curved = 0;
    for (auto edge : vd.edges())
    {
        if (!edge.is_primary())
        {
            continue;
        }
        if (internal_edges_only && !edge.is_finite())
        {
            continue;
        }
        // std::vector<cv::Point2i> samples;
        if (!edge.is_finite())
        {
            // clip_infinite_edge(*it, &samples);
        }
        else
        {
            cv::Point2i vertex0(edge.vertex0()->x(), edge.vertex0()->y());
            cv::Point2i vertex1(edge.vertex1()->x(), edge.vertex1()->y());
            // dont show all edges which are curved or which endpoints are outside the image or occupied
            if (edge.is_curved())
            {
                count_curved++;
                continue;
            }
            if (vertex0.x > output_img.cols || vertex0.x < 0 || vertex0.y < 0 || vertex0.y > output_img.rows)
            {
                continue;
            }
            if (vertex1.x > output_img.cols || vertex1.x < 0 || vertex1.y < 0 || vertex1.y > output_img.rows)
            {
                continue;
            }
            cv::line(all_lines_img, vertex0, vertex1, cv::Scalar(rng.uniform(0,255)), 2, 8);
            if (is_surrounding_occupied(input_img, vertex0, 2))
            {
                continue;
            }
            if (is_surrounding_occupied(input_img, vertex1, 2))
            {
                continue;
            }
            // samples.push_back(vertex0);
            // samples.push_back(vertex1);
            cv::line(output_img, vertex0, vertex1, cv::Scalar(255), 1, 8);
            // img_out.at<uchar>(vertex0) = 255;
            // img_out.at<uchar>(vertex1) = 255;
        }
    }

    BOOST_FOREACH (const cv::Point2i point, points)
    {
        output_img.at<uchar>(point) = 0;
    }
    // cv::bitwise_and(output_img, ~input_img, output_img);
    cv::imwrite("/home/soenke/voronoi_all_lines.png", all_lines_img);
}

int iterate_primary_edges1(const voronoi_diagram<double>& vd)
{
    int result = 0;
    for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
    {
        if (it->is_primary())
        {
            ++result;
            it->rot_next();
        }
    }
    return result;
}

void get_contour_points(const cv::Mat& img_binary_contours, std::vector<cv::Point2i>& points)
{
    cv::Mat non_zero_coordinates;
    cv::findNonZero(img_binary_contours, non_zero_coordinates);
    for (int i = 0; i < non_zero_coordinates.total(); i++)
    {
        points.push_back(non_zero_coordinates.at<cv::Point2i>(i));
    }
}

void get_contour_segments(const std::vector<std::vector<cv::Point>>& contours, std::vector<Segment>& segments)
{
    for (auto contour : contours)
    {
        // segments.push_back();
    }
}

bool create_boost_voronoi(const cv::Mat& img_binary, cv::Mat& voronoi_img, bool debug_output = false)
{
    if (img_binary.empty())
    {
        ROS_ERROR("Got empty image, aborting!");
        return false;
    }
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    cv::Mat img_denoised(img_binary);
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // cv::morphologyEx(img_binary, img_denoised, cv::MORPH_CLOSE, kernel);
    // cv::morphologyEx(img_denoised, img_denoised, cv::MORPH_OPEN, kernel);
    cv::Mat canny_output;
    cv::Canny(img_denoised, canny_output, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat img_binary_contours = cv::Mat(canny_output.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat img_binary_contours_out = cv::Mat(canny_output.size(), CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(255);
        cv::drawContours(img_binary_contours, contours, (int)i, color, 1, cv::LINE_4, hierarchy, 0);
        cv::drawContours(img_binary_contours_out, contours, (int)i, color, 3, cv::LINE_4, hierarchy, 0);
    }

    std::vector<cv::Point2i> points;
    get_contour_points(img_binary_contours, points);
    std::vector<Segment> segments;
    // get_contour_segments(contours, segments);
    std::chrono::steady_clock::time_point preprocess_end = std::chrono::steady_clock::now();

    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
    std::chrono::steady_clock::time_point voronoi_end = std::chrono::steady_clock::now();

    draw_edges(vd, img_binary, voronoi_img, points);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    if (debug_output)
    {
        std::string debug_img_dir_path = ros::package::getPath("voronoi_generation") + "/images/";
        cv::imwrite(debug_img_dir_path + "boost_01_denoised_input.png", img_denoised);
        cv::imwrite(debug_img_dir_path + "boost_02_contours.png", img_binary_contours_out);
        cv::imwrite(debug_img_dir_path + "boost_03_output.png", voronoi_img);
        std::chrono::steady_clock::time_point debug_end = std::chrono::steady_clock::now();
        double preprocess_secs =
            (std::chrono::duration_cast<std::chrono::microseconds>(preprocess_end - begin).count()) / 1000000.0;
        double voronoi_secs =
            (std::chrono::duration_cast<std::chrono::microseconds>(voronoi_end - preprocess_end).count()) / 1000000.0;
        double postprocess_secs =
            (std::chrono::duration_cast<std::chrono::microseconds>(end - voronoi_end).count()) / 1000000.0;
        double debug_secs =
            (std::chrono::duration_cast<std::chrono::microseconds>(debug_end - end).count()) / 1000000.0;
        double total_secs =
            (std::chrono::duration_cast<std::chrono::microseconds>(debug_end - begin).count()) / 1000000.0;
        ROS_INFO_STREAM("Time taken for preprocessing: " << preprocess_secs);
        ROS_INFO_STREAM("Time taken for boost voronoi: " << voronoi_secs);
        ROS_INFO_STREAM("Time taken for postprocessing: " << postprocess_secs);
        ROS_INFO_STREAM("Time taken for debug writing: " << debug_secs);
        ROS_INFO_STREAM("Time taken total: " << total_secs);
    }
    return true;
}
} // namespace voronoi_generation
