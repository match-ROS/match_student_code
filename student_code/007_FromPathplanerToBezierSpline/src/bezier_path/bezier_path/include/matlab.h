#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <string> 


void exportBezierSpline(const std::vector<Eigen::Vector2f>& spline, const std::string& filename);
void export3f(const std::vector<Eigen::Vector3f>& spline, const std::string& filename);
std::vector<float> calculateCumulativeLengths(const std::vector<Eigen::Vector2f>& points);
void matlabData(const std::vector<Eigen::Vector2f>& bezier_spline, const std::vector<Eigen::Vector2f>& first_derivative_bezier_spline, 
                const std::vector<Eigen::Vector2f>& second_derivative_bezier_spline, const std::vector<float>& curvature_bezier_spline,
                const std::vector<Eigen::Vector3f>& analyse);