#include <matlab.h>


void matlabData(const std::vector<Eigen::Vector2f>& bezier_spline, const std::vector<Eigen::Vector2f>& first_derivative_bezier_spline, 
                const std::vector<Eigen::Vector2f>& second_derivative_bezier_spline, const std::vector<float>& curvature_bezier_spline,
                const std::vector<Eigen::Vector3f>& analyse) {
    // create Matlab data
    std::vector<Eigen::Vector2f> normalizedPointsX;
    std::vector<Eigen::Vector2f> normalizedPointsY;
    std::vector<Eigen::Vector2f> normalizedPointsXfirst;
    std::vector<Eigen::Vector2f> normalizedPointsYfirst;
    std::vector<Eigen::Vector2f> normalizedPointsXsecond;
    std::vector<Eigen::Vector2f> normalizedPointsYsecond;
    std::vector<Eigen::Vector2f> curvaturePoints;
    std::vector<float> cumulativeLengths = calculateCumulativeLengths(bezier_spline);

    for (size_t i = 0; i < bezier_spline.size(); ++i) {
        Eigen::Vector2f normalizedPointX(cumulativeLengths[i], bezier_spline[i].x());
        Eigen::Vector2f normalizedPointY(cumulativeLengths[i], bezier_spline[i].y());
        normalizedPointsX.push_back(normalizedPointX);
        normalizedPointsY.push_back(normalizedPointY);
    }

    for (size_t i = 0; i < first_derivative_bezier_spline.size(); ++i) {
        Eigen::Vector2f normalizedPointXfirst(cumulativeLengths[i], first_derivative_bezier_spline[i].x());
        Eigen::Vector2f normalizedPointYfirst(cumulativeLengths[i], first_derivative_bezier_spline[i].y());
        normalizedPointsXfirst.push_back(normalizedPointXfirst);
        normalizedPointsYfirst.push_back(normalizedPointYfirst);
    }

    for (size_t i = 0; i < second_derivative_bezier_spline.size(); ++i) {
        Eigen::Vector2f normalizedPointXsecond(cumulativeLengths[i], second_derivative_bezier_spline[i].x());
        Eigen::Vector2f normalizedPointYsecond(cumulativeLengths[i], second_derivative_bezier_spline[i].y());
        normalizedPointsXsecond.push_back(normalizedPointXsecond);
        normalizedPointsYsecond.push_back(normalizedPointYsecond);
    }

    for (size_t i = 0; i < curvature_bezier_spline.size(); ++i) {
        Eigen::Vector2f curvaturePoint(cumulativeLengths[i], curvature_bezier_spline[i]);
        curvaturePoints.push_back(curvaturePoint);;
    }
    // export Matlab data
    exportBezierSpline(bezier_spline, "/home/henrik/Dokumente/Studienarbeit/matlab_files/bezier_spline.csv");
    exportBezierSpline(first_derivative_bezier_spline, "/home/henrik/Dokumente/Studienarbeit/matlab_files/first_derivative_bezier_spline.csv");
    exportBezierSpline(second_derivative_bezier_spline, "/home/henrik/Dokumente/Studienarbeit/matlab_files/second_derivative_bezier_spline.csv");
    exportBezierSpline(normalizedPointsX, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsX.csv");
    exportBezierSpline(normalizedPointsY, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsY.csv");
    exportBezierSpline(normalizedPointsXfirst, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsXfirst.csv");
    exportBezierSpline(normalizedPointsYfirst, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsYfirst.csv");
    exportBezierSpline(normalizedPointsXsecond, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsXsecond.csv");
    exportBezierSpline(normalizedPointsYsecond, "/home/henrik/Dokumente/Studienarbeit/matlab_files/normalizedPointsYsecond.csv");
    exportBezierSpline(curvaturePoints, "/home/henrik/Dokumente/Studienarbeit/matlab_files/curvaturePoints.csv");
    // export3f(analyse, "/home/henrik/Dokumente/Studienarbeit/matlab_files/analyse_1523.csv"); // analyse 3 dimensional vector if necessary
}

void exportBezierSpline(const std::vector<Eigen::Vector2f>& spline, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Fehler beim Öffnen der Datei!" << std::endl;
        return;
    }
    for (const Eigen::Vector2f& point : spline) {
        file << point.x() << "," << point.y() << std::endl;
    }
    file.close();
}

void export3f(const std::vector<Eigen::Vector3f>& spline, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Fehler beim Öffnen der Datei!" << std::endl;
        return;
    }
    for (const Eigen::Vector3f& point : spline) {
        file << point.x() << "," << point.y() << "," << point.z() << std::endl;
    }
    file.close();
}

std::vector<float> calculateCumulativeLengths(const std::vector<Eigen::Vector2f>& points) {
    std::vector<float> cumulativeLengths(points.size());
    float totalLength = 0.0f;
    cumulativeLengths[0] = 0.0f;

    for (size_t i = 1; i < points.size(); ++i) {
        float segmentLength = (points[i] - points[i - 1]).norm();
        totalLength += segmentLength;
        cumulativeLengths[i] = totalLength;
    }

    for (float& length : cumulativeLengths) {
        length /= totalLength;
    }
    return cumulativeLengths;
}