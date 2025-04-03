#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>

#include <bezier_splines/base_bezier_spline_HW.h>
#include <visualization_helper/visualization_helper_HW.h>


namespace bezier_splines
{
	class CubicBezierSplines : public BaseBezierSpline
    {
        public:
			// using BaseBezierSpline::BaseBezierSpline; // Use constructors of base class
			CubicBezierSplines();
			CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper);
			CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
							   Eigen::Vector2f start_pose,
							   Eigen::Vector2f end_pose);
			CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
							   visualization_helper::VisualizationHelper *visu_helper,
							   Eigen::Vector2f start_pose,
							   Eigen::Vector2f end_pose);
			CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
							   size_t j,
							   Eigen::Vector2f start_pose,
							   Eigen::Vector2f start_tangent,
							   Eigen::Vector2f end_pose,
							   Eigen::Vector2f end_tangent,
							   Eigen::Vector2f goalTangent);
			CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
							   size_t j,
							   visualization_helper::VisualizationHelper *visu_helper,
							   Eigen::Vector2f start_pose,
							   Eigen::Vector2f start_tangent,
							   Eigen::Vector2f end_pose,
							   Eigen::Vector2f end_tangent,
							   Eigen::Vector2f goalTangent);

#pragma region BezierMethods
			void calcControlPoints() override;

			Eigen::Vector2f calcPointOnBezierSpline(float iterator) override;
			std::shared_ptr<BaseBezierSpline> clone() override;

			Eigen::Vector2f calcFirstDerivativeValue(float iterator) override;
			Eigen::Vector2f calcSecondDerivativeValue(float iterator) override;
			Eigen::Vector2f calcCurrStartSecondDerivative() override;
			Eigen::Vector2f calcCurrEndSecondDerivative() override;
			#pragma endregion

			void printInfo() override;

    };
}