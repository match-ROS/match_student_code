#pragma once

#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <math.h>

#include <visualization_helper/visualization_helper_HW.h>
#include <bezier_splines/base_bezier_spline_HW.h>
#include <bezier_splines/cubic_bezier_spline_HW.h>

namespace bezier_splines
{
	class QuinticBezierSplines : public BaseBezierSpline
    {
		public:
			// using BaseBezierSpline::BaseBezierSpline; // Use constructors of base class
			#pragma region Constructors
			QuinticBezierSplines();
			QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper);
			QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
								 Eigen::Vector2f start_pose,
								 Eigen::Vector2f end_pose);
			QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
								 visualization_helper::VisualizationHelper *visu_helper,
								 Eigen::Vector2f start_pose,
								 Eigen::Vector2f end_pose);
			QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
								 size_t j, // number of segment
								 Eigen::Vector2f start_pose,
								 Eigen::Vector2f start_tangent,
								 Eigen::Vector2f end_pose,
								 Eigen::Vector2f end_tangent,
								 Eigen::Vector2f goalTangent);
			QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
								 size_t j,
								 visualization_helper::VisualizationHelper *visu_helper,
								 Eigen::Vector2f start_pose,
								 Eigen::Vector2f start_tangent,
								 Eigen::Vector2f end_pose,
								 Eigen::Vector2f end_tangent,
								 Eigen::Vector2f goalTangent);
			#pragma endregion


			#pragma region BezierMethods
			void calcControlPoints() override;

            Eigen::Vector2f calcPointOnBezierSpline(float iterator) override;
			std::shared_ptr<BaseBezierSpline> clone() override;
			Eigen::Vector2f calcFirstDerivativeValue(float iterator) override;
			Eigen::Vector2f calcSecondDerivativeValue(float iterator) override;		
			Eigen::Vector2f calcCurrEndSecondDerivative() override;
			Eigen::Vector2f calcCurrStartSecondDerivative() override;				

			#pragma endregion

			std::shared_ptr<bezier_splines::CubicBezierSplines> createCubicBezierSpline(std::shared_ptr<bezier_splines::BaseBezierSpline> base_spline);

			void printInfo() override;

    };
}