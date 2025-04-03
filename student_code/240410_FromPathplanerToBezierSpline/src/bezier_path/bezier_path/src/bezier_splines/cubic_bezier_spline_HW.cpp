#include <bezier_splines/cubic_bezier_spline_HW.h>

namespace bezier_splines
{
	CubicBezierSplines::CubicBezierSplines()
		: BaseBezierSpline(3) {}
	CubicBezierSplines::CubicBezierSplines(visualization_helper::VisualizationHelper *visu_helper)
		: BaseBezierSpline(3, visu_helper) {}
	CubicBezierSplines::CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f end_pose)
		: BaseBezierSpline(3, waypoints, start_pose, end_pose) {} // calls 3
	CubicBezierSplines::CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
										   visualization_helper::VisualizationHelper *visu_helper,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f end_pose)
		: BaseBezierSpline(3, waypoints, visu_helper, start_pose, end_pose) {} // calls 4
	CubicBezierSplines::CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
										   size_t j,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f start_tangent,
										   Eigen::Vector2f end_pose,
										   Eigen::Vector2f end_tangent,
										   Eigen::Vector2f goalTangent)
		: BaseBezierSpline(3,
						   waypoints,
						   j,
						   start_pose,
						   start_tangent,
						   end_pose,
						   end_tangent,
						   goalTangent) {}// calls 5 auf
	CubicBezierSplines::CubicBezierSplines(std::vector<Eigen::Vector2f> waypoints,
										   size_t j,
										   visualization_helper::VisualizationHelper *visu_helper,
										   Eigen::Vector2f start_pose,
										   Eigen::Vector2f start_tangent,
										   Eigen::Vector2f end_pose,
										   Eigen::Vector2f end_tangent,
										   Eigen::Vector2f goalTangent)
		: BaseBezierSpline(3,
						   waypoints,
						   j,
						   visu_helper,
						   start_pose, 
						   start_tangent,
						   end_pose, 
						   end_tangent,
						   goalTangent) {} // calls 6

	void CubicBezierSplines::calcControlPoints()
    {
		this->control_points_[1] = (1.0 / 3.0) * this->getStartTangent() +
								   this->control_points_.front();
		this->control_points_[2] = -(1.0 / 3.0) * this->getEndTangent() +
								   this->control_points_.back();
	}

    Eigen::Vector2f CubicBezierSplines::calcPointOnBezierSpline(float iterator)
    {
        Eigen::Matrix<float, 4, 4> bezier_basis_matrix;
        Eigen::Matrix<float, 1, 4> iterator_matrix;

        Eigen::Matrix<float, 6, 1> bernstein_polynom_vector;

		for(int counter = this->BEZIER_DEGREE; counter >= 0; counter--)
		{
			bernstein_polynom_vector[counter] = this->calcBinomialCoefficient(this->BEZIER_DEGREE, counter) *
												std::pow(iterator, counter) *
												std::pow((1 - iterator), this->BEZIER_DEGREE - counter);
		}
		
        Eigen::Vector2f result_vector;
        result_vector << 0, 0;
        for(int counter = 0; counter <= this->BEZIER_DEGREE; counter++)
        {
            result_vector = result_vector + (bernstein_polynom_vector[counter] * this->control_points_[counter]);
        }
        return result_vector;
    }

	Eigen::Vector2f CubicBezierSplines::calcFirstDerivativeValue(float iterator)
	{
		Eigen::Vector2f first_derivative_value;

		first_derivative_value = 3 * std::pow(1 - iterator, 2) * (this->control_points_[1] - this->control_points_.front()) +
								 6 * (1 - iterator) * iterator * (this->control_points_[2] - this->control_points_[1]) +
								 3 * std::pow(iterator, 2) * (this->control_points_.back() - this->control_points_[2]);

		return first_derivative_value;								 
	}

	Eigen::Vector2f CubicBezierSplines::calcSecondDerivativeValue(float iterator)
	{
		Eigen::Vector2f second_derivative_value;
				
		second_derivative_value = 6 * (1 - iterator) *
									  (this->control_points_.front() - 2 * this->control_points_[1] + this->control_points_[2]) +
								  6 * iterator *
									  (this->control_points_[1] - 2 * this->control_points_[2] + this->control_points_.back());
		return second_derivative_value;
	}

	Eigen::Vector2f CubicBezierSplines::calcCurrStartSecondDerivative() 
	{
		return {0,0};
	}

	Eigen::Vector2f CubicBezierSplines::calcCurrEndSecondDerivative() 
	{
		return {0,0};
	}

	std::shared_ptr<BaseBezierSpline> CubicBezierSplines::clone() {
        auto new_spline = std::make_shared<CubicBezierSplines>();
		new_spline->setStartPose(this->getStartPose());
		new_spline->setEndPose(this->getEndPose());
		new_spline->setStartTangent(this->getStartTangent());
		new_spline->setEndTangent(this->getEndTangent());
		return new_spline;
    }

	void CubicBezierSplines::printInfo()
	{
		BaseBezierSpline::printInfo();
	}
}