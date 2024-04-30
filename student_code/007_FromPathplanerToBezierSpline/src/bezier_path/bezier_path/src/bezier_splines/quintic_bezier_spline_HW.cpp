#include <bezier_splines/quintic_bezier_spline_HW.h>

namespace bezier_splines
{
	QuinticBezierSplines::QuinticBezierSplines() // calls 1
		: BaseBezierSpline(5) {}
	QuinticBezierSplines::QuinticBezierSplines(visualization_helper::VisualizationHelper *visu_helper) // calls 2 -> 1
		: BaseBezierSpline(5, visu_helper) {}
	QuinticBezierSplines::QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f end_pose) // calls 3 -> 1
		: BaseBezierSpline(5, waypoints, start_pose, end_pose) {}
	QuinticBezierSplines::QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
											   visualization_helper::VisualizationHelper *visu_helper,
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f end_pose) // calls 4 -> 3 -> 1
		: BaseBezierSpline(5, waypoints, visu_helper, start_pose, end_pose) {}
	QuinticBezierSplines::QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
											   size_t j,
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f start_tangent,
											   Eigen::Vector2f end_pose,
											   Eigen::Vector2f end_tangent,
											   Eigen::Vector2f goalTangent) //calls 5 -> 3 -> 1
		: BaseBezierSpline(5,
						   waypoints,
						   j,
						   start_pose,
						   start_tangent,
						   end_pose,
						   end_tangent,
						   goalTangent) {}
	QuinticBezierSplines::QuinticBezierSplines(std::vector<Eigen::Vector2f> waypoints,
											   size_t j,
											   visualization_helper::VisualizationHelper *visu_helper,											   
											   Eigen::Vector2f start_pose,
											   Eigen::Vector2f start_tangent,
											   Eigen::Vector2f end_pose,
											   Eigen::Vector2f end_tangent,
											   Eigen::Vector2f goalTangent) // calls 6 -> 5 -> 3 -> 1
		: BaseBezierSpline(5,
						   waypoints,
						   j,
						   visu_helper,
						   start_pose,
						   start_tangent,
						   end_pose,
						   end_tangent,
						   goalTangent) {}

#pragma region BezierMethods
    void QuinticBezierSplines::calcControlPoints()
    {
		this->control_points_[1] = (1.0 / 5.0) * this->getStartTangent() + this->control_points_.front();
        this->control_points_[4] = -(1.0 / 5.0) * this->getEndTangent() + this->control_points_.back();

		std::shared_ptr<bezier_splines::CubicBezierSplines> previous_spline = nullptr; // initialise non member variables
		std::shared_ptr<bezier_splines::CubicBezierSplines> current_spline = nullptr;
		std::shared_ptr<bezier_splines::CubicBezierSplines> next_spline = nullptr;

		if (this->previous_spline_ != nullptr)
		{
			previous_spline = this->createCubicBezierSpline(this->previous_spline_);
		}
		current_spline = this->createCubicBezierSpline(std::make_shared<bezier_splines::QuinticBezierSplines>(*this));

		if(this->next_spline_ != nullptr)
		{
			next_spline = this->createCubicBezierSpline(this->next_spline_);
		}

		if(this->previous_spline_ != nullptr) //previous spline exists
		{
			previous_spline->setNextSpline(current_spline);
			current_spline->setPreviousSpline(previous_spline);
			previous_spline->setStartTangent(this->previous_spline_->getStartTangent());	
			previous_spline->setEndTangent(this->previous_spline_->getEndTangent());	
			previous_spline->calcControlPoints(); // Everything is set for the previous spline. Control points can be calculated			
		}
		else // first segment
		{
			current_spline->setStartTangent(this->start_tangent_);
		}

		if(this->next_spline_ != nullptr) //next spline exists
		{
			current_spline->setNextSpline(next_spline);
			next_spline->setPreviousSpline(current_spline);
			next_spline->setStartTangent(this->next_spline_->getStartTangent());
			next_spline->setEndTangent(this->next_spline_->getEndTangent());
			next_spline->calcControlPoints(); // Everything is set for the next spline. Control points can be calculated
		}
		else // last segment
		{
			current_spline->setEndTangent(this->end_tangent_);
		}

		current_spline->calcControlPoints(); // Everything is set for the current spline. Control points can be calculated

		Eigen::Vector2f CurrStartSecDer;

		if(previous_spline != nullptr)
		{
			CurrStartSecDer = this->calcCurrStartSecondDerivative();
		}
		else 
		{
			CurrStartSecDer = {0,0};
		}
		this->control_points_[2] = (1.0 / 20.0) * CurrStartSecDer + 2.0 * this->control_points_[1] - this->control_points_.front();

		Eigen::Vector2f CurrEndSecDer;
		if(next_spline_ != nullptr)
		{
			CurrEndSecDer = this->calcCurrEndSecondDerivative();
		}
		else 
		{
			CurrEndSecDer = current_spline->calcSecondDerivativeValue(1.0);
		}
		this->control_points_[3] = (1.0 / 20.0) * CurrEndSecDer + 2.0 * this->control_points_[4] - this->control_points_.back();
		j_++;
		// std::vector<Eigen::Vector2f> previousCP = this->getPreviousControlPoints();
		// if(previous_spline_!=nullptr) {
		// 	std::cout<<"CP0 prev: "<<previousCP[0].x()<<", "<<previousCP[0].y()<<std::endl;
		// 	std::cout<<"CP1 prev: "<<previousCP[1].x()<<", "<<previousCP[1].y()<<std::endl;
		// 	std::cout<<"CP2 prev: "<<previousCP[2].x()<<", "<<previousCP[2].y()<<std::endl;
		// 	std::cout<<"CP3 prev: "<<previousCP[3].x()<<", "<<previousCP[3].y()<<std::endl;
		// 	std::cout<<"CP4 prev: "<<previousCP[4].x()<<", "<<previousCP[4].y()<<std::endl;
		// 	std::cout<<"CP5 prev: "<<previousCP[5].x()<<", "<<previousCP[5].y()<<std::endl;
		// 	std::cout<<std::endl;
		// }
		// std::vector<Eigen::Vector2f> nextCP = this->getNextControlPoints();
		// if(next_spline_!=nullptr) {
		// 	std::cout<<"CP0 next: "<<nextCP[0].x()<<", "<<nextCP[0].y()<<std::endl;
		// 	std::cout<<"CP1 next: "<<nextCP[1].x()<<", "<<nextCP[1].y()<<std::endl;
		// 	std::cout<<"CP2 next: "<<nextCP[2].x()<<", "<<nextCP[2].y()<<std::endl;
		// 	std::cout<<"CP3 next: "<<nextCP[3].x()<<", "<<nextCP[3].y()<<std::endl;
		// 	std::cout<<"CP4 next: "<<nextCP[4].x()<<", "<<nextCP[4].y()<<std::endl;
		// 	std::cout<<"CP5 next: "<<nextCP[5].x()<<", "<<nextCP[5].y()<<std::endl;
		// 	std::cout<<std::endl;
		// }
    }

	std::shared_ptr<BaseBezierSpline> QuinticBezierSplines::clone() {
        auto new_spline = std::make_shared<QuinticBezierSplines>();
		new_spline->setStartPose(this->getStartPose());
		new_spline->setEndPose(this->getEndPose());
		new_spline->setStartTangent(this->getStartTangent());
		new_spline->setEndTangent(this->getEndTangent());
		return new_spline;
    }


    Eigen::Vector2f QuinticBezierSplines::calcPointOnBezierSpline(float iterator) // iterator = t, between 0 and 1
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

	Eigen::Vector2f QuinticBezierSplines::calcFirstDerivativeValue(float iterator)
	{
		Eigen::Vector2f first_derivative_value =
			5 * std::pow(1 - iterator, 4) * (this->control_points_[1] - this->control_points_.front()) +
			20 * std::pow(1 - iterator, 3) * iterator * (this->control_points_[2] - this->control_points_[1]) +
			30 * std::pow(1 - iterator, 2) * std::pow(iterator, 2) * (this->control_points_[3] - this->control_points_[2]) +
			20 * (1 - iterator) * std::pow(iterator, 3) * (this->control_points_[4] - this->control_points_[3]) +
			5 * std::pow(iterator, 4) * (this->control_points_.back() - this->control_points_[4]);

		return first_derivative_value;									   
	}

	Eigen::Vector2f QuinticBezierSplines::calcSecondDerivativeValue(float iterator)
	{
		Eigen::Vector2f second_derivative_value =
			20 * std::pow(1 - iterator, 3) * (this->control_points_.front() - 2 * this->control_points_[1] + this->control_points_[2]) +
			60 * std::pow(1 - iterator, 2) * iterator * (this->control_points_[1] - 2 * this->control_points_[2] + this->control_points_[3]) +
			60 * (1 - iterator) * std::pow(iterator, 2) * (this->control_points_[2] - 2 * this->control_points_[3] + this->control_points_[4]) +
			20 * std::pow(iterator, 3) * (this->control_points_[3] - 2 * this->control_points_[4] + this->control_points_.back());

		return second_derivative_value;												  
	}

	Eigen::Vector2f QuinticBezierSplines::calcCurrEndSecondDerivative() 
	{
		Eigen::Vector2f currEndSecondDerivative;
		Eigen::Vector2f A;
		Eigen::Vector2f B;
		if(j_ < waypoints_.size()-1) {
			A = 6 * waypoints_[j_-1] + 2 * this->getStartTangent() + 4 * this->getEndTangent() - 6 * waypoints_[j_];
			B = -6 * waypoints_[j_] - 2 * next_spline_->getStartTangent() - 4 * next_spline_->getEndTangent() + 6 * waypoints_[j_+1];
			currEndSecondDerivative = (A + B) / 2;
		}
		else {
			currEndSecondDerivative={0,0}; //wird nicht aktiviert
		}		
		return currEndSecondDerivative;
	}

	Eigen::Vector2f QuinticBezierSplines::calcCurrStartSecondDerivative() 
	{
		Eigen::Vector2f currStartSecondDerivative;
		Eigen::Vector2f A;
		Eigen::Vector2f B;
		if(previous_spline_ != nullptr) {
			A = 6 * waypoints_[j_-2] + 2 * previous_spline_->getStartTangent() + 4 * previous_spline_->getEndTangent() - 6 * waypoints_[j_-1];
			B = -6 * waypoints_[j_-1] - 2 * this->getStartTangent() - 4 * this->getEndTangent() + 6 * waypoints_[j_];
			currStartSecondDerivative = (A + B) / 2;
		}
		else {
			currStartSecondDerivative={0,0};
		}
		return currStartSecondDerivative;
	}

	#pragma endregion

	std::shared_ptr<bezier_splines::CubicBezierSplines> QuinticBezierSplines::createCubicBezierSpline(std::shared_ptr<bezier_splines::BaseBezierSpline> base_spline)
	{
		std::shared_ptr<bezier_splines::CubicBezierSplines> cubic_spline =
			std::make_shared<bezier_splines::CubicBezierSplines>(base_spline->getWaypoints(), // change
																 base_spline->getJ(),
																 base_spline->getStartPose(),
																 base_spline->getStartTangent(),
																 base_spline->getEndPose(),
																 base_spline->getEndTangent(),
																 base_spline->getGoalTangent());
		return cubic_spline;
	}

	void QuinticBezierSplines::printInfo()
	{
		BaseBezierSpline::printInfo();
	}
}