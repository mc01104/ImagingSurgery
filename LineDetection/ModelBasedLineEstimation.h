# pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "ValveModel.h"


class ModelBasedLineEstimation
{
		double robot_position[3];
		double robot_velocity[3];
		double robot_predicted_position[3];
		double delta_t;

		ValveModel	valveModel;
		
		::cv::Mat	current_img;
		int			crop;

		::std::vector<::cv::Point>	pointsToFit;
		::std::vector<::cv::Point>	highProbPointsToFit;

		::cv::Vec4f		fittedLine;
		::cv::Vec2f		centroid;
		double			line_covariance;
		double			covariance_threshold;

		::cv::Vec4f		predictedLine;
		::cv::Vec2f		predictedcentroid;
		double			pred_line_covariance;

		int				channel_pixel_position_unrotated[2];
		double			inner_tube_rotation;
		int				channel_pixel_position_rotated[2];
		double			scaling_factor;
		::Eigen::Vector3d			projected_robot_position;
		::Eigen::Vector3d			centroid_position_3d;

		double			init_image_rotation;
		int				image_center;

		bool			line_detected;
		bool			update_model;

	public:

		ModelBasedLineEstimation();

		~ModelBasedLineEstimation();

		bool step(double robot_position[3], double robot_desired_velocity[3], const ::cv::Mat& img, double innerTubeRotation, 
			::cv::Vec4f& line, ::cv::Vec2f& centroid, bool update = true);

		bool stepBenchtop(double robot_position[3], double robot_desired_velocity[3], const ::cv::Mat& img, double innerTubeRotation, 
			::cv::Vec4f& line, ::cv::Vec2f& centroid, bool update = true);

		void resetModel();

		ValveModel getModel() {return this->valveModel; };

		double*	getCurrentPoint() {return this->robot_predicted_position;};
		void	getClosestPointOnCircle(double point[3]);
		bool	getTangent(double p1[3], double p2[3]);
		bool	getPredictedTangent(::cv::Vec4f& line);

	protected:
		void predict(double robot_position[3], double robot_desired_velocity[3]);

		void update(const ::cv::Mat& img);
		void updateBenchtop(const ::cv::Mat& img);

		void rejectOutliers();

		void addPointToModel();

		void computeResidualVariance();

		void computePredictedTangent();

		void computeClosestCirclePoint();

		void computePointsForFitting();
		void computePointsForFittingBenchtop();

		bool fitLine();

		bool checkLineFitting();

		void thresholdImageAllChannels(const ::cv::Mat& img,::cv::Mat& thresholded);
		void thresholdImageSynthetic(const ::cv::Mat& img,::cv::Mat& thresholded);

		bool RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3);

		void computeCentroid();

		void convertLineCentroidTo3DWorkspace(double tmp_point[3]);

		void computeVariance(); 

		void computePredictionCovariance();

		void rejectSmallAreaImageRegions(const ::cv::Mat& img, ::cv::Mat& output);

};