#include "stdafx.h"
#include "Utilities.h"
#include "ModelBasedLineEstimation.h"
#include "HTransform.h"

ModelBasedLineEstimation::ModelBasedLineEstimation():
	delta_t(1.0/46),
	crop(15),
	covariance_threshold(30),				// TO TUNE!!!
	init_image_rotation(-90),
	image_center(125),
	pred_line_covariance(0),
	line_detected(false)	,
	scaling_factor(26.27),
	update_model(true)
{
	for(int i = 0; i < 3; ++i)
		robot_position[i] = robot_velocity[i] = robot_predicted_position[i] = 0;

	// mask the working channel
	// scope 1
	channel_pixel_position_unrotated[0] = 149;
	channel_pixel_position_unrotated[1] = 29;
	// scope 2
	channel_pixel_position_unrotated[0] = 149;
	channel_pixel_position_unrotated[1] = 215;
	// scope 3
	channel_pixel_position_unrotated[0] = 115;
	channel_pixel_position_unrotated[1] = 47;
	// scope 4
	channel_pixel_position_unrotated[0] = 116;
	channel_pixel_position_unrotated[1] = 43;


}

ModelBasedLineEstimation::~ModelBasedLineEstimation()
{
	delete[] robot_position;
	delete[] robot_velocity;
	delete[] robot_predicted_position;
}


bool 
ModelBasedLineEstimation::step(double robot_position[3], double robot_desired_velocity[3], const ::cv::Mat& img, double innerTubeRotation, ::cv::Vec4f& line, ::cv::Vec2f& centroid_out, bool update)
{
	this->inner_tube_rotation = innerTubeRotation;
	this->update_model = update;
	// using the current robot position and the velocity from the visual servoing controller -> predict when the line should be in the next iteration
	this->predict(robot_position, robot_desired_velocity);

	// use the image to compute the line
	this->update(img);
	
	if (!this->line_detected)
		return false;
	line = this->fittedLine;

	centroid_out[0] = this->centroid[0];
	centroid_out[1] = this->centroid[1];

	//temporary!!!!
	this->predictedcentroid[0] = this->centroid[0];
	this->predictedcentroid[1] = this->centroid[1];

	this->predictedLine[2] = this->centroid[0];
	this->predictedLine[3] = this->centroid[1];

	this->computeResidualVariance();


	return this->checkLineFitting();
}

void 
ModelBasedLineEstimation::predict(double robot_position[3], double robot_desired_velocity[3])
{
	memcpy(this->robot_position, robot_position, 3 * sizeof(double));
	memcpy(this->robot_velocity, robot_desired_velocity, 3 * sizeof(double));

	for (int i = 0; i < 3; ++i)
		this->robot_predicted_position[i] = this->robot_position[i] + this->robot_velocity[i] * delta_t;

	if (this->valveModel.isInitialized())
	{
		// not sure if this function is correct
		this->computePredictedTangent();
		//this->computeClosestCirclePoint();

		this->computePredictionCovariance();
	}
}

void 
ModelBasedLineEstimation::update(const ::cv::Mat& img)
{
	img.copyTo(this->current_img);

	//this->computePointsForFitting();
	this->computePointsForFittingWire();
	//this->computePointsForFittingNew();


	if (false)//this->valveModel.isInitialized())
		this->rejectOutliers();
	else
	{
		this->highProbPointsToFit.resize(this->pointsToFit.size());
		::std::copy(this->pointsToFit.begin(), this->pointsToFit.end(), this->highProbPointsToFit.begin());
	}

	this->line_detected = this->fitLineHough();
	//this->line_detected = this->fitLine();
	if (this->checkLineFitting() && this->update_model)
		this->addPointToModel();
}

// =========OK=============//
void 
ModelBasedLineEstimation::computePointsForFitting()
{
	::cv::Mat img_crop = this->current_img(::cv::Rect(this->crop, this->crop, this->current_img.cols - 2 * this->crop, this->current_img.rows - 2 * this->crop));

    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(this->current_img.size(), CV_8UC1);

	this->thresholdImageAllChannels(this->current_img, thresholded);

	thresholded.convertTo(thresholded_binary,CV_8UC1);
	
    ::cv::findNonZero(thresholded_binary, this->pointsToFit);

	::cv::Mat	 output;
	::cv::cvtColor(thresholded, output, CV_GRAY2BGR);

	::cv::imshow("thresholded", output);
	::cv::waitKey(1);
}

void 
ModelBasedLineEstimation::rejectSmallAreaImageRegions(const ::cv::Mat& img, ::cv::Mat& output)
{
	int numCC = cv::connectedComponents(img, output);
	::cv::imshow("connected components", output);
}

bool
ModelBasedLineEstimation::fitLine()
{

	if (this->highProbPointsToFit.size() > 50)
	{
        ::cv::fitLine(this->highProbPointsToFit, this->fittedLine, CV_DIST_L2, 0, 0.01, 0.01);

		this->computeCentroid();

		::Eigen::Matrix3d rot1 = RotateZ(0*this->init_image_rotation * M_PI/180.0 - this->inner_tube_rotation);
		::Eigen::Vector3d predTang;

        ::cv::line( this->current_img, ::cv::Point(fittedLine[2],fittedLine[3]), ::cv::Point(fittedLine[2]+fittedLine[0]*100,fittedLine[3]+fittedLine[1]*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
        ::cv::line( this->current_img, ::cv::Point(fittedLine[2],fittedLine[3]), ::cv::Point(fittedLine[2]+fittedLine[0]*(-100),fittedLine[3]+fittedLine[1]*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::circle(this->current_img, ::cv::Point(this->centroid[0], centroid[1]), 5, ::cv::Scalar(255,0,0));

		return true;
	}

 	return false;
}

void ModelBasedLineEstimation::computeCentroid()
{
	float distance;
	float totalX=0.0, totalY=0.0;    
	for(int i = 0; i < this->highProbPointsToFit.size(); ++i) 
	{
	    totalX+=this->highProbPointsToFit[i].x;
	    totalY+=this->highProbPointsToFit[i].y;
	}

	this->centroid[0] = totalX/this->highProbPointsToFit.size();
	this->centroid[1] = totalY/this->highProbPointsToFit.size();

	//centroid[0] += crop;
	//centroid[1] += crop;
}

bool 
ModelBasedLineEstimation::checkLineFitting()
{

	return (this->line_covariance > this->covariance_threshold ? false : true);
}

void
ModelBasedLineEstimation::addPointToModel()
{
	double tmp_point[3];

	this->convertLineCentroidTo3DWorkspace(tmp_point);

	this->valveModel.updateModel(tmp_point[0], tmp_point[1], tmp_point[2]);
}


void
ModelBasedLineEstimation::computeResidualVariance()
{
	::Eigen::VectorXd point_on_line(2), tangent(2), tmp_point(2);
	point_on_line[0] = this->fittedLine[2];
	point_on_line[1] = this->fittedLine[3];

	tangent[0] = this->fittedLine[0];
	tangent[1] = this->fittedLine[1];

	// compute residuals
	::Eigen::VectorXd res(this->highProbPointsToFit.size());
	for (int i = 0; i < this->highProbPointsToFit.size(); ++i)
	{
		tmp_point[0] = this->highProbPointsToFit[i].x;
		tmp_point[1] = this->highProbPointsToFit[i].y;

		distancePointToLine(tmp_point, point_on_line, tangent, res[i]); 
	}

	// compute variance
	::Eigen::VectorXd res_centered = res - res.mean() * Eigen::VectorXd::Ones(res.size());
	double min = res_centered.minCoeff();
	double max = res_centered.maxCoeff();
	this->line_covariance = ::std::sqrt(res_centered.array().pow(2).sum()/res.size());
	//::std::cout << "covariance" << line_covariance << ::std::endl;
}


void 
ModelBasedLineEstimation::thresholdImageAllChannels(const ::cv::Mat& img,::cv::Mat& thresholded)
{
    ::cv::Mat O1, O2, O3;
    this->RGBtoOpponent(img, O1, O2, O3); 

	double min, max;
    ::cv::minMaxLoc(O1, &min, &max);
	O1 = O1 - min;
	::cv::minMaxLoc(O1, &min, &max);
	O1 /= max;
	O1 *= 255;

    ::cv::minMaxLoc(O2, &min, &max);
	O2 = O2 - min;
	::cv::minMaxLoc(O2, &min, &max);
	O2 /= max;
	O2 *= 255;

    ::cv::minMaxLoc(O3, &min, &max);
	O3 = O3 - min;
	::cv::minMaxLoc(O3, &min, &max);
	O3 /= max;
	O3 *= 255;

	// normalize image
	O1.convertTo(O1, CV_8UC1);
	O2.convertTo(O2, CV_8UC1);
	O3.convertTo(O3, CV_8UC1);

	// histogram equalization
	cv::equalizeHist(O1, O1);
	cv::equalizeHist(O2, O2);
	cv::equalizeHist(O3, O3);

	// invert image
	::cv::minMaxLoc(O1, &min, &max);
	O1 = 255 - O1;
	O2 = 255 - O2;
	O3 = 255 - O3;

	::cv::threshold(O1, O1, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	::cv::threshold(O2, O2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	::cv::threshold(O3, O3, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	// mask circle
    ::cv::Mat circle_img = ::cv::Mat::zeros(O1.size(), CV_32FC1);
    ::cv::circle(circle_img, ::cv::Size(O1.size[0]/2, O1.size[1]/2), 100, ::cv::Scalar(255, 255, 255), -1);
	::cv::normalize(circle_img, circle_img,1 , ::cv::NORM_L2);
	circle_img = circle_img * 255;
	circle_img.convertTo(circle_img, CV_8UC1);

	O1 = O1.mul(circle_img);
	O2 = O2.mul(circle_img);
	O3 = O3.mul(circle_img);


	// combine all channels
	::cv::Mat out;
    ::cv::bitwise_and(O1, O2, out);
    ::cv::bitwise_and(out, O3, out);

	//threshold
	::cv::threshold(out, thresholded, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

}

bool ModelBasedLineEstimation::RGBtoOpponent(const ::cv::Mat &img, ::cv::Mat &O1, ::cv::Mat &O2, ::cv::Mat &O3)
{

	::std::vector< ::cv::Mat> channels;
	::cv::split(img,channels);

    const int rows = img.rows;
    const int cols = img.cols;
    O1 = cv::Mat(img.size(),CV_32FC1);
    O2 = cv::Mat(img.size(),CV_32FC1);
    O3 = cv::Mat(img.size(),CV_32FC1);

    for(int r=0;r<rows;r++)
    {
        for(int c=0;c<cols;c++)
        {
            const cv::Vec3b &bgr = img.at<cv::Vec3b>(r,c);
            O1.at<float>(r,c) = (1.0f/sqrt(2)) * ((float)bgr[2] - (float)bgr[1]);
            O2.at<float>(r,c) = (1.0f/sqrt(6)) * ((float)bgr[2] + (float)bgr[1] - 2.0f * (float)bgr[0]);
            O3.at<float>(r,c) = (1.0f/sqrt(3)) * ((float)bgr[2] + (float)bgr[1] + (float)bgr[0]);
        }
    }


    return true;
}


void
ModelBasedLineEstimation::rejectOutliers()
{
	double distance = 0;
	::Eigen::VectorXd point_on_line(2), tangent(2), tmp_point(2);
	//point_on_line[0] = this->predictedLine[2];
	//point_on_line[1] = this->predictedLine[3];
	point_on_line[0] = centroid[0];
	point_on_line[1] = centroid[1];
	tangent[0] = this->predictedLine[0];
	tangent[1] = this->predictedLine[1];


	this->highProbPointsToFit.clear();

	for (int i = 0; i < this->pointsToFit.size(); ++i)
	{
		distancePointToLine(tmp_point, point_on_line, tangent, distance);
		
		if (distance > 500)
			continue;
		
		this->highProbPointsToFit.push_back(this->pointsToFit[i]);
		//::std::cout << "point " << i << ", distance:" << distance << ", covariance:" << this->line_covariance << ::std::endl;
	}
	//::std::cout << distance << ::std::endl;
	//::std::cout << "num of points:" << this->highProbPointsToFit.size() << ::std::endl;
}


void 
ModelBasedLineEstimation::convertLineCentroidTo3DWorkspace(double tmp_point[3])
{

	::Eigen::Vector2d DP;
	DP(0) = this->centroid[0] - this->channel_pixel_position_unrotated[0];    // in pixels in unrotated image frame
	DP(1) = this->centroid[1] - this->channel_pixel_position_unrotated[1];

	::Eigen::Matrix3d rot1 = RotateZ(this->init_image_rotation * M_PI/180.0 - this->inner_tube_rotation);
	DP = rot1.block(0, 0, 2, 2).transpose() * DP;	// in pixels in rotated image frame


	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);
	DP = rot.block(0, 0, 2, 2).transpose() * DP;	// in pixels in image frame aligned with robot base frame
	DP /= this->scaling_factor;    // in mm in world frame

	::Eigen::MatrixXd proj;
	this->getModel().getProjectionMatrixToPlane(proj);
	DP = proj.block(0, 0, 2, 2) * DP;
	//this->projected_robot_position[0] += DP[0];
	//this->projected_robot_position[1] += DP[1];

	// bring back to 3D
	this->centroid_position_3d.segment(0, 2) = ::Eigen::Map<::Eigen::Vector3d>(this->robot_position, 3).segment(0, 2) + DP;
	this->centroid_position_3d(2) = this->robot_position[2];

	memcpy(tmp_point, this->centroid_position_3d.data(), 3 * sizeof(double));
}

void ModelBasedLineEstimation::computePredictedTangent()
{
	//::Eigen::Vector3d tangent, point_on_circle;
	//this->valveModel.getTangentEstimate(this->robot_predicted_position[0], this->robot_predicted_position[1], this->robot_predicted_position[2], tangent, point_on_circle);

	////::Eigen::MatrixXd proj;
	////this->valveModel.getProjectionMatrixToPlane(proj);
	::cv::Vec4f line;
	if (this->getPredictedTangent(line))
	{
		this->predictedLine[0] = line[0];
		this->predictedLine[1] = line[1];
	}
	else
	{
		this->predictedLine[0] = 1;
		this->predictedLine[1] = 0;
	}
	//tangent = proj * tangent;
}


void
ModelBasedLineEstimation::computeClosestCirclePoint()
{
	::Eigen::Vector3d  tmpPoint;
	this->valveModel.getClosestPointOnCircle(this->robot_predicted_position[0], this->robot_predicted_position[1], this->robot_predicted_position[2], tmpPoint);

	//CONVERT TO IMAGE FRAME!!!!
	this->predictedLine[2] = tmpPoint(1);
	this->predictedLine[3] = tmpPoint(2);
}


void
ModelBasedLineEstimation::computePredictionCovariance()
{
	//TO BE IMPLEMENTED
	pred_line_covariance = 50;
}


void
ModelBasedLineEstimation::getClosestPointOnCircle(double point[3])
{
	::Eigen::Vector3d tmp;
	valveModel.getClosestPointOnCircle(this->robot_predicted_position[0], this->robot_predicted_position[1], this->robot_predicted_position[2], tmp);
	memcpy(point, tmp.data(), 3*sizeof(double));
}

bool 
ModelBasedLineEstimation::getTangent(double p1[3], double p2[3])
{
	double x = this->robot_predicted_position[0];
	double y = this->robot_predicted_position[1];
	double z = this->robot_predicted_position[2];

	::Eigen::Vector3d tangent, point_on_circle;
	this->valveModel.getTangentEstimate(x, y, z, tangent, point_on_circle);

	::Eigen::Vector3d p1Eig, p2Eig;
	p1Eig = point_on_circle - 10 * tangent;
	p2Eig = point_on_circle + 10 * tangent;

	if (p1Eig == p2Eig)
		return false;
	memcpy(p1, p1Eig.data(), 3 * sizeof(double));
	memcpy(p2, p2Eig.data(), 3 * sizeof(double));

	return true;
}

bool	
ModelBasedLineEstimation::getPredictedTangent(::cv::Vec4f& line)
{
	double p1[3]; 
	double p2[3];
	this->getTangent(p1, p2);

	::Eigen::Vector3d tangent = ::Eigen::Map<::Eigen::Vector3d> (p1, 3) - ::Eigen::Map<::Eigen::Vector3d> (p2, 3);
	tangent.normalize();

	::Eigen::Matrix3d rot = RotateZ( -90 * M_PI/180.0);
	::Eigen::Vector2d tangentEig = rot.block(0, 0, 2, 2) * tangent;

	line[0] = tangentEig[0];
	line[1] = tangentEig[1];

	return this->valveModel.isInitialized();
}


bool 
ModelBasedLineEstimation::stepBenchtop(double robot_position[3], double robot_desired_velocity[3], const ::cv::Mat& img, double innerTubeRotation, ::cv::Vec4f& line,
									::cv::Vec2f& centroid_out, bool update)
{
	this->inner_tube_rotation = innerTubeRotation;
	this->update_model = update;
	// using the current robot position and the velocity from the visual servoing controller -> predict when the line should be in the next iteration
	this->predict(robot_position, robot_desired_velocity);

	// use the image to compute the line
	this->updateBenchtop(img);
	
	if (!this->line_detected)
		return false;
	line = this->fittedLine;

	centroid_out[0] = this->centroid[0];
	centroid_out[1] = this->centroid[1];

	//temporary!!!!
	this->predictedcentroid[0] = this->centroid[0];
	this->predictedcentroid[1] = this->centroid[1];

	this->predictedLine[2] = this->centroid[0];
	this->predictedLine[3] = this->centroid[1];

	this->computeResidualVariance();


	return this->checkLineFitting();

}

void 
ModelBasedLineEstimation::updateBenchtop(const ::cv::Mat& img)
{
	img.copyTo(this->current_img);

	this->computePointsForFittingBenchtop();

	if (this->valveModel.isInitialized())
	{
		//::std::cout << "rejecting outliers" << ::std::endl;
		this->rejectOutliers();
	}
	else
	{
		this->highProbPointsToFit.resize(this->pointsToFit.size());
		::std::copy(this->pointsToFit.begin(), this->pointsToFit.end(), this->highProbPointsToFit.begin());
	}

	this->line_detected = this->fitLine();

	if (this->checkLineFitting() && this->update_model)
		this->addPointToModel();
}

void 
ModelBasedLineEstimation::computePointsForFittingBenchtop()
{
	::cv::Mat img_crop = this->current_img(::cv::Rect(this->crop, this->crop, this->current_img.cols - 2 * this->crop, this->current_img.rows - 2 * this->crop));

    ::cv::Mat thresholded;
	
    ::cv::Mat thresholded_binary(this->current_img.size(), CV_8UC1);

	this->thresholdImageAllChannels(this->current_img, thresholded);
    thresholded.convertTo(thresholded_binary,CV_8UC1);
	
    ::cv::findNonZero(thresholded_binary, this->pointsToFit);

	::cv::Mat	 output;
	::cv::cvtColor(thresholded, output, CV_GRAY2BGR);

	::cv::imshow("thresholded", output);
	::cv::waitKey(1);
}

void ModelBasedLineEstimation::thresholdImageSynthetic(const ::cv::Mat& img,::cv::Mat& thresholded)
{
	::cv::Mat img_grey;
	::cv::cvtColor(img, img_grey, ::cv::ColorConversionCodes::COLOR_RGB2GRAY); 
	::cv::threshold(img_grey, thresholded, 50, 255, ::cv::ThresholdTypes::THRESH_BINARY_INV);
}


void ModelBasedLineEstimation::resetModel()
{
	this->valveModel.resetModel();
}


void ModelBasedLineEstimation::computePointsForFittingNew()
{
	int radius = 125;
	int numOfPoints = (int) 3.14 * radius*radius *0.04;
    ::cv::Mat thresholded_mask, intermediate_img;

    // Blur  the image to remove small artifacts (color defects from camera, bood flow ..)
    ::cv::Mat img_blur;
    ::cv::GaussianBlur(this->current_img,img_blur,::cv::Size(15,15),0);

    // Mask the optical window using provided center and radius
    ::cv::Mat ow_mask = ::cv::Mat::zeros(this->current_img.rows,this->current_img.cols, CV_8UC1);
    ::cv::circle(ow_mask,::cv::Point(this->current_img.rows/2,this->current_img.cols/2),radius,255,-1);

    img_blur.copyTo(intermediate_img,ow_mask);
    this->thresholdImage(intermediate_img,thresholded_mask);

	::cv::Mat thresholded_binary(thresholded_mask.size(),CV_8UC1);
    thresholded_mask.convertTo(thresholded_binary,CV_8UC1);
	
    ::cv::findNonZero(thresholded_binary, this->pointsToFit);

	::cv::Mat	 output;
	::cv::cvtColor(thresholded_binary, output, CV_GRAY2BGR);

	::cv::imshow("thresholded", output);
	::cv::waitKey(1);
}


bool ModelBasedLineEstimation::thresholdImage(const cv::Mat &img, ::cv::Mat &output)
{

    output = ::cv::Mat::zeros(img.rows,img.cols, CV_8UC1);

    ::cv::Mat S, A, V;
    convertImage(img,S,A, V);

    // Apply thresholds
    const int thresh_S = 64;
	//const int thresh_V = 200; -> worked first sucess
	const int thresh_V = 250;
    const int min_a = 130, max_a = 145;

    ::cv::Mat mask_s;
    ::cv::threshold(S,mask_s,thresh_S,255,::cv::THRESH_BINARY_INV);

	double min, max;
	::cv::minMaxLoc(A, &min, &max);
	//::std::cout << "min:" << min << " max:" << max << ::std::endl;

	::cv::Mat mask_v;
	::cv::threshold(V, mask_v, thresh_V, 255, ::cv::THRESH_BINARY_INV);

    ::cv::Mat mask_a;
    ::cv::inRange(A,min_a,max_a,mask_a);

	::cv::bitwise_and(mask_s, mask_v, output); 
    ::cv::bitwise_and(output,mask_a,output);

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(9,9));
    ::cv::morphologyEx(output,output,::cv::MORPH_OPEN,kernel);

    return true;
}

bool ModelBasedLineEstimation::convertImage(const cv::Mat &img, cv::Mat& S, cv::Mat& A, ::cv::Mat& V)
{	
    ::cv::Mat hsv, lab;

    ::std::vector< ::cv::Mat> hsv_split;
    ::std::vector< ::cv::Mat> lab_split;

    ::cv::cvtColor(img,hsv, CV_BGR2HSV);
    ::cv::cvtColor(img,lab, CV_BGR2Lab);

    ::cv::split(hsv,hsv_split);
    ::cv::split(lab,lab_split);


    S = hsv_split[1];
	V = hsv_split[2];
    A = lab_split[1];

    return true;
}


void ModelBasedLineEstimation::computePointsForFittingWire()
{
	::cv::Mat hsv, out;
	::std::vector<::cv::Mat> hsv_split;

	::cv::cvtColor(this->current_img, hsv, CV_BGR2HSV);
	::cv::split(hsv, hsv_split);

    ::cv::Mat mask_h, mask_s, mask_v;
	const int min_h = 10, max_h = 110;
	const int min_s = 1, max_s = 255;
	const int min_v = 1, max_v = 255;

	//const int min_h = 70, max_h = 115;
	//const int min_s = 28, max_s = 89;
	//const int min_v = 185, max_v = 255;

    ::cv::inRange(hsv_split[0] ,min_h,max_h,mask_h);
    ::cv::inRange(hsv_split[1] ,min_s,max_s,mask_s);
    ::cv::inRange(hsv_split[2] ,min_v,max_v,mask_v);

	::cv::bitwise_and(mask_s, mask_h, out); 
	::cv::bitwise_and(mask_v, out, out);

	//::cv::Mat channel_mask = ::cv::Mat::ones(this->current_img.rows, this->current_img.cols, CV_8UC1)*255;
	::cv::Mat channel_mask = ::cv::Mat::zeros(this->current_img.rows, this->current_img.cols, CV_8UC1);
	::cv::circle(channel_mask, ::cv::Point(125, 125), 125,  255, -1);
	::cv::bitwise_and(out, channel_mask, out); 
	//// mask the working channel
	//// scope 1
	//::cv::circle(channel_mask, ::cv::Point(29, 149), 40,  0, -1);
	//// scope 2
	//::cv::circle(channel_mask, ::cv::Point(215, 149), 40,  0, -1);
	//// scope 3
	//::cv::circle(channel_mask, ::cv::Point(47, 115), 40,  0, -1);
	//// scope 4
	//::cv::circle(channel_mask, ::cv::Point(43, 116), 46,  0, -1);

	//::cv::bitwise_and(out, channel_mask, out); 

    // Apply morphological opening to remove small things
    ::cv::Mat kernel = ::cv::getStructuringElement(::cv::MORPH_ELLIPSE,::cv::Size(5,5));
    ::cv::morphologyEx(out,out,::cv::MORPH_OPEN,kernel);

	//cv::Mat thresholded_binary;
    out.convertTo(this->thresholded_binary,CV_8UC1);
	
    ::cv::findNonZero(this->thresholded_binary, this->pointsToFit);

	::cv::Mat	 output;
	::cv::cvtColor(out, output, CV_GRAY2BGR);

	::cv::imshow("thresholded", output);
	::cv::waitKey(1);
}

bool
ModelBasedLineEstimation::fitLineHough()
{

	::std::vector<::cv::Vec4i> lines;
	//::cv::imshow("thresholded", output);
	//::cv::waitKey(1);

	if (this->highProbPointsToFit.size() > 0)
	{

		::cv::HoughLinesP(this->thresholded_binary, lines, 1, 1 * M_PI/180.0, 30, 5, 3);
		if (lines.size() > 0)
			this->averageTangentPCA(lines, this->fittedLine);
		else 
			return false;

		this->computeCentroid();

		//if (lines.size() > 0)
		//	this->averageTangentPCA(lines, this->fittedLine);
		//else
		//	return false;

        ::cv::line( this->current_img, ::cv::Point(fittedLine[2],fittedLine[3]), ::cv::Point(fittedLine[2]+fittedLine[0]*100,fittedLine[3]+fittedLine[1]*100), ::cv::Scalar(0, 255, 0), 2, CV_AA);
        ::cv::line( this->current_img, ::cv::Point(fittedLine[2],fittedLine[3]), ::cv::Point(fittedLine[2]+fittedLine[0]*(-100),fittedLine[3]+fittedLine[1]*(-100)), ::cv::Scalar(0, 255, 0), 2, CV_AA);
		::cv::circle(this->current_img, ::cv::Point(this->centroid[0], centroid[1]), 5, ::cv::Scalar(255,0,0));

		return true;
	}

 	return false;
}


void ModelBasedLineEstimation::averageTangentPCA(::std::vector<::cv::Vec4i>& lines, ::cv::Vec4f& line)
{
	::Eigen::MatrixXd data;
	::Eigen::Vector2d tmp;
	for (int i = 0; i < lines.size(); ++i)
	{
		tmp(0) = lines[i][2] - lines[i][0];
		tmp(1) = lines[i][3] - lines[i][1];
		tmp.normalize();

		appendRowEigen(data, tmp);

		tmp(0) *= -1;
		tmp(1) *= -1;

		appendRowEigen(data, tmp);
	}

	::Eigen::JacobiSVD<::Eigen::MatrixXd> svd(data.transpose() * data, ::Eigen::ComputeThinU | ::Eigen::ComputeThinV);

	line(0) = svd.matrixU()(0, 0);
	line(1) = svd.matrixU()(1, 0);

}

