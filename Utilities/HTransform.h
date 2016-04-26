#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>


::Eigen::Matrix3d RotateZ(double angle);


class HTransform
{
	::Eigen::Matrix3d rotation;

	::Eigen::Vector3d translation;


public:

	HTransform();

	HTransform(const ::Eigen::Matrix3d& rotation, const ::Eigen::Vector3d& translation);

	HTransform(HTransform& htransform);

	HTransform(HTransform* htransform);

	const ::Eigen::Matrix3d& getRotation() const;

	const ::Eigen::Vector3d& getTranslation() const;

	void setRotation(const ::Eigen::Matrix3d& rotation);

	void setTranslation(const ::Eigen::Vector3d& translation);

	HTransform inverse();

	friend ::std::ostream& operator<<(::std::ostream& os, const HTransform& htransform);

	static void applyHTransform(const HTransform& htransform, const ::std::vector<double>& inputVector, ::std::vector<double>& outputVector);

	static void applyRotation(const HTransform& htransform, const ::std::vector<double>& inputVector, ::std::vector<double>& outputVector);
};



