#include "HTransform.h"

	HTransform::HTransform():
		rotation(),
		translation()
	{
	}

	HTransform::HTransform(const ::Eigen::Matrix3d& rotation, const ::Eigen::Vector3d& translation):
		rotation(rotation),
		translation(translation)
	{
	}

	HTransform::HTransform(HTransform& htransform)
	{
		rotation = htransform.getRotation();
		translation = htransform.getTranslation();
	}

	HTransform::HTransform(HTransform* htransform)
	{
		rotation = htransform->getRotation();
		translation = htransform->getTranslation();
	}

	const ::Eigen::Matrix3d&
	HTransform::getRotation() const
	{
		return rotation;
	}

	const ::Eigen::Vector3d& 
	HTransform::getTranslation() const
	{
		return translation;
	}

	void 
	HTransform::setRotation(const ::Eigen::Matrix3d& rotation)
	{
		this->rotation = rotation;
	}

	void 
	HTransform::setTranslation(const ::Eigen::Vector3d& translation)
	{
		this->translation = translation;
	}

	HTransform
	HTransform::inverse()
	{
		return HTransform(this->rotation.transpose(), -this->rotation.transpose() * this->translation);
	}


	void HTransform::applyHTransform(const HTransform& htransform, const ::std::vector<double>& inputVector, ::std::vector<double>& outputVector)
	{
		::Eigen::Vector3d inputEigen(inputVector.data());
		::Eigen::Vector3d outputEigen = htransform.getRotation() * inputEigen + htransform.getTranslation();
		outputVector = ::std::vector<double> (outputEigen.data(), outputEigen.data() + 3);
	}

	void HTransform::applyRotation(const HTransform& htransform, const ::std::vector<double>& inputVector, ::std::vector<double>& outputVector)
	{
		::Eigen::Vector3d inputEigen(inputVector.data());
		::Eigen::Vector3d outputEigen = htransform.getRotation() * inputEigen;
		outputVector = ::std::vector<double> (outputEigen.data(), outputEigen.data() + 3);
	}

	::Eigen::Matrix3d
	RotateZ(double angle)
	{
		::Eigen::Matrix3d rotation;

		rotation << cos(angle), - sin(angle), 0,
					sin(angle),   cos(angle), 0,
				 			0 ,			0	, 1;
		return rotation;
	}

	::std::ostream& operator<<(::std::ostream& os, const HTransform& htransform)
	{
		
		::Eigen::IOFormat OctaveFmt(::Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");

		os << "------Rotation------" << ::std::endl;
		os << htransform.getRotation() << ::std::endl;

		os << "------Translation------" << ::std::endl;
		os << htransform.getTranslation() << ::std::endl;

		return os; 
	}