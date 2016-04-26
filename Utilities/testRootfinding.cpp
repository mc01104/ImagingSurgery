#include "lwpr.hh"
#include "Utilities.h"

#include <iostream>
#include <Dense>

typedef ::std::vector< ::std::vector<double> > DoubleMat;

void printDoubleMatrix(const doubleMat& matrixToBePrinted);
void printEigMatrix(const Eigen::MatrixXd& matrixToBePrinted);
doubleVec computeInverseRoot(LWPR_Object& model, const doubleVec& inputVec);
void stepGradientDescent(LWPR_Object& model, const doubleVec& inputVec, const doubleVec& initVector, doubleVec& resultVec, double& positionError, double& orientationError);
void updateSolution(LWPR_Object& model, double step, const Eigen::VectorXd& xOld, const Eigen::MatrixXd& invJ, const Eigen::VectorXd& fOld, const Eigen::VectorXd& offset, doubleVec& xNew); 
void computeError(LWPR_Object& model, const doubleVec& posOrt, const doubleVec& jAng, double& positionError, double& orientationError);

using namespace std;

int main(int argc, char* argv[])
{
	//path to model definition
	string pathToModel("C:/Users/RC/Dropbox/Boston/BCH/concentric_tubes/George/forward_kinematics/models/model_ct_2015_10_22_11_30_46.bin");

	// load the LWPR model
	LWPR_Object model(pathToModel.c_str());
			
	double data[3] = {0.17, 0.02, 0.32};

	doubleVec inputData(data, data + model.nIn());

	cout << "input to the model" << endl;
	printVector(inputData);

	cout << "model output" << endl;
	doubleVec result = model.predict(inputData, 0.0000001);
	printVector(result);

	// use gradient descent to do inverse
	doubleVec inverseResult = computeInverseRoot(model, result);
	
	cout << "inverse computation" << endl;
	printVector(inverseResult);


	EXIT_SUCCESS;
}


doubleVec computeInverseRoot(LWPR_Object& model, const doubleVec& inputVec)
{
	double positionError = 10000.0;
	double orientationError = 10000.0;
	double step = 1.0;

	int iterations = 0;

	double init[3] = {0, 0.23, 0.23};
	doubleVec initVec(init, init + 3);
	doubleVec resultVec;
	
	while ((positionError > 0.01 || orientationError > 0.01) && iterations < 1000)
	{
			stepGradientDescent(model, inputVec, initVec, resultVec, positionError, orientationError);
			iterations++;
			initVec = resultVec;
	}

	if (iterations >= 1000) {::std::cout << "mode iterations needed" << ::std::endl;}
	return resultVec;
}

void 
stepGradientDescent(LWPR_Object& model, const doubleVec& offset, const doubleVec& xOld, doubleVec& resultVec, double& positionError, double& orientationError)
{
	 
	::std::vector<double> fOld = model.predict(xOld);

	double step = 1;

	Eigen::VectorXd xOldEig = ::Eigen::Map< Eigen::VectorXd> (const_cast<doubleVec&> (xOld).data(), model.nIn());
	Eigen::VectorXd fOldEig = ::Eigen::Map< Eigen::VectorXd> (fOld.data(), model.nOut());
	Eigen::VectorXd offsetEig = Eigen::Map< Eigen::VectorXd> (const_cast<doubleVec&> (offset).data(), model.nOut());

	Eigen::MatrixXd J = model.computeJacobianNumerical(xOld);
	
	::Eigen::MatrixXd invJ = pseudoInverse(J);

	updateSolution(model, step, xOldEig, invJ, fOldEig, offsetEig, resultVec) ; 

	computeError(model, offset, resultVec, positionError, orientationError);
}

void
updateSolution(LWPR_Object& model, double step, const Eigen::VectorXd& xOld, const Eigen::MatrixXd& invJ, const Eigen::VectorXd& fOld, const Eigen::VectorXd& offset, doubleVec& xNew)
{

	// update estimation of solution
	Eigen::VectorXd xNewEig = xOld - step * invJ * fOld;
	xNew.assign(xNewEig.data(), xNewEig.data() + 3);

	// new value for objective function
	::std::vector<double> fNew = model.predict(xNew);
	Eigen::VectorXd fNewEig = ::Eigen::Map<Eigen::VectorXd > (fNew.data(), model.nOut());

	// if the error increases when updating the solution, we need to decrease the step size of the gradient descent
	Eigen::VectorXd error = fNewEig - offset;
	Eigen::VectorXd errorOld = fOld - offset;

	if (error.squaredNorm() - errorOld.squaredNorm() > 0.0001)
		return updateSolution(model, 0.5 * step, xOld, invJ, fOld, offset, xNew);   
	
	return;
}

void 
computeError(LWPR_Object& model, const doubleVec& posOrt, const doubleVec& jAng, double& positionError, double& orientationError)
{
	doubleVec outputData = model.predict(jAng);
	
	doubleVec error = posOrt - outputData;
	doubleVec tmp(posOrt);

	positionError = norm2(error);

	::Eigen::Map <::Eigen::Vector3d> vec1( outputData.data() + 3, 3);
	::Eigen::Map <::Eigen::Vector3d> vec2( tmp.data() + 3, 3);

	orientationError = vec1.cross(vec2).norm();

}

void printDoubleMatrix(const doubleMat& matrixToBePrinted)
{
	for (DoubleMat::const_iterator it = matrixToBePrinted.begin(); it != matrixToBePrinted.end(); ++it)
	{
		for (::std::vector<double>::const_iterator it2 = it->begin(); it2 != it->end(); ++it2)
			::std::cout << *it2 << " ";
		::std::cout << ::std::endl;
	}
	
	::std::cout << ::std::endl;	
}

void printEigMatrix(const Eigen::MatrixXd& matrixToBePrinted)
{

	std::string sep = "\n----------------------------------------\n";
	Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	
	std::cout << matrixToBePrinted.format(OctaveFmt) << sep;
}