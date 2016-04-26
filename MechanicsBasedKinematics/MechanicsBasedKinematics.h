#pragma once

#include "CTR.h"
#include"LieGroup.h"

#include <Eigen/dense>

class MechanicsBasedKinematics
{
	CTR* robot;
	Eigen::VectorXd arcLengthGrid, normalizedArcLengthGrid;
	Eigen::MatrixXd BVPSolutionGrid, perturbedSolGrid;
	Eigen::MatrixXd jacobianBC;
	
	std::vector<SE3> bishopFrames, perturbedBishopFrames;
	Eigen::VectorXd boundaryConditionTip;
	int maxIter;
	double stepSize;

	bool isUsingIVPJacobian;

public:
	MechanicsBasedKinematics(CTR* _robot, int numOfGridPoints = 1000);
	~MechanicsBasedKinematics();
	bool ComputeKinematics(double* rotation, double* translation);
	void GetBishopFrame(SE3& bishopFrame);
	void GetBishopFrame(double s, SE3& bishopFrame);
	void GetBishopFrame(std::vector<double> s, std::vector<SE3>& bishopFrame);
	double GetInnerTubeRotation(double s);
	double GetInnerTubeRotation();
	bool GetControlJacobian(double s, Eigen::MatrixXd& controlJacobian);

	void printSolution(string filename = "../solution.txt") const;
	void printBishopFrame(string filename = "../frame.txt") const;
	void ActivateIVPJacobian() {this->isUsingIVPJacobian = true;};
	static void RelativeToAbsolute(const CTR* const robot, const double* const& relativeConf, double* const &rotation, double* const &translation);

private:
	bool updateConfiguration (double* rotation, double* translation);
	bool solveBVP (Eigen::MatrixXd& solution);
	void solveIVP (Eigen::MatrixXd& solution, const Eigen::VectorXd& boundaryConditions);
	void updateBC (Eigen::VectorXd& errorBC);
	void computeBCJacobian (Eigen::MatrixXd& solution);
	bool hasBVPConverged (Eigen::MatrixXd& solution, Eigen::VectorXd& errorBC);
	void propagateBishopFrame (std::vector<SE3>& bishopFramesToPropagate, Eigen::MatrixXd& solution);
	void findNearestGridPoint(double s, int* beginIdx, double* fracFromBegin);
	void GetBishopFrame(double s, SE3& bishopFrame, std::vector<SE3>& frames);
	void ComputeBCJacobianNumerical(Eigen::MatrixXd& solution);
	void ComputeBCAtBase(const Eigen::VectorXd& solutionAtBase, const double* tubeTranslation, Eigen::VectorXd& estimatedBC);
	void Initialize(int numOfGridPoints);
	void checkJacobianCondition();
};
