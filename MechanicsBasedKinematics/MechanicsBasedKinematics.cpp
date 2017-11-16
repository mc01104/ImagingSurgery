#include "MechanicsBasedKinematics.h"
#include <fstream>
#include "../Utilities/Utilities.h"

#define CTR_EPSILON 0.0001

MechanicsBasedKinematics::MechanicsBasedKinematics(CTR* _robot, int numOfGridPoints)
	: maxIter(100), stepSize(1.0), isUsingIVPJacobian(false)
{
	this->robot = _robot;

	this->Initialize(numOfGridPoints);

	this->ActivateIVPJacobian();
}

MechanicsBasedKinematics::~MechanicsBasedKinematics()
{
	// delete this->robot;
}

bool MechanicsBasedKinematics::ComputeKinematics(double* rotation, double* translation)
{
	if(!this->robot->UpdateConfiguration(rotation, translation))
	{
		::std::cout << "translation: ";
		PrintCArray(translation, 3);
		return false;
	}
	
	if(!this->solveBVP(this->BVPSolutionGrid))
		return false;

	this->propagateBishopFrame(this->bishopFrames, this->BVPSolutionGrid);
	
	return true;
}

void MechanicsBasedKinematics::GetBishopFrame(SE3& bishopFrame)
{
	bishopFrame = this->bishopFrames.back();
}

void MechanicsBasedKinematics::GetBishopFrame(double s, SE3& bishopFrame)
{
	GetBishopFrame(s, bishopFrame, this->bishopFrames);
}

void MechanicsBasedKinematics::GetBishopFrame(std::vector<double> s, std::vector<SE3>& bishopFrame)
{
	int sizeS = s.size();
	if(bishopFrame.size() != sizeS)
		bishopFrame.resize(sizeS);

	for(int i = 0; i < sizeS; ++i)
		this->GetBishopFrame(s[i], bishopFrame[i]);
}

double MechanicsBasedKinematics::GetInnerTubeRotation(double s)
{
	int numTube = robot->GetNumOfTubes();
	
	int beginIdx;
	double frac;
	this->findNearestGridPoint(s, &beginIdx, &frac);

	double rotation0 = this->BVPSolutionGrid(numTube-1, beginIdx);
	double rotation1 = this->BVPSolutionGrid(numTube-1, beginIdx+1);

	return frac*rotation1 + (1-frac)*rotation0;
}

double MechanicsBasedKinematics::GetInnerTubeRotation()
{
	return this->BVPSolutionGrid(robot->GetNumOfTubes()-1, this->BVPSolutionGrid.cols()-1);
}




bool MechanicsBasedKinematics::GetControlJacobian(double s, Eigen::MatrixXd& controlJacobian)
{
	int numTubes = this->robot->GetNumOfTubes();

	// Check the size of controlJacobian, and resize if necessary.
	if(controlJacobian.cols() != 2*numTubes || controlJacobian.rows() != 6)
		controlJacobian.resize(6,2*numTubes);
	
	double* rotation = this->robot->GetRotation();
	double* translation = this->robot->GetTranslation();

	double* perturbedRot = new double[numTubes];
	double* perturbedTrans = new double[numTubes];

	memcpy(perturbedRot, rotation, sizeof(double)*numTubes);
	memcpy(perturbedTrans, translation, sizeof(double)*numTubes);

	
	SE3 frameBeforePerturb, frameAfterPerturb;
	
	this->GetBishopFrame(s, frameBeforePerturb, this->bishopFrames);

	double invEps = (1.0/CTR_EPSILON);
	for(int i = 0; i < numTubes; ++i)
	{
		perturbedRot[i] += CTR_EPSILON;
		perturbedTrans[i] += CTR_EPSILON;
		
		// Jacobian w.r.t. rotation
		this->robot->UpdateConfiguration(perturbedRot, translation);
		if(!this->solveBVP(perturbedSolGrid))
			return false;
		this->propagateBishopFrame(this->perturbedBishopFrames, perturbedSolGrid);
		this->GetBishopFrame(s, frameAfterPerturb, this->perturbedBishopFrames);

		se3 TmpColumn = Log(Inv(frameBeforePerturb)*frameAfterPerturb) * invEps;
		controlJacobian.col(i) = Eigen::Map<Eigen::VectorXd>(&TmpColumn[0],6);	// This couldn't work.

		// Jacobin w.r.t. translation
		this->robot->UpdateConfiguration(rotation, perturbedTrans);
		if(!this->solveBVP(perturbedSolGrid))
			return false;
		this->propagateBishopFrame(this->perturbedBishopFrames, perturbedSolGrid);
		this->GetBishopFrame(s, frameAfterPerturb, this->perturbedBishopFrames);

		TmpColumn = Log(Inv(frameBeforePerturb)*frameAfterPerturb) * invEps;
		controlJacobian.col(i + numTubes) = Eigen::Map<Eigen::VectorXd>(&TmpColumn[0],6);	// This couldn't work.

		perturbedRot[i] = rotation[i];
		perturbedTrans[i] = translation[i];
	}
	this->robot->UpdateConfiguration(rotation, translation);

	delete perturbedRot, perturbedTrans;

	return true;
}


bool MechanicsBasedKinematics::solveBVP (Eigen::MatrixXd& solution)
{
	this->stepSize = 1.0;
	//int halfMaxIter = this->maxIter/2;
	int subMaxIter = this->maxIter/5;

	// scale arcLengthGrid to robot length
	this->arcLengthGrid = this->normalizedArcLengthGrid * this->robot->GetLength();
	
	Eigen::VectorXd errorBC;
	for(int i = 0; i < this->maxIter; ++i)
	{
		this->solveIVP(solution, this->boundaryConditionTip);
		
		if(this->hasBVPConverged(solution, errorBC))
			return true;
		
		this->updateBC(errorBC);

		if(i % subMaxIter == subMaxIter-1)
		{
			this->boundaryConditionTip = Eigen::VectorXd::Random(this->robot->GetNumOfTubes(),1) * M_PI;
			//cout << "random initial generated. [" << this->boundaryConditionTip.transpose() << "]" << endl;
			this->stepSize = 0.01;
		}
		
	}

	//std::cout << "BVP Failed!" <<std::endl;
	return false;
}

void MechanicsBasedKinematics::solveIVP(Eigen::MatrixXd& solution, const Eigen::VectorXd& boundaryConditions)
{	
	int numTubes = this->robot->GetNumOfTubes();
	int numGridPoints = this->arcLengthGrid.size();

	solution.col(numGridPoints-1).setZero();
	solution.block(0,numGridPoints-1, boundaryConditions.size(),1) = boundaryConditions;

	//std::cout << "solution at Tip = " << solution.col(numGridPoints-1).transpose() << std::endl;
	//std::cout << "boundaryConditions = " << boundaryConditions.transpose() << std::endl;
	
	::std::vector<double> stiffness(numTubes);
	::std::vector<double> poissonsRatio(numTubes);
	for (int i = 0; i < numTubes; ++i)
	{
		stiffness[i] = this->robot->GetStiffness(i);
		poissonsRatio[i] = this->robot->GetPoissonsRatio(i);
	}


	vector<SO3> Rz(numTubes);
	vector<Vec3> u_hat(numTubes);
	std::vector<double> nu(numTubes);
	std::vector<bool> existingTubeIDs(numTubes);
	std::vector<Vec3> Rzk_RotZpi_uhatk(numTubes);
	
	// Jacobian computation
	Eigen::MatrixXd Phi_tt, Phi_tt_temp, Phi_ut, A;
	SO3 RotZ_pi = RotZ(M_PI).GetOrientation();
	if(isUsingIVPJacobian)
	{
		Phi_tt.resize(numTubes,numTubes);
		Phi_tt.setIdentity();
		Phi_tt_temp = Phi_tt;
		
		Phi_ut.resize(numTubes,numTubes);
		Phi_ut.setZero();
		
		A.resize(numTubes,numTubes);
		A.setZero();
	}	

	double s, ds, momentSum, firstKz, first_u_hat_z, sumkxy, sumkz, duzds, nuj;
	Vec3 sumRKu, u, u_curTube, Rzj_uhatj;
	for(int i = numGridPoints-1; i > 0; --i)
	{
		s = this->arcLengthGrid[i];
		
		// uz, theta

		this->robot->GetExistingTubes(s, existingTubeIDs);

		momentSum = 0;
		firstKz = 0;
		first_u_hat_z = 0;

		ds = s - this->arcLengthGrid[i-1];

		// integrate theta
		solution.block(0,i-1, numTubes,1) = solution.block(0,i, numTubes,1) - solution.block(numTubes,i, numTubes,1) * ds;

		// compute uxy
		sumkxy = 0, sumkz = 0;
		sumRKu.SetValues(0,0,0);

		for (int j = 0; j < numTubes; ++j)
		{
			if (!existingTubeIDs[j])
				continue;

			const double* precurvature;
			this->robot->ComputePrecurvature(s, j, &precurvature);
			u_hat[j] = Vec3(precurvature[0], precurvature[1], precurvature[2]);

			nu[j] = poissonsRatio[j];
			double kz = stiffness[j] / (1 + nu[j]);
			sumkxy += stiffness[j];
			sumkz += kz;

			Rz[j] = RotZ(solution(j, i)).GetOrientation(); //Exp(0, 0, theta);
			sumRKu += Rz[j] * Vec3(stiffness[j] * u_hat[j][0], stiffness[j] * u_hat[j][1], kz * u_hat[j][2]);
		}
		sumRKu[0] += solution(2*numTubes,i);
		sumRKu[1] += solution(2*numTubes+1,i);
		sumRKu[2] += solution(2*numTubes+2,i);

		//Vec3 u(sumRKu[0] / sumkxy, sumRKu[1] / sumkxy, sumRKu[2] / sumkz);	// z components doesn't seem to be used any where.
		u.SetValues(sumRKu[0] / sumkxy, sumRKu[1] / sumkxy, sumRKu[2] / sumkz);	// z components doesn't seem to be used any where.

		solution(solution.rows()-2,i) = u[0];
		solution(solution.rows()-1,i) = u[1];
		
		
		// integrate uz
		for(int j = 0; j < numTubes; j++)
		{
			if (!existingTubeIDs[j])
				continue;

			u_curTube = Inv(Rz[j]) * u;

			duzds = (1+nu[j]) * (u_curTube[0]*u_hat[j][1] - u_curTube[1]*u_hat[j][0]);

			solution(numTubes + j, i-1) = solution(numTubes + j, i) - duzds * ds;
		}

		// TODO: integrate m (moment) and n (force)


		// Computing A
		if(isUsingIVPJacobian)
		{
			for(int k = 0; k < numTubes; ++k)
				Rzk_RotZpi_uhatk[k] = Rz[k] * (RotZ_pi * u_hat[k]);

			for(int j = 0; j < numTubes; ++j)
			{
				if(!existingTubeIDs[j])
					continue;

				nuj = 1+nu[j];
				Rzj_uhatj = Rz[j]*u_hat[j];

				for(int k = 0; k < numTubes; ++k)
				{
					if(!existingTubeIDs[k])
						continue;

					A(j,k) = nuj * stiffness[k]/sumkxy * Inner(Rzj_uhatj, Rzk_RotZpi_uhatk[k]);
				
					if (j == k)
						A(j,k) += nuj * Inner(Rzj_uhatj, u);
						/*A(j,k) += (1+nu[j]) * ((Rz[j](0,0)*u_hat[j][0] + Rz[j](0,1)*u_hat[j][1])*u[0]
												+ (Rz[j](1,0)*u_hat[j][0] + Rz[j](1,1)*u_hat[j][1])*u[1]);*/
				}
			}

			Phi_tt_temp = Phi_tt;
			Phi_tt += ds*Phi_ut;
			Phi_ut += ds*A*Phi_tt_temp;
		}

	}
	
	// BVP Jacobian
	if(isUsingIVPJacobian)
	{
		double* translation = this->robot->GetTranslation();
		for(int i = 0; i< numTubes; ++i)
			Phi_ut.row(i) *= translation[0] - translation[i];

		jacobianBC = Phi_tt + Phi_ut;
	}

	// uxy at s = 0
	sumkxy = 0, sumkz = 0;
	sumRKu.SetValues(0,0,0);
	//vector<SO3> Rz(numTubes);
	//vector<Vec3> u_hat(numTubes);
	//std::vector<double> nu(numTubes);
	for (int j = 0; j < numTubes; ++j)
	{
		//double precurvature[3];
		const double* precurvature;
		this->robot->ComputePrecurvature(this->arcLengthGrid[0], j, &precurvature);
		u_hat[j] = Vec3(precurvature[0], precurvature[1], precurvature[2]);

		nu[j] = this->robot->GetPoissonsRatio(j);
		double kz = stiffness[j] / (1 + nu[j]);
		sumkxy += stiffness[j];
		sumkz += kz;

		Rz[j] = RotZ(solution(j, 0)).GetOrientation();
		sumRKu += Rz[j] * Vec3(stiffness[j] * u_hat[j][0], stiffness[j] * u_hat[j][1], kz * u_hat[j][2]);
	}
	sumRKu[0] += solution(2*numTubes,0);
	sumRKu[1] += solution(2*numTubes+1,0);
	sumRKu[2] += solution(2*numTubes+2,0);

	u.SetValues(sumRKu[0] / sumkxy, sumRKu[1] / sumkxy, sumRKu[2] / sumkz);

	solution(solution.rows()-2,0) = u[0];
	solution(solution.rows()-1,0) = u[1];
		


	//std::cout<< "sol column(0) = " << solution.col(0).transpose() << std::endl;
	//std::cout<< "sol column(1) = " << solution.col(1).transpose() << std::endl;
	//std::cout<< "sol = " << solution << std::endl;
}

void MechanicsBasedKinematics::updateBC(Eigen::VectorXd& errorBC)
{
	checkJacobianCondition();

	boundaryConditionTip += this->stepSize * this->jacobianBC.inverse() * errorBC;
	//std::cout << "BC at Tip = [" << boundaryConditionTip.transpose() << "]" << std::endl;
}

void MechanicsBasedKinematics::computeBCJacobian(Eigen::MatrixXd& solution)
{
	if(!isUsingIVPJacobian)
		this->ComputeBCJacobianNumerical(solution);
}


void MechanicsBasedKinematics::printSolution(string filename) const
{
	std::ofstream file(filename);
	file << this->arcLengthGrid.transpose() << std::endl;
	file << this->BVPSolutionGrid;
	file.close();
}

void MechanicsBasedKinematics::printBishopFrame(string filename) const
{
	std::ofstream file(filename);
	for(int i = 0; i < bishopFrames.size(); ++i)
	{
		for(int j = 0; j < 12 ; ++j)
			file << this->bishopFrames[i][j] << "\t";
		file << std::endl;
	}
	file.close();
}


bool MechanicsBasedKinematics::hasBVPConverged(Eigen::MatrixXd& solution, Eigen::VectorXd& errorBC)
{
	int numTubes = this->robot->GetNumOfTubes();

	//double* robotRotation, *robotTranslation;
	//this->robot->GetConfiguration(robotRotation, robotTranslation);
	double* robotRotation = this->robot->GetRotation();
	double* robotTranslation = this->robot->GetTranslation();


	Eigen::VectorXd solutionAtBase = solution.block(0,0, 2 * numTubes , 1);
	//Eigen::VectorXd solutionAtBase = solution.col(0);
	//std::cout << "solution at base = " << solutionAtBase.transpose() << std::endl;
	Eigen::VectorXd estimatedBC;
	estimatedBC.resize(numTubes);

	this->ComputeBCAtBase(solutionAtBase, robotTranslation, estimatedBC);

	Eigen::VectorXd desiredBC = Eigen::Map<Eigen::VectorXd>(robotRotation, numTubes);
	errorBC = desiredBC - estimatedBC;
	double integralPart = 0;
	for(int i = 0; i < 3; ++i)
		errorBC[i] = 2 * M_PI * ::std::modf((errorBC[i] + M_PI)/(2*M_PI), &integralPart) - M_PI;

	//for (int i = 0; i < 3; ++i)
	//	errorBC[i] = atan2(sin(errorBC[i]), cos(errorBC[i])) + M_PI;

	double errorNorm = errorBC.norm();
	
	//std::cout << "desired BC at Base = [" << desiredBC.transpose() << "], estimated BC at base = [" << estimatedBC.transpose() << "]"<< std::endl;
	//std::cout << "estimated BC at base = [" << estimatedBC.transpose() << "]"<< std::endl;
	//std::cout<< "Convergence error = " << errorNorm << std::endl;
	
	if (errorNorm < 0.001)
		return true;

	return false;
}

void MechanicsBasedKinematics::propagateBishopFrame (std::vector<SE3>& bishopFramesToPropagate, Eigen::MatrixXd& solution)
{
	int numGridPoints = this->arcLengthGrid.size();
	if(bishopFramesToPropagate.size() != numGridPoints)
		bishopFramesToPropagate.resize(numGridPoints);

	//double* robotRotation, *robotTranslation;
	//this->robot->GetConfiguration(robotRotation, robotTranslation);
	double* robotRotation = this->robot->GetRotation();
	double* robotTranslation = this->robot->GetTranslation();

	bishopFramesToPropagate[0].SetEye();
	bishopFramesToPropagate[0].SetPosition(Vec3(0, 0, robotTranslation[0]));
	Eigen::Vector2d uxy;
	for(int i = 1; i < numGridPoints; ++i)
	{
		uxy = solution.block(solution.rows()-2,i,2,1);
		bishopFramesToPropagate[i] = bishopFramesToPropagate[i-1] * Exp(se3(uxy[0],uxy[1],0, 0,0,1) * (this->arcLengthGrid[i] - this->arcLengthGrid[i-1]) );
	}

}

void MechanicsBasedKinematics::findNearestGridPoint(double s, int* beginIdx, double* fracFromBegin)
{
	// TODO: change it to be binary search
	for(int i = 0; i < this->arcLengthGrid.size(); ++i)
		if(s < arcLengthGrid[i])
		{
			*beginIdx = i-1;
			*fracFromBegin = (s - arcLengthGrid[i-1]) / (arcLengthGrid[i] - arcLengthGrid[i-1]);
			return;
		}
	
	*beginIdx = this->arcLengthGrid.size() - 1;
	fracFromBegin = 0;

}

void MechanicsBasedKinematics::GetBishopFrame(double s, SE3& bishopFrame, std::vector<SE3>& frames)
{
	try
	{
		int beginIdx;
		double frac;
		this->findNearestGridPoint(s, &beginIdx, &frac);

		if (beginIdx == this->arcLengthGrid.size() - 1)
		{
			bishopFrame = frames[beginIdx];
			return;
		}

		se3 w1 = Log(Inv(frames[beginIdx])*frames[beginIdx+1]);
		bishopFrame = frames[beginIdx] * Exp(frac*w1);
	}
	catch (runtime_error& ex) 
	{
		::std::cout << "Exception in bishop frames"  <<  ex.what() << ::std::endl;
	} 
}

void MechanicsBasedKinematics::ComputeBCJacobianNumerical(Eigen::MatrixXd& solution)
{
	Eigen::VectorXd perturbedBCTip(this->boundaryConditionTip);
		
	//double* translation;
	//this->robot->GetTranslation(translation);
	double* translation = this->robot->GetTranslation();

	Eigen::VectorXd estimatedBCBase, perturbedBCBase;
	estimatedBCBase.resize(this->robot->GetNumOfTubes());
	
	this->ComputeBCAtBase(solution.col(0), translation, estimatedBCBase);
	perturbedBCBase = estimatedBCBase;

	for (int i = 0; i < perturbedBCTip.size(); ++i)
	{
		perturbedBCTip[i] += 0.001;

		this->solveIVP(solution, perturbedBCTip);

		this->ComputeBCAtBase(solution.col(0), translation, perturbedBCBase);
				
		this->jacobianBC.col(i) = (perturbedBCBase - estimatedBCBase) / 0.001;

		perturbedBCTip[i] = this->boundaryConditionTip[i];
	}

	//std::cout << "Jacobian = [" << this->jacobianBC << "]" << std::endl;

}

void MechanicsBasedKinematics::ComputeBCAtBase(const Eigen::VectorXd& solutionAtBase, const double* tubeTranslation, Eigen::VectorXd& estimatedBC)
{
	int numTubes = this->robot->GetNumOfTubes();
	for(int i = 0; i < numTubes; ++i)
		estimatedBC[i] = solutionAtBase[i] - solutionAtBase[i + numTubes] * (tubeTranslation[0] - tubeTranslation[i]);

}

void MechanicsBasedKinematics::Initialize(int numOfGridPoints)
{
	int numTubes = this->robot->GetNumOfTubes();

	this->arcLengthGrid.resize(numOfGridPoints);
	this->normalizedArcLengthGrid.resize(numOfGridPoints);
	this->BVPSolutionGrid.resize(2*numTubes+8, numOfGridPoints);
	this->perturbedSolGrid.resize(2*numTubes+8, numOfGridPoints);
	this->jacobianBC.resize(numTubes, numTubes);

	this->bishopFrames.resize(numOfGridPoints);
	this->perturbedBishopFrames.resize(numOfGridPoints);

	this->boundaryConditionTip.resize(numTubes);

	// set zero
	this->arcLengthGrid.setZero();
	this->BVPSolutionGrid.setZero();
	this->perturbedSolGrid.setZero();
	this->jacobianBC.setZero();
	//this->boundaryConditionTip.setZero();
	this->boundaryConditionTip.setOnes();

	// normalizedArcLengthGrid
	for(int i = 0; i < numOfGridPoints; ++i)
		normalizedArcLengthGrid[i] = 1.0/((double)numOfGridPoints-1.0)*(double)i;
}

void MechanicsBasedKinematics::checkJacobianCondition()
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianBC, Eigen::ComputeThinU | Eigen::ComputeThinV);
	
	double maxSingularValue = svd.singularValues()(0);
	double minSingularValue = svd.singularValues()(this->jacobianBC.cols() - 1);
	double conditionNumber = maxSingularValue / minSingularValue;

	double conditionThreshold = 10;
	if(conditionNumber > conditionThreshold)
	{
		//double epsilon = conditionThreshold * minSingularValue - maxSingularValue;
		//epsilon /= 1 - conditionThreshold;
		//this->jacobianBC += epsilon * svd.matrixU() * svd.matrixV().transpose();
		for(int i = 0 ; i < this->jacobianBC.rows() ; ++i)
			this->jacobianBC(i,i) += 0.1*maxSingularValue;
		//this->jacobianBC += epsilon * svd.matrixU() * svd.matrixV().transpose();

		//
		//Eigen::JacobiSVD<Eigen::MatrixXd> svd2(this->jacobianBC, Eigen::ComputeThinU | Eigen::ComputeThinV);
	
		//double maxSingularValue2 = svd2.singularValues()(0);
		//double minSingularValue2 = svd2.singularValues()(this->jacobianBC.cols() - 1);
		//double conditionNumber2 = maxSingularValue2 / minSingularValue2;

		//std::cout<< "condition number = " << conditionNumber2 << std::endl;

	}


}

// This is not really general for any robot. It assumes a 3-tube robot whose first two tubes form a balanced pair
void MechanicsBasedKinematics::RelativeToAbsolute(const CTR* const robot, const double* const& relativeConf, double* const &rotation, double* const &translation)
{
	int numOfTubes = robot->GetNumOfTubes();
	double collarLength = robot->GetTubes().front().GetCollarLength();
	double translationLowerLimit = robot->GetLowerTubeJointLimits()[numOfTubes - 1];

	rotation[0] = relativeConf[3];
	translation[0] = relativeConf[4];

	rotation[1] = relativeConf[0] + rotation[0];
	rotation[2] = relativeConf[1] + rotation[0];
	
	translation[1] = -collarLength + translation[0];
	translation[2] = relativeConf[2] -35 + translation[0] -2*collarLength;

	//::std::cout << "tube 3 relative: " << relativeConf[2] << " - rigid body:" << relativeConf[4] << ::std::endl;
	//PrintCArray(translation, 3);


}