#include "CTR.h"
#include <iostream>

#define MAX_TRANSLATION 100000000000000000

CTR::CTR(): numTubes(0), length(0.0)
{
}

CTR::~CTR()
{
	delete tubeRotation, tubeTranslation, upperTubeTranslationLimit, lowerTubeTranslationLimit;
}

void CTR::UpdateLength ()
{
	length = (--tubes.end())->GetTubeLength() - (tubeTranslation[0] - tubeTranslation[numTubes-1]);
}

void CTR::ComputeJointLimits ()
{
	double accCollarLength = this->tubes[0].GetCollarLength();
	for(int i = 1; i < this->numTubes; ++i)
	{	
		this->upperTubeTranslationLimit[i] = -accCollarLength + this->tubeTranslation[0];
		this->lowerTubeTranslationLimit[i] = this->tubes[0].GetTubeLength() - this->tubes[i].GetTubeLength() + this->tubeTranslation[0];

		accCollarLength += this->tubes[i].GetCollarLength();
	}
}

bool CTR::CheckJointLimits(const double* translation) const
{
	for( int i = 0; i < this->numTubes; ++i)
		if(translation[i] < this->lowerTubeTranslationLimit[i] || translation[i] > this->upperTubeTranslationLimit[i])
		{
			std::cout << "Out of joint limit!" << std::endl;
			return false;
		}

	return true;
}

// After initialization, stop adding tubes.
void CTR::Initialize ()
{
	this->tubeRotation = new double[numTubes];
	this->tubeTranslation = new double[numTubes];
	for( int i = 0; i < numTubes; ++i)
		this->tubeRotation[i] = this->tubeTranslation[i] = 0.0;
	

	this->upperTubeTranslationLimit = new double[numTubes];
	this->lowerTubeTranslationLimit = new double[numTubes];

	upperTubeTranslationLimit[0] = MAX_TRANSLATION;
	lowerTubeTranslationLimit[0] = -MAX_TRANSLATION;

    this->ComputeJointLimits();
}

bool CTR::TubeExists (double s, int tubeID) const
{
	if( s < 0)
		return false;

	if( s > tubes[tubeID].GetTubeLength() - (tubeTranslation[0] - tubeTranslation[tubeID]) )
		return false;

	return true;
}

bool CTR::UpdateConfiguration (const double* rotation, const double* translation)
{
	memcpy(this->tubeRotation, rotation, sizeof(double)*this->numTubes);
	memcpy(this->tubeTranslation, translation, sizeof(double)*this->numTubes);

	this->ComputeJointLimits();
	if(!this->CheckJointLimits(translation))
		return false;

	this->UpdateLength();

	return true;
}

void CTR::AddTube (Tube tube)
{
    this->numTubes++;
	this->tubes.push_back(tube);
}

bool CTR::ComputePrecurvature (double s, int tubeID, const double* precurvature[3])
{
	if(!this->TubeExists(s, tubeID))
		return false;

    std::vector<Section>& sections = this->tubes[tubeID].GetSections();
	 
	double accSectionLength = 0;
	double relativeTrans = (this->tubeTranslation[0] - this->tubeTranslation[tubeID]);
	for(std::vector<Section>::const_iterator it = sections.begin(); it != sections.end(); ++it)
	{
		accSectionLength += it->GetSectionLength();
		if(s + relativeTrans <= accSectionLength)
		{
			//const double* sectionPrecurvature = it->GetPrecurvature();
			//memcpy(precurvature, sectionPrecurvature, sizeof(double)*3);
			*precurvature = it->GetPrecurvature();

			return true;
		} 
	}

	return false;
}

const std::vector<Tube>& CTR::GetTubes () const
{
    return this->tubes;
}

void CTR::GetExistingTubes(const double s, std::vector<bool>& tubeIDs) const
{
	//tubeIDs.clear();
	for(int i = 0; i < this->numTubes; ++i)
		if(this->TubeExists (s, i))
			tubeIDs[i] = true;
}