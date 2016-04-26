#include "Section.h"

Section::Section(double _sectionLength, double _precurvature[3])
	: sectionLength(_sectionLength)
{
	memcpy(this->precurvature, _precurvature, sizeof(double)*3);
}

double Section::GetSectionLength() const
{
	return sectionLength;
}

const double* Section::GetPrecurvature() const
{
	return precurvature;
}