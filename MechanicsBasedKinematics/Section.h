#pragma once

#include "Part.h"

class Section :  public Part
{
friend class CTRFactory;

public:
	Section(double _sectionLength, double _precurvature[3]);
	double GetSectionLength() const;
	const double* GetPrecurvature() const;
	
private:
	double sectionLength;
	double precurvature[3];

	
};
