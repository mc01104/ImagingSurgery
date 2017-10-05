#include "Tube.h"

Tube::Tube(double bendingStiffness, double PoissonsRatio, std::vector<Section> _sections)
	: kxy(bendingStiffness), nu(PoissonsRatio), length(0.0), collarLength(17)
{
	if(!_sections.empty())
		this->sections = _sections;
}

std::vector<Section>& Tube::GetSections ()
{
    return this->sections;
}

void Tube::UpdateLength ()
{
	double L = 0;
	for ( std::vector<Section>::iterator it = this->sections.begin(); it != this->sections.end(); it++)
		L += it->GetSectionLength();

	this->length = L;
}

void Tube::AddSection (Section section)
{
	this->sections.push_back(section);
	this->UpdateLength();
}