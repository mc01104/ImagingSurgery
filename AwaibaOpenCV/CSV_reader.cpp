#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "CSV_reader.h"

using namespace csv;

std::string const& CSVRow::operator[](std::size_t index) const
{
    return m_data[index];
}
        
std::size_t CSVRow::size() const
{
    return m_data.size();
}


void CSVRow::readNextRow(std::istream& str)
{
    std::string         line;
    std::getline(str,line);

    std::stringstream   lineStream(line);
    std::string         cell;

    m_data.clear();
    while(std::getline(lineStream,cell,','))
    {
		if(!(cell[0] == '%')) // comment
			m_data.push_back(cell);
    }
}
    
std::istream& operator>>(std::istream& str,CSVRow& data)
{
    data.readNextRow(str);
    return str;
}   


ParseOptions::ParseOptions(std::string options_file)
{
	m_parsed = true;
	try
	{

		std::ifstream file(options_file);
		csv::CSVRow row;
		while(file >> row)
		{
			if (row.size()>0)
			{
				if (row[0] == "whitebal")
				{
					float g_r = stof(row[1]);
					float g_g = stof(row[2]);
					float g_b = stof(row[3]);
					m_whiteBal.push_back(g_r);
					m_whiteBal.push_back(g_g);
					m_whiteBal.push_back(g_b);
				}
				if (row[0] == "savedir")
				{
					m_saveDir = row[1];
				}
				if (row[0] == "rotation")
				{
					m_rotation = stof(row[1]);
				}

			}
		}
	}
	catch(...)
	{
		m_parsed = false;
		std::cout << "Error reading the CSV file" << std::endl;
	}
}

ParseOptions::ParseOptions(){}
ParseOptions::~ParseOptions(){}


std::string ParseOptions::getSaveDir()
{
	return m_saveDir;
}
std::vector<float> ParseOptions::getWhiteBalance()
{
	return m_whiteBal;
}
float ParseOptions::getRotation()
{
	return m_rotation;
}
bool ParseOptions::getStatus()
{
	return m_parsed;
}




