#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
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


void ParseOptions::parseFile(std::string options_file)
{
    try
    {
        std::ifstream file(options_file);
        csv::CSVRow row;
        while(file >> row)
        {
            m_csv_rows.push_back(row);
        }
    }
    catch(...)
    {
        m_file_parsed = false;
        std::cout << "Error reading the CSV file" << std::endl;
    }

    m_file_parsed = true;
}


void ParseOptions::readHeaders()
{
    if (!m_file_parsed) m_headers_parsed=false;

    else
    {
        try
        {
            for (csv::CSVRow row : m_csv_rows)
            {
                if (row.size()>0) // need to have at least a header and a value
                {
                    std::string name = row[0];
                    m_headers[name] = row;
                }
            }
        }
        catch(...)
        {
            m_headers_parsed = false;
            std::cout << "Error reading the CSV file" << std::endl;
        }

        m_headers_parsed = true;
    }
}


bool ParseOptions::isHeaderPresent(std::string name)
{

    if (!m_headers_parsed) return false;

    try
    {
        std::map<std::string,csv::CSVRow>::iterator it;
        it = m_headers.find(name);

        if (it != m_headers.end())
        {
            return true;
        }
        else return false;
    }
    catch(...)
    {
        return false;
    }
}


bool ParseOptions::getData(std::string header_name, std::vector<std::string>& return_value)
{
    return_value.clear();

    if (isHeaderPresent(header_name))
    {
        try
        {
            csv::CSVRow row = m_headers[header_name];

            for (int i=1;i<row.size();i++)
            {
                return_value.push_back(row[i]);
            }
            return true;
        }
        catch(...)
        {
            return false;
        }
    }
    else return false;
}


bool ParseOptions::getData(std::string header_name, std::vector<float>& return_value)
{

    return_value.clear();

    if (isHeaderPresent(header_name))
    {
        try
        {
            csv::CSVRow row = m_headers[header_name];

            for (int i=1;i<row.size();i++)
            {
                return_value.push_back(stof(row[i]));
            }
            return true;

        }
        catch(...)
        {
            return false;
        }
    }
    else return false;
}





ParseOptions::ParseOptions(std::string options_file)
{
	
    m_headers_parsed = false;
    m_file_parsed = false;
    m_data_parsed = false;

	try
	{
        parseFile(options_file);
        readHeaders();
	}
	catch(...)
	{
        m_file_parsed = false;
		std::cout << "Error reading the CSV file" << std::endl;
	}
}

ParseOptions::ParseOptions()
{
    m_headers_parsed = false;
    m_file_parsed = false;
    m_data_parsed = false;
}
ParseOptions::~ParseOptions(){}



