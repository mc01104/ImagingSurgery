#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


namespace csv
{
	class CSVRow
	{
		public:
			std::string const& operator[](std::size_t index) const;
			std::size_t size() const;
			void readNextRow(std::istream& str);
		private:
			std::vector<std::string>    m_data;
	};

};

std::istream& operator>>(std::istream& str,csv::CSVRow& data);

class ParseOptions
{
public:
	ParseOptions(std::string options_file);
	ParseOptions();
	~ParseOptions();

	std::string getSaveDir();
	std::vector<float> getWhiteBalance();
	float getRotation();
	bool getStatus();

private:
	std::string m_saveDir;
	std::vector<float> m_whiteBal;
	float m_rotation;
	bool m_parsed;
};