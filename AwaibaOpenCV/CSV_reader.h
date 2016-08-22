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
	std::string getSVMDir();
	std::vector<float> getWhiteBalance();
	std::vector<float> getKFParams();
	float getRotation();
	std::string getIPAddress();
	bool getRenderShape();
	bool getStatus();

private:
	std::string m_saveDir;
	std::string m_SVMDir;
	std::vector<float> m_whiteBal;
	std::vector<float> m_KFParams;
	float m_rotation;
	std::string m_ipaddress;
	bool m_rendershape;
	bool m_parsed;
};