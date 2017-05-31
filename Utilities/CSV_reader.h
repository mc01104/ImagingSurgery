#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>



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

}

std::istream& operator>>(std::istream& str,csv::CSVRow& data);

class ParseOptions
{
public:
	ParseOptions(std::string options_file);
	ParseOptions();
	~ParseOptions();

    void parseFile(std::string options_file);
    void readHeaders();
    bool isHeaderPresent(std::string name);

    bool getData(std::string header_name, std::vector<std::string>& return_value);
    bool getData(std::string header_name, std::vector<float>& return_value);


private:

    std::map<std::string,csv::CSVRow> m_headers;
    std::vector<csv::CSVRow> m_csv_rows;

    bool m_file_parsed;
    bool m_headers_parsed;
    bool m_data_parsed;
};
