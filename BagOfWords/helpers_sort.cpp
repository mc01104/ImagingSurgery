#include "helpers_sort.h"

bool is_not_digit(char c)
{
    return !std::isdigit(c);
}


bool numeric_string_compare(const std::string& s1, const std::string& s2)
{
    // handle empty strings...

	const std::string s1_ = s1.substr(0, s1.find_last_of("."));
	const std::string s2_ = s2.substr(0, s2.find_last_of("."));

    std::string::const_iterator it1 = s1_.begin(), it2 = s2_.begin();

    if (std::isdigit(s1_[0]) && std::isdigit(s2_[0])) {

		double n1 = 0;
		std::istringstream ss(s1_);
		ss >> n1;

		double n2 = 0;
		std::istringstream ss2(s2_);
		ss2 >> n2;

        if (n1 != n2) return n1 < n2;

        it1 = std::find_if(s1_.begin(), s1_.end(), is_not_digit);
        it2 = std::find_if(s2_.begin(), s2_.end(), is_not_digit);
    }

    return std::lexicographical_compare(it1, s1_.end(), it2, s2_.end());
}