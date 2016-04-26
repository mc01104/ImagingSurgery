#pragma once

#include <string>

class Parameter
{
public:
	Parameter(): value(0.0), isFree(false), name("") {}
	Parameter(double _value, bool _isFree = false, std::string _name = ""): value(_value), isFree(_isFree), name(_name) {}

public:
    double value;
    bool isFree;
    std::string name;
};
