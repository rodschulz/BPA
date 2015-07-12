/**
 * Author: rodrigo
 * 2015
 */
#include "Config.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <vector>
#include <boost/algorithm/string.hpp>

Config::Config()
{
	level = NONE;
	radius = 0.002;
	spheres = true;
}

Config::~Config()
{
}

void Config::load(const std::string &_filename)
{
	std::string line;
	std::ifstream inputFile;
	inputFile.open(_filename.c_str(), std::fstream::in);
	if (inputFile.is_open())
	{
		while (getline(inputFile, line))
		{
			// Parse string line
			std::vector<std::string> tokens;
			std::istringstream iss(line);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

			parse(tokens[0], tokens[1]);
		}
		inputFile.close();
	}
	else
		std::cout << "Unable to open input: " << _filename;
}

void Config::parse(const std::string _key, const std::string _value)
{
	if (boost::iequals(_key, "ballRadius"))
		getInstance()->radius = atof(_value.c_str());
	else if (boost::iequals(_key, "debug"))
	{
		if (boost::iequals(_value, "low"))
			getInstance()->level = LOW;
		else if (boost::iequals(_value, "medium"))
			getInstance()->level = MEDIUM;
		else if (boost::iequals(_value, "high"))
			getInstance()->level = HIGH;
		else
			getInstance()->level = NONE;
	}
	else if (boost::iequals(_key, "drawSpheres"))
	{
		getInstance()->spheres = boost::iequals(_value, "true");
	}
}

double Config::getBallRadius()
{
	return getInstance()->radius;
}

bool Config::drawSpheres()
{
	return getInstance()->spheres;
}
