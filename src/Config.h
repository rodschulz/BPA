/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>

enum DebugLevel
{
	NONE, LOW, MEDIUM, HIGH
};

class Config
{
public:
	~Config();

	// Returns the instance of the singleton
	static Config *getInstance()
	{
		static Config *instance = new Config();
		return instance;
	}
	// Loads the configuration file
	static void load(const std::string &_filename);

	static inline DebugLevel getDebugLevel()
	{
		return getInstance()->level;
	}

	static double getBallRadius();
	static bool drawSpheres();

private:
	Config();

	// Parses the given value according to the given key
	static void parse(const std::string _key, const std::string _value);

	DebugLevel level;
	double radius;
	bool spheres;
};

