#pragma once

#include <forward_list>
#include <string>
#include<mutex>

class SaveData
{

private:
	std::forward_list<std::string> *log;
	int numVal;
	int numImu;
	double **values;
	double **imuQuaternion;
	double **viconQuaternion;

	bool *imuStarted;
	uint32_t *initailTimestamp;
	uint32_t *timestamp;

	void sampleLog(int j);
	std::mutex mutex;

public:
	void saveLog(std::string file);
	void setValues(int i, double* val, double* viconQuat, double*imuQuat, uint32_t tstamp);
	void setValues(int i, double* val, double*imuQuat, uint32_t tstamp);
	void setValues(int i, double* val, uint32_t tstamp);

	SaveData(int nImu, int nVal, bool, bool);

};