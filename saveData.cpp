
#include"saveData.h"
#include<fstream>
#include <iostream>

SaveData::SaveData(const int nImu, const int nVal, bool imuQ, bool viconQ)
{
	numImu = nImu;
	numVal = nVal;
	std::cout << "imuRawData ";
	viconQuaternion = NULL;
	imuQuaternion = NULL;
	values = NULL;
	
	if (nImu > 0) 
	{
		values = new double*[nImu];
		imuStarted = new bool[nImu];
		initailTimestamp = new uint32_t[nImu];
		timestamp = new  uint32_t[nImu];
		log = new std::forward_list<std::string>[nImu];

		for (int i = 0; i < nImu; i++)
		{
			values[i] = new double[nVal];
			imuStarted[i] = false;
			initailTimestamp[i] = UINT32_MAX;
			timestamp[i] = 0;
		}

		if (imuQ) {
			std::cout << "imuQuaternion "; ;
			imuQuaternion = new double*[nImu];
			for (int i = 0; i < nImu; i++)
			{
				imuQuaternion[i] = new double[4];
			}
		}
	}

	if(viconQ){
		std::cout << "viconQuaternion ";

		viconQuaternion = new double*[nImu];
		for (int i = 0; i < nImu; i++)
		{
			viconQuaternion[i] = new double[4];
		}
	}
		
}

void SaveData::setValues(int i, double* val, double* imuQuat, double*viconQuat, uint32_t tstamp)
{
	mutex.lock();
	if(val != NULL)
		memcpy(values[i], val, numVal * sizeof(double));
	if(imuQuat!= NULL)
		memcpy(imuQuaternion[i], imuQuat, 4 * sizeof(double));
	if(viconQuat != NULL)
		memcpy(viconQuaternion[i], viconQuat, 7 * sizeof(double));
	mutex.unlock();

	// Barcelli's soloution. Non va
	/*
	int sumStarted = 0;
	for (int k = 0; k < numImu; k++)
	{
		sumStarted += imuStarted[k];
	}
	//std::cout << "Imu:" << i << " sum: " << sumStarted << std::endl;

	if (sumStarted < numImu)
	{
		initailTimestamp[i] = tstamp;
		imuStarted[i] = true;
	}
	else {
		timestamp[i] = tstamp - initailTimestamp[i];

		sampleLog(i);
	}
	*/

	/* Tommi */
	
	if (initailTimestamp[i] == UINT32_MAX)
	{
		initailTimestamp[i] = tstamp;
	}
	else
		timestamp[i] = tstamp - initailTimestamp[i];

	int sumTs = 0;
	for (int k = 0; k < numImu; k++) {
		sumTs += timestamp[i];
	}
	if (sumTs)
		sampleLog(i);
	//*/


	
}

void SaveData::setValues(int i, double* val, double*imuQuat, uint32_t tstamp)
{
	setValues(i,val,imuQuat,NULL,tstamp);

}

void SaveData::setValues(int i, double* val, uint32_t tstamp)
{
	setValues(i, val, NULL, NULL, tstamp);

}

void SaveData::sampleLog(int j) {
	std::string str ="";
	mutex.lock();
	str += std::to_string(timestamp[j]) + " ";
	//for (int j = 0; j < numImu; j++) {

		for (int i = 0; i < numVal; i++)
		{
			str += std::to_string(values[j][i])+ " ";
		}

		if (imuQuaternion!= NULL)
		{	
		
			for (int i = 0; i < 4; i++)
			{
				str += std::to_string(imuQuaternion[j][i]) + " ";
			}
		}

		if (viconQuaternion != NULL)
		{
			for (int i = 0; i < 7; i++)
			{
				str += std::to_string(viconQuaternion[j][i]) + " ";
			}
		}

	//	str += "; ";
	//}
	mutex.unlock();
	str = str.substr(0, str.length() - 1);
	log[j].push_front(str);
}


void SaveData::saveLog(std::string file) {
	
	std::ofstream f;
	f.open(file, std::fstream::out);

	for (int i = 0; i < numImu; i++)
	{
		log[i].reverse();
	}

	std::forward_list<std::string>::iterator *it = new std::forward_list<std::string>::iterator[numImu];
	for (int i = 0; i<numImu; ++i)
		it[i] = log[i].begin();
	bool endMe = false;
	while (!endMe)
	{
		//controllo che nessuna lista sia finita
		for (int i = 0; i < numImu; ++i)
			if (it[i] == log[0].end())
				endMe = true;
		if (endMe)
			break;

		//USO LE MIE numImu LISTE
		for (int i=0; i < numImu;i++)
		{
			f << *it[i]<< " ; ";
			
		}
		f << std::endl;
		//scorro tutte le liste
		for (int i = 0; i < numImu; ++i)
			++it[i];
	}
	/*
		for (auto it = log[0].begin(); it != log[0].end(); ++it)
			f << *it << "\n";
	*/
	f.close();
}
