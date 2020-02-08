#pragma once

#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <fstream>
#include <sstream>
#include <string>
#include <conio.h>
#include <iostream>
#include <iomanip>

#include <queue>          // std::queue
#include <list>


using namespace Eigen;
using namespace std;

struct acquisition {
	uint32_t timestamp;
	Vector3d gyroscope;
	Vector3d accelerometer;
	Vector4d vicon;
//public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
	
class mekfQueue : public list<acquisition> {
private:
	unsigned mySize;

public:
	mekfQueue(int s)
	{
		mySize = s;
	};

	mekfQueue() {};

	void setSize(int s)
	{
		mySize = s;
	};


	void insert(acquisition &newAcquisition) {
		if (this->size() < mySize)
		{
			this->push_back(newAcquisition);
		}
		else
		{
			this->pop_front();
			this->push_back(newAcquisition);
		}
	};


	int computeVariance(Eigen::Vector3d* accVariance, Eigen::Vector3d* gyroVariance) {

		if (this->size() < mySize) 
		{
			accVariance->setZero();
			gyroVariance->setZero();
			return 0;
		}
		else
		{
			Eigen::Vector3d tmpAverageAcc(0,0,0 );
			Eigen::Vector3d tmpAverageGyr(0, 0, 0);

			Eigen::Vector3d tmpVarianceAcc(0, 0, 0);
			Eigen::Vector3d tmpVarianceGyr(0, 0, 0);


			for (std::list<acquisition>::iterator it = this->begin(); it != this->end(); it++)
			{
				tmpAverageAcc += (*it).accelerometer /mySize;
				tmpAverageGyr += (*it).gyroscope / mySize;
			}

			for (std::list<acquisition>::iterator it = this->begin(); it != this->end(); it++)
			{
				tmpVarianceAcc += ((*it).accelerometer - tmpAverageAcc).asDiagonal()*(((*it).accelerometer - tmpAverageAcc))/ mySize;

				tmpVarianceGyr += ((*it).gyroscope - tmpAverageGyr).asDiagonal()*((*it).gyroscope - tmpAverageGyr) / mySize;
			}

			(*accVariance) = tmpVarianceAcc;
			(*gyroVariance) = tmpVarianceGyr;
		}



	}


};

class mekf
{
private:



	void PrintAcquisition(acquisition* a); 

	Matrix3d skew_symmetric(Vector3d a);
	Quaterniond angularspeed2quaternion(Vector3d a, double t);

	acquisition raw_data;
	acquisition raw_data_old;


	//calibration 
	double accelerometer_magnitude;
	Vector3d accelerometer_variance;
	Vector3d gyroscope_bias;
	Vector3d gyroscope_variance;
	Vector3d gyroscope_bias_variance;

	Vector3d M2_accelerometer_variance;
	Vector3d delta_accelerometer_variance;
	Vector3d mean_accelerometer;
	// Gyroscope
	Vector3d M2_gyroscope_variance;
	Vector3d delta_gyroscope_variace;

	Vector3d diff_measure;
	Vector3d M2_gyroscope_bias_variance;
	Vector3d delta_gyroscope_bias_variance;
	Vector3d mean_gyroscope_bias_variance;
	Vector3d reference_acceleration;

	VectorXd y;
	int stationary_samples;


	//mekf
	int S;
	MatrixXd Q, R;

	MatrixXd P;
	VectorXd x;

	// Correction
	double accelerometer_magnitude_rolling_mean;
	Vector3d gyroscope_rolling_mean;
	mekfQueue rawValuesQueue;

	Vector3d local_a;
	Vector3d local_vb;
	VectorXd local_h;
	VectorXd local_x;
	MatrixXd local_P, local_K, local_K_aux;


	bool filterRunning;
	Quaterniond q;

	int t;
	int iCalibration;

	bool firstcicleCalibration;

public:


	mekf();
	

	void mekfOnline();
	void calibrationOnline();
	void setRawData(double* rawValues, uint32_t timestamp);
	void getQuaternion(double* quaternion);
	void getGyroscopeBias(Vector3d*);
	void setCovarianceMatrices();
	void resetQuaternion();
	void setStationary_samples(int);


	void updateCalibration();
	void stopCalibration();
	void get_a(Vector3d*);
	void get_vb(Vector3d*);
	void get_h(VectorXd*);
	void get_x(VectorXd*);
	void get_P(MatrixXd*);
	void get_K(MatrixXd*);
	void get_K_aux(MatrixXd*);

};
