#include"mekf.h"

mekf::mekf(bool HR) {
	if (HR)
		stationary_samples = 500;
	else
		stationary_samples = 50;

	t = 0;
	iCalibration = 0;
	firstcicleCalibration = false;
	q.setIdentity();
	
	//std::cout << q.coeffs() << std::endl;

	raw_data.accelerometer = {0,0,0};
	raw_data.gyroscope = { 0,0,0 };
	raw_data.vicon = {1, 0,0,0 };
	raw_data.timestamp = 0;
	//raw_data_old = new acquisition;

	accelerometer_magnitude = 0.0;
	accelerometer_variance.setZero();
	gyroscope_variance.setZero();
	gyroscope_bias.setZero();
	gyroscope_bias_variance.setZero();

	M2_accelerometer_variance.setZero();
	delta_accelerometer_variance.setZero();
	mean_accelerometer.setZero();
	M2_gyroscope_variance.setZero();
	delta_gyroscope_variace.setZero();
	diff_measure.setZero();
	M2_gyroscope_bias_variance.setZero();
	delta_gyroscope_bias_variance.setZero();
	mean_gyroscope_bias_variance.setZero();

	S = 6;
	P = MatrixXd::Zero(S, S);
	P.setIdentity();
	P = 0.001 * P;
	x = VectorXd::Zero(6,1);
	y = VectorXd::Zero(6, 1);
	Q = MatrixXd::Zero(S, S);
	R = MatrixXd::Zero(S, S);


	// Correction
	accelerometer_magnitude_rolling_mean = 0;
	gyroscope_rolling_mean.setZero();
	rawValuesQueue.setSize(stationary_samples);
	local_a = { 0,0,0 };
	local_vb = { 0,0,0 };
	local_h = VectorXd::Zero(6, 1);
	local_x = VectorXd::Zero(6, 1);
	local_P = MatrixXd::Zero(S, S);
	local_K = MatrixXd::Zero(S, S);
	local_K_aux = MatrixXd::Zero(S, S);

};


mekf::mekf(string s) {

	infile_for_numbering.open(s);
	infile.open(s);

	string line_for_numbering;
	number_of_lines = 0;
	while (getline(infile_for_numbering, line_for_numbering)) {
		number_of_lines += 1;
	};
	std::cout << "Number of samples: " << number_of_lines << endl;
	infile_for_numbering.close();


};


void mekf::calibrationOnline()
{

	// Calibration
	if(iCalibration == 0)
		std::cout << "Calibrating.... Press c to end the calibration procedure" << endl;

	// Accelerometer

	double aux_magnitude;
	double delta_magnitude;
	
		// Accelerometer
		aux_magnitude = raw_data.accelerometer.norm();
		delta_magnitude = aux_magnitude - accelerometer_magnitude;
		accelerometer_magnitude += delta_magnitude / (iCalibration + 1);
		delta_accelerometer_variance = raw_data.accelerometer - mean_accelerometer;
		mean_accelerometer += delta_accelerometer_variance / (iCalibration + 1);
		M2_accelerometer_variance += delta_accelerometer_variance.cwiseProduct(raw_data.accelerometer - mean_accelerometer);
		accelerometer_variance = M2_accelerometer_variance / iCalibration;

		// Gyroscope
		delta_gyroscope_variace = raw_data.gyroscope - gyroscope_bias;
		gyroscope_bias += delta_gyroscope_variace / (iCalibration + 1);
		M2_gyroscope_variance += delta_gyroscope_variace.cwiseProduct(raw_data.gyroscope - gyroscope_bias);
		gyroscope_variance = M2_gyroscope_variance / iCalibration;

		// Gyroscope bias drift variance
		if (iCalibration>0) {
			diff_measure = raw_data.gyroscope - raw_data_old.gyroscope;
			delta_gyroscope_bias_variance = diff_measure - mean_gyroscope_bias_variance;
			mean_gyroscope_bias_variance += delta_gyroscope_bias_variance / (iCalibration);
			M2_gyroscope_bias_variance += delta_gyroscope_bias_variance.cwiseProduct(diff_measure - mean_gyroscope_bias_variance);
			gyroscope_bias_variance = M2_gyroscope_bias_variance / (iCalibration - 1);
		}

	iCalibration++;

}


void mekf::mekfOnline() {
	
	// minimum stationary time for correction
	float g_acc = 0.1;

	y.setZero();

	Vector3d a(0, 0, 0);
	Vector3d omega(0, 0, 0);
	MatrixXd F(6, 6), F_aux(6, 6), G(6, 6), G_aux(6, 6), eye6(6, 6), eye3(3, 3);
	F.setZero();
	F_aux.setZero();
	G.setZero();
	G_aux.setIdentity();
	G_aux.block(0, 0, 3, 3) *= -1;
	eye6.setIdentity();
	eye3.setIdentity();
	F_aux.block(0, 3, 3, 3) = -eye3;

	Vector3d vB;
	VectorXd h(6, 1);
	vB.setZero();
	h.setZero();
	MatrixXd H(6, 6), K(6, 6), K_aux(6, 6);
	H.setIdentity();
	K.setZero();
	K_aux.setZero();

	Vector3d gyroscope(0, 0, 0), accelerometer(0, 0, 0);

	//cout << G_aux << endl;
	int l = 1, c = 1;
	bool prediction_on = true;
	//double accelerometer_magnitude_rolling_mean = 0;

	double dt;
	bool condition_accelerometer_magnitude, condition_gyroscope, condition_acc_variance, condition_gyro_variance;
	
	//ofstream quaternion_output;
	//quaternion_output.open("quaternion_output.txt");
	//quaternion_output << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
	

	rawValuesQueue.insert(raw_data);


	//for (int t = 0; t<number_of_lines - 1; t++) {
	//while(filterRunning){
	// Prediction step
	dt = (raw_data.timestamp - raw_data_old.timestamp) * 1e-4;
	x.head(3) << 0, 0, 0;
	gyroscope = raw_data.gyroscope - gyroscope_bias; // remove bias from gyroscope data
	omega = gyroscope - x.tail(3);
	F_aux.block(0, 0, 3, 3) = -skew_symmetric(omega);

	F = eye6 + dt * F_aux;
	G = dt * G_aux;

	local_P = P;
	//prediction_on = true;
	if (prediction_on) {
		P = F * P * F.transpose() + G * Q * G.transpose();
	}
	q = q * angularspeed2quaternion(omega, dt);


	// End of prediction step

	// Correction check
	if (t<stationary_samples) {
		accelerometer_magnitude_rolling_mean = (accelerometer_magnitude_rolling_mean*t + raw_data.accelerometer.norm()) / (t + 1);
		//std::cout << accelerometer_magnitude_rolling_mean<< " = " << accelerometer_magnitude_rolling_mean*t << " t "<< t << " raw_data.accelerometer.norm() "<< raw_data.accelerometer.norm()<< " (t+1) "<<(t+1)<< std::endl;
		gyroscope_rolling_mean += gyroscope / stationary_samples;
	}
	else {
		accelerometer_magnitude_rolling_mean = (accelerometer_magnitude_rolling_mean*(stationary_samples - 1) + raw_data.accelerometer.norm()) / stationary_samples;
		gyroscope_rolling_mean = (gyroscope_rolling_mean*(stationary_samples - 1) + gyroscope) / stationary_samples;

		condition_accelerometer_magnitude = abs((accelerometer_magnitude_rolling_mean - accelerometer_magnitude) / accelerometer_magnitude) <= 0.01;
		condition_gyroscope = gyroscope_rolling_mean.norm() <= 0.01;


		Vector3d gyrVariance(0, 0, 0);
		Vector3d accVariance(0, 0, 0);
		rawValuesQueue.computeVariance(&accVariance, &gyrVariance);

		condition_acc_variance = true; // ((accVariance.array() <= accelerometer_variance.array()));
		condition_gyro_variance = true; // ((gyrVariance.array() <= gyroscope_variance.array()).isOnes());
			
		for (int i = 0; i < 3; i++) {
			if (accVariance(i) > accelerometer_variance(i)) {
				condition_acc_variance = false;
			}

			if (gyrVariance(i) > gyroscope_variance(i)) {
				condition_gyro_variance = false;
			}
		} 

		
	}
	//End of correction check




	// Correction
	if (t >= stationary_samples && condition_accelerometer_magnitude && condition_acc_variance && condition_gyro_variance) 
	{
			
		prediction_on = false;
		y.head(3) = raw_data.accelerometer;
		y.tail(3) = gyroscope;
		vB = q.toRotationMatrix().transpose() * reference_acceleration;
		h.head(3) = vB;
		h.tail(3) = x.tail(3);
			
		H.block(0, 0, 3, 3) = skew_symmetric(vB);

		K_aux = H * P * H.transpose() + R;
		K = P * H.transpose() * K_aux.inverse();
		x = x + K * (y - h);
		P = P - K * K_aux * K.transpose();
		a = x.head(3);

		local_h = h;
		local_a = a;
		local_vb = vB;
		local_x = x;
		local_K = K;
		local_K_aux = K_aux;

		//a(2) = 0;
		q = q * angularspeed2quaternion(a, 1);
	}
	else {
		prediction_on = true;
	}
	t++;
//}


}




Matrix3d mekf::skew_symmetric(Vector3d a)
{
	Matrix3d A;
	A << 0, -a(2), a(1),
		a(2), 0, -a(0),
		-a(1), a(0), 0;
	return A;
}


Quaterniond mekf::angularspeed2quaternion(Vector3d a, double t) {
	Quaterniond Q;
	double aux = a.norm()*t / 2;
	Q.w() = cos(aux);
	Q.x() = a(0) / a.norm()*sin(aux);
	Q.y() = a(1) / a.norm()*sin(aux);
	Q.z() = a(2) / a.norm()*sin(aux);

	return Q;
}

void mekf::PrintAcquisition(acquisition* a)
{
	cout << a->timestamp << " " << a->gyroscope << " " << a->accelerometer << " " << a->vicon << endl;
}

void mekf::setFilterRunning(bool b)
{
	filterRunning = b;
}


void mekf::setRawData(double* rawValues, uint32_t timestamp)
{
	raw_data_old = raw_data;
	raw_data.timestamp= timestamp;
	for (int i = 0; i < 3; i++)
	{
		raw_data.accelerometer(i) =  rawValues[i + 3];
		raw_data.gyroscope(i) = rawValues[i];
	}

}

void mekf::getQuaternion(double* quaternion)
{
	quaternion[0] = q.w();
	quaternion[1] = q.x();
	quaternion[2] = q.y();
	quaternion[3] = q.z();

}

void mekf::getGyroscopeBias(Vector3d* gyroBias)
{
	(*gyroBias) =gyroscope_bias;


}

void mekf::setCovarianceMatrices() {

	Q.block(0, 0, 3, 3) = gyroscope_variance.asDiagonal();
	Q.block(3, 3, 3, 3) = gyroscope_bias_variance.asDiagonal();

	R.block(0, 0, 3, 3) = accelerometer_variance.asDiagonal();
	R.block(3, 3, 3, 3) = gyroscope_variance.asDiagonal();

	reference_acceleration = { 0, 0, accelerometer_magnitude };

}

void mekf::resetQuaternion()
{
	q.setIdentity();
	
}

void mekf::updateCalibration()
{
	if (firstcicleCalibration)
	{
		iCalibration = 0;
		firstcicleCalibration = false;
	}
}

void mekf::stopCalibration()
{
	if(!firstcicleCalibration)
	{
		firstcicleCalibration = true;
	}
	
}

void mekf::get_a(Vector3d* a_) {
	*a_ = local_a;

}

void mekf::get_vb(Vector3d* b_) {
	*b_ = local_vb;

}

void mekf::get_x(VectorXd*x_) {
	*x_ = local_x;
}


void mekf::get_h(VectorXd* h_) {
	*h_ = local_h;
}


void mekf::get_P(MatrixXd *P_) {
	*P_ = local_P;
}

void mekf::get_K(MatrixXd* K_) {
	*K_ = local_K;
}

void mekf::get_K_aux(MatrixXd* Ka_) {
	*Ka_ = local_K_aux;
}