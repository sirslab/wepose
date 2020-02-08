#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <thread>
#include "saveData.h"
#include "mekf.h"
#define _USE_MATH_DEFINES // for C++  
#include <cmath>  
#include <math.h>

#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif


#include <stdarg.h>
#define GL_GLEXT_PROTOTYPES
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include"glUtils.h"
#include"stickman.h"


// ----------------------------------------------------------
//  GLUT
// ----------------------------------------------------------
#define GUI

// Visualization options
bool rectangles = true;
bool g_b_3DStickman = false;


// Glut functions
void display();
void glutIdle(void);
void mykey(unsigned char key, int x, int y);
void initGL();

void renderBitmapString(float x, float y, float z, void *font, char *string);
void cleanExit();

// Glut variables
double rotate_y = 0;
double rotate_x = 0;

double **qCubo = NULL;

bool *g_b_kalman = NULL;
bool *g_b_calibration = NULL;
bool *g_b_calibrated = NULL;

bool g_b_startGUI = false;


// ----------------------------------------------------------
//  IMU
// ----------------------------------------------------------

int numImu; // number of IMUs

bool g_bReadIMU = true;
bool *g_b_ImuReset;

SaveData *logger;
mutex mutexSave;

mutex mutexQuat;

//IMU configuration
bool orientation = false;
bool HR = false;
bool magnetometer = true;


// Vicon?
bool vicon = false;
bool viconLoop = false;


// Data to save
bool save = false;
bool imuQ = true;
bool viconQ = false;


//Functions
int readVicon(double**, int, bool*);
void readIMU(DeviceClass*, XsPortInfo*, int, double*);


//Threads
std::thread* t; // each imu will have a sapate thread

int valuesToSave = 9; // all values -> 136;


int mainthread() {

	DeviceClass* deviceArray;
	double** viconQuaternion = NULL;

	std::cout << "Select the visualization modality (0: blocks; 1: Stickman): "; // << std::endl;
	int visualMode = 0;
	std::cin >> visualMode;
	rectangles = !visualMode;
	g_b_3DStickman = visualMode;

	std::cout << "Do you want to save the output quaternion? (y/N):  "; // << std::endl;
	char answ;
	std::cin >> answ;

	if (answ == 'y')
		save = true;
	std::cout << "save: " << save;



	try
	{
		// Scan for connected USB devices
		std::cout << "Scanning for USB devices... " << std::endl;

		XsPortInfoArray portInfoArray;
		xsEnumerateUsbDevices(portInfoArray);
		numImu = portInfoArray.size();
		std::string portName;

		if (!portInfoArray.size())
		{
			int baudRate;
			std::cout << "No USB Motion Tracker automatically found." << std::endl;

			std::cout << "Please enter number of devices: ";
			std::cin >> numImu;

			for (int iDev = 0; iDev < numImu; iDev++)
			{
				std::cout << "\n ---- insert  parameters for Device " << iDev << " ----" << std::endl;
#ifdef WIN32
				std::cout << "Please enter COM port name (eg. COM1): "; // << std::endl;
#else
				std::cout << "Please enter COM port name (eg. /dev/ttyUSB0): " << std::endl;
#endif
				std::cin >> portName;
				string porteCom[3];


				std::cout << "Please enter baud rate (eg. 115200.). HR works only with 2000000: ";
				std::cin >> baudRate;

				XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
				portInfoArray.push_back(portInfo);
			}
		}

		deviceArray = new DeviceClass[numImu];
		t = new std::thread[numImu];


		g_b_kalman = new bool[numImu];
		g_b_calibration = new bool[numImu];
		g_b_calibrated = new bool[numImu];
		g_b_ImuReset = new bool[numImu];

		for (int i = 0; i < numImu; i++)
		{
			g_b_kalman[i] = false;
			g_b_calibration[i] = true;
			g_b_calibrated[i] = false;
			g_b_ImuReset[i] = false;
			std::cout << "IMU: " << i << "calib init\n";
		}


		std::cout << "Device added: " << numImu << std::endl;


		std::cout << "\n Valori salvati: ";

		if (save)
		{
			logger = new SaveData(numImu, valuesToSave, imuQ, viconQ);
		}

		std::cout << std::endl;


		// Use the first detected device
		//XsPortInfo mtPort;
		//DeviceClass device;
		if (portInfoArray.size() != numImu)
		{
			std::runtime_error("Num Port and Num device error");
			return -1;
		}

		for (int iDev = 0; iDev < numImu; iDev++)
		{

			//mtPort = portInfoArray.at(iDev);
			std::cout << std::endl << "Opening port: " << iDev << " ..." << std::endl;


			// Open the port with the detected device
			if (!deviceArray[iDev].openPort(portInfoArray.at(iDev)))
				throw std::runtime_error("Could not open port. Aborting.");

			// Put the device in configuration mode
			std::cout << "Putting device " << iDev << " into configuration mode..." << std::endl;
			if (!deviceArray[iDev].gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("Could not put device into configuration mode. Aborting.");
			}

			// Request the device Id to check the device type
			portInfoArray.at(iDev).setDeviceId(deviceArray[iDev].getDeviceId());

			// Check if we have an MTi / MTx / MTmk4 device
			if (!portInfoArray.at(iDev).deviceId().isMt9c() && !portInfoArray.at(iDev).deviceId().isLegacyMtig() && !portInfoArray.at(iDev).deviceId().isMtMk4() && !portInfoArray.at(iDev).deviceId().isFmt_X000())
			{
				throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
			}
			std::cout << "Found a device with id: " << portInfoArray.at(iDev).deviceId().toString().toStdString() << " @ port: " << portInfoArray.at(iDev).portName().toStdString() << ", baudrate: " << portInfoArray.at(iDev).baudrate() << std::endl;
		}

		try
		{
			for (int iDev = 0; iDev < numImu; iDev++)
			{
				// Print information about detected MTi / MTx / MTmk4 device
				std::cout << "Device: " << deviceArray[iDev].getProductCode().toStdString() << " opened." << std::endl;

				// Configure the device. Note the differences between MTix and MTmk4
				std::cout << "Configuring the device..." << std::endl;



				if (portInfoArray.at(iDev).deviceId().isMt9c() || portInfoArray.at(iDev).deviceId().isLegacyMtig())
				{
					std::cout << "Mt9c" << std::endl;

					XsOutputMode outputMode = XOM_Orientation; // output orientation data
					XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

																					  // set the device configuration
					if (!deviceArray[iDev].setDeviceMode(outputMode, outputSettings))
					{
						throw std::runtime_error("Could not configure MT device. Aborting.");
					}
				}
				else if (portInfoArray.at(iDev).deviceId().isMtMk4() || portInfoArray.at(iDev).deviceId().isFmt_X000())
				{
					std::cout << "Mt4c" << std::endl;

					XsOutputConfigurationArray configArray;
					if (orientation)
					{
						XsOutputConfiguration quat(XDI_Quaternion, 100);
						configArray.push_back(quat);
					}

					if (HR)
					{
						XsOutputConfiguration rate(XDI_RateOfTurnHR, 1000);
						configArray.push_back(rate);
						XsOutputConfiguration acc(XDI_AccelerationHR, 1000);
						configArray.push_back(acc);

					}

					else {

						XsOutputConfiguration rate_(XDI_RateOfTurn, 100);
						configArray.push_back(rate_);
						XsOutputConfiguration acc_(XDI_Acceleration, 100);
						configArray.push_back(acc_);
						XsOutputConfiguration mag_(XDI_MagneticField, 100);
						configArray.push_back(mag_);


					}

					XsOutputConfiguration time(XDI_SampleTimeFine, 100);
					configArray.push_back(time);

					if (!deviceArray[iDev].setOutputConfiguration(configArray))
					{
						throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
					}
				}
				else
				{
					throw std::runtime_error("Unknown device while configuring. Aborting.");
				}


				std::cout << "\nMain loop (press any key to quit)" << std::endl;
				std::cout << std::string(79, '-') << std::endl;


				//readIMU(DeviceClass* device, XsPortInfo* mtPort) {
				double* vq = NULL;

				if (viconQuaternion != NULL)
					vq = viconQuaternion[iDev];

				qCubo = new double*[numImu];
				for (int i = 0; i < numImu; i++)
					qCubo[i] = new double[4];

				t[iDev] = std::thread(readIMU, &deviceArray[iDev], &portInfoArray.at(iDev), iDev, vq);
				std::cout << "Thread Creato\n";

				//readIMU(&deviceArray[iDev], &portInfoArray.at(iDev), iDev, vq);

			}

			g_b_startGUI = true;


			std::cout << std::endl;
			for (int i = 0; i < numImu; i++)
			{
				t[i].join();
			}
			std::cout << "All Threads joined" << std::endl;



		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		std::cout << "Closing port..." << std::endl;
		for (int iDev = 0; iDev < numImu; iDev++)
		{
			deviceArray[iDev].close();
		}



	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}


	std::cout << "Successful exit." << std::endl;

	
	std::cout << "Press [ENTER] to continue." << std::endl; //std::cin.get();


	return 0;


};




int main(int argc, char* argv[])
{

#ifdef GUI

	// start imu's thread (main thread for data orientation estimation)
	std::thread mainthread_Trd;

	mainthread_Trd = std::thread(mainthread);

	//  Inizializzare il GLUT e processare i parametri degli utenti
	glutInit(&argc, argv);

	//  Richiedi una finestra a colori veri con doppio buffering e buffer Z
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 800);
	glutInitWindowPosition(50, 200);	// Creare la finestra

	// Create window with the given title
	glutCreateWindow("WePosE  GUI");

	//Callback
	glutKeyboardFunc(mykey);

	// Register callback handler for window re-paint event
	glutDisplayFunc(display);

	// Register callback handler for window re-size event
	//glutReshapeFunc(reshape);       

	//register idle function
	glutIdleFunc(glutIdle);

	//glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	// Our own OpenGL initialization
	initGL();

	// Passare il controllo alla GLUT per gli eventi
	while (!g_b_startGUI)
	{
		Sleep(1);
	}
	glutMainLoop();


	//wait for master thread :-O
	mainthread_Trd.join();

#else
	avviaTutto(); // no GUI
#endif


	return 0;
}


void readIMU(DeviceClass* device, XsPortInfo* mtPort, int numDev, double* viconQuat) {

	mekf kalmanFilter(HR);
	double imuQuat[4] = { 1,0,0,0 };

	XsByteArray data;
	XsMessageArray msgs;
	double counter = 0;
	bool timeSet = false;
	bool readAcc = false;
	bool readGyr = false;
	bool readMag = false;

	uint32_t timestamp = 0;
	uint32_t startTime = 0;

	// Put the device in measurement mode
	std::cout << "Putting device " << numDev << " into measurement mode..." << std::endl;
	//return;
	if (!device->gotoMeasurement())
	{
		throw std::runtime_error("Could not put device into measurement mode. Aborting.");
	}
	double rawValues[132];

	for (int i = 0; i < valuesToSave; i++)
	{
		rawValues[i] = 0;
	}


	while (!kbhit() && g_bReadIMU)
	{
		device->readDataToBuffer(data);
		//(*device).readDataToBuffer(data);
		device->processBufferedData(data, msgs);
		for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
		{
			// Retrieve a packet
			XsDataPacket packet;

			if ((*it).getMessageId() == XMID_MtData) {
				LegacyDataPacket lpacket(1, false);
				lpacket.setMessage((*it));
				lpacket.setXbusSystem(false);
				lpacket.setDeviceId(mtPort->deviceId(), 0);
				lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion, 0);	//lint !e534
				XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
			}
			else if ((*it).getMessageId() == XMID_MtData2) {
				packet.setMessage((*it));
				packet.setDeviceId(mtPort->deviceId());
			}
			// Process data packet
			if (orientation)
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				std::cout << "\r"
					<< "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.w()
					<< ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.x()
					<< ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.y()
					<< ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.z()
					;

				// Convert packet to euler
				XsEuler euler = packet.orientationEuler();
				std::cout << ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.roll()
					<< ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.pitch()
					<< ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.yaw()
					;
			}
			if (HR)
			{
				if (packet.containsRateOfTurnHR()) {

					XsVector angvel = packet.rateOfTurnHR();
					//XsVector angvel = packet.calibratedGyroscopeData();
					//std::cout << angvel[0] << std::endl;

					/*std::cout << " "
						<< ",omega_x:" << std::setw(5) << std::fixed << std::setprecision(2) << angvel[0]
						<< ",omega_y:" << std::setw(5) << std::fixed << std::setprecision(2) << angvel[1]
						<< ",omega_z:" << std::setw(5) << std::fixed << std::setprecision(2) << angvel[2]
						;
				//	*/
					for (int i = 0; i < 3; i++)
					{
						rawValues[i] = angvel[i];
						//rawValues[i] = counter+i;// *M_PI / 180;
					}

			//		rawValues[0] = angvel[0];
			//		rawValues[2] = angvel[1];
			//		rawValues[1] = angvel[2];

					readGyr = true;
					counter++;
				}
				if (packet.containsAccelerationHR())
				{

					uint32_t time = packet.sampleTimeFine();
					//if (!timeSet)
					{
						timeSet = true;
						timestamp = (uint32_t)time;
					}

					XsVector accvel = packet.accelerationHR();
					/*	std::cout << " "
							<< ",acc_x:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[0]
							<< ",acc_y:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[1]
							<< ",acc_z:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[2]
							;
							// */
					for (int i = 3; i < 6; i++)
					{
						rawValues[i] = accvel[i - 3];
					}


				//	rawValues[3] = accvel[0];
				//	rawValues[5] = accvel[1];
				//	rawValues[4] = accvel[2];
					readAcc = true;
				}

				for (int i = 6; i < 9; i++)
				{
					rawValues[i] = 0;
					//std::cout << (double)magval[i-6] << " ";
				}
				//	std::cout << std::endl;
				readMag = true;





			}

			else
			{
				if (packet.containsCalibratedGyroscopeData()) {
					XsVector angvel = packet.calibratedGyroscopeData();

					for (int i = 0; i < 3; i++)
					{
						rawValues[i] = angvel[i];// *M_PI / 180;
												 //rawValues[i] = counter+i;// *M_PI / 180;
					}
					readGyr = true;
					counter++;
				}
				if (packet.containsCalibratedAcceleration())
				{

					uint32_t time = packet.sampleTimeFine();
					//if (!timeSet)
					{
						timeSet = true;
						timestamp = (uint32_t)time;
					}

					XsVector accvel = packet.calibratedAcceleration();
					/*	std::cout << " "
					<< ",acc_x:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[0]
					<< ",acc_y:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[1]
					<< ",acc_z:" << std::setw(5) << std::fixed << std::setprecision(2) << accvel[2]
					;
					// */
					for (int i = 3; i < 6; i++)
					{
						rawValues[i] = accvel[i - 3];
					}
					readAcc = true;
				}

				if (packet.containsCalibratedMagneticField()) {
					XsVector magval = packet.calibratedMagneticField();
					//std::cout << "mag: ";
					for (int i = 6; i < 9; i++)
					{
						rawValues[i] = magval[i - 6];
						//std::cout << (double)magval[i-6] << " ";
					}
					//	std::cout << std::endl;
					readMag = true;
				}

			}

			if (readAcc && readGyr && readMag) {

				if (startTime == 0)
				{
					startTime = timestamp;

				}


				if (g_b_calibration[numDev])
				{
					kalmanFilter.setRawData(rawValues, (timestamp - startTime));
					kalmanFilter.calibrationOnline();
					kalmanFilter.getQuaternion(imuQuat);
#ifndef GUI
					if (kbhit() && getch() == 'c')

#else // !GUI
				}
				if (g_b_calibrated[numDev] && !g_b_calibration[numDev])
				{
#endif
					{
						g_b_calibrated[numDev] = false;

#ifndef GUI
						g_b_calibration = false;
						g_b_kalman = true;
#endif
						kalmanFilter.setCovarianceMatrices();
						kalmanFilter.setFilterRunning(true);
						std::cout << "IMU: " << numDev << "Calibration completed" << endl;
						Eigen::Vector3d gb;
						kalmanFilter.getGyroscopeBias(&gb);
						std::cout << "Gyroscope Byas = " << std::fixed << std::setprecision(5) << gb << std::endl;
					}

				}
				else if (g_b_kalman[numDev])
				{
					kalmanFilter.setRawData(rawValues, (timestamp - startTime));
					//std::cout << "Timestamp main " << (timestamp - startTime);
					kalmanFilter.mekfOnline();
					kalmanFilter.getQuaternion(imuQuat);


				


					mutexQuat.lock();
					qCubo[numDev] = imuQuat;
					mutexQuat.unlock();


					if (g_b_ImuReset[numDev])
					{
						kalmanFilter.resetQuaternion();
						g_b_ImuReset[numDev] = false;

					}

				}

				if (save)
				{
					if (imuQ && !viconQ)
					{
						mutexSave.lock();
						logger->setValues(numDev, rawValues, imuQuat, timestamp);
						mutexSave.unlock();

					}

					else
					{
						logger->setValues(numDev, rawValues, timestamp);
						//std::cout << "salvo";
					}
				}


				timeSet = false;
				readAcc = false;
				readGyr = false;
				readMag = false;
			}

			if (numDev == numImu - 1)
			{

				std::cout << "\r" << std::fixed << std::setprecision(2) << (double)(timestamp - startTime) / 10000 << " s";
			}

		}
		//std::wcout << "\r";
		std::cout << std::flush;


	}


	if (save) {
		if (numDev == numImu - 1)
			logger->saveLog("output.txt");
		std::cout << "File Saved!!!!" << std::endl;
	}

	std::cout << "Putting device into configuration mode..." << std::endl;

	if (!device->gotoConfig())
	{
		throw std::runtime_error("Could not put device into configuration mode. Aborting.");
	}
	std::cout << "Done! " << std::endl;

}



// ----------------------------------------------------------
// Funzione di richiamo display()
// ----------------------------------------------------------
void display() {
	int font = (int)GLUT_BITMAP_8_BY_13;


	if (rectangles) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

		glLoadIdentity();                 // Reset the model-view matrix
		//glTranslatef(0.0f, 0.0f, 7.0f);  // Move right and into the screen

		glScalef(0.2f, 0.2f, 0.2f);


		// angle of rotation
		Eigen::Quaterniond cQuaternion;
		cQuaternion.setIdentity();
		Matrix3d T1, T2;
		std::vector<Matrix3d> allT(numImu);



		for (int i = 0; i < numImu; i++) {
			//std::cout << "g_b_kalman[i] " << g_b_kalman[i] << std::endl;
			if (g_b_kalman[i])
			{
				glColor3f(1.0, 1.0, 1.0);

				renderBitmapString(-3.0, 4.0, 0.5, (void *)font, "Align the IMU with the OpenGL reference system and the press 'r' ");
				renderBitmapString(-3.0, 3.8, 0.5, (void *)font, "The Z axes point into the screen");
				renderBitmapString(-3.0, 3.4, 0.5, (void *)font, "press [ESC] to exit");


				mutexQuat.lock();
				cQuaternion.w() = qCubo[i][0];
				cQuaternion.x() = qCubo[i][1];
				cQuaternion.y() = qCubo[i][2];
				cQuaternion.z() = qCubo[i][3];
				mutexQuat.unlock();

				double a = 3.0;
				double b = 0.7;
				double c = 0.5;

				Eigen::MatrixXd P(8, 3);

				//rotation around the center of mass
				if (numImu == 1) { 
					P << a, b, c,
						a, b, -c,
						a, -b, -c,
						a, -b, c,
						-a, b, c,
						-a, b, -c,
						-a, -b, -c,
						-a, -b, c;
				}
				else
				{

					P << a, b, c,
						a, b, -c,
						a, -b, -c,
						a, -b, c,
						0, b, c,
						0, b, -c,
						0, -b, -c,
						0, -b, c;
				}
				Matrix3d T;		
				Matrix3d Rotx90, Roty90, Rotz90;

				Rotx90 << 1, 0, 0,
					0, 0, -1,
					0, 1, 0;

				Roty90 << 0, 0, 1,
					0, 1, 0,
					-1, 0, 0;


				T = cQuaternion.toRotationMatrix();
				allT[i] = T;
				//std::cout << T << endl;



	//			Eigen::Vector3d xc(-2.5*i, 0, 0);
				Eigen::Vector3d xc(0, 0, 0);

				Eigen::Vector3d v;
				Eigen::VectorXd X(8, 1), Y(8, 1), Z(8, 1);

				if (i > 0) {
					Eigen::Vector3d translation(a, 0, 0);
					xc << allT[i - 1].inverse().transpose() * (xc + translation);
					//draw_sphere(xc(0), xc(1), xc(2), (b + c));

				}



				for (int k = 0; k < 8; k++)
				{
					v = xc.transpose() + (P.row(k)*T.inverse());

					X(k) = v(0);
					Y(k) = v(1);
					Z(k) = v(2);
				}





				// Lato multicolore - FRONTALE

				glBegin(GL_QUADS);

				glColor3f(0.0, 1.0, 0.0);

				glVertex3f(X(5), Y(5), Z(5));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(4), Y(4), Z(4));

				//  POSTERIORE
				glColor3f(0.0, 1.0, 0.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(3), Y(3), Z(3));
				glVertex3f(X(7), Y(7), Z(7));

				//DESTRO
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(3), Y(3), Z(3));

				// SINISTRO
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(5), Y(5), Z(5));
				glVertex3f(X(4), Y(4), Z(4));
				glVertex3f(X(7), Y(7), Z(7));

				// Lato Blu - SUPERIORE
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f(X(7), Y(7), Z(7));
				glVertex3f(X(3), Y(3), Z(3));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(4), Y(4), Z(4));

				// Lato Rosso - INFERIORE
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(5), Y(5), Z(5));

				glEnd();

				//draw_cylinder(1, 1, 1, 0, 0, 0,0,0);
				//drawCylinder(2, 10, 10);
				//cylinder(1,0,0);



			}
			else
				//if (i == 0)
					renderBitmapString(-0.5, -0.5, 0.5, (void *)font, "Click here and then press 'c' to end the calibration");

		}



		//render();

	}

	else if (g_b_3DStickman) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

		glLoadIdentity();                 // Reset the model-view matrix
										  //glTranslatef(0.0f, 0.0f, 7.0f);  // Move right and into the screen

		glScalef(0.2f, 0.2f, 0.2f);

		// angle of rotation
		Eigen::Quaterniond cQuaternion;
		cQuaternion.setIdentity();
		Matrix3d T1, T2;
		std::vector<Matrix3d> allT(numImu);

		for (int i = 0; i < numImu; i++) {

			//std::cout << "g_b_kalman[i] " << g_b_kalman[i] << std::endl;
			if (g_b_kalman[i])
			{

				mutexQuat.lock();
				cQuaternion.w() = qCubo[i][0];
				cQuaternion.x() = qCubo[i][1];
				cQuaternion.y() = qCubo[i][2];
				cQuaternion.z() = qCubo[i][3];
				mutexQuat.unlock();

				double a = 1.5;
				double b = 0.4;
				double c = 0.4;


				Eigen::MatrixXd P(8, 3);
				/*P << a, b, c,
				a, b, -c,
				a, -b, -c,
				a, -b, c,
				-a, b, c,
				-a, b, -c,
				-a, -b, -c,
				-a, -b, c;
				//	*/


				P << a, b, c,
					a, b, -c,
					a, -b, -c,
					a, -b, c,
					0, b, c,
					0, b, -c,
					0, -b, -c,
					0, -b, c;
				// */


				Matrix3d T;
				//T.setIdentity();


				Eigen::Quaterniond qStickMan;
				qStickMan.w() = 0.5;
				qStickMan.x() = 0.5;
				qStickMan.y() = -0.5;
				qStickMan.z() = 0.5;
				qStickMan.normalize();

				T = qStickMan.toRotationMatrix()*cQuaternion.toRotationMatrix();
				allT[i] = T;
				//std::cout << T << endl;



				//			Eigen::Vector3d xc(-2.5*i, 0, 0);

				Eigen::Vector3d xc;
				
				if(numImu <2)
					xc << Eigen::Vector3d (-1.3, -1.0, 0);
				else
					xc << Eigen::Vector3d(-1.3,1.0, 0.0);
				

				//1.7, 1.05, -4.5 + y_offsest, -3.05 + y_offsest, -1.0, 1.0

				Eigen::Vector3d v;
				Eigen::VectorXd X(8, 1), Y(8, 1), Z(8, 1);

				if (i > 0)

				{
					Eigen::Vector3d translation(a,0.0,0.0);
					xc << (allT[i - 1].inverse().transpose() * (translation))+xc;
				}



				for (int k = 0; k < 8; k++)
				{
					v = xc.transpose() + (P.row(k)*T.inverse());

					X(k) = v(0);
					Y(k) = v(1);
					Z(k) = v(2);
				}



				// Lato multicolore - FRONTALE

				glBegin(GL_QUADS);

				glColor3f(0.0, 1.0, 0.0);

				glVertex3f(X(5), Y(5), Z(5));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(4), Y(4), Z(4));

				//  POSTERIORE
				glColor3f(0.0, 1.0, 0.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(3), Y(3), Z(3));
				glVertex3f(X(7), Y(7), Z(7));

				//DESTRO
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(3), Y(3), Z(3));

				// SINISTRO
				glColor3f(1.0, 0.0, 0.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(5), Y(5), Z(5));
				glVertex3f(X(4), Y(4), Z(4));
				glVertex3f(X(7), Y(7), Z(7));

				// Lato Blu - SUPERIORE
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f(X(7), Y(7), Z(7));
				glVertex3f(X(3), Y(3), Z(3));
				glVertex3f(X(0), Y(0), Z(0));
				glVertex3f(X(4), Y(4), Z(4));

				// Lato Rosso - INFERIORE
				glColor3f(0.0, 0.0, 1.0);
				glVertex3f(X(6), Y(6), Z(6));
				glVertex3f(X(2), Y(2), Z(2));
				glVertex3f(X(1), Y(1), Z(1));
				glVertex3f(X(5), Y(5), Z(5));

				glEnd();



			}
		}




		float y_offsest = 2;


		glColor3f(1.0, 0.2, 0.2);
		draw_gray_box(-1.0, 1.0, -3.7 + y_offsest, -1.0 + y_offsest, 1.0, -1.0);
		
		//Arms
		if(numImu<2)
			draw_gray_box(-1.7, -1.05, -2.5 + y_offsest, -1.0 + y_offsest, 1.0, -1.0);
		
		draw_gray_box(1.7, 1.05, -2.5 + y_offsest, -1.0 + y_offsest, -1.0, 1.0);
		//Legs
		draw_gray_box(-0.8, -0.1, -6.0 + y_offsest, -3.75 + y_offsest, -1.0, 1.0);
		draw_gray_box(0.8, 0.1, -6.0 + y_offsest, -3.75 + y_offsest, -1.0, 1.0);

		//forearms

		glColor3f(0.87, 0.87, 0.87);
		draw_gray_box(1.7, 1.05, -4.0 + y_offsest, -2.55 + y_offsest, -1.0, 1.0);

		///joints
		/*glColor3f(1.0, 0.0, 0.8);

		glTranslatef(-1.35, -3.0 + y_offsest, 0.0);
		drawSphere(0.1);
		glTranslatef(0.35, 2.0, 0.0);
		drawSphere(0.1);

		*/
		glTranslatef(0.0, 2.0, 0.0);

		glColor3f(0.78, 0.78, 0.78);
		drawSphere(1.0);

		//*/
		glFlush();

	}

	glutSwapBuffers();



}




void cleanExit()
{
	g_bReadIMU = false;
	viconLoop = false;

	Sleep(5000);
	exit(0);
}




void mykey(unsigned char key, int x, int y)
{
	switch (key) {
	case 27:
		cleanExit();
		break;

	case 'r':
		for (int numDev = 0; numDev < numImu; numDev++) {
			g_b_ImuReset[numDev] = true;
			std::cout << " IMU " << numDev << " RESET!" << std::endl;
		}
		break;

	case 'c':
		for (int numDev = 0; numDev < numImu; numDev++) {
			g_b_calibration[numDev] = !g_b_calibration[numDev];
			g_b_kalman[numDev] = !g_b_calibration[numDev];
			g_b_calibrated[numDev] = !g_b_calibration[numDev];
			g_b_ImuReset[numDev] = true;
			std::cout << "IMU " << numDev << " Calibrartion: " << (g_b_calibration[numDev] ? " START\n" : " END\n") << std::endl;
		}
		break;

	default:
		break;
	}
	glutPostRedisplay();
}


void renderBitmapString(float x, float y, float z, void *font, char *string)
{
	char *c;
	glRasterPos3f(x, y, z);
	for (c = string; *c != '\0'; c++)
		glutBitmapCharacter(font, *c);
}