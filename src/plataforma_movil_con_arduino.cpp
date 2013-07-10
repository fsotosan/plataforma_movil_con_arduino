#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "Serial_Q.h"

#define SERIALDEVICE	"/dev/ttyUSB0"

using namespace std;

void joyCallback(const sensor_msgs::Joy& inJoyCommand);
bool processPtuComm();
void getVelCommand(int inVelLineal, int velAngular, char* inComando);

ros::NodeHandle* myNodeHandle = 0;

Serial::Serial_Q *Ptu;

char tmpChar;
char status;
char mySerialDevice[] = "/dev/ttyUSB0";

sensor_msgs::JointState thePtuJointState;


int main(int argc, char** argv) {

	ros::init(argc, argv, "plataforma_movil_con_arduino");
	ros::NodeHandle theNodeHandle;
	myNodeHandle = &theNodeHandle;
	ros::Subscriber theJoySubscriber = theNodeHandle.subscribe("/joy", 10, joyCallback);

	ROS_INFO("Nodo plataforma_movil_con_arduino iniciado");

	Ptu = new Serial::Serial_Q(mySerialDevice, B9600);

	if (Ptu->LastError != NULL) {
		ROS_ERROR("Error conectando con plataforma: %s",Ptu->LastError);
		return -1;
	}

	ROS_INFO("Puerto serie abierto %d",B9600);

	ros::Rate r(1); // 1 hz
	while (myNodeHandle->ok()) {

		//processPtuComm();
		ros::spinOnce();
	}

	cout << "Programa terminado" << endl;
	return 0;

}

void getVelCommand(int inVelLineal, int inVelAngular, char* inComando) {

	int i;
	char zero = '0';

	//ROS_INFO("Lin: %d. Ang: %d", inVelLineal, inVelAngular);
	inComando[0] = 'A';
	inComando[1] = (unsigned char)((inVelLineal&0xFF00) >> 8);
	inComando[2] = (unsigned char)(inVelLineal&0x00FF);
	inComando[3] = (unsigned char)((inVelAngular&0xFF00) >> 8);
	inComando[4] = (unsigned char)(inVelAngular&0x00FF);
	inComando[5] = 'Z';
	inComando[6] = 'Z';
	inComando[7] = 0;

	// Los ceros no llegan a través del cable USB a TTL (investigar motivo, de momemto reemplazar)
	for(i=1;i<=4;i++) {
		if (inComando[i] == 0) {
			inComando[i] = zero;
		}
	}

}


void joyCallback(const sensor_msgs::Joy& inJoyCommand) {

	char theCommand[8];

	float theAxesH = (float)inJoyCommand.axes[0];
	float theAxesV = (float)inJoyCommand.axes[1];

	printf("Recibido comando joystick (%g,%g)\n",theAxesV,theAxesH);

	int theLinVel = -theAxesV * 626;
	int theAngVel = theAxesH * 448;

	if (theLinVel != 0) {
		theAngVel = -theAngVel / 4;
	}

	getVelCommand(theLinVel,theAngVel,theCommand);

	//do {
		Ptu->send(theCommand,7);
	//
	//} while (!processPtuComm());

	ros::Duration(0.1).sleep();

	processPtuComm();

}


bool processPtuComm() {

	string theResp;
	std::size_t theFound;

	if (Ptu->checkDataAndEnqueue()) {
		theResp.assign((char *)Ptu->getFullQueueContent(true));
		ROS_INFO("Plataforma móvil dice: %s",theResp.c_str());
	}

	theFound=theResp.find("*");
	return (theFound!=std::string::npos);

}
