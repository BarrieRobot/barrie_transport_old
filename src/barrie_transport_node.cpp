//
// Created by casper on 4-1-18.
//

#include <iostream>
#include "ros/ros.h"
#include <threemxl/C3mxl.h>
#include <threemxl/LxFTDI.h>
#include <boost/scoped_ptr.hpp>
#include <threemxl/platform/hardware/dynamixel/3mxl/3mxlControlTable.h>

/**
 * Most important doc:
 * http://docs.ros.org/indigo/api/threemxl/html/classC3mxl.html
 * This page states all the methods of the C3mxl class.
 * In this file, the C3mxl instance is called 'motor'
 */

#define CONNECTION_FTDI 0
#define CONNECTION_SERIAL 1
#define CONNECTION_MODE CONNECTION_SERIAL

bool openPort(LxSerial *serial_port) {
    bool is_open = false;

#if CONNECTION_MODE == CONNECTION_FTDI
    // Create new instance for serial port
    LxSerial *serial_port = new LxFTDI();

    // Open serial port
    is_open = serial_port->port_open("i:0x0403:0x6001", LxSerial::RS485_FTDI);
    // Set connection speed. This cannot be higher than 500 kbaut on V2 of the board
    serial_port->set_speed_int(921600);
#elif CONNECTION_MODE == CONNECTION_SERIAL
    LxSerial *serial_port = new LxSerial();

    is_open serial_port->port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port->set_speed(LxSerial::S500000);
#endif

    return is_open;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "barrie_transport");

    // Create new 3Mxl class instance
    C3mxl *motor = new C3mxl();
    // Create config to send to board
    CDxlConfig *config = new CDxlConfig();

    // Open serial/FTDI port
    LxSerial *serial_port;
    bool is_open = openPort(serial_port);

    // Check if the port opened successfully
    if (!is_open) {
        ROS_ERROR("Port could not be opened");
    }

    // Pass serialport handle to the 3Mxl class instance
    motor->setSerialPort(serial_port);

    // ID 108 for testing motor
    motor->setConfig(config->setID(108));

    // Initialise class and don't (?) send configuration
    motor->init(false);

    // Standard ROS loop
    ros::Rate loop_rate(10);
    // Set motor position reference to 0. Function also accepts speed.
    motor->setPos(0);

    do {
        loop_rate.sleep();
        // getPos() function gets retrieves the position but does not return it according to the docs
        // presentPos() returnes this cached value.
        motor->getPos();
    } while (ros::ok() && fabs(motor->presentPos()) > 0.01);
}
