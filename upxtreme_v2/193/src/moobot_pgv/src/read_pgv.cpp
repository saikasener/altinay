#include "ros/ros.h"
#include <moobot_pgv/pgv_scan_data.h>
// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWRs
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <bitset>
#include <string>
#include <stdlib.h>     /* strtoull */
#include <signal.h>
#include <sstream>
#include <cmath> // to use pow

using namespace std;

int serial_port = open("/dev/cnv_pgv", O_RDWR);

unsigned char dir_straight[ 2 ] = {0xEC, 0x13}; // Straight ahead 
unsigned char dir_left[ 2 ] = { 0xE8, 0x17}; // Follow Left
unsigned char dir_right[ 2 ] = { 0xE4, 0x1B}; // Follow Right
unsigned char dir_nolane[ 2 ] = { 0xE0, 0x1F}; // No lane is selected
unsigned char pos_req[ 2 ] = { 0xC8, 0x37}; // Position Request
double agv_ang_des = 0.0; // degree
double agv_x_pos_des = 0.0; // mm
double agv_y_pos_des; // mm
bool read_barcode; // if true there is barcode
double cal_err_x = 0;
double cal_err_y = 0;
double cal_err_ang = 0;
int tag_num = 0;
int lane_detected_msg;
moobot_pgv::pgv_scan_data pgv_msg;
void update_pgv_data(){
    pgv_msg.x_pos = (agv_x_pos_des - cal_err_x) / 10.0; // mm
    pgv_msg.y_pos = (agv_y_pos_des - cal_err_y) / 10.0; // mm
    pgv_msg.read_barcode = read_barcode;
    pgv_msg.orientation = agv_ang_des - cal_err_ang; //deg
    pgv_msg.tag_num = tag_num; // int
    pgv_msg.lane_detected = lane_detected_msg; //int
}

void my_handler(int s){
    close(serial_port);
    printf("\n\nCaught signal %d\n Port closed.\n",s);
    exit(1); 
}

unsigned long int string2decimal(string input){
    int strlength = input.length();
    char input_char[strlength + 1];
    strcpy(input_char, input.c_str());
    char *pEnd;
    unsigned long int out_dec = strtoull(input_char, &pEnd, 2);
    return out_dec;
}


int main(int argc, char **argv){
    // Handle Ctrl+C signal
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n - Check your device is connected or not.\n", errno, strerror(errno));
        return 1;
    }

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    //////////////////////////////////////////////////
    // Serial Communication Configuration
    //////////////////////////////////////////////////
    tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS7; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 2;
    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    ros::init(argc, argv, "pgv100_node");
    ros::NodeHandle nh;
    ros::Publisher pgv_data_pub = nh.advertise<moobot_pgv::pgv_scan_data>("/pgv_scan", 100);
    ros::Rate loop_rate(10);
    write(serial_port,  dir_nolane , sizeof( dir_straight));  
    write(serial_port, dir_straight, sizeof(dir_straight));
    while (ros::ok()){
        ////////////////////////////////////////
        //////////////// Read pgv///////////////
        //////////////////////////////////////////
        write(serial_port, pos_req , 2);
        char read_buf [21];
        memset(&read_buf, '\0', sizeof(read_buf));
        int byte_count = read(serial_port, &read_buf, sizeof(read_buf));
        
        // Get Lane-Detection from the byte array [Bytes 1-2]
        bitset<7> lane_detect_byte1(read_buf[0]);
        bitset<7> lane_detect_byte0(read_buf[1]);
        string agv_lane_detect_str = lane_detect_byte1.to_string() + lane_detect_byte0.to_string();
        

        // Get tag detected
        string tag_detected = agv_lane_detect_str.substr(7,1);
        read_barcode = string2decimal(tag_detected); 

        // Get pos detected
        string agv_no_pos_str = agv_lane_detect_str.substr(5, 1);
        int agv_no_pos_des = string2decimal(agv_no_pos_str);
        lane_detected_msg = agv_no_pos_des;//string2decimal(agv_lane_detect_str);
        // Get the angle from the byte array [Byte 11-12]
        bitset<7> ang_1(read_buf[10]);
        bitset<7> ang_0(read_buf[11]);
        string agv_ang_str = ang_1.to_string() + ang_0.to_string();
        agv_ang_des = string2decimal(agv_ang_str) / (10.0);
        if (agv_ang_des > 180.0) // this makes x-pos zero centered
            agv_ang_des -= 360.0;

            // Get the X-Position from the byte array [Bytes 3-4-5-6]
            bitset<3> x_pos_3(read_buf[2]);
            bitset<7> x_pos_2(read_buf[3]);
            bitset<7> x_pos_1(read_buf[4]);
            bitset<7> x_pos_0(read_buf[5]);
            string agv_x_pos_str = x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();
            agv_x_pos_des = string2decimal(agv_x_pos_str);
        if(read_barcode != 0){
            if (agv_x_pos_des > 2000.0) // this makes x-pos zero centered
              agv_x_pos_des = agv_x_pos_des - pow(2,24) - 1;
        }
        
        // Get Y-Position from the byte array [Bytes 7-8]
        bitset<7> y_pos_1(read_buf[6]);
        bitset<7> y_pos_0(read_buf[7]);
        string agv_y_pos_str = y_pos_1.to_string() + y_pos_0.to_string();
        agv_y_pos_des = string2decimal(agv_y_pos_str);
        if (agv_y_pos_des > 2000.0) // this makes y-pos zero centered
          agv_y_pos_des = agv_y_pos_des - 16383.0;
        // We get opposite values when we try the read the y-pos value from the colored and code strip.
        // So this is checking which strip that we're reading.
        if(agv_no_pos_des) agv_y_pos_des *= -1;

        // Get tag number from the byte array [Byte 15-18]
        if(read_barcode == true){
            //bitset<7> tag_num1(read_buf[15]);
            bitset<7> tag_num2(read_buf[17]);
            //bitset<7> tag_num3(read_buf[17]);
            //bitset<7> tag_num4(read_buf[18]);
            string tag_num_str = tag_num2.to_string();
            tag_num = string2decimal(tag_num_str);
        }
        
    ///////////////////////////////////////////////////////
        update_pgv_data();
        pgv_data_pub.publish(pgv_msg);
        ros::spinOnce();
        sigaction(SIGINT, &sigIntHandler, NULL);

    }


    return 0;
}