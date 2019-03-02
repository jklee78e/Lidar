/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/LaserScan.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();
        sensor_msgs::LaserScan scan;

        uint8_t testStr[] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x27, 0xE1, 0xD0};
        ser.write(testStr,8);
        uint8_t start_count=0;
        uint8_t raw_bytes[200];
        ROS_INFO("Start Streaming Sent");

        while(ser.available())
        {

            ser.read(&raw_bytes[start_count++],1);
        }
        if(start_count)
       {
            scan.header.frame_id = "vu8";
            scan.intensities.resize(8);
            scan.ranges.resize(8);
           for(uint16_t i = 0; i < 8; i++)
            {
                const uint16_t bias=33;
            uint8_t byte0 = raw_bytes[bias+i*2];
            uint8_t byte1 = raw_bytes[bias+i*2+1];
            uint8_t byte2 = raw_bytes[bias+16+i*2];
            uint8_t byte3 = raw_bytes[bias+16+i*2+1];
            uint16_t ranges = (byte0<<8)+byte1;
            uint16_t intencity = (byte2<<8)+byte3;
            uint8_t Flag=raw_bytes[bias+32+i*2+1];
            ROS_INFO("%d Seg : Range %d, Intencity %d, Flag %d", i, ranges, intencity, Flag);

            scan.ranges[i]=ranges/100.0;
            scan.intensities[i]=intencity/64.0;

            }

            char const hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
            std::string str;
            for(int i=0; i<start_count;i++)
            {
                char const byte = raw_bytes[i];
                str.append(&hex[(byte & 0xF0) >> 4] , 1);
                str.append(&hex[(byte & 0x0F)] , 1);
            }
            ROS_INFO_STREAM("Stream : " << str);
            ROS_INFO("Parsing Ends %d chars Received", start_count);

            scan.header.stamp = ros::Time::now();
            scan.angle_min = -3.14/3.0;
            scan.angle_max = 3.14/3.0;
            scan.angle_increment = 3.14/3*2/8.0;
            scan.time_increment = 0.1;
            scan.range_min = 0;
            scan.range_max = 100;
            scan_pub.publish(scan);

       }

/*
       for(uint16_t i = 3; i < start_count; i=i+6)
        {
          uint8_t byte0 = raw_bytes[i];
          uint8_t byte1 = raw_bytes[i+1];
          uint8_t byte2 = raw_bytes[i+2];
          uint8_t byte3 = raw_bytes[i+3];
          uint16_t ranges = (byte1<<8)+byte0;
          uint16_t intencity = (byte3<<8)+byte2;
          uint8_t Flag=raw_bytes[i+4];
          uint8_t Segment =raw_bytes[i+5];
          ROS_INFO("%d Seg : Range %d, Intencity %d", Segment, ranges, intencity);
        }
*/


        /*        
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        */
        loop_rate.sleep();

    }
}

