#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "nord_messages/IRSensors.h"
#include <cmath>

float front_inverse(int x){
		return (-40.19*std::exp(x*(-0.004891))+41.03*std::exp(x*(-0.004897)));
}
float back_inverse(int x){
		return (0.05347*std::exp(x*(0.0004147))+1.9*std::exp(x*(-0.008829)));
}

float lfront_inverse(int x){
		return (28.73*std::exp(x*(-0.04203))+0.4839*std::exp(x*(-0.006104)));
}

float lback_inverse(int x){
		return (25.57*std::exp(x*(-0.03562))+0.2736*std::exp(x*(-0.003856)));
}

float rback_inverse(int x){
		return (0.3986*std::exp(x*(-0.04507))+0.515*std::exp(x*(-0.00663)));
}

float rfront_inverse(int x){
		return (0.4628*std::exp(x*(-0.1022))+0.515*std::exp(x*(-0.00663)));
}

int main(int argc, char** argv)
{
    using nord_messages::IRSensors;
    using ras_arduino_msgs::ADConverter;
    ros::init(argc, argv, "nord_sensors");
    ros::NodeHandle n;

    ros::Publisher ir_pub = n.advertise<IRSensors>("/nord/sensors/ir", 10);

    ros::Subscriber adc_sub = n.subscribe<ADConverter>("/arduino/adc", 10,
        [&](const ADConverter::ConstPtr& msg) {
             IRSensors ir;
			 
			ir.back = back_inverse(msg->ch5);
			
			ir.front = front_inverse(msg->ch6);
/*ir.back = back_inverse(0);
			
			ir.front = front_inverse(0);*/
			 
			if(msg->ch1>320){
				ir.left_front = lfront_inverse(0);
			}else{
				ir.left_front = lfront_inverse(msg->ch1);
			}
			//ir.left_front = lfront_inverse(0);
			if(msg->ch3>320){
				ir.left_back = lback_inverse(0);
			}else{
				ir.left_back = lback_inverse(msg->ch3);
			}
			//ir.left_back = lback_inverse(0);
			if(msg->ch7>400){
				ir.right_back = rback_inverse(0);
			}else{
				ir.right_back = rback_inverse(msg->ch7);
			}
			//ir.right_back = rback_inverse(0);
			if(msg->ch8>320){
				ir.right_front = rfront_inverse(0);
			}else{
				ir.right_front = rfront_inverse(msg->ch8);
			}
			//ir.right_front = rfront_inverse(0);
            ir_pub.publish(ir);
        });

    ros::spin();

    return 0;
}
