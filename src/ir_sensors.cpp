#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "nord_messages/IRSensors.h"
#include <cmath>


float ir_back_temp;
float ir_front_temp;
float ir_left_front_temp;
float ir_left_back_temp;
float ir_right_back_temp;
float ir_right_front_temp;

const float ALPHA=0.5f;

template<class T>
std::function<T(T)> make_lorentz(T height, T center, T hwhm)
{
    return [=](T x) {
        return height / (1 + std::pow((x - center) / hwhm, 2));
    };
}

int main(int argc, char** argv)
{
    using nord_messages::IRSensors;
    using ras_arduino_msgs::ADConverter;
    ros::init(argc, argv, "nord_sensors");
    ros::NodeHandle n;

    auto lorentz1 = make_lorentz(0.1396f, 111.639f, 12.251f);
    auto lorentz2 = make_lorentz(0.6645f, 84.85f, 133.49f);
    auto back_lorentz = [&](float x) { return lorentz1(std::max(x, 110.0f)) + lorentz2(std::max(x, 110.0f)) + 0.003f; };

    auto lorentz3 = make_lorentz(0.7685f, -6.6394f, 147.44f);
    auto lorentz9 = make_lorentz(0.0366f, 412.53f, -159.43f);
    auto front_lorentz = [&](float x) { return lorentz3(x) + lorentz9(x); };

	
    auto lorentz5 = make_lorentz(0.6607f, -36.32f, 131.3f);
   // auto lorentz6 = make_lorentz(0.227f, 29.78f, 98.66f);
    auto Rback_lorentz = [&](float x) { return lorentz5(std::max(x, 20.0f)) -0.1293+std::max(x, 20.0f)*0.000166 + 0.085f; };

    auto lorentz4 = make_lorentz(0.963f, 19.24f, 97.91f);
    auto Lfront_lorentz = [&](float x) { return lorentz4(std::max(x, 75.0f))-0.333f+0.000761*std::max(x, 75.0f) + 0.097f; };
	
    auto lorentz7 = make_lorentz(0.3448f, 95.51f, 25.52f);
    auto Lback_lorentz = [&](float x) { return lorentz7(std::max(x, 100.0f))+0.09874-0.0003979*std::max(x, 100.0f) + 0.1f; };
	
    auto lorentz8 = make_lorentz(127.31f, -77.53f, 6.32f);
    auto Rfront_lorentz = [&](float x) { return lorentz8(x) +0.09f; };
	

    ros::Publisher ir_pub = n.advertise<IRSensors>("/nord/sensors/ir", 10);

    ros::Subscriber adc_sub = n.subscribe<ADConverter>("/arduino/adc", 10,
        [&](const ADConverter::ConstPtr& msg) {
             IRSensors ir;
             ir.back = back_lorentz(msg->ch5);
             ir.back=ir.back+ALPHA*(ir_back_temp-ir.back);
             ir_back_temp=back_lorentz(msg->ch5);

             ir.front = front_lorentz(msg->ch6);
             ir.front=ir.front+ALPHA*(ir_front_temp-ir.front);
             ir_front_temp=front_lorentz(msg->ch6);

             ir.left_front = Lfront_lorentz(msg->ch1);
             ir.left_front=ir.left_front+ALPHA*(ir_left_front_temp-ir.left_front);
             ir_left_front_temp=Lfront_lorentz(msg->ch1);

             ir.left_back = Lback_lorentz(msg->ch3);
             ir.left_back=ir.left_back+ALPHA*(ir_left_back_temp-ir.left_back);
             ir_left_back_temp=Lback_lorentz(msg->ch3);

             ir.right_back = Rback_lorentz(msg->ch7);
             ir.right_back=ir.right_back+ALPHA*(ir_right_back_temp-ir.right_back);
             ir_right_back_temp=Rback_lorentz(msg->ch7);

             ir.right_front = Rfront_lorentz(msg->ch8);
             ir.right_front=ir.right_front+ALPHA*(ir_right_front_temp-ir.right_front);
             ir_right_front_temp=Rfront_lorentz(msg->ch8);

             ir_pub.publish(ir);
        });

    ros::spin();

    return 0;
}
