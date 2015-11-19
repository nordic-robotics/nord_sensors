#include "ros/ros.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "nord_messages/IRSensors.h"
#include <cmath>

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

    auto lorentz1 = make_lorentz(0.228f, 73.63f, 13.0f);
    auto lorentz2 = make_lorentz(62.31f, -184.0f, 25.10f);
    auto long_lorentz = [&](float x) { return lorentz1(x) + lorentz2(x); };

    auto lorentz3 = make_lorentz(126.8f, -86.93f, 8.824f);
    auto short_lorentz = [&](float x) { return x > 35 ? lorentz3(x) : 0; };

    ros::Publisher ir_pub = n.advertise<IRSensors>("/nord/sensors/ir", 10);

    ros::Subscriber adc_sub = n.subscribe<ADConverter>("/arduino/adc", 10,
        [&](const ADConverter::ConstPtr& msg) {
             IRSensors ir;
             ir.back = long_lorentz(msg->ch5);
             ir.front = long_lorentz(msg->ch6);
             ir.left_front = short_lorentz(msg->ch1);
             ir.left_back = short_lorentz(msg->ch3);
             ir.right_back = short_lorentz(msg->ch7);
             ir.right_front = short_lorentz(msg->ch8);
             ir_pub.publish(ir);
        });

    ros::spin();

    return 0;
}