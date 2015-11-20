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

const float ALPHA=0.15f;

template<class T>
std::function<T(T)> make_lorentz(T height, T center, T fwhm)
{
    return [=](T x) {
        return height / (1 + std::pow((x - center) / fwhm, 2));
    };
}

int main(int argc, char** argv)
{
    using nord_messages::IRSensors;
    using ras_arduino_msgs::ADConverter;
    ros::init(argc, argv, "nord_sensors");
    ros::NodeHandle n;

    auto lorentz1 = make_lorentz(0.337f, 107.55f, 43.76f);
    auto lorentz2 = make_lorentz(0.479f, 140.18f, 231.33f);
    auto back_lorentz = [&](float x) { return lorentz1(std::max(x, 110.0f)) + lorentz2(std::max(x, 110.0f)) + 0.003f; };

    auto lorentz3 = make_lorentz(0.4157f, 109.9f, 68.31f);
    auto front_lorentz = [&](float x) { return lorentz3(std::max(x, 110.0f)); };
	
	auto lorentz5 = make_lorentz(0.42f, -1.157f, 29.7f);
    auto lorentz6 = make_lorentz(0.227f, 29.78f, 197.32f);
    auto Rback_lorentz = [&](float x) { return lorentz5(x) + lorentz6(x) + 0.085f; };

    auto lorentz4 = make_lorentz(0.864f, 80.48f, 59.85f);
    auto Lfront_lorentz = [&](float x) { return lorentz4(std::max(x, 80.0f)) + 0.097f; };
	
	auto lorentz7 = make_lorentz(0.569f, 101.96f, 62.96f);
    auto Lback_lorentz = [&](float x) { return lorentz7(std::max(x, 100.0f)) + 0.1f; };
	
	auto lorentz8 = make_lorentz(132.3f, -77.17f, 12.35f);
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
             ir_front_temp=back_lorentz(msg->ch6);

             ir.left_front = Lfront_lorentz(msg->ch1);
             ir.left_front=ir.left_front+ALPHA*(ir_left_front_temp-ir.left_front);
             ir_left_front_temp=back_lorentz(msg->ch1);

             ir.left_back = Lback_lorentz(msg->ch3);
             ir.left_back=ir.left_back+ALPHA*(ir_left_back_temp-ir.left_back);
             ir_left_back_temp=back_lorentz(msg->ch3);

             ir.right_back = Rback_lorentz(msg->ch7);
             ir.right_back=ir.right_back+ALPHA*(ir_right_back_temp-ir.right_back);
             ir_right_back_temp=back_lorentz(msg->ch7);

             ir.right_front = Rfront_lorentz(msg->ch8);
             ir.right_front=ir.right_front+ALPHA*(ir_right_front_temp-ir.right_front);
             ir_right_front_temp=back_lorentz(msg->ch8);

             ir_pub.publish(ir);
        });

    ros::spin();

    return 0;
}
