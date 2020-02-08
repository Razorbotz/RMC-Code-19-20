#define Phoenix_No_WPI
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/platform/can/PlatformCAN.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <unistd.h>

constexpr int CONFIG_TIMEOUT_MS = 30;

ctre::phoenix::motorcontrol::can::TalonSRX* talonSRX;

void setSpeed(const std_msgs::Float32 speed)
{
	talonSRX->Set(ControlMode::PercentOutput, speed.data);	
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "talon");
	ros::NodeHandle privateNodeHandle("~");
	ros::NodeHandle nodeHandle;

	int motorNumber = 0;
	privateNodeHandle.getParam("motor_number", motorNumber);
	
	std::string infoTopic;
	privateNodeHandle.getParam("info_topic", infoTopic);
	
	std::string speedTopic;
	privateNodeHandle.getParam("speed_topic", speedTopic);

	double kF;
	privateNodeHandle.getParam("kF", kF);

	double kP;
	privateNodeHandle.getParam("kP", kP);

	double kI;
	privateNodeHandle.getParam("kI", kI);

	double kD;
	privateNodeHandle.getParam("kD", kD);

	ctre::phoenix::platform::can::setCANInterface("can0");
	
	talonSRX = new TalonSRX(motorNumber);

	talonSRX->SelectProfileSlot(0,0);
	talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, CONFIG_TIMEOUT_MS);
	talonSRX->ConfigClosedloopRamp(2);

	talonSRX->ConfigNominalOutputForward(0, CONFIG_TIMEOUT_MS);
	talonSRX->ConfigNominalOutputReverse(0, CONFIG_TIMEOUT_MS);
	talonSRX->ConfigPeakOutputForward(0, CONFIG_TIMEOUT_MS);
	talonSRX->ConfigPeakOutputReverse(0, CONFIG_TIMEOUT_MS);

	talonSRX->Config_kF(PID_LOOP_IDX, kF, CONFIG_TIMEOUT_MS);
	talonSRX->Config_kP(PID_LOOP_IDX, kP, CONFIG_TIMEOUT_MS);
	talonSRX->Config_kI(PID_LOOP_IDX, kI, CONFIG_TIMEOUT_MS);
	talonSRX->Config_kD(PID_LOOP_IDX, kD, CONFIG_TIMEOUT_MS);

	talonSRX->ConfigAllowableClosedLoopError(PID_LOOP_IDX, 0, CONFIG_TIMEOUT_MS);

	talonSRX->Set(ControlMode::PercentOutput, 0);

	ros::Subscriber speedSubscriber = nodeHandle.subscribe(speedTopic, 1, setSpeed);

	while (ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(100); 
		//usleep(20); // Why is this necessary?
		ros::spinOnce();
	}

	return 0;
}
