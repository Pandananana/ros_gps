#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <libgpsmm.h>

using namespace std::chrono_literals;

class GpsPublisher : public rclcpp::Node
{
public:
	GpsPublisher()
		: Node("gps_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
		timer_ = this->create_wall_timer(
			500ms, std::bind(&GpsPublisher::timer_callback, this));

		// Initialize GPS
		gps_rec.stream(WATCH_ENABLE | WATCH_JSON);
	}

private:
	void timer_callback()
	{
		auto message = sensor_msgs::msg::NavSatFix();

		struct gps_data_t *gpsd_data;

		if (gpsd_data->fix.mode > MODE_NO_FIX)
		{ // Check for a valid GPS fix using mode
			message.header.stamp = this->now();
			message.header.frame_id = "gps";

			message.latitude = gpsd_data->fix.latitude;
			message.longitude = gpsd_data->fix.longitude;
			message.altitude = gpsd_data->fix.altitude;

			// Set the status to indicate a fix
			message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // Ensure this is the correct way to set a fix status

			// Set some default covariance
			message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
			message.position_covariance[0] = 10.0;
			message.position_covariance[4] = 10.0;
			message.position_covariance[8] = 10.0;

			RCLCPP_INFO(this->get_logger(), "Publishing: Lat: %.6f, Lon: %.6f, Alt: %.2f",
						message.latitude, message.longitude, message.altitude);

			publisher_->publish(message);
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "No GPS fix");
		}
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
	size_t count_;
	gpsmm gps_rec{"localhost", DEFAULT_GPSD_PORT};
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GpsPublisher>());
	rclcpp::shutdown();
	return 0;
}