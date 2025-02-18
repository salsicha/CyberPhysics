
// create virtual serial port:
// sudo socat -d -d PTY,link=/tmp/my_gps,raw,echo=0,ispeed=38400 PTY,link=/tmp/my_gps_ctl,raw,echo=0,ispeed=38400
// stdbuf -oL ./your_cpp_program > /tmp/my_gps
// sudo gpsd /tmp/my_gps -F /var/run/gpsd.sock
// cgps -s

// Accelerometer and Gyro
#include <iostream>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// I2C address of the accelerometer/gyroscope (check datasheet for specific address)
#define I2C_ADDR 0x68 

int main() {
    int file_i2c;
    if ((file_i2c = open("/dev/i2c-1", O_RDWR)) < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
        return 1;
    }

    if (ioctl(file_i2c, I2C_SLAVE, I2C_ADDR) < 0) {
         std::cerr << "Failed to acquire bus access and/or talk to slave" << std::endl;
        return 1;
    }
    
    // Read accelerometer and gyroscope data (example: reading X-axis acceleration)
    char reg_addr = 0x3B; // Register address for Accel X-axis High byte
    char buffer[2];

    if (write(file_i2c, &reg_addr, 1) != 1) {
        std::cerr << "Failed to write to the i2c bus" << std::endl;
        return 1;
    }

    if (read(file_i2c, buffer, 2) != 2) {
        std::cerr << "Failed to read from the i2c bus" << std::endl;
        return 1;
    }

    int16_t acc_x = (buffer[0] << 8) | buffer[1];
    std::cout << "Accelerometer X-axis: " << acc_x << std::endl;

    close(file_i2c);
    return 0;
}


// Access GPS
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    //Open the serial port (modify as needed, e.g., /dev/ttyAMA0)
    int serial_fd = open("/dev/ttyS0", O_RDWR);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port" << std::endl;
        return 1;
    }

    //Configure serial port
    struct termios tty;
    tcgetattr(serial_fd, &tty);
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    tty.c_oflag &= ~OPOST; // Raw output mode
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // Timeout of 1 second
    cfsetospeed(&tty, B115200); // Set baud rate
    cfsetispeed(&tty, B115200);
    tcsetattr(serial_fd, TCSANOW, &tty);

   // Send AT command to request GPS NMEA data (example)
    std::string cmd = "AT+CGPSINF=0\r";
    write(serial_fd, cmd.c_str(), cmd.length());

    //Read and process the GPS data
    char read_buf[256];
    int bytes_read = read(serial_fd, read_buf, sizeof(read_buf) - 1);
    if (bytes_read > 0) {
        read_buf[bytes_read] = '\0';
        std::cout << "GPS Data: " << read_buf << std::endl;
    } else {
        std::cout << "No GPS data received" << std::endl;
    }

    close(serial_fd);
    return 0;
}


// ROS2 version
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <serial/serial.h>
// Add I2C library include if needed

class SaraR5Node : public rclcpp::Node {
public:
    SaraR5Node() : Node("sara_r5_node") {
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_data", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
        
        // Initialize serial port for GPS (adjust port name as needed)
        try {
            serial_.setPort("/dev/ttyACM1"); 
            serial_.setBaudrate(115200); // Adjust baud rate if needed
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            // Handle error
        }
    }

private:
    void readAndPublishData() {
        // Read GPS data
        if (serial_.available()) {
            std::string nmea_line = serial_.readline();
            // Parse NMEA data and populate sensor_msgs::msg::NavSatFix
            // Publish GPS data
            auto gps_msg = sensor_msgs::msg::NavSatFix();
            // ... populate gps_msg ...
            gps_publisher_->publish(gps_msg);
        }

        // Read IMU data (example - adapt for your specific sensor)
        // ... read from I2C ...
        auto imu_msg = sensor_msgs::msg::Imu();
        // ... populate imu_msg ...
        imu_publisher_->publish(imu_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    serial::Serial serial_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SaraR5Node>();
    rclcpp::Rate rate(10); // Publish at 10 Hz
    while (rclcpp::ok()) {
        node->readAndPublishData();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

