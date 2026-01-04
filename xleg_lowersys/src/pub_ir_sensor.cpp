/*
 * 红外模块控制节点 - RK3588 GPIO版本
 * 功能：
 * 1. 直接通过/sys/class/gpio控制RK3588的GPIO引脚
 * 2. 读取红外传感器的数字输出（检测障碍物）
 * 3. 控制GPIO输出（控制红外模块）
 * 
 * 使用说明：
 * 1. RK3588 GPIO编号计算：GPIO编号 = 控制器编号 × 32 + 端口编号 × 8 + 引脚索引
 *    端口编号：A=0, B=1, C=2, D=3
 *    示例：GPIO1_D0 = 1×32 + 3×8 + 0 = 56
 * 2. 修改launch文件中的GPIO参数（ir_gpio_input, ir_gpio_output）
 * 3. 启动节点：roslaunch xleg_lowersys ir_sensor.launch
 * 4. 需要root权限或加入gpio用户组
 * 
 * Topic:
 * - 发布：xleg/ir_obstacle (std_msgs/Bool) - 障碍物检测结果
 * - 订阅：xleg/ir_io_control (std_msgs/UInt8) - IO控制命令
 *         数据格式：高4位为GPIO编号的低位，低4位为值（0或1）
 *         注意：当前实现使用GPIO编号的低8位，如需完整编号请修改消息格式
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <map>

using namespace std;

// RK3588 GPIO控制类
class RK3588GPIO {
private:
    int gpio_number;
    string gpio_path;
    bool is_exported;
    bool is_input;

public:
    RK3588GPIO(int gpio_num) : gpio_number(gpio_num), is_exported(false), is_input(false) {
        gpio_path = "/sys/class/gpio/gpio" + to_string(gpio_number);
    }

    // 导出GPIO
    bool exportGPIO() {
        if (is_exported) return true;
        
        // 检查是否已经导出
        ifstream check(gpio_path + "/direction");
        if (check.good()) {
            is_exported = true;
            check.close();
            return true;
        }
        check.close();
        
        ofstream export_file("/sys/class/gpio/export");
        if (!export_file.is_open()) {
            ROS_ERROR("Failed to open /sys/class/gpio/export (may need root permission)");
            return false;
        }
        export_file << gpio_number;
        export_file.close();
        usleep(200000); // 等待200ms让系统创建文件
        
        // 验证是否成功导出
        ifstream verify(gpio_path + "/direction");
        if (verify.good()) {
            is_exported = true;
            verify.close();
            ROS_INFO("GPIO %d exported successfully", gpio_number);
            return true;
        } else {
            ROS_ERROR("GPIO %d export failed", gpio_number);
            return false;
        }
    }

    // 取消导出GPIO
    void unexportGPIO() {
        if (!is_exported) return;
        
        ofstream unexport_file("/sys/class/gpio/unexport");
        if (unexport_file.is_open()) {
            unexport_file << gpio_number;
            unexport_file.close();
            is_exported = false;
            ROS_INFO("GPIO %d unexported", gpio_number);
        }
    }

    // 设置方向：in或out
    bool setDirection(const string& direction) {
        if (!is_exported) {
            if (!exportGPIO()) return false;
        }
        
        string dir_path = gpio_path + "/direction";
        ofstream dir_file(dir_path);
        if (!dir_file.is_open()) {
            ROS_ERROR("Failed to set direction for GPIO %d", gpio_number);
            return false;
        }
        dir_file << direction;
        dir_file.close();
        is_input = (direction == "in");
        return true;
    }

    // 读取GPIO值
    bool readValue() {
        if (!is_exported) {
            if (!exportGPIO()) {
                ROS_WARN("GPIO %d not exported", gpio_number);
                return false;
            }
            if (!setDirection("in")) {
                return false;
            }
        }
        
        string value_path = gpio_path + "/value";
        ifstream value_file(value_path);
        if (!value_file.is_open()) {
            ROS_ERROR("Failed to read GPIO %d", gpio_number);
            return false;
        }
        
        string value;
        value_file >> value;
        value_file.close();
        
        return (value == "1");
    }

    // 写入GPIO值
    bool writeValue(bool value) {
        if (!is_exported) {
            if (!exportGPIO()) return false;
        }
        
        if (is_input) {
            if (!setDirection("out")) {
                return false;
            }
        }
        
        string value_path = gpio_path + "/value";
        ofstream value_file(value_path);
        if (!value_file.is_open()) {
            ROS_ERROR("Failed to write GPIO %d", gpio_number);
            return false;
        }
        value_file << (value ? "1" : "0");
        value_file.close();
        return true;
    }

    ~RK3588GPIO() {
        unexportGPIO();
    }
};

// 全局GPIO对象管理（使用map支持多个GPIO）
map<int, RK3588GPIO*> gpio_map;

// 获取或创建GPIO对象
RK3588GPIO* getGPIO(int gpio_num) {
    if (gpio_map.find(gpio_num) == gpio_map.end()) {
        gpio_map[gpio_num] = new RK3588GPIO(gpio_num);
    }
    return gpio_map[gpio_num];
}

// 红外传感器状态
bool ir_obstacle_detected = false;  // 是否检测到障碍物

// 读取数字IO状态
bool readDigitalIO(int gpio_num) {
    RK3588GPIO* gpio = getGPIO(gpio_num);
    if (gpio == nullptr) {
        return false;
    }
    return gpio->readValue();
}

// 控制IO输出
bool writeDigitalIO(int gpio_num, bool value) {
    RK3588GPIO* gpio = getGPIO(gpio_num);
    if (gpio == nullptr) {
        return false;
    }
    return gpio->writeValue(value);
}

// 控制IO输出的回调函数
void ioControlCallback(const std_msgs::UInt8::ConstPtr& msg) {
    // msg->data 格式：高4位为GPIO编号的低位，低4位为值（0或1）
    // 注意：当前实现使用GPIO编号的低8位
    // 如果需要完整的GPIO编号，建议修改消息格式或使用服务
    uint8_t gpio_low = (msg->data >> 4) & 0x0F;
    uint8_t value = msg->data & 0x0F;
    
    // 实际使用时，建议从参数或消息中获取完整GPIO编号
    // 这里简化处理：假设GPIO编号在0-15范围内
    int gpio_num = gpio_low;
    
    bool success = writeDigitalIO(gpio_num, value != 0);
    if(success) {
        ROS_INFO("GPIO Control: GPIO %d set to %s", gpio_num, (value ? "HIGH" : "LOW"));
    } else {
        ROS_WARN("GPIO Control: Failed to set GPIO %d", gpio_num);
    }
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "ir_sensor_gpio_node");
    ros::NodeHandle n;
    ros::NodeHandle nh_private("~");

    // 获取参数 - RK3588 GPIO编号
    int ir_gpio_input;   // 输入GPIO（读取红外传感器）
    int ir_gpio_output; // 输出GPIO（控制红外模块，可选）
    double publish_rate;
    
    // RK3588 GPIO编号示例：
    // GPIO1_D0 = 1×32 + 3×8 + 0 = 56
    // GPIO2_B3 = 2×32 + 1×8 + 3 = 75
    // GPIO0_A5 = 0×32 + 0×8 + 5 = 5
    nh_private.param<int>("ir_gpio_input", ir_gpio_input, 56);   // 默认GPIO1_D0
    nh_private.param<int>("ir_gpio_output", ir_gpio_output, 75); // 默认GPIO2_B3（可选）
    nh_private.param<double>("publish_rate", publish_rate, 10.0);

    ROS_INFO("IR Sensor GPIO Node (RK3588) Starting...");
    ROS_INFO("Input GPIO: %d, Output GPIO: %d", ir_gpio_input, ir_gpio_output);

    // 初始化输入GPIO（用于读取红外传感器）
    RK3588GPIO* input_gpio = getGPIO(ir_gpio_input);
    if (!input_gpio->setDirection("in")) {
        ROS_ERROR("Failed to configure input GPIO %d", ir_gpio_input);
        ROS_ERROR("Please check: 1) GPIO number is correct, 2) Have root permission, 3) GPIO not used by other driver");
        return -1;
    }

    // 初始化输出GPIO（可选，用于控制红外模块）
    RK3588GPIO* output_gpio = nullptr;
    if (ir_gpio_output > 0) {
        output_gpio = getGPIO(ir_gpio_output);
        if (!output_gpio->setDirection("out")) {
            ROS_WARN("Failed to configure output GPIO %d, output control disabled", ir_gpio_output);
            output_gpio = nullptr;
        } else {
            ROS_INFO("Output GPIO %d configured successfully", ir_gpio_output);
        }
    }

    // 创建发布者
    ros::Publisher ir_obstacle_pub = n.advertise<std_msgs::Bool>("xleg/ir_obstacle", 10);

    // 创建订阅者（用于控制IO输出）
    ros::Subscriber io_control_sub = n.subscribe("xleg/ir_io_control", 10, ioControlCallback);

    // 消息对象
    std_msgs::Bool obstacle_msg;

    // 主循环
    ros::Rate loop_rate(publish_rate);
    ROS_INFO("IR Sensor GPIO Node is running...");

    while(ros::ok()) {
        // 读取GPIO状态（检测障碍物）
        bool current_state = readDigitalIO(ir_gpio_input);
        
        // 状态变化时输出日志
        static bool last_state = false;
        if(current_state != last_state) {
            last_state = current_state;
            ir_obstacle_detected = current_state;
            ROS_INFO("IR Obstacle Detection: %s", ir_obstacle_detected ? "DETECTED" : "CLEAR");
        } else {
            ir_obstacle_detected = current_state;
        }
        
        // 发布障碍物检测结果
        obstacle_msg.data = ir_obstacle_detected;
        ir_obstacle_pub.publish(obstacle_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 清理GPIO对象
    for (auto& pair : gpio_map) {
        delete pair.second;
    }
    gpio_map.clear();

    ROS_INFO("IR Sensor GPIO Node shutdown");
    return 0;
}
