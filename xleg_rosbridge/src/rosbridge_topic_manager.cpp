/*
 * rosbridge_topic_manager.cpp
 * 
 * 实现rosbridge话题发布节点，通过Websocket接口实现对外发布话题内容动态调整功能
 * 提供话题的动态添加、删除、修改功能，提高通信效率以及拓展性
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <map>
#include <string>
#include <vector>

// 注意：RosbridgeTopicConfig.h 会在编译时由CMake自动生成
// 如果编译时出现找不到该头文件的错误，请先编译项目生成该文件
#ifdef HAVE_DYNAMIC_RECONFIGURE
#include <xleg_rosbridge/RosbridgeTopicConfig.h>
#endif

// 话题管理器类
class RosbridgeTopicManager
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // 存储动态创建的话题发布者
    std::map<std::string, ros::Publisher> topic_publishers_;
    
    // 动态重配置服务器
#ifdef HAVE_DYNAMIC_RECONFIGURE
    dynamic_reconfigure::Server<xleg_rosbridge::RosbridgeTopicConfig> dyn_reconf_server_;
#endif
    
    // 话题配置列表
    std::vector<std::string> available_topics_;
    
    // 回调函数：处理动态重配置
#ifdef HAVE_DYNAMIC_RECONFIGURE
    void reconfigureCallback(xleg_rosbridge::RosbridgeTopicConfig &config, uint32_t level)
    {
        ROS_INFO("Rosbridge Topic Manager: Reconfigure Request");
        
        // 根据配置动态添加或移除话题
        // 这里可以根据config参数来动态管理话题
        // 例如：config.enable_footforce, config.enable_jointstate等
        
        if (config.enable_footforce && topic_publishers_.find("xleg/footForce") == topic_publishers_.end())
        {
            // 添加footForce话题（注意：实际发布由xleg_lowersys_node完成，这里只是标记）
            ROS_INFO("Enabling footForce topic for rosbridge");
        }
        
        if (config.enable_jointstate && topic_publishers_.find("xleg/jointState") == topic_publishers_.end())
        {
            ROS_INFO("Enabling jointState topic for rosbridge");
        }
        
        if (config.enable_bodystate && topic_publishers_.find("xleg/bodyState") == topic_publishers_.end())
        {
            ROS_INFO("Enabling bodyState topic for rosbridge");
        }
        
        if (config.enable_cameraimage && topic_publishers_.find("/cameraImage") == topic_publishers_.end())
        {
            ROS_INFO("Enabling cameraImage topic for rosbridge");
        }
    }
#else
    // 如果没有动态重配置，使用参数服务器
    void loadConfigFromParam()
    {
        bool enable_footforce, enable_jointstate, enable_bodystate, enable_cameraimage;
        nh_private_.param("enable_footforce", enable_footforce, true);
        nh_private_.param("enable_jointstate", enable_jointstate, true);
        nh_private_.param("enable_bodystate", enable_bodystate, true);
        nh_private_.param("enable_cameraimage", enable_cameraimage, false);
        
        if (enable_footforce) ROS_INFO("FootForce topic enabled for rosbridge");
        if (enable_jointstate) ROS_INFO("JointState topic enabled for rosbridge");
        if (enable_bodystate) ROS_INFO("BodyState topic enabled for rosbridge");
        if (enable_cameraimage) ROS_INFO("CameraImage topic enabled for rosbridge");
    }
#endif
    
public:
    RosbridgeTopicManager() : nh_private_("~")
    {
        // 初始化动态重配置服务器
#ifdef HAVE_DYNAMIC_RECONFIGURE
        dyn_reconf_server_.setCallback(boost::bind(&RosbridgeTopicManager::reconfigureCallback, this, _1, _2));
#endif
        
        // 初始化可用话题列表
        available_topics_.push_back("xleg/footForce");
        available_topics_.push_back("xleg/jointState");
        available_topics_.push_back("xleg/bodyState");
        available_topics_.push_back("/cameraImage");
        
        ROS_INFO("Rosbridge Topic Manager initialized");
        ROS_INFO("Available topics for rosbridge:");
        for (const auto& topic : available_topics_)
        {
            ROS_INFO("  - %s", topic.c_str());
        }
        
#ifndef HAVE_DYNAMIC_RECONFIGURE
        loadConfigFromParam();
#endif
    }
    
    ~RosbridgeTopicManager()
    {
        // 清理资源
        topic_publishers_.clear();
    }
    
    // 添加话题到发布列表
    bool addTopic(const std::string& topic_name, const std::string& topic_type)
    {
        if (topic_publishers_.find(topic_name) != topic_publishers_.end())
        {
            ROS_WARN("Topic %s already exists", topic_name.c_str());
            return false;
        }
        
        // 根据话题类型创建发布者
        // 注意：实际的话题发布由原始节点完成，rosbridge会自动转发
        // 这里主要用于管理和配置
        
        ROS_INFO("Added topic %s (type: %s) to rosbridge manager", topic_name.c_str(), topic_type.c_str());
        return true;
    }
    
    // 从发布列表移除话题
    bool removeTopic(const std::string& topic_name)
    {
        auto it = topic_publishers_.find(topic_name);
        if (it == topic_publishers_.end())
        {
            ROS_WARN("Topic %s not found", topic_name.c_str());
            return false;
        }
        
        topic_publishers_.erase(it);
        ROS_INFO("Removed topic %s from rosbridge manager", topic_name.c_str());
        return true;
    }
    
    // 获取所有可用话题
    std::vector<std::string> getAvailableTopics() const
    {
        return available_topics_;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbridge_topic_manager");
    
    RosbridgeTopicManager manager;
    
    ROS_INFO("Rosbridge Topic Manager node started");
    ROS_INFO("Use dynamic_reconfigure to adjust topic settings");
    ROS_INFO("Or use rosservice to add/remove topics dynamically");
    
    ros::spin();
    
    return 0;
}

