//
// Created by xuhao on 5/21/19.
//

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <inf_uwb_ros/incoming_broadcast_data.h>
#include <inf_uwb_ros/data_buffer.h>
#include <swarmtal_msgs/drone_onboard_command.h>
#include <swarmtal_msgs/drone_commander_state.h>
#include <mavlink/swarm/mavlink.h>
#include <inf_uwb_ros/remote_uwb_info.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

using namespace inf_uwb_ros;
using namespace swarmtal_msgs;

class SwarmPilot {
    ros::NodeHandle nh;

    ros::Subscriber incoming_data_sub, drone_cmd_state_sub, uwb_remote_sub, uwb_timeref_sub;
    ros::Publisher onboardcmd_pub;
    ros::Publisher uwb_send_pub;
    ros::Publisher planning_tgt_pub;
    ros::Publisher traj_pub;


    int accept_cmd_node_id = -1; //-1 Accept all, >=0 accept corresponding
    double send_drone_status_freq = 1.0;

    drone_commander_state cmd_state;
    ros::Time last_send_drone_status;

    int self_id = -1;
    uint8_t buf[1000] = {0};
    sensor_msgs::TimeReference uwb_time_ref;
    void on_uwb_timeref(const sensor_msgs::TimeReference &ref) {
        uwb_time_ref = ref;
    }

    ros::Time LPS2ROSTIME(const int32_t &lps_time) {
        ros::Time base = uwb_time_ref.header.stamp - ros::Duration(uwb_time_ref.time_ref.toSec());
        return base + ros::Duration(lps_time / 1000.0);
    }

    int32_t ROSTIME2LPS(ros::Time ros_time) {
        double lps_t_s = (ros_time - uwb_time_ref.header.stamp).toSec() + uwb_time_ref.time_ref.toSec();
        return (int32_t)(lps_t_s * 1000);
    }
public:

    SwarmPilot(ros::NodeHandle & _nh):
        nh(_nh) {
//        ROS_INFO()
        nh.param<int>("drone_id", self_id, -1);
        nh.param<int>("acpt_cmd_node", accept_cmd_node_id, -1);
        nh.param<double>("send_drone_status_freq", send_drone_status_freq, 5);

        uwb_timeref_sub = nh.subscribe("/uwb_node/time_ref", 1, &SwarmPilot::on_uwb_timeref, this, ros::TransportHints().tcpNoDelay());

        onboardcmd_pub = nh.advertise<drone_onboard_command>("/drone_commander/onboard_command", 1);
        uwb_send_pub = nh.advertise<data_buffer>("/uwb_node/send_broadcast_data", 10);
        planning_tgt_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        traj_pub = nh.advertise<std_msgs::Int8>("/swarm_traj_start_trigger", 1);
        incoming_data_sub = nh.subscribe("/uwb_node/incoming_broadcast_data", 10, &SwarmPilot::incoming_broadcast_data_sub, this, ros::TransportHints().tcpNoDelay());
        drone_cmd_state_sub = nh.subscribe("/drone_commander/swarm_commander_state", 1, &SwarmPilot::on_drone_commander_state, this, ros::TransportHints().tcpNoDelay());
        uwb_remote_sub = nh.subscribe("/uwb_node/remote_nodes", 1, &SwarmPilot::on_uwb_remote_node, this, ros::TransportHints().tcpNoDelay());
        last_send_drone_status = ros::Time::now() - ros::Duration(10);
     
        
    }

    void on_uwb_remote_node(const remote_uwb_info & info) {
        self_id = info.self_id;
    }

    void send_planning_command(const drone_onboard_command & cmd) {
        if (cmd.command_type == drone_onboard_command::CTRL_PLANING_TGT_COMMAND) {
            geometry_msgs::PoseStamped pose_tgt;
            pose_tgt.header.stamp = ros::Time::now();
            pose_tgt.header.frame_id = "world";
            
            pose_tgt.pose.position.x = cmd.param1 / 10000.0;
            pose_tgt.pose.position.y = cmd.param2 / 10000.0;
            pose_tgt.pose.position.z = cmd.param3 / 10000.0;

            Eigen::Quaterniond _quat(Eigen::AngleAxisd(-cmd.param4/10000.0, Eigen::Vector3d::UnitZ()));
            // pose_tgt.
            pose_tgt.pose.orientation.w = _quat.w();
            pose_tgt.pose.orientation.x = _quat.x();
            pose_tgt.pose.orientation.y = _quat.y();
            pose_tgt.pose.orientation.z = _quat.z();

            planning_tgt_pub.publish(pose_tgt);
        }
    }

    void traj_mission_callback(uint32_t cmd_type) {
        int _cmd = cmd_type - 100;
        std_msgs::Int8 cmd;
        cmd.data = _cmd;
        traj_pub.publish(cmd);
    }

    void on_mavlink_msg_remote_cmd(ros::Time stamp, int node_id, const mavlink_swarm_remote_command_t & cmd) {
        if (cmd.command_type >= 100) {
            traj_mission_callback(cmd.command_type);
            return;
        }

        drone_onboard_command onboardCommand;
        
        if (node_id != accept_cmd_node_id && accept_cmd_node_id >= 0) {
            ROS_WARN("Command from unacceptable drone %d, reject", node_id);
            return;
        } 
        if (cmd.target_id >=0 && cmd.target_id != self_id) {
            ROS_WARN("Control target %d not this drone, reject", cmd.target_id);            
            return;
        }

        ROS_INFO("Recv onboard cmd from %d type %d with 1-3 %d %d %d 4-6 %d %d %d, 7-10 %d %d %d %d",
                    node_id,  cmd.command_type,
                    cmd.param1, cmd.param2, cmd.param3,
                    cmd.param4, cmd.param5, cmd.param6,
                    cmd.param7, cmd.param8, cmd.param9,
                    cmd.param10);
        
        onboardCommand.command_type = cmd.command_type;
        onboardCommand.param1 = cmd.param1;
        onboardCommand.param2 = cmd.param2;
        onboardCommand.param3 = cmd.param3;
        onboardCommand.param4 = cmd.param4;
        onboardCommand.param5 = cmd.param5;
        onboardCommand.param6 = cmd.param6;
        onboardCommand.param7 = cmd.param7;
        onboardCommand.param8 = cmd.param8;
        onboardCommand.param9 = cmd.param9;
        onboardCommand.param10 = cmd.param10;


        if (cmd.command_type >= drone_onboard_command::CTRL_PLANING_TGT_COMMAND) {
            send_planning_command(onboardCommand);
        } else {
            onboardcmd_pub.publish(onboardCommand);
        }
    }

    void on_drone_commander_state(const drone_commander_state & _state) {
        mavlink_message_t msg;
        if (self_id < 0) {
            ROS_WARN("Don't have self id, unable to send drone status");
        }
        cmd_state = _state;

        if ((ros::Time::now() - last_send_drone_status).toSec() > (1 / send_drone_status_freq) ) {
            last_send_drone_status = ros::Time::now();
            double bat_remain = -1;
            mavlink_msg_drone_status_pack(self_id, 0, &msg, ROSTIME2LPS(ros::Time::now()), _state.flight_status, _state.control_auth,
                    _state.commander_ctrl_mode, _state.ctrl_input_state, _state.rc_valid, _state.onboard_cmd_valid, _state.djisdk_valid,
                    _state.vo_valid, _state.bat_vol, bat_remain, _state.pos.x, _state.pos.y, _state.pos.z, _state.yaw);
            ROS_INFO("Sending swarm status");
            send_mavlink_message(msg);
        }

    }

    void send_mavlink_message(mavlink_message_t & msg) {
        int len = mavlink_msg_to_send_buffer(buf , &msg);
        data_buffer buffer;
        buffer.data = std::vector<uint8_t>(buf, buf+len);
        buffer.send_method = 2;
        uwb_send_pub.publish(buffer);
    }

    void incoming_broadcast_data_callback(std::vector<uint8_t> data, int sender_drone_id, ros::Time stamp) {
//        ROS_INFO("Recv incoming msg %d", data.data.size());
        mavlink_message_t msg;
        mavlink_status_t status;
        bool ret = false;
        for (int i = 0; i <data.size(); i++){
            uint8_t  c = data[i];
            if (mavlink_parse_char(0, c, &msg, &status)) {
                switch(msg.msgid) {
                    case MAVLINK_MSG_ID_SWARM_REMOTE_COMMAND:
                        mavlink_swarm_remote_command_t cmd;
                        mavlink_msg_swarm_remote_command_decode(&msg, &cmd);
                        on_mavlink_msg_remote_cmd(stamp, sender_drone_id, cmd);
                        break;
                    default:
                        break;
                }
            }
        }

    }

    void incoming_broadcast_data_sub(const incoming_broadcast_data & data) {
        incoming_broadcast_data_callback(data.data, data.remote_id, data.header.stamp);
    }

};


int main(int argc, char** argv)
{

    ROS_INFO("swarm pilot\nIniting\n");

    ros::init(argc, argv, "swarm_pilot");

    ros::NodeHandle nh("swarm_pilot");

    SwarmPilot sp(nh);

    ROS_INFO("swarm_pilot node ready");
    ros::spin();

}
