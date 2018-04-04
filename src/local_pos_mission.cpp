/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>



#include <atomic>
#include <thread>
#include <mutex>

class MAVROS_PoseCtrl
{
  public:

    typedef double TPrecision;

    template<typename TData = TPrecision>
    struct TVector3
    {
      TVector3() : x(0), y(0), z(0) { }

      TVector3(TData const& x, TData const& y, TData const& z) : x(x), y(y), z(z) { }

      TVector3(TVector3 const& rhs)
      {
        *this = rhs;
      }

      TVector3 operator=(TVector3 const& rhs) const
      {
        return TVector3(rhs.x, rhs.y, rhs.z);
      }
      TData x;
      TData y;
      TData z;
    };

    typedef TVector3<TPrecision> Vector3;

    MAVROS_PoseCtrl(ros::NodeHandle& nh, double const rate_Hz = 10) : nh_(nh)
    {
      rate_Hz_ = rate_Hz;
      shutdown_ = false;
      run_ = false;

      state_sub_ = nh_.subscribe<mavros_msgs::State>
                   ("/mavros/state", 1, &MAVROS_PoseCtrl::state_cb, this);
      pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
                  ("/poseStamped", 1, &MAVROS_PoseCtrl::pose_stamped_cb, this);
      local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
                       ("/mavros/setpoint_position/local", 1);
      arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
                       ("/mavros/cmd/arming");
      set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
                         ("/mavros/set_mode");

      connect();
      arm();
      take_off();
      sleep(4);
      set_point();

      mpThread = (new std::thread(&MAVROS_PoseCtrl::run, this));

    }

    ~MAVROS_PoseCtrl()
    {
      ROS_INFO("land...");
      ////////////////////////////////////////////
      ///////////////////LAND/////////////////////
      ////////////////////////////////////////////
      ros::ServiceClient land_cl = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
      mavros_msgs::CommandTOL srv_land;
      srv_land.request.altitude = 0;
      srv_land.request.latitude = 0;
      srv_land.request.longitude = 0;
      srv_land.request.min_pitch = 0;
      srv_land.request.yaw = 0;
      if(land_cl.call(srv_land))
      {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
      }
      else
      {
        ROS_ERROR("Failed Land");
      }


      stop();
      mpThread->join();
      ROS_ERROR("thread killed!");
      delete mpThread;
    }

    void start()
    {
      run_ = true;
    }
    void stop()
    {
      shutdown_ = true;
    }

    void run()
    {
      ros::Rate rate(rate_Hz_);
      ros::Time last_request = ros::Time::now() - ros::Duration(5.0);
      bool request_send = false;
      int cnt = 100;

      mavros_msgs::SetMode set_mode_msg;
      set_mode_msg.request.custom_mode = "OFFBOARD";
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;

      while(!shutdown_ && ros::ok())
      {
        ros::spinOnce();
        request_send = false;

        if(run_)
        {


          if( current_state_.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(5.0)))
          {
            if( set_mode_client_.call(set_mode_msg) &&
                set_mode_msg.response.mode_sent)
            {
              ROS_INFO("Offboard enabled");
            }
            request_send = true;
          }

          if( !current_state_.armed &&
              (ros::Time::now() - last_request > ros::Duration(5.0)))
          {
            if( arming_client_.call(arm_cmd) &&
                arm_cmd.response.success)
            {
              ROS_INFO("Vehicle armed");
            }
            request_send = true;
          }

          if(request_send)
          {
            last_request = ros::Time::now();
          }


          Vector3 p;
          double yaw;
          get_pos(p, yaw);

          geometry_msgs::PoseStamped msg;
          msg.pose.position.x = p.x;
          msg.pose.position.y = p.y;
          msg.pose.position.z = p.z;
          msg.header.stamp = ros::Time::now();
          msg.header.seq = cnt;
          msg.header.frame_id = 1;

          double t0 = cos(yaw * 0.5);
          double t1 = sin(yaw * 0.5);;
          double t2 = 1;
          double t3 = 0;
          double t4 = 1;
          double t5 = 0;

          msg.pose.orientation.w = t2 * t4 * t0 + t3 * t5 * t1;
          msg.pose.orientation.x = t3 * t4 * t0 - t2 * t5 * t1;
          msg.pose.orientation.y = t2 * t5 * t0 + t3 * t4 * t1;
          msg.pose.orientation.z = t2 * t4 * t1 - t3 * t5 * t0;

          local_pos_pub_.publish(msg);
          ROS_INFO("point published");
          cnt++;
        }
        rate.sleep();
      }
    }

    void absPosition(Vector3 const& p, double const yaw = 0)
    {
      ROS_INFO("abs pos cb");
      set_pos(p, yaw);
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
      ROS_INFO("state cb");
      set_state(*msg);
    }

    void pose_stamped_cb(geometry_msgs::PoseStampedConstPtr const& msg)
    {
      ROS_INFO("pos cb");
      geometry_msgs::Quaternion q = msg->pose.orientation;
      set_pos(Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
              std::atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z));
    }

    void connect()
    {
      ROS_INFO("try to connect...");
      ros::Rate rate(rate_Hz_);
      // wait for FCU connection
      while(ros::ok() && !get_state().connected)
      {
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("connected");
    }
    bool arm()
    {

      ros::ServiceClient arming_cl = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
      mavros_msgs::CommandBool srv;
      srv.request.value = true;
      if(arming_cl.call(srv))
      {
        ROS_INFO("ARM send ok %d", srv.response.success);
        return true;
      }
      else
      {
        ROS_ERROR("Failed arming or disarming");
        return false;
      }
    }

    bool take_off()
    {
      ////////////////////////////////////////////
      /////////////////TAKEOFF////////////////////
      ////////////////////////////////////////////
      ros::ServiceClient takeoff_cl = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
      mavros_msgs::CommandTOL srv_takeoff;
      srv_takeoff.request.altitude = 3;
      srv_takeoff.request.latitude = 0;
      srv_takeoff.request.longitude = 0;
      srv_takeoff.request.min_pitch = 0;
      srv_takeoff.request.yaw = 0;
      if(takeoff_cl.call(srv_takeoff))
      {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
      }
      else
      {
        ROS_ERROR("Failed Takeoff");
      }
    }

  private:
    ros::NodeHandle nh_;
    double rate_Hz_ = 10;

    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    std::mutex mtx_state_;
    mavros_msgs::State current_state_;
    Vector3 last_pos_;
    double last_yaw_;
    std::mutex mtx_pos;
    std::atomic_bool shutdown_;
    std::atomic<bool>  run_;

    std::thread        *mpThread;

    void set_point(uint const num = 10)
    {

      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = 1;
      msg.pose.position.x = last_pos_.x;
      msg.pose.position.y = last_pos_.y;
      msg.pose.position.z = last_pos_.z;
      msg.pose.orientation.x = 0;
      msg.pose.orientation.y = 0;
      msg.pose.orientation.z = 0;
      msg.pose.orientation.w = 1;
      ros::Rate rate(rate_Hz_);
      //send a few setpoints before starting
      for(uint i = 0; ros::ok() && i < num; i++)
      {
        msg.header.stamp = ros::Time::now();
        msg.header.seq = i;

        local_pos_pub_.publish(msg);
        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("point set");
    }

    void set_state(mavros_msgs::State const& s)
    {
      std::lock_guard<std::mutex> lock(mtx_state_);
      current_state_ = s;
    }
    mavros_msgs::State get_state()
    {
      std::lock_guard<std::mutex> lock(mtx_state_);
      return current_state_;
    }

    void set_pos(Vector3 const& p, double const yaw = 0)
    {
      ROS_INFO("set pos: %f, %f, %f", p.x, p.y, p.z);
      std::lock_guard<std::mutex> lock(mtx_pos);
      last_pos_ = p;
      last_yaw_ = yaw;
    }

    void get_pos(Vector3 &p, double &yaw)
    {
      std::lock_guard<std::mutex> lock(mtx_pos);
      p = last_pos_;
      yaw = last_yaw_;
    }

};


int main(int argc, char **argv)
{

  // disable RC failsafe: param set NAV_RCL_ACT 0 in the ekf2/iris file
  ros::init(argc, argv, "local_node");
  ros::NodeHandle nh("~");

  ROS_INFO("create instance");
  MAVROS_PoseCtrl pos_ctrl(nh, 5);

  pos_ctrl.start();

  ros::Rate r(0.5);

  MAVROS_PoseCtrl::Vector3 v(5,5,5);

  bool sign = true;
  while(ros::ok())
  {
    if(sign)
    {
      v.x += 1;
    }
    else
    {
      v.x -= 1;
    }

    if(v.x >= 5)
    {
      sign = false;
    }
    else if (v.x <= 0)
    {
      sign = true;
    }
    pos_ctrl.absPosition(v);
    r.sleep();
  }
  pos_ctrl.stop();
  return 0;
}
