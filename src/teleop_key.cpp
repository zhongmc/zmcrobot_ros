#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class TeleopZMCRobot
{
    public:
        TeleopZMCRobot();
        void keyLoop();

    private:
        ros::NodeHandle nh_;
        float linear_, angular_, v_max, w_max, v_step, w_step;

        ros::Publisher twist_pub_;
};

    TeleopZMCRobot::TeleopZMCRobot():
        linear_(0),
        angular_(0),
        v_max(0.2),
        v_step(0.01),
        w_max(2),
        w_step(0.1)
        {
            nh_.param("v_max", v_max, v_max);
            nh_.param("v_step", v_step, v_step);
            nh_.param("w_max", w_max, w_max);
            nh_.param("w_step", w_step, w_step);

            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        }

    int kfd = 0;
    struct termios cooked, raw;

    void quit(int sig)
    {
        (void) sig;
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
    }

    int main(int argc, char **argv )
    {
        ros::init(argc, argv, "teleop_zmcrobot");
        TeleopZMCRobot teleop_zmcrobot;
        signal(SIGINT, quit);

        teleop_zmcrobot.keyLoop();
        quit(0);
        return (0);

    }

    void TeleopZMCRobot::keyLoop()
    {
        char c;
        bool dirty = false;

        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof( struct termios ));
        raw.c_lflag &=~(ICANON | ECHO );
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts("reading from keyboard");
        puts("-------------------------------");
        puts("use arrow keys to move the robot. space to brake; 'q' to quit.");

        linear_ = angular_ = 0;

        for(;;)
        {
            if(read(kfd, &c, 1) < 0 )
            {
                perror("read():");
                exit(-1);
            }
            switch(c)
            {
            case KEYCODE_L:
                angular_ -=w_step;
                if( angular_ < -w_max )
                angular_ = -w_max;
                dirty = true;
                break;
            case KEYCODE_R:
                angular_+=w_step;
                if( angular_ > w_max )
                    angular_ = w_max;
                dirty = true;
                break;
            case KEYCODE_U:
                linear_+=v_step;
                if( linear_ > v_max )
                    linear_ = v_max;
                dirty = true;
                break;
            case KEYCODE_D:
                linear_-=v_step;
                if( linear_ < - v_max )
                    linear_ = - v_max;
                dirty = true;
                break;

            case KEYCODE_SPACE:
                if( angular_ != 0 )
                {
                    angular_ = 0;
                    dirty = true;
                }
                else if( linear_ != 0 )
                {
                    linear_ = 0;
                    dirty = true;
                }
                break;
            case KEYCODE_Q:
                return;
            }

            if( dirty == true )
            {
                geometry_msgs::Twist twist;
                twist.angular.z =  angular_;
                twist.linear.x =  linear_;
                twist_pub_.publish(twist);
                dirty = false;
             //   fprintf(stdout, "%02f, %02f", linear_, angular_);
            }
        }

    }


