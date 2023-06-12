#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

const char* msg = R"(
Control your omo_r3mini~~
----------------------------------------------------------------------
Moving around:
        w
    a   s   d
        x
w/x: increase/decrease linear velocity (omo_r3mini: ~0.3m/s)
a/d: increase/decrease angular velocity (omo_r3mini: ~3.5rad/s)
space, s: force stop
----------------------------------------------------------------------
CTRL+C to quit
)";

int getch()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt); // store old settings and copy to new setting
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_iflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar(); // get the current character

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // reapply old settings

  return ch;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_keyboard");
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  while(rclcpp::ok())
  {
    key = getch();
  }


  // (void) argc;
  // (void) argv;

  // printf("hello world r3mini_teleop package\n");
  // return 0;
}
