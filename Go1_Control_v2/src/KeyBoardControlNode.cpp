/** Created by Amir on 03/02/23 **/

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Char>("keyBoardControl", 1000);

  // Set terminal to raw mode
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  std::cout << "Type characters on the keyboard. Press Ctrl-C to exit." << std::endl;

  while (ros::ok())
  {
    // Read character input from the terminal
    char c;
    if (read(STDIN_FILENO, &c, 1) < 0) break;

    // Create a new message with the input character
    std_msgs::Char msg;
    msg.data = c;

    // Publish the message on the topic
    pub.publish(msg);
  }

  // Reset terminal to normal mode
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return 0;
}




// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <iostream>

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "keyboard_publisher");
//   ros::NodeHandle nh;

//   ros::Publisher pub = nh.advertise<std_msgs::String>("keyBoardControl", 1000);

//   std::cout << "Type your message and press enter. Type 'exit' to quit." << std::endl;

//   while (ros::ok())
//   {
//     std_msgs::String msg;
//     std::getline(std::cin, msg.data);

//     if (msg.data == "exit") break;

//     pub.publish(msg);
//   }

//   return 0;
// }