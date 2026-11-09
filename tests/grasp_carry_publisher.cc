#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

int main()
{
  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::StringMsg>("/drink/carry");

  // Keep one Gazebo Transport publisher alive for the complete test stream.
  // Short-lived "gz topic -p" processes can exit before their individual
  // messages have been delivered, even when the CLI itself exits cleanly.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::string command;
  while (std::getline(std::cin, command))
  {
    if (command.empty())
      continue;
    gz::msgs::StringMsg message;
    message.set_data(command);
    if (!publisher.Publish(message))
    {
      std::cerr << "failed to publish command: " << command << std::endl;
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Allow the transport layer to flush the final message before teardown.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return 0;
}
