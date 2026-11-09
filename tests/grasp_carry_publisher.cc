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

  // Do not consume the command FIFO until the plugin subscriber has been
  // discovered. Advertise() alone only creates a local publisher endpoint;
  // an immediate Publish() can otherwise race transport discovery.
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(10);
  while (!publisher.HasConnections() &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!publisher.HasConnections())
  {
    std::cerr << "timed out waiting for /drink/carry subscriber discovery"
              << std::endl;
    return 2;
  }

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
    // Keep enough spacing for the asynchronous Gazebo Transport worker to
    // deliver every command before the next one is queued. The 512-command
    // saturation burst still completes in about five seconds, while physics
    // remains paused so the plugin's bounded queue cannot drain.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Allow the transport layer to flush the final message before teardown.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return 0;
}
