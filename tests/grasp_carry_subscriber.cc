#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char **argv)
{
  const std::string marker =
      argc > 1 ? argv[1] : "/tmp/grasp-carry-first-ack-dropped";
  gz::transport::Node node;
  std::atomic<bool> dropped{false};
  std::mutex outputMutex;

  const auto callback =
      [&](const gz::msgs::StringMsg &_message)
      {
        std::lock_guard<std::mutex> lock(outputMutex);
        if (!dropped.exchange(true))
        {
          std::cout << "FIRST_ACK_DROPPED " << _message.data()
                    << std::endl;
          std::ofstream markerFile(marker);
          return;
        }
        std::cout << "data: \"" << _message.data() << "\""
                  << std::endl;
      };

  if (!node.Subscribe("/drink/carry/ack", callback))
  {
    std::cerr << "failed to subscribe to /drink/carry/ack" << std::endl;
    return 2;
  }

  std::cout << "ACK_SUBSCRIBER_READY" << std::endl;
  while (true)
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
