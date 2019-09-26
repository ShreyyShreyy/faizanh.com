/*! @file
 *
 *  @brief Main entry point for assignment 3.
 *
 *  41012 Programming For Mechatronica Systems Spring 2019
 *
 *  @author {Shreynit Prasad 12602520}
 *  @date {27/09/19}
*/

#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"
#include"PurePursuit.h"

int main(void)
{
 std::vector<std::thread> threads;
 std::shared_ptr<Simulator> sim(new Simulator());
 std::shared_ptr<PurePursuit> friendly_control(new PurePursuit(sim));

 threads.push_back(sim->spawn());

 threads.push_back(std::thread(std::bind(&PurePursuit::Watchdog, friendly_control)));
 threads.push_back(std::thread(std::bind(&PurePursuit::DistanceFromFriendly, friendly_control)));
 threads.push_back(std::thread(std::bind(&PurePursuit::DistanceFromBase, friendly_control)));
 threads.push_back(std::thread(std::bind(&PurePursuit::FriendlyControl, friendly_control)));
 threads.push_back(std::thread(std::bind(&PurePursuit::findBogie,friendly_control)));

  for(auto & t: threads){
    t.join();
  }

  return 0;
}

