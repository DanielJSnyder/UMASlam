#include "../lcmtypes/particles_t.hpp"
#include "../src/Constants.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

using namespace std;

class Printer
{
public:
void printHandle(const lcm::ReceiveBuffer * rbuf, const std::string & chan, const particles_t * p)
{
  cout << "Printing " << p->num_particles << " particles" << endl;
  for(int i = 0; i < p->num_particles;++i)
  {
    cout << p->particles[i].x << "\t"
       << p->particles[i].y << "\t"
       << p->particles[i].theta << "\t"
       << p->particles[i].likelihood << endl;
  }
  cout << endl;
}
};

int main()
{
  Printer p;
  lcm::LCM lcm;
  lcm.subscribe(SLAM_PARTICLE_CHANNEL, &Printer::printHandle, &p);

  while(1)
  {
    lcm.handle();
  }
  return 0;
}
