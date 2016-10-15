#ifndef __SLAM_FAKE_COMPASS_HPP__
#define __SLAM_FAKE_COMPASS_HPP__

#include "../lcmtypes/state_t.hpp"
#include <vector>
#include <lcm/lcm-cpp.hpp>

class FakeCompass
{
public:
	double getNorthLocation();

	void handleState(const lcm::ReceiveBuffer * rbuf,
					 const std::string & chan,
					 const common::LMC::types::state_t * state);

	size_t getNumStates() const;

private:
	std::vector<common::LCM::types::state_t> states;
};

#endif
