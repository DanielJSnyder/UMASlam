#include "FakeCompass.hpp" 

using namespace std;
using namespace common::LCM::types;


void FakeCompass::handleState(const lcm::ReceiveBuffer * rbuf,
							  const string & chan,
							  const state_t * state)
{
	states.push_back(*state);
}

double FakeCompass::getNorthLocation()
{
	double total_north = 0;
	size_t num_norths = 0;
	for(size_t i = 0; i < 
}

size_t FakeCompass::getNumStates() const
{
	return states.size();
}
