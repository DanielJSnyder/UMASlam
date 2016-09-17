#ifndef __SLAM_UTILITES_HPP__
#define __SLAM_UTILITES_HPP__

#include <string>
#include <iostream>

/*
	writes msg to screen if the level is less than or equal to the debug level set
*/
template <typename Printable>
void logDebugMsg(const Printable & msg, int level)
{
	#ifdef SLAM_DEBUG_LEVEL
	if(level <= SLAM_DEBUG_LEVEL)
	{
		std::cerr << msg << std::endl;
	}
	#endif
}

#endif
