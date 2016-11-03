#ifndef WAYPOINT_NAVIGATION_TASK_TASK_HPP
#define WAYPOINT_NAVIGATION_TASK_TASK_HPP

#include "waypoint_navigation/TaskBase.hpp"
#include <base/waypoint.h>

namespace waypoint_navigation_lib{
	class WaypointNavigation;
}

namespace waypoint_navigation {
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	///Instance of the dumbTrajectoryFollower driver
	waypoint_navigation_lib::WaypointNavigation *pathTracker;
	///Trajectory in the format of the driver
	std::vector<base::Waypoint> trajectory;
	
    public:
        Task(std::string const& name = "WaypointNavigation::Task");

        bool configureHook();

        void updateHook();

        void cleanupHook();

    private:
    //	void preprocessPath		(std::vector<base::Waypoint*>& 	waypoints);
    };
}

#endif
