/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class encapsulating the pure-pursuit based path 
 * follower, which was implemented for the six-wheeled ExoTeR rover.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Dec 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */


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
    base::commands::Motion2D mc_prev;
    bool positionValid;
	
    public:
        Task(std::string const& name = "WaypointNavigation::Task");
        bool configureHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

    private:
        void stopRover();
    //	void preprocessPath		(std::vector<base::Waypoint*>& 	waypoints);
    };
}

#endif
