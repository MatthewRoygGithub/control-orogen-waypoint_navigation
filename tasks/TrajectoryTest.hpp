/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef WAYPOINT_NAVIGATION_TRAJECTORYTEST_TASK_HPP
#define WAYPOINT_NAVIGATION_TRAJECTORYTEST_TASK_HPP

#include "waypoint_navigation/TrajectoryTestBase.hpp"

namespace waypoint_navigation {

   class TrajectoryTest : public TrajectoryTestBase
    {
	friend class TrajectoryTestBase;
    protected:
        std::vector<int> mSelectedWaypoints;
        std::vector<base::Waypoint> trajectory; 
        bool waypointsConfigured;
        // Predefined grid of waypoints marekd in the lab environment
        std::vector<double> xpos, ypos;

    public:
        TrajectoryTest(std::string const& name = "waypoint_navigation::TrajectoryTest");

        TrajectoryTest(std::string const& name, RTT::ExecutionEngine* engine);

	~TrajectoryTest();

        bool configureHook();

        bool startHook();

        void updateHook();

        void errorHook();

        void stopHook();

        void cleanupHook();
    };
}

#endif

