#include "Task.hpp"
#include <WaypointNavigation.hpp>

using namespace waypoint_navigation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    pathTracker = NULL;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
	if (! TaskBase::configureHook())
      		return false;
	pathTrackerConfig ptConfig = _ptConfig.value();
	pathTracker = new waypoint_navigation_lib::WaypointNavigation();
        assert(	pathTracker->configure(
		ptConfig.minTurnRadius,
		ptConfig.translationalVelocity,
		ptConfig.rotationalVelocity,
		ptConfig.corridor,
		ptConfig.lookaheadDistance));
	trajectory.clear();
        std::cout << "Path Tracker configured using " <<
        pathTracker->getLookaheadDistance() << "m lookahead distance." << std::endl;

  return true;
}

void Task::updateHook()
{
    std::cout << "Task::updateHook() called." << std::endl;
    int rtt_return = _trajectory.readNewest(trajectory);  
    if(rtt_return  == RTT::NewData ) { // Trajectory input contains new data
        //convert to driver format
        std::cout << "Task::updateHook(), Task has  "
                  << trajectory.size() << " points in trajectory." << std::endl;

        // Pass the waypoints to the library using pointers
        std::vector<base::Waypoint*> waypoints;
        for (std::vector<base::Waypoint>::const_iterator it = trajectory.begin();
                it != trajectory.end(); ++it) // Iterate through trajectory received: [1st to Nth].
        {
            waypoints.push_back(new base::Waypoint(*it)); // Add element to the end of the vector
        }
        pathTracker->setTrajectory(waypoints);
        std::cout << "Task::updateHook(), Trajectory set to path tracker." << std::endl;
    }

     // Set pose if the trajectory is not empty and Pose contains data
    base::samples::RigidBodyState pose;
    if (!trajectory.empty() && _pose.readNewest(pose) != RTT::NoData)
    {
       // Create zero motion command
       base::commands::Motion2D mc;
       mc.translation = 0; mc.rotation = 0;

       std::cout << "Task::updateHook(), with xr = (" <<
                pose.position(0) << ", "<< 
                pose.position(1) << ")" << std::endl;
      // If data are valid, calculate the motion command
       if (pathTracker->setPose(pose)){
           pathTracker->update(mc);
       }

      // Propagate the Path Tracker state from the library to the component
      waypoint_navigation_lib::NavigationState curentState = pathTracker->getNavigationState();
      switch(curentState) {
          case waypoint_navigation_lib::DRIVING:{
            state(DRIVING);
            break;
          }
          case waypoint_navigation_lib::ALIGNING:{
            state(ALIGNING);
            break;
          }
          case waypoint_navigation_lib::TARGET_REACHED:{
            state(TARGET_REACHED);
            break;
          }
          case waypoint_navigation_lib::OUT_OF_BOUNDARIES:{
            state(OUT_OF_BOUNDARIES);
            break;
          }
          default:{
            break;
          }
      }
      // WRITE TO OUTPUTS
      _currentWaypoint.write(*(pathTracker->getLookaheadPoint()));
      _motion_command.write(mc);
  }
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete pathTracker;
}
