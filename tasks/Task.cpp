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

    Config configuration = _tracker_config.get();
    pathTracker = new WaypointNavigation(configuration);
    trajectory.clear();

    uint N = 3;
    trajectory.resize(N);
    std::vector<base::Waypoint*> ptrajectory(N);
    for (size_t i = 0; i < N; i++)
    {
        trajectory.at(i).position = Eigen::Vector3d(i+1.0,6,0);
        trajectory.at(i).heading  = 0.0/180.0*M_PI;
        trajectory.at(i).tol_position = 0.1;
        ptrajectory.at(i) = &trajectory.at(i);
    }
    std::cout << "Trajectory created" << std::endl;
    pathTracker->setTrajectory(ptrajectory);
    std::cout << "Trajectory set" << std::endl;
    return true;

}

void Task::updateHook()
{
    if(_trajectory.readNewest(trajectory) != RTT::NoData) { // Trajectory input contains new data
        //convert to driver format
        std::cerr << "DTF: got " << trajectory.size() << " points in trajectory" << std::endl;

        std::vector<base::Waypoint*> waypoints;
        for (std::vector<base::Waypoint>::const_iterator it = trajectory.begin();
                it != trajectory.end(); ++it) // Iterate through trajectory received: [1st to Nth].
        {
            waypoints.push_back(new base::Waypoint(*it)); // Add element to the end of the vector
        }
        pathTracker->setTrajectory(waypoints);
    }

     // Set pose if the trajectory is not empty and Pose contains data
    base::samples::RigidBodyState pose;
    if (!trajectory.empty() && _pose.readNewest(pose) != RTT::NoData)
    {
	     pathTracker->setPose(pose);

       // Get motion command
       base::commands::Motion2D mc;
       pathTracker->update(mc);

      // Propagate the Path Tracker state from the library to the component
      NavigationState curentState = pathTracker->getNavigationState();
      switch(curentState) {
          case DRIVING:{
            state(DRIVING);
            break;
          }
          case ALIGNING:{
            state(ALIGNING);
            break;
          }
          case TARGET_REACHED:{
            state(TARGET_REACHED);
            break;
          }
          case OUT_OF_BOUNDARIES:{
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
