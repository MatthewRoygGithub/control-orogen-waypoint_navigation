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
    /*
1,2093.531,7419.216
2,3936.113,7199.626
3,5602.180,7409.183
4,7834.344,7899.822
5,2150.851,5739.723
6,3906.638,5781.423
7,5843.060,5952.044
8,7567.428,5951.296
9,1305.340,4224.239
10,4283.850,4015.380
11,5741.045,3961.448
12,7483.661,3926.435
13,1014.269,2683.210
14,2950.443,2549.871
15,4300.017,2388.352
16,5806.631,2396.875
17,7227.912,2404.997
18,1208.888,978.873
19,2914.739,1386.479
20,4405.077,1069.489
21,7051.609,1132.802

*/
        // Trajectory loop
	//double xpos[] = {2.093,	 3.936,	 5.602,	 7.567,	7.483,	5.806, 4.405, 2.914, 1.208, 1.014, 1.305, 2.150};
	//double ypos[] = {7.419,	 7.199,	 7.409,	 5.951,	3.926,	2.396, 1.069, 1.386, 0.978, 2.683, 4.224, 5.739};
        // Trajectory 1
        //double xpos[] = {5.806, 4.405, 2.914, 1.208, 1.014, 1.305, 2.150};
	//double ypos[] = {2.396, 1.069, 1.386, 0.978, 2.683, 4.224, 5.739};
        // Redone using indexing
        double xpos[] = {2.093, 3.936, 5.602, 7.834, 2.150, 3.906, 5.843, 7.567, 1.305, 4.283, 5.741, 7.483, 1.014, 2.950, 4.300, 5.806, 7.227, 1.208, 2.914, 4.405, 7.051};
        double ypos[] = {7.419, 7.199, 7.409, 7.899, 5.739, 5.781, 5.952, 5.951, 4.224, 4.015, 3.961, 3.926, 2.683, 2.549, 2.388, 2.396, 2.404, 0.978, 1.386, 1.069, 1.132};
        
        // Big loop
        //uint selectWaypoints[] = {1, 2, 3, 8, 12, 16, 20, 19, 19, 13, 9, 5};
        //uint N = 12;
        // Small loop, worked well, point turn once at #13
        //uint selectWaypoints[] = {5, 6, 11, 16, 20, 19, 13, 9, 5};
        //uint N = 9;
        // Force point turns
        uint selectWaypoints[] = {7, 3, 1};
        uint N = 3;


    trajectory.resize(N);
    std::vector<base::Waypoint*> ptrajectory(N);
    for (size_t i = 0; i < N; i++)
    {
        trajectory.at(i).position = Eigen::Vector3d(xpos[ selectWaypoints[i] -1],ypos[ selectWaypoints[i] -1],0);
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
	// Get motion command
       base::commands::Motion2D mc;
       if ( mc.translation != 0 || mc.rotation != 0){
           std::cout << "MC was not zeroed!" << std::endl;
           mc.translation = 0; mc.rotation = 0;
       }
       // If data are valid
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
