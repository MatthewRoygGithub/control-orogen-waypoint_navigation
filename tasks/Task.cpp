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
		ptConfig.lookaheadDistance,
        ptConfig.backwards));
    positionValid = false;
	trajectory.clear();
	std::cout << "Path Tracker configured using " <<
    pathTracker->getLookaheadDistance() << "m lookahead distance." << std::endl;

  return true;
}

void Task::updateHook()
{
    // -------------------  TRAJECTORY SETTING   ---------------
    if(_trajectory.readNewest(trajectory) == RTT::NewData ) { // Trajectory input contains new data
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
        std::cout << "Task::updateHook(), Trajectory set to path tracker." 
                  << std::endl;
    }

    // -------------------   NEW POSE READING   ----------------
    base::samples::RigidBodyState pose;
    if (_pose.readNewest(pose) != RTT::NoData ){
        positionValid = pathTracker->setPose(pose);
    }
    
    std::cout << "Task::updateHook(), with xr = (" <<
        pose.position(0) << ", "<< 
        pose.position(1) << ")" << std::endl;
    // Create zero motion command
    base::commands::Motion2D mc;
    mc.translation = 0; mc.rotation = 0;

    // -------------------   MOTION UPDATE      ----------------    
    if (!trajectory.empty() && positionValid ){
        // If position data are valid, calculate the motion command
        pathTracker->update(mc);
        // Write lookahead point to the output (only if there is the trajectory)
        _currentWaypoint.write(*(pathTracker->getLookaheadPoint()));
    }
    // Write motion command to the ouput
    _motion_command.write(mc);

    //-------------- State Update from the library to the component
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
                mc.translation = 0.00001; 
                _motion_command.write(mc);
                 break;
            }
            case waypoint_navigation_lib::OUT_OF_BOUNDARIES:{
                state(OUT_OF_BOUNDARIES);
                break;
            }
            case waypoint_navigation_lib::NO_TRAJECTORY:{
                state(NO_TRAJECTORY);
                break;
            }
            default:{
                break;
            }
        }

}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete pathTracker;
}

/*
void Task::preprocessPath(std::vector<base::Waypoint*>& waypoints){
	// Add all waypoints to the path
	if ( !downsampleGridBasedPath){
		// Iterate through trajectory received: [1st to Nth] and add each point
	    for (std::vector<base::Waypoint>::const_iterator it = trajectory.begin();
	            it != trajectory.end(); ++it) 
	        {
	        	waypoints.push_back(new base::Waypoint(*it)); 
	        }
	} else {
		// Downsampling requested/ 
		int dxPrev, dyPrev, dxNext, dyNext;
		int unitScale = 100; // to cm
		base::Vector3d currWp, nextWp;
		base::Waypoint* pWaypointToAdd;

		// Initialize //
		currWp = trajectory.at(0).position;
		nextWp = trajectory.at(1).position;
		dxPrev = round(unitScale*(nextWp.x() - currWp.x()));
		dyPrev = round(unitScale*(nextWp.y() - currWp.y()));
		pWaypointToAdd = &trajectory.at(0);
		waypoints.push_back(pWaypointToAdd); 

		// Iterate //
		for (size_t it = 1; it < trajectory.size() - 1 ; it++) 
	    {
        	// Shift the waypoints being examined //
        	currWp = nextWp;
        	nextWp = trajectory.at(it+1).position;

        	// Get the [dx, dy] vector to the next //
        	dxNext = round(unitScale*(nextWp.x() - currWp.x()));
			dyNext = round(unitScale*(nextWp.y() - currWp.y()));

			// Is the waypoint significant or lies on a line defined by the prev and next?
			if (dxPrev != dxNext || dyPrev != dyNext){
				// Significant, will be added
				pWaypointToAdd = &trajectory.at(it);
				waypoints.push_back(pWaypointToAdd); 
				// Otherwise does not have to be on the path.
			}
			// Shift the calculated [dx, dy] to be used in next iteration //
			dxPrev = dxNext;
			dyPrev = dyNext;
	    }
	    // Terminate //
	    pWaypointToAdd = &trajectory.back();
		waypoints.push_back(pWaypointToAdd);
	}
}
*/
