/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryTest.hpp"

using namespace waypoint_navigation;

TrajectoryTest::TrajectoryTest(std::string const& name)
    : TrajectoryTestBase(name),mSelectedWaypoints(5,0)
{
}

TrajectoryTest::TrajectoryTest(std::string const& name, RTT::ExecutionEngine* engine)
    : TrajectoryTestBase(name, engine),mSelectedWaypoints(5,0)

{
}

TrajectoryTest::~TrajectoryTest()
{
}

bool TrajectoryTest::configureHook()
{
    if (! TrajectoryTestBase::configureHook())
        return false;
    mSelectedWaypoints = _selectedWaypoints.get();
    int N = mSelectedWaypoints.size();
    trajectory.resize(N);

    // Predefined grid of waypoints marekd in the lab environment
    double xpos[] = {2.093, 3.936, 5.602, 7.834, 2.150, 3.906, 5.843, 7.567, 0.705, 4.283, 5.741, 7.483, 1.014, 2.950, 4.300, 5.806, 7.227, 1.208, 2.914, 4.405, 7.051};
    double ypos[] = {7.419, 7.199, 7.409, 7.899, 5.739, 5.781, 5.952, 5.951, 4.224, 4.015, 3.961, 3.926, 2.683, 2.549, 2.388, 2.396, 2.404, 0.978, 1.386, 1.069, 1.132};

    // Now iteratively set the trajectory so that each previous waypoints has a heading towards the next waypoint.
    int it;
    for(it= N-1; it >= 0; it--){
       trajectory.at(it).position = Eigen::Vector3d(
            xpos[mSelectedWaypoints.at(it)-1],ypos[mSelectedWaypoints.at(it)-1],0);

        // Final waypoint has its own handling of heading
        if ( it < N-1 ){
            trajectory.at(it).heading  = atan2(
                 ypos[mSelectedWaypoints.at(it+1)-1]-ypos[mSelectedWaypoints.at(it)-1],
                 xpos[mSelectedWaypoints.at(it+1)-1]-xpos[mSelectedWaypoints.at(it)-1]);
            trajectory.at(it).tol_position = 0.1;
            trajectory.at(it).tol_heading = 5.0/180*M_PI; 
        } else {
            // TODO add those in config file!
            trajectory.at(it).tol_position = 0.1;
            trajectory.at(it).tol_heading  = 3.0/180*M_PI; 
            trajectory.at(it).heading  = 45.0/180*M_PI;
        }
    }
   std::cout << "TrajectoryTest::configureHook(), Trajectory created." << std::endl;
   return true;
}

bool TrajectoryTest::startHook()
{
    if (! TrajectoryTestBase::startHook())
        return false;
    std::cout << "TrajectoryTest::startHook called" << std::cout; 
//     _trajectory.write(trajectory); 
//    std::cout << "Trajectory written out." << std::cout;
    return true;
}
void TrajectoryTest::updateHook()
{
    TrajectoryTestBase::updateHook();
    std::cout << "TrajectoryTest::updateHook called." << std::cout;
    _trajectory.write(trajectory); 
//    std::cout << "Trajectory written out." << std::cout;
   
 }
void TrajectoryTest::errorHook()
{
    TrajectoryTestBase::errorHook();
}
void TrajectoryTest::stopHook()
{
    TrajectoryTestBase::stopHook();
}
void TrajectoryTest::cleanupHook()
{
    TrajectoryTestBase::cleanupHook();
}
