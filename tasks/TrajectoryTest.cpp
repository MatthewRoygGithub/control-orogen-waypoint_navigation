/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for generation of test path (list of Waypoints)
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

    xpos = _xpos.get();
    ypos = _ypos.get();

	// Sizes of available waypoint lists are consistent
    if ( xpos.size() != ypos.size()){
    	std::cerr << "\nError in TrajectoryTest::configureHook()\n" 
    			<< "xpos size ("<< xpos.size() << ") does not match ypos ("<< ypos.size() << ") size!\n" 
    			<< "Check config/waypoint_navigation::TrajectoryTest.yml.\n"
    			<< std::endl;
    	return false;
    }
    std::vector<int>::iterator result;

    // Selected waypoints indexes are valid:
    result = std::max_element(mSelectedWaypoints.begin(), mSelectedWaypoints.end());
    if ( *result > xpos.size() ){
    	std::cerr << "\nError in TrajectoryTest::configureHook()\n"
    			<< "Index: " << *result << " exceeds the number of available waypoints (" << xpos.size() << ").\n"
    			<< "Check selectedWaypoints in config/waypoint_navigation::TrajectoryTest.yml.\n" 
    			<< std::endl;
    	return false;
    }
    result = std::min_element(mSelectedWaypoints.begin(), mSelectedWaypoints.end());
    if ( *result < 1 ){
    	std::cerr << "\nError in TrajectoryTest::configureHook()\n" 
    			<< "Index: " << *result << " is under the limit, must be >0.\n"
    			<< "Check selectedWaypoints in config/waypoint_navigation::TrajectoryTest.yml.\n" 
    			<< std::endl;
    	return false;
    }

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
            trajectory.at(it).tol_heading  = 3.0/180.0*M_PI; 
            trajectory.at(it).heading      = (_goal_heading.get())/180.0*M_PI;
        }
    }
   std::cout << "TrajectoryTest::configureHook(), Trajectory created." << std::endl;
   return true;
}

bool TrajectoryTest::startHook()
{
    if (! TrajectoryTestBase::startHook())
        return false;
//    std::cout << "TrajectoryTest::startHook called" << std::cout; 
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
