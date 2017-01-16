/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for simple simulation (only integration of 
 * velocity commands) of robot motions.
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

#ifndef WAYPOINT_NAVIGATION_FOLLOWINGTEST_TASK_HPP
#define WAYPOINT_NAVIGATION_FOLLOWINGTEST_TASK_HPP

#include "waypoint_navigation/FollowingTestBase.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <base/Timeout.hpp>

namespace waypoint_navigation {

    
    class FollowingTest : public FollowingTestBase
    {
	friend class FollowingTestBase;
    protected:
        base::samples::RigidBodyState mStartPose;
        base::samples::RigidBodyState mCurrentPose;
        bool mStartPoseReceived;
        base::Time mTimeStart;
        base::commands::Motion2D mc;


    public:
        FollowingTest(std::string const& name = "waypoint_navigation::FollowingTest");
        FollowingTest(std::string const& name, RTT::ExecutionEngine* engine);

	   ~FollowingTest();

        
        bool configureHook();

        
        bool startHook();

        
        void updateHook();

        
        void errorHook();

        
        void stopHook();

        void cleanupHook();
    };
}

#endif

