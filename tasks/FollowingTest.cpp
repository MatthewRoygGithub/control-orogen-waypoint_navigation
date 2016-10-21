/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FollowingTest.hpp"

using namespace waypoint_navigation;

FollowingTest::FollowingTest(std::string const& name)
    : FollowingTestBase(name), mStartPose(), mStartPoseReceived(false), 
	mTimeStart()
{
}

FollowingTest::FollowingTest(std::string const& name, RTT::ExecutionEngine* engine)
    : FollowingTestBase(name, engine), mStartPose(), mStartPoseReceived(false), 
	mTimeStart()
{
}

FollowingTest::~FollowingTest()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FollowingTest.hpp for more detailed
// documentation about them.

bool FollowingTest::configureHook()
{
    if (! FollowingTestBase::configureHook())
        return false;
	mStartPose = _start_pose.value();
	mStartPoseReceived = true;
	mCurrentPose = mStartPose;
        std::cout << "Config of robot pose done!" << std::endl;
	std::cout << "Robot = (" << mCurrentPose.position.x() <<","
                << mCurrentPose.position.y() <<","
                << mCurrentPose.position.z() <<"), "
                << "yaw = "<<  mCurrentPose.getYaw()*180/M_PI << " deg."
                << std::endl << std::endl;
    return true;
}
bool FollowingTest::startHook()
{
    if (! FollowingTestBase::startHook())
        return false;
    _robot_pose.write(mCurrentPose);	
    return true;
}

void FollowingTest::updateHook()
{
    FollowingTestBase::updateHook();
	// MOTION SIMULATION HERE -------------------------------------
	
    if(!mStartPoseReceived) {
        return;
    }
    
    base::commands::Motion2D mc;
    if(_motion_command.readNewest(mc) == RTT::NewData) { 
        base::Time time_now = base::Time::now();
        double time_sec = (time_now - mTimeStart).toSeconds();
        
        // Avoids the big initial cap.
        if(time_sec > 2) {
            mTimeStart = base::Time::now();
            time_sec = 0;
        }
         
        mTimeStart = time_now;


	double dt = time_sec;;
        double yaw = mCurrentPose.getYaw();
        
	Eigen::AngleAxisd toWCF, robotRot;
        toWCF = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());


      if ( fabs(mc.translation) < 0.000001 ){
        // Point turn
        //std::cout << "PT of " << (mc.rotation*dt)*180/M_PI << "deg" << std::endl;
        robotRot = Eigen::AngleAxisd(mc.rotation*dt, Eigen::Vector3d::UnitZ());
        mCurrentPose.orientation = Eigen::Quaterniond(robotRot) * mCurrentPose.orientation;
      } else if ( fabs(mc.rotation) < 0.000001){
        // Straight line
        //std::cout << "SL" << std::endl;
        mCurrentPose.position += (mc.translation*dt)*(toWCF*Eigen::Vector3d::UnitX());
      } else {
        // Ackermann
        //  std::cout << "ACK" << std::endl;
        Eigen::Vector3d turnCenter;
        turnCenter << 0.0, mc.translation/mc.rotation, 0.0;
        turnCenter = toWCF*(turnCenter) + mCurrentPose.position;
        robotRot = Eigen::AngleAxisd(mc.rotation*dt, Eigen::Vector3d::UnitZ());
        mCurrentPose.position    = robotRot*(mCurrentPose.position - turnCenter) + turnCenter;
        mCurrentPose.orientation = Eigen::Quaterniond(robotRot)*mCurrentPose.orientation;
      }

      	std::cout << "Robot = (" << mCurrentPose.position.x() <<","
                << mCurrentPose.position.y() <<","
                << mCurrentPose.position.z() <<"), "
                << "yaw = "<<  mCurrentPose.getYaw()*180/M_PI << " deg."
                << std::endl << std::endl;
           
        _robot_pose.write(mCurrentPose);
	}
	//-------------------------------------------------------------

}
void FollowingTest::errorHook()
{
    FollowingTestBase::errorHook();
}
void FollowingTest::stopHook()
{
    FollowingTestBase::stopHook();
}
void FollowingTest::cleanupHook()
{
    FollowingTestBase::cleanupHook();
}
