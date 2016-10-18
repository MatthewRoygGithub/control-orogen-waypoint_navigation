#ifndef WAYPOINT_NAVIGATION_TYPE_HPP
#define WAYPOINT_NAVIGATION_TYPE_HPP

namespace waypoint_navigation {

    /** Configuration **/
	struct pathTrackerConfig
	{	
		// Paramater							Unit
		//-----------------------------------------------------------------------
		// Minimum radius of robot's Ackermann turn			 (m)
		double minTurnRadius;

		// Robot's translational velocity during Ackermann turns	 (m/s)
		double translationalVelocity;
		
		// Robot's rotational velocity during point turns		 (rad/s)
		double rotationalVelocity;

		// Safety corridor (1/2 width) arround the nominal trajectory	 (m)
		double corridor;		

		// Maximum allowed disalignment from target heading in the final pose (rad)
		double maxDisalignment;	
		
		// Distance to the lookahead point of the Pure Pursuit Algorithm (m)
		double lookaheadDistance;	
    };
}
#endif
