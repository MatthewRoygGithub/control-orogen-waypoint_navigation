require 'orocos'
require 'vizkit'
require 'readline'
require 'rock/bundle'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos::CORBA.max_message_size = 120000000
Bundles.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Bundles.run 'waypoint_navigation::Task' => 'pathTracker',
            'waypoint_navigation::FollowingTest' => 'move',
            "valgrind" => false,
            'output' => nil, 
            "wait" => 1000 					do

    pathTracker = TaskContext::get 'pathTracker'
    move = TaskContext::get 'move'

    # load property from configuration file
    pathTracker.apply_conf_file("config/waypoint_navigation::Task.yml", 
        ["exoter"])
    move.apply_conf_file("config/waypoint_navigation::FollowingTest.yml", 
        ["default"])

    pathTracker.configure
    move.configure

    pathTracker.motion_command.connect_to move.motion_command, :type => :buffer, :size => 10
    move.robot_pose.connect_to pathTracker.pose, :type => :buffer, :size => 10
    
   # Vizkit.display planner.trajectory
    # Vizkit.display move.start_pose, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display move.robot_pose, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display move.robot_pose, :widget => Vizkit.default_loader.TrajectoryVisualization
    Vizkit.display pathTracker.currentWaypoint, :widget => Vizkit.default_loader.WaypointVisualization
    
     Readline::readline("Hit ENTER to start")    
    
    pathTracker.start
    move.start
    Vizkit.exec    # Busy waiting

 

    Readline::readline("Hit ENTER to stop")    
end
