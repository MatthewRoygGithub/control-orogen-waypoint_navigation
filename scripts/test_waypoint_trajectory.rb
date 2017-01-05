require 'orocos'
require 'vizkit'
require 'readline'
require 'rock/bundle'

include Orocos
Orocos::CORBA.max_message_size = 120000000
Bundles.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Bundles.run 'waypoint_navigation::Task'          => 'pathTracker',
            'waypoint_navigation::FollowingTest' => 'move',
            'waypoint_navigation::TrajectoryTest' => 'trajectoryGen',
            "valgrind" => false,
            'output' => nil, 
            "wait" => 60 					do

    pathTracker   = TaskContext::get 'pathTracker'
    move          = TaskContext::get 'move'
    trajectoryGen = TaskContext::get 'trajectoryGen'

    # load property from configuration file
    pathTracker.apply_conf_file("config/waypoint_navigation::Task.yml",   ["default"])
    move.apply_conf_file("config/waypoint_navigation::FollowingTest.yml", ["default"])
    trajectoryGen.apply_conf_file("config/waypoint_navigation::TrajectoryTest.yml", ["testing"])

    pathTracker.configure
    trajectoryGen.configure
    move.configure

    trajectoryGen.trajectory.connect_to         pathTracker.trajectory, :type => :buffer, :size => 1
    pathTracker.motion_command.connect_to       move.motion_command,    :type => :buffer, :size => 5
    move.robot_pose.connect_to                  pathTracker.pose,       :type => :buffer, :size => 5

    pathTracker.start
    move.start  
    trajectoryGen.start

    view3d = Vizkit.vizkit3d_widget
    view3d.show
 
    Vizkit.display move.robot_pose, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display move.robot_pose, :widget => Vizkit.default_loader.TrajectoryVisualization
    Vizkit.display pathTracker.currentWaypoint,  :widget => Vizkit.default_loader.WaypointVisualization
    Vizkit.display trajectoryGen.trajectory,     :widget => Vizkit.default_loader.WaypointVisualization
    Vizkit.display trajectoryGen.trajectory 

#   Readline::readline("Hit ENTER to start")    
    
    trajectoryGen.trigger 
 
    Vizkit.exec    # Busy waiting
    Readline::readline("Hit ENTER to stop")    

end
