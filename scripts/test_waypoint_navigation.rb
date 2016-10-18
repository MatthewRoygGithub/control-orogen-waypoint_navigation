require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run  'waypoint_navigation::Task' => 'pathTracker' do
        
    pathTracker = TaskContext::get 'pathTracker'
    # load property from configuration file
    pathTracker.apply_conf_file("config/waypoint_navigation::Task.yml", 
        ["exoter"])
    pathTracker.configure
    pathTracker.start

    Readline::readline("Hit ENTER to stop")    
end
