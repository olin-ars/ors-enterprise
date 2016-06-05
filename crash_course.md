# How to make the boat go

## ssh into boat
- cd to `ors-git-repo/shell_scripts`
- connect to bat with `./connect_to_fitpc.sh`

## start code
- use `roslaunch fit_pc_pkg auto_code.launch`
  - use this for either auto or rc things (it runs the most up to date code)
- for simulator: `roslaunch fit_pc_pkg simulator_auto.launch`

## launch webpage thing
- cd to `ors-git-repo/fit_pc_pkg/scritps/html-gui`
  - alternately, from anywhere `roscd fit_pc_pg/scripts/html-gui`
- use a web browser to open `index.html`
  - (in my case `chromium-web-browser index.html`)
- there is a prompt, the default number is correct for connecting to the boat
  - for running simulator, change that to `localhost`

## change mode
- `rostopic pub /opperating_mode std_msgs/Int16 "data: #"
  - key:
    - 0: testing
    - 1: RC
    - 2: auto
    - 3: semi-auto (autonomous sails, rc rudder)

## set waypoint

### waypoint using lat-lon
- ```
rostopic pub /raw_waypoints geometry_msgs/Pose2D "x: lat
y: lon
theta: 1"
'''
- ps. use tab-complete

### waypoint using local coordinates
- '''
rostopic pub /local_waypoints geometry_msgs/Pose2D "x: east
y: north
theta: 1"
'''
- ps. use tab-grom	
### other waypoint things
- publish true to `/clear_waypoints` to clear waypoints
- publish true to `/rm_waypoint` to remove the most recently added waypoint
- publish true to `/skip_waypoint` to skip the next waypoint set for the boat