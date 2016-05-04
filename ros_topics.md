# Namespaces and Topics
### where they go, what they do

#### General boat topics:
* `location` The boat's location relative to startup, in meters with North at +y and
  * Pose2D (pub from simulator for now) (x and y orthogonal) (angle Range 0-359.999)
* `velocity` The boat's smoothed current velocity, relative to location
  * Pose2D (pub from simulator for now) (x is speed, in m/s; theta is angle, in deg CW from North)
* `true_wind` The motion of the wind relative to the fixed local coordinate system, smoothed.
  * Pose2D (pub from simulator for now) (x is speed, in m/s; theta is angle, in deg CW from boat heading)
* `relative_wind` The motion of the wind relative to the boat, smoothed.
  * Pose2D (pub from simulator for now) (x is speed, in m/s; theta is angle, in deg CW from boat heading)

#### Arbiter:
* `operating_mode` Controls boat's operating mode
  * Int16 (sub)
    * DEFAULT = 0 : Unimplemented
    * RC_MODE = 1 : Implemented by `RC-control-master.py`
    * AUTO_MODE = 2 : Unimplemented
    * TEST_MODE = 3 : Implemented by `publish-test-rudderCommands.py`

##### Each operating mode gets its own namespace
* `rc_mode/`
 * `rc_mode/rudder/set_point` This is forwarded to `rudder/set_point` if RC mode is active
 * `rc_mode/sail/set_point` etc...
* `test_mode/`
 * `rc_mode/rudder/set_point`
 * `rc_mode/sail/set_point` etc...
* `auto_mode/`
 * `auto_mode/rudder/set_point`
 * `auto_mode/sail/set_point`

#### Each teensy has it's own namespace:
* `rc/` for the teensy recieveing RC signals
* `rudder/` for teensy controlling the rudder motor
* `sails/` for teensy controlling the sail motor

#### RC topics:
* `rc/rudder_in` RC input for rudder position
 * Float32 (pub)
* `rc/sails_in` RC input for sail position
 * Float32 (pub)
* `rc/switch_in` RC input for switch on controller
 * Bool (pub)
* `rc/debug_dial_in` value of potentiometer connected to rc teensy
 * Float32 (pub)

#### Rudder topics:
* `rudder/pos` encoder/pot position in degrees
  * Int16 (pub)
* `rudder/motor_direction` direction of motor
  * Int16 (pub)
* `rudder/set_point` set point for rudder in degrees
  * Int16 (sub)

#### Sail topics:
* `sail/pos` magnet sensor position
  * Float32 (pub)
* `sail/motor_direction` direction of actuator
  * Int16 (pub)
* `sail/set_point` set point for linear actuator
  * Float32 (sub)

### Information Flow

1. The RC teensy gets information from the RC controller and sends that information to the FitPC over rostopics outlined above.
2. A node on the FitPC then determines what set points to send to both the sail and the rudder using the rostopics outlined above (this arbiter node will hold logic for choosing between RC control and various autonomous modes)
* Topics published by the rudder and sail teensys, as well as the debug dial are currently used only for debugging, but may be useful later.
