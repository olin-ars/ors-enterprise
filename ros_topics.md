# Namespaces and Topics
### where they go, what they do
note: none of this is implemented yet

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
  * Int32 (pub)
* `rudder/motor_direction` direction of motor
  * Int32 (pub)
* `rudder/set_point` set point for rudder in degrees
  * Int32 (sub)

#### Sail topics:
* `sail/pos` magnet sensor position
  * Int32 (pub)
* `sail/motor_direction` direction of actuator
  * Int32 (pub)
* `sail/set_point` set point for linear actuator
  * Int32 (sub)

### Information Flow

1. The RC teensy gets information from the RC controller and sends that information to the FitPC over rostopics outlined above.
2. A node on the FitPC then determines what set points to send to both the sail and the rudder using the rostopics outlined above (this arbiter node will hold logic for choosing between RC control and various autonomous modes)
* Topics published by the rudder and sail teensys, as well as the debug dial are currently used only for debugging, but may be useful later.
