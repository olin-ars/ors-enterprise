# Namespaces and Topics
### where they go, what they do
note: none of this is implemented yet

#### Each teensy has it's own namespace:
* `rc/` for the teensy recieveing RC signals
* `rudder/` for teensy controlling the rudder motor
* `sails/` for teensy controlling the sail motor

#### RC topics:
* `rc/rudder_in` RC input for rudder position
* `rc/sails_in` RC input for sail position
* `rc/switch_in` RC input for switch on controller
* `rc/debug_dial_in` value of potentiometer connected to rc teensy

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
