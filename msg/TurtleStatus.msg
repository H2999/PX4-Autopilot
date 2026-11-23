# Experimental ground recovery, not ordinary armed flight. Command delivery is not ESC confirmation.
uint64 timestamp
uint8 state       # turtle::State: idle,stop-reverse,reverse,drive,stop-normal,normal,coast,wait-exit
uint8 reason      # turtle::Reason: none,disabled,inputs,geometry,not-inverted,complete,timeout,cancelled,too-fast,outside-recovery-range
bool active       # Driver owns outputs, including direction restoration
bool ready_for_arming # Mode exited, direction command sequence completed, valid geometry
uint32 motor_mask # Selected PHYSICAL DShot output bits, not logical motor indices
float32 tilt      # Angle from upright [rad]
float32 power     # Normalised recovery throttle, not measured thrust
bool levelling    # Upright levelling: saved normal direction, physically lower pair
