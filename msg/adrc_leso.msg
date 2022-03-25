# ADRC LESO states message
uint64 timestamp          # time since system start (microseconds)

float32 r		  # Reference setpoint
float32 y		  # Measurement input
float32 u		  # Control input

float32 z1                # State 1 of LESO
float32 z2                # State 2 of LESO
float32 z3                # State 3 of LESO (disturbance)

bool ctrl_en		  # Denotes whether controller is enabled or not

float32 b                 # Feedback gain
float32 bwc	          # Controller bandwidth
float32 bwo		  # Observer bandwidth
# TOPICS adrc_leso adrc_leso_1
