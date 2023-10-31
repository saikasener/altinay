# Motor errors
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Motor torque
uint8 TORQUE_ON  = 1
uint8 TORQUE_OFF = 2

########################################
# Messages
########################################
Header  header
uint8   led

int64  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)
int64  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)
#bool   torque
float32  battery
