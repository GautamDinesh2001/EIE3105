enum wheels_action { STOP, FORWARD, BACKWARD, LEFT, RIGHT };

void wheels(wheels_action, unsigned speed);

/*
  if speed is greater that 10, it becomes direct PWM values.
*/