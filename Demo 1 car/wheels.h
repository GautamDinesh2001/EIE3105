void pwms(unsigned, unsigned,char); 
enum wheels_action { STOP, FORWARD, BACKWARD, LEFT, RIGHT };

void wheels(wheels_action, unsigned speed);
