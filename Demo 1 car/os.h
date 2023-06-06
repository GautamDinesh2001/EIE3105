void init(void), wait(unsigned), toggleA0(void), setLEDs(char);
void pwmServo(unsigned);
char getPath(void);
unsigned getVoltage(void), getCount(void), getDistance(unsigned);
unsigned wait_count(unsigned start, unsigned), wait_count(unsigned);

//the extra functions needed
void loop_rx_rmt(void);
void on_buttons(char);
void on_x(unsigned);
void on_y(unsigned);
void sr04(unsigned);
unsigned getDistance(void);

void pwmLeft(unsigned);
void pwmRight(unsigned);
void dirLeft(bool);
void dirRight(bool);

bool uart3_print(const char*);