void init(void), wait(unsigned), toggleA0(void), setLEDs(char);
void pwmServo(unsigned);
char getPath(void);
unsigned getVoltage(void), getCount(void);
unsigned wait_count(unsigned start, unsigned), wait_count(unsigned);

//Some functions used
unsigned getDistance(void);
void pwms(unsigned left_cnt, unsigned right_cnt, char path);


void pwmLeft(unsigned i);
void pwmRight(unsigned i);
void dirRight(bool b);
void dirLeft(bool b);


void loop_remote(void);
int getSpeed(void);
int getAngularSpeed(void);

//USART
bool uart3_print(const char* p);
bool uart2_print(const char* p);


//SPI
char getSpiVal (void);
void LED8(unsigned char ch);

//motor control
int getRight();
int getLeft();

void Delay (int);
void __attribute__((weak)) on_button(bool);
void controller(void);
