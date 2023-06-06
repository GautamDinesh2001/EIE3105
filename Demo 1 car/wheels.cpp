#include "wheels.h"

void pwmLeft(unsigned), pwmRight(unsigned), dirLeft(bool), dirRight(bool);

namespace { //anonymous*************************************

class Speed {
public:
  Speed(void) { last = speed8 = 0; }
  unsigned speed(unsigned cnt) {      // cnt = new counter value
    unsigned u = cnt - last;          // change in 2.5 ms
    last = cnt;
    unsigned s, s8 = speed8;
    for (s = u; s8; s8 >>= 4) s += s8 & 15;
    speed8 <<= 4;                     // shift out old data
    speed8 |= u;                      // save new data
    return s;
  }
private:
  unsigned speed8, last;
} left, right;

class Pid {
public:
  Pid(unsigned p, unsigned i, unsigned d):p(p),i(i),d(d) { reset(); }
  int control(int error) {
    int delta = error - last;
    last = error;
    acc += error;
    if (acc > MAX) acc = MAX;
    if (acc < MIN) acc = MIN;
    return error * p + acc * i + delta * d;
  }
  void reset(void) { acc = last = 0; }
private:
  enum { MAX=2123456789, MIN=-2123456789 };
  unsigned p, i, d;
  int acc, last;
} speed(22345, 100, 1000), steer(12345, 200, 1000);

unsigned speed_set;

} //anonymous***********************************************


void pwms(unsigned left_cnt, unsigned right_cnt, char path) {
  int lt = left.speed(left_cnt);
  int rt = right.speed(right_cnt);
  if (speed_set < 100) {
    if (speed_set) {
      int sp = speed.control(speed_set - lt - rt);
      int st = steer.control(lt - rt);
      lt = sp - st; rt = sp + st;
      if (lt < 0) lt = 0;
      if (rt < 0) rt = 0;
      if (lt > 44999) lt = 44999;
      if (rt > 44999) rt = 44999;
    } else lt = rt = 0;
  } else lt = rt = speed_set;
  pwmLeft(lt); pwmRight(rt);
}

void wheels(wheels_action a, unsigned s) {
  speed_set = s;
  switch (a) {
    case FORWARD: dirLeft(true); dirRight(true); break;
    case BACKWARD: dirLeft(false); dirRight(false); break;
    case LEFT: dirLeft(false); dirRight(true); break;
    case RIGHT: dirLeft(true); dirRight(false); break;
    default: speed_set = 0;
  }
  speed.reset(); steer.reset();
}
