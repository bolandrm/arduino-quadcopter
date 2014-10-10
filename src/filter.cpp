// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#include "Filter.h"

Filter::Filter(){
  inv_gain = 1 / GAIN;
}

float Filter::update(float input) {
  xv[0] = xv[1];
  xv[1] = xv[2];
  xv[2] = xv[3];
  xv[3] = xv[4];
  xv[4] = xv[5];
  xv[5] = xv[6];
  xv[6] = xv[7];
  xv[7] = xv[8];

  xv[8] = input * inv_gain;

  yv[0] = yv[1];
  yv[1] = yv[2];
  yv[2] = yv[3];
  yv[3] = yv[4];
  yv[4] = yv[5];
  yv[5] = yv[6];
  yv[6] = yv[7];
  yv[7] = yv[8];

  yv[8] = (xv[0] + xv[8])
          + 8 * (xv[1] + xv[7])
          + 28 * (xv[2] + xv[6])
          + 56 * (xv[3] + xv[5])
          + 70 * xv[4]
          + ( -0.1817387267 * yv[0])
          + (  1.7618254533 * yv[1])
          + ( -7.5108024552 * yv[2])
          + ( 18.3974693970 * yv[3])
          + (-28.3318491710 * yv[4])
          + ( 28.1024051670 * yv[5])
          + (-17.5430988560 * yv[6])
          + (  6.3057235769 * yv[7]);

  return  yv[8];
}
