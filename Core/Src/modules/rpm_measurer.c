#include "rpm_measurer.h"
#include "irq.h"
#include <string.h> // for memset

#define MASK_POS_OVERFLOW 0x1
#define MASK_NEG_OVERFLOW 0x2

void rpm_measurer_init(struct speed_measure_t * s)
{
    memset(s, 0, sizeof(*s));
    // by default speed_coef forces to receive the pure difference in position
    s->speed_coef = 1;
}

void rpm_measurer_set_speed_coef(struct speed_measure_t * rpm, float speed_coef)
{
  rpm->speed_coef = speed_coef;
}

static void _position_changed(struct speed_measure_t * rpm, uint16_t cur_pos)
{
  int32_t pos = cur_pos + 
  ((rpm->overlow & MASK_POS_OVERFLOW) ? MAX_WHEEL_POS : 0) +
  ((rpm->overlow & MASK_NEG_OVERFLOW) ? -MAX_WHEEL_POS : 0);
  rpm->speed = (pos - rpm->pos) * rpm->speed_coef;
  rpm->pos = cur_pos;
  rpm->overlow = 0;
}

void position_changed(struct speed_measure_t * rpm, uint16_t cur_pos)
{
  wrp_disable_irq();
  _position_changed(rpm, cur_pos);
  wrp_enable_irq();
}

static void _rpm_measurer_set_overflow_mask(struct speed_measure_t * s, uint8_t mask)
{
  s->overlow |= mask;
}

void position_changed_detect_overflow(struct speed_measure_t * rpm, uint16_t cur_pos)
{
  const uint16_t half = MAX_WHEEL_POS / 2;
  wrp_disable_irq();
  // try to detect overflow/underflow
  if (rpm->speed * rpm->speed_coef > 0 && rpm->pos > half && cur_pos < half)
  {
    _rpm_measurer_set_overflow_mask(rpm, MASK_POS_OVERFLOW);
  }
  else if (rpm->speed * rpm->speed_coef <= 0 && rpm->pos < half && cur_pos > half)
  {
    _rpm_measurer_set_overflow_mask(rpm, MASK_NEG_OVERFLOW);
  }
  // calculate the position change
  _position_changed(rpm, cur_pos);
  wrp_enable_irq();
}

static void rpm_measurer_set_overflow_mask(struct speed_measure_t * s, uint8_t mask)
{
  wrp_disable_irq();
  _rpm_measurer_set_overflow_mask(s, mask);
  wrp_enable_irq();
}

void rpm_measurer_positive_overflow(struct speed_measure_t * s)
{
  rpm_measurer_set_overflow_mask(s, MASK_POS_OVERFLOW);
}

void rpm_measurer_negative_overflow(struct speed_measure_t * s)
{
  rpm_measurer_set_overflow_mask(s, MASK_NEG_OVERFLOW);
}

float current_speed(struct speed_measure_t * rpm)
{
  return rpm->speed;
}
