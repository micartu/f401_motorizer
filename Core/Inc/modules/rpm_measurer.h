#ifndef __RPM_MEASURER_H__
#define __RPM_MEASURER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define MAX_WHEEL_POS 65535

struct speed_measure_t
{
	/// current position of the wheel
	int32_t pos;
	/// bit mask which indicates whether a positive
	/// or negative overflow happened
	uint8_t overlow;
	/// measured speed at time <last_measured_time>
	float speed;
	/// speed convertion coefficient
	float speed_coef;
	/// how many over(under)-flows have happened already (negative=under)
	int32_t overflown_counter;
};

/**
 * Initializes measurer unit struct
 * which contains all information needed to calculate the RPM of a wheel
 *
 * @param s structure to be initialized
 */
void rpm_measurer_init(struct speed_measure_t * s);

/**
 * Sets custom speed koefficient
 * it'll be used to multiply position difference in order 
 * to get proper speed units
 *
 * @param rpm structure to be initialized
 * @param speed_koef custom speed koefficient to install
 */
void rpm_measurer_set_speed_coef(struct speed_measure_t * rpm, float speed_coef);

/**
 * @brief returns the current estimated speed of the wheel
 * 
 * @param rpm all data needed to estimate the spee
 * @return currenly estimated speed
 */
float current_speed(struct speed_measure_t * rpm);

/**
 * @brief calculates speed based on current wheel measurement
 * must be called every tick in order to calculate
 * speed properly
 * 
 * @param rpm all data needed to estimate the speed
 * @param cur_pos current measuremnt of wheel position
 */
void position_changed(struct speed_measure_t * rpm, uint16_t cur_pos);

/**
 * @brief calculates speed based on current wheel measurement
 * must be called every tick in order to calculate
 * speed properly
 * the version also finds out whether overflow or underflow happened
 * based on current speed and position
 * 
 * @param rpm all data needed to estimate the speed
 * @param cur_pos current measuremnt of wheel position
 */
void position_changed_detect_overflow(struct speed_measure_t * rpm, uint16_t cur_pos);

/**
 * gives hint to the measurement system that positive overflow happened
 *
 * @param s structure to be initialized
 */
void rpm_measurer_positive_overflow(struct speed_measure_t * s);


/**
 * gives hint to the measurement system that negative overflow happened
 *
 * @param s structure to be initialized
 */
void rpm_measurer_negative_overflow(struct speed_measure_t * s);

#ifdef __cplusplus
}
#endif

#endif /* __RPM_MEASURER_H__ */
