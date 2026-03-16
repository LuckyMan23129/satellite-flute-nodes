//=============================================================================#
/**
 * @brief: Read MCU Supply Voltage
 *
 * Reads the supply voltage powering the CircuitDojo nRF9160.
 * Used when the system operates without a boost converter.
 */
//=============================================================================#

#ifndef APPLICATION_READ_VOLATGE_SUPPLY_H
#define APPLICATION_READ_VOLATGE_SUPPLY_H


/** Enable or disable measurement of the battery voltage.
 *
 * @param enable true to enable, false to disable
 *
 * @return zero on success, or a negative error code.
 */
int16_t battery_measure_enable(bool enable);

/** Measure the battery voltage.
 *
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
int16_t battery_sample(void);

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	uint16_t lvl_pptt;

	/** Battery voltage at #lvl_pptt remaining life. */
	uint16_t lvl_mV;
};

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 *
 * @return the estimated remaining capacity in parts per ten
 * thousand.
 */
uint16_t battery_level_pptt(uint16_t batt_mV,
				const struct battery_level_point *curve);

int16_t  battery_setup(void);
uint16_t read_Vsupp_mv(void);

#endif /* APPLICATION_READ_VOLATGE_SUPPLY_H */