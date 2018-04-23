/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * ADIS16477.hpp
 *
 */

#ifndef DRIVERS_IMU_ADIS16477_ADIS16477_HPP_
#define DRIVERS_IMU_ADIS16477_ADIS16477_HPP_

#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <drivers/device/integrator.h>
#include <lib/conversion/rotation.h>
#include <systemlib/perf_counter.h>

#define ADIS16477_GYRO_DEFAULT_RATE					100
#define ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16477_ACCEL_DEFAULT_RATE				100
#define ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16477_ACCEL_MAX_OUTPUT_RATE              1221
#define ADIS16477_GYRO_MAX_OUTPUT_RATE               1221

class ADIS16477_gyro;

class ADIS16477 : public device::SPI
{
public:
	ADIS16477(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation);
	virtual ~ADIS16477();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver and sensor.
	 */
	void			print_info();

	void 			print_calibration_data();

protected:
	virtual int		probe();

	friend class ADIS16477_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	ADIS16477_gyro		*_gyro{nullptr};

	uint16_t			_product{0};	/** product code */

	struct hrt_call		_call {};
	unsigned			_call_interval{0};

	ringbuffer::RingBuffer			*_gyro_reports{nullptr};

	struct gyro_calibration_s	_gyro_scale {};
	float				_gyro_range_scale{0.0f};
	float				_gyro_range_rad_s{0.0f};

	ringbuffer::RingBuffer			*_accel_reports{nullptr};

	struct accel_calibration_s	_accel_scale {};
	float				_accel_range_scale{0.0f};
	float				_accel_range_m_s2{0.0f};

	orb_advert_t		_accel_topic{nullptr};

	int					_accel_orb_class_instance{-1};
	int					_accel_class_instance{-1};

	unsigned			_sample_rate{100};

	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;

	math::LowPassFilter2p	_gyro_filter_x{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_gyro_filter_y{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_gyro_filter_z{ADIS16477_GYRO_DEFAULT_RATE, ADIS16477_GYRO_DEFAULT_DRIVER_FILTER_FREQ};

	math::LowPassFilter2p	_accel_filter_x{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_accel_filter_y{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};
	math::LowPassFilter2p	_accel_filter_z{ADIS16477_ACCEL_DEFAULT_RATE, ADIS16477_ACCEL_DEFAULT_DRIVER_FILTER_FREQ};

	Integrator			_accel_int{1000000 / ADIS16477_ACCEL_MAX_OUTPUT_RATE, false};
	Integrator			_gyro_int{1000000 / ADIS16477_GYRO_MAX_OUTPUT_RATE, true};

	enum Rotation		_rotation;

	perf_counter_t		_controller_latency_perf;

#pragma pack(push, 1)
	/**
	 * Report conversation with in the ADIS16477, including command byte and interrupt status.
	 */
	struct ADISReport {
		uint16_t		cmd;
		uint16_t		status;
		uint16_t		gyro_x;
		uint16_t		gyro_y;
		uint16_t		gyro_z;
		uint16_t		accel_x;
		uint16_t		accel_y;
		uint16_t		accel_z;
		uint16_t		temp;

		ADISReport():
			cmd(0),
			status(0),
			gyro_x(0),
			gyro_y(0),
			gyro_z(0),
			accel_x(0),
			accel_y(0),
			accel_z(0),
			temp(0) {}
	};
#pragma pack(pop)

	/**
	 * Start automatic measurement.
	 */
	void		start();

	/**
	 * Stop automatic measurement.
	 */
	void		stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			measure();

	/**
	 * Read a register from the ADIS16477
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16477
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void 			write_reg16(unsigned reg, uint16_t value);

	/**
	 * Modify a register in the ADIS16477
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Swap a 16-bit value read from the ADIS16477 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * convert 12 bit integer format to int16.
	 */
	int16_t			convert12BitToINT16(int16_t word);

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set IMU to factory default
	 */
	void _set_factory_default();

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/*
	  set the gyroscope dynamic range
	*/
	void _set_gyro_dyn_range(uint16_t desired_gyro_dyn_range);

	ADIS16477(const ADIS16477 &);
	ADIS16477 operator=(const ADIS16477 &);
};

#endif /* DRIVERS_IMU_ADIS16477_ADIS16477_HPP_ */
