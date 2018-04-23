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

/**
 * @file adis16497.cpp
 *
 * Driver for the Analog device ADIS16497 connected via SPI.
 *
 */

#include <px4_config.h>
#include <ecl/geo/geo.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#define DIR_READ				0x00
#define DIR_WRITE				0x80

#define ADIS16497_DEVICE_PATH_ACCEL		"/dev/adis16497_accel"
#define ADIS16497_DEVICE_PATH_GYRO		"/dev/adis16497_gyro"

//  ADIS16497 registers
#define ADIS16497_FLASH_CNT  	0x00  /* Flash memory write count */
#define ADIS16497_SUPPLY_OUT 	0x02  /* Power supply measurement */
#define ADIS16497_XGYRO_OUT 	0x04  /* X-axis gyroscope output */
#define ADIS16497_YGYRO_OUT 	0x06  /* Y-axis gyroscope output */
#define ADIS16497_ZGYRO_OUT 	0x08  /* Z-axis gyroscope output */
#define ADIS16497_XACCL_OUT 	0x0A  /* X-axis accelerometer output */
#define ADIS16497_YACCL_OUT 	0x0C  /* Y-axis accelerometer output */
#define ADIS16497_ZACCL_OUT 	0x0E  /* Z-axis accelerometer output */
#define ADIS16497_TEMP_OUT  	0x18  /* Temperature output */

/* Calibration parameters */
#define ADIS16497_XGYRO_OFF 	0x1A  /* X-axis gyroscope bias offset factor */
#define ADIS16497_YGYRO_OFF 	0x1C  /* Y-axis gyroscope bias offset factor */
#define ADIS16497_ZGYRO_OFF 	0x1E  /* Z-axis gyroscope bias offset factor */
#define ADIS16497_XACCL_OFF 	0x20  /* X-axis acceleration bias offset factor */
#define ADIS16497_YACCL_OFF 	0x22  /* Y-axis acceleration bias offset factor */
#define ADIS16497_ZACCL_OFF 	0x24  /* Z-axis acceleration bias offset factor */

#define ADIS16497_GPIO_CTRL 	0x32  /* Auxiliary digital input/output control */
#define ADIS16497_MSC_CTRL  	0x34  /* Miscellaneous control */
#define ADIS16497_SMPL_PRD  	0x36  /* Internal sample period (rate) control */
#define ADIS16497_SENS_AVG  	0x38  /* Dynamic range and digital filter control */
#define ADIS16497_SLP_CNT   	0x3A  /* Sleep mode control */
#define ADIS16497_DIAG_STAT 	0x3C  /* System status */

/* Alarm functions */
#define ADIS16497_GLOB_CMD  	0x3E  /* System command */
#define ADIS16497_ALM_SMPL1 	0x44  /* Alarm 1 sample size */
#define ADIS16497_ALM_SMPL2 	0x46  /* Alarm 2 sample size */
#define ADIS16497_ALM_CTRL  	0x48  /* Alarm control */

#define ADIS16334_LOT_ID1   	0x52  /* Lot identification code 1 */
#define ADIS16334_LOT_ID2   	0x54  /* Lot identification code 2 */
#define ADIS16497_PRODUCT_ID 	0x56  /* Product identifier */
#define ADIS16334_SERIAL_NUMBER 0x58  /* Serial number, lot specific */

#define ADIS16497_Product		0x4040/* Product ID Description for ADIS16497 */

#define BITS_SMPL_PRD_NO_TAP_CFG 	(0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG	 	(1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG	 	(2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG	 	(3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG	(4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG	 (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG	 (1<<8)

#define BITS_FIR_NO_TAP_CFG		(0<<0)
#define BITS_FIR_2_TAP_CFG		(1<<0)
#define BITS_FIR_4_TAP_CFG		(2<<0)
#define BITS_FIR_8_TAP_CFG		(3<<0)
#define BITS_FIR_16_TAP_CFG		(4<<0)
#define BITS_FIR_32_TAP_CFG		(5<<0)
#define BITS_FIR_64_TAP_CFG		(6<<0)
#define BITS_FIR_128_TAP_CFG	(7<<0)


#define ADIS16497_GYRO_DEFAULT_RATE					100
#define ADIS16497_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16497_ACCEL_DEFAULT_RATE				100
#define ADIS16497_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16497_ACCEL_MAX_OUTPUT_RATE              1221
#define ADIS16497_GYRO_MAX_OUTPUT_RATE               1221

#define FW_FILTER									false

#define SPI_BUS_SPEED								1000000
#define T_STALL										9

#define GYROINITIALSENSITIVITY						250
#define ACCELINITIALSENSITIVITY						(1.0f / 1200.0f)
#define ACCELDYNAMICRANGE							18.0f

class ADIS16497_gyro;

class ADIS16497 : public device::SPI
{
public:
	ADIS16497(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation);
	virtual ~ADIS16497();

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

	friend class ADIS16497_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	ADIS16497_gyro		*_gyro;

	uint16_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned			_call_interval;

	ringbuffer::RingBuffer			*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
	float				_gyro_range_scale;
	float				_gyro_range_rad_s;

	ringbuffer::RingBuffer			*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float				_accel_range_scale;
	float				_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int					_accel_orb_class_instance;
	int					_accel_class_instance;

	unsigned			_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;


	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;
	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	Integrator			_accel_int;
	Integrator			_gyro_int;

	enum Rotation		_rotation;

	perf_counter_t		_controller_latency_perf;

#pragma pack(push, 1)
	/**
	 * Report conversation with in the ADIS16497, including command byte and interrupt status.
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
	 * Read a register from the ADIS16497
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16497
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void 			write_reg16(unsigned reg, uint16_t value);

	/**
	 * Modify a register in the ADIS16497
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Swap a 16-bit value read from the ADIS16497 to native byte order.
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

	ADIS16497(const ADIS16497 &);
	ADIS16497 operator=(const ADIS16497 &);
};

/**
 * Helper class implementing the gyro driver node.
 */
class ADIS16497_gyro : public device::CDev
{
public:
	ADIS16497_gyro(ADIS16497 *parent, const char *path);
	virtual ~ADIS16497_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16497;

	void			parent_poll_notify();
private:
	ADIS16497			*_parent;
	orb_advert_t		_gyro_topic;
	int					_gyro_orb_class_instance;
	int					_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	ADIS16497_gyro(const ADIS16497_gyro &);
	ADIS16497_gyro operator=(const ADIS16497_gyro &);

};

/** driver 'main' command */
extern "C" { __EXPORT int adis16497_main(int argc, char *argv[]); }

ADIS16497::ADIS16497(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	SPI("ADIS16497", path_accel, bus, device, SPIDEV_MODE3, SPI_BUS_SPEED),
	_gyro(new ADIS16497_gyro(this, path_gyro)),
	_product(0),
	_call{},
	_call_interval(0),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_sample_rate(100),																			/* Init sampling frequency set to 100Hz */
	_accel_reads(perf_alloc(PC_COUNT, "adis16497_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "adis16497_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "adis16497_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "adis16497_bad_transfers")),
	_gyro_filter_x(ADIS16497_GYRO_DEFAULT_RATE, ADIS16497_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(ADIS16497_GYRO_DEFAULT_RATE, ADIS16497_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(ADIS16497_GYRO_DEFAULT_RATE, ADIS16497_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_x(ADIS16497_ACCEL_DEFAULT_RATE, ADIS16497_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(ADIS16497_ACCEL_DEFAULT_RATE, ADIS16497_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(ADIS16497_ACCEL_DEFAULT_RATE, ADIS16497_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / ADIS16497_ACCEL_MAX_OUTPUT_RATE, false),
	_gyro_int(1000000 / ADIS16497_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16497;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16497;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

ADIS16497::~ADIS16497()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
}

int
ADIS16497::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* fetch an initial set of measurements for advertisement */
	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	struct gyro_report grp;

	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

int ADIS16497::reset()
{
	/* Set gyroscope scale to default value */
	_set_gyro_dyn_range(GYROINITIALSENSITIVITY);

	/* Set digital FIR filter tap */
	_set_dlpf_filter(BITS_FIR_16_TAP_CFG);

	/* Set IMU sample rate */
	_set_sample_rate(_sample_rate);

	_accel_range_scale = CONSTANTS_ONE_G * ACCELINITIALSENSITIVITY;
	_accel_range_m_s2  = CONSTANTS_ONE_G * ACCELDYNAMICRANGE;

	/* settling time */
	up_udelay(50000);

	return OK;
}

int
ADIS16497::probe()
{
	uint16_t serial_number;

	/* retry 5 time to get the ADIS16497 PRODUCT ID number */
	for (int i = 0; i < 5; i++) {
		/* recognize product ID */
		_product = read_reg16(ADIS16497_PRODUCT_ID);

		if (_product != 0) {
			break;
		}
	}

	/* recognize product serial number */
	serial_number = (read_reg16(ADIS16334_SERIAL_NUMBER) & 0xfff);

	/* verify product ID */
	switch (_product) {
	case ADIS16497_Product:
		DEVICE_DEBUG("ADIS16497 is detected ID: 0x%02x, Serial: 0x%02x", _product, serial_number);
		modify_reg16(ADIS16497_GPIO_CTRL, 0x0200, 0x0002);			/* Turn on ADIS16497 adaptor board led */
		return OK;
	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product);
	return -EIO;
}

/* set sample rate for both accel and gyro */
void
ADIS16497::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	if (desired_sample_rate_hz <= 51) {
		smpl_prd = BITS_SMPL_PRD_16_TAP_CFG;

	} else if (desired_sample_rate_hz <= 102) {
		smpl_prd = BITS_SMPL_PRD_8_TAP_CFG;

	} else if (desired_sample_rate_hz <= 204) {
		smpl_prd = BITS_SMPL_PRD_4_TAP_CFG;

	} else if (desired_sample_rate_hz <= 409) {
		smpl_prd = BITS_SMPL_PRD_2_TAP_CFG;

	} else {
		smpl_prd = BITS_SMPL_PRD_NO_TAP_CFG;
	}

	modify_reg16(ADIS16497_SMPL_PRD, 0x1f00, smpl_prd);

	if ((read_reg16(ADIS16497_SMPL_PRD) & 0x1f00) != smpl_prd) {
		DEVICE_DEBUG("failed to set IMU sample rate");
	}

}

/* set the DLPF FIR filter tap. This affects both accelerometer and gyroscope. */
void
ADIS16497::_set_dlpf_filter(uint16_t desired_filter_tap)
{
	modify_reg16(ADIS16497_SENS_AVG, 0x0007, desired_filter_tap);

	/* Verify data write on the IMU */

	if ((read_reg16(ADIS16497_SENS_AVG) & 0x0007) != desired_filter_tap) {
		DEVICE_DEBUG("failed to set IMU filter");
	}

}

/* set IMU to factory defaults. */
void
ADIS16497::_set_factory_default()
{
	write_reg16(ADIS16497_GLOB_CMD, 0x02);
}

/* set the gyroscope dynamic range */
void
ADIS16497::_set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{

	uint16_t gyro_range_selection = 0;

	if (desired_gyro_dyn_range <= 250) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_250_CFG;

	} else if (desired_gyro_dyn_range <= 500) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_500_CFG;

	} else {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_1000_CFG;
	}

	modify_reg16(ADIS16497_SENS_AVG, 0x0700, gyro_range_selection);

	/* Verify data write on the IMU */

	if ((read_reg16(ADIS16497_SENS_AVG) & 0x0700) != gyro_range_selection) {
		DEVICE_DEBUG("failed to set gyro range");

	} else {
		_gyro_range_rad_s  = ((float)(gyro_range_selection >> 8) * 250.0f / 180.0f) * M_PI_F;
		_gyro_range_scale  = (float)(gyro_range_selection >> 8) / 100.0f;
	}
}


ssize_t
ADIS16497::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
ADIS16497::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
ADIS16497::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
ADIS16497::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

ssize_t
ADIS16497::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

int
ADIS16497::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16497_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);

					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_accel_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case ACCELIOCGSAMPLERATE:
		return _sample_rate;

	case ACCELIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return -EINVAL;

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / CONSTANTS_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

	case ACCELIOCTYPE:
		return (ADIS16497_Product);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16497::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_gyro_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case GYROIOCGSAMPLERATE:
		return _sample_rate;

	case GYROIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		_set_gyro_dyn_range(arg);
		return OK;

	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

	case GYROIOCTYPE:
		return (ADIS16497_Product);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint16_t
ADIS16497::read_reg16(unsigned reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16497::write_reg16(unsigned reg, uint16_t value)
{
	uint16_t	cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

void
ADIS16497::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t	val;

	val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}

int16_t
ADIS16497::convert12BitToINT16(int16_t word)
{

	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}

void
ADIS16497::start()
{
	/* make sure we are stopped first */
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	/* discard any stale data in the buffers */
	_gyro_reports->flush();
	_accel_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&ADIS16497::measure_trampoline, this);
}

void
ADIS16497::stop()
{
	hrt_cancel(&_call);
}

void
ADIS16497::measure_trampoline(void *arg)
{
	ADIS16497 *dev = reinterpret_cast<ADIS16497 *>(arg);

	/* make another measurement */
	dev->measure();
}

int
ADIS16497::measure()
{
	struct ADISReport adis_report;

	struct Report {
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the ADIS16497 in one pass (burst read).
	 */

	adis_report.cmd = ((ADIS16497_GLOB_CMD | DIR_READ) << 8) & 0xff00;

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		return -EIO;
	}

	report.gyro_x  = (int16_t) adis_report.gyro_x;
	report.gyro_y  = (int16_t) adis_report.gyro_y;
	report.gyro_z  = (int16_t) adis_report.gyro_z;
	report.accel_x = (int16_t) adis_report.accel_x;
	report.accel_y = (int16_t) adis_report.accel_y;
	report.accel_z = (int16_t) adis_report.accel_z;
	report.temp    = convert12BitToINT16(adis_report.temp);

	if (report.gyro_x == 0 && report.gyro_y == 0 && report.gyro_z == 0 &&
	    report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0) {

		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	/*
	 * Report buffers.
	 */
	accel_report arb;
	gyro_report grb;

	arb.timestamp = grb.timestamp = hrt_absolute_time();

	grb.error_count = arb.error_count = perf_event_count(_bad_transfers);

	/* Gyro report: */
	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float xraw_f = report.gyro_x;
	float yraw_f = report.gyro_y;
	float zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	if (FW_FILTER) {
		grb.x = _gyro_filter_x.apply(x_gyro_in_new);
		grb.y = _gyro_filter_y.apply(y_gyro_in_new);
		grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	} else {
		grb.x = x_gyro_in_new;
		grb.y = y_gyro_in_new;
		grb.z = z_gyro_in_new;
	}

	grb.scaling = _gyro_range_scale * M_PI_F / 180.0f;
	grb.range_rad_s = _gyro_range_rad_s;

	/* Accel report: */
	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	xraw_f = report.accel_x;
	yraw_f = report.accel_y;
	zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	if (FW_FILTER) {
		arb.x = _accel_filter_x.apply(x_in_new);
		arb.y = _accel_filter_y.apply(y_in_new);
		arb.z = _accel_filter_z.apply(z_in_new);

	} else {
		arb.x = x_in_new;
		arb.y = y_in_new;
		arb.z = z_in_new;
	}

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	/* Temperature report: */
	grb.temperature_raw = report.temp;
	grb.temperature 	= (report.temp * 0.07386f) + 31.0f;

	arb.temperature_raw = report.temp;
	arb.temperature 	= (report.temp * 0.07386f) + 31.0f;

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	/* return device ID */
	arb.device_id = _device_id.devid;
	grb.device_id = _gyro->_device_id.devid;

	_gyro_reports ->force(&grb);
	_accel_reports->force(&arb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
	return OK;
}

void
ADIS16497::print_calibration_data()
{
	uint16_t XGYRO_OFF = read_reg16(ADIS16497_XGYRO_OFF);
	uint16_t YGYRO_OFF = read_reg16(ADIS16497_YGYRO_OFF);
	uint16_t ZGYRO_OFF = read_reg16(ADIS16497_ZGYRO_OFF);
	uint16_t XACCL_OFF = read_reg16(ADIS16497_XACCL_OFF);
	uint16_t YACCL_OFF = read_reg16(ADIS16497_YACCL_OFF);
	uint16_t ZACCL_OFF = read_reg16(ADIS16497_ZACCL_OFF);

	PX4_INFO("single calibration value read:");
	PX4_INFO("XGYRO_OFF =:  \t%8.4x\t", XGYRO_OFF);
	PX4_INFO("YGYRO_OFF =:  \t%8.4x\t", YGYRO_OFF);
	PX4_INFO("ZGYRO_OFF =:  \t%8.4x\t", ZGYRO_OFF);
	PX4_INFO("XACCL_OFF =:  \t%8.4x\t", XACCL_OFF);
	PX4_INFO("YACCL_OFF =:  \t%8.4x\t", YACCL_OFF);
	PX4_INFO("ZACCL_OFF =:  \t%8.4x\t", ZACCL_OFF);
}

void
ADIS16497::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");

	PX4_INFO("DEVICE ID:\nACCEL:\t%d\nGYRO:\t%d\n\n", _device_id.devid, _gyro->_device_id.devid);
}

ADIS16497_gyro::ADIS16497_gyro(ADIS16497 *parent, const char *path) :
	CDev("ADIS16497_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
}

ADIS16497_gyro::~ADIS16497_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
ADIS16497_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	return ret;
}

void
ADIS16497_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ADIS16497_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
ADIS16497_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace adis16497
{

ADIS16497	*g_dev;

void	start(enum Rotation rotation);
void	test();
void	reset();
void	info();
void 	info_cal();
void	usage();
/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16497(PX4_SPI_BUS_EXT, ADIS16497_DEVICE_PATH_ACCEL, ADIS16497_DEVICE_PATH_GYRO, PX4_SPIDEV_EXT_MPU,
			      rotation);
#else
	PX4_ERR("External SPI not available");
	exit(0);
#endif

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != (g_dev)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(ADIS16497_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);
	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	accel_report a_report;
	gyro_report  g_report;

	ssize_t sz;

	/* get the driver */
	int fd = open(ADIS16497_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed", ADIS16497_DEVICE_PATH_ACCEL);
	}

	/* get the gyro driver */
	int fd_gyro = open(ADIS16497_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", ADIS16497_DEVICE_PATH_GYRO);
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	close(fd_gyro);
	close(fd);

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(ADIS16497_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a sensor calibration info.
 */
void
info_cal()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	g_dev->print_calibration_data();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'info_cal', 'reset',\n");
	warnx("options:");
	warnx("    -R rotation");
}

}
// namespace

int
adis16497_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16497::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		adis16497::start(rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		adis16497::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		adis16497::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		adis16497::info();
	}

	/*
	 * Print sensor calibration information.
	 */
	if (!strcmp(verb, "info_cal")) {
		adis16497::info_cal();
	}

	adis16497::usage();
	exit(1);
}
