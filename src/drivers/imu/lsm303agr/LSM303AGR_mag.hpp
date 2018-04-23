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

#ifndef DRIVERS_IMU_LSM303AGR_LSM303AGR_MAG_HPP_
#define DRIVERS_IMU_LSM303AGR_LSM303AGR_MAG_HPP_

#include "LSM303AGR.hpp"

#include <drivers/device/CDev.hpp>
#include <drivers/drv_mag.h>

#define LSM303AGR_DEVICE_PATH_MAG		"/dev/LSM303AGR_mag"

/**
 * Helper class implementing the mag driver node.
 */
class LSM303AGR_mag : public device::CDev
{
public:
	LSM303AGR_mag(LSM303AGR *parent);
	~LSM303AGR_mag();

	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class LSM303AGR;

	void				parent_poll_notify();
private:
	LSM303AGR				*_parent;

	orb_advert_t			_mag_topic;
	int				_mag_orb_class_instance;
	int				_mag_class_instance;

	void				measure();

	void				measure_trampoline(void *arg);

	/* this class does not allow copying due to ptr data members */
	LSM303AGR_mag(const LSM303AGR_mag &);
	LSM303AGR_mag operator=(const LSM303AGR_mag &);
};

#endif /* DRIVERS_IMU_LSM303AGR_LSM303AGR_MAG_HPP_ */
