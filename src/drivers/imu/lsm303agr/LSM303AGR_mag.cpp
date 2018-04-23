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

#include "LSM303AGR_mag.hpp"

LSM303AGR_mag::LSM303AGR_mag(LSM303AGR *parent) :
	CDev("LSM303AGR_mag", LSM303AGR_DEVICE_PATH_MAG),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
}

LSM303AGR_mag::~LSM303AGR_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}
}

int
LSM303AGR_mag::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		goto out;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

out:
	return ret;
}

void
LSM303AGR_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
LSM303AGR_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
LSM303AGR_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->mag_ioctl(filp, cmd, arg);
	}
}

void
LSM303AGR_mag::measure()
{
	_parent->mag_measure();
}

void
LSM303AGR_mag::measure_trampoline(void *arg)
{
	_parent->mag_measure_trampoline(arg);
}
