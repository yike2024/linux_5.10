/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2025, Sophgo inc
 */
#ifndef SECURE_OTP_H
#define SECURE_OTP_H

typedef struct otp_config {
	uint32_t segment;
	uint32_t addr;
	uint32_t size;
	uint32_t value[32];	
} otp_config_s;

#define IOCTL_OTP_BASE							'O'
#define IOCTL_OTP_VERSION                       _IOR(IOCTL_OTP_BASE, 0, uint32_t)
#define IOCTL_OTP2_READ_CONFIG                  _IOWR(IOCTL_OTP_BASE, 1, struct otp_config)
#define IOCTL_OTP2_WRITE_CONFIG                 _IOW(IOCTL_OTP_BASE, 2, struct otp_config)
#define IOCTL_OTP3_READ_CONFIG                  _IOWR(IOCTL_OTP_BASE, 3, struct otp_config)
#define IOCTL_OTP3_WRITE_CONFIG                 _IOW(IOCTL_OTP_BASE, 4, struct otp_config)

#endif /*SECURE_OTP_H*/
