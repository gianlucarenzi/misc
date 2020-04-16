#ifndef __SERIAL_INCLUDED__
#define __SERIAL_INCLUDED__

#include <termios.h>
#include "debug.h"

extern int serial_device_init(const char *name, int baudrate, int pre, int post);
extern int serial_device_reset(int fd, int baudrate, int pre, int post);
extern void serial_device_status(int fd);
extern int serial_send_break(int fd);

// String oriented functions (EOL /r/n terminated)
extern int serial_send_string(int fd, const unsigned char *string);
extern int serial_read_string(int fd, unsigned char *buf, long to);

// Byte oriented function (length oriented)
extern int serial_send_raw(int fd, const unsigned char *buf, int len);
extern int serial_read_raw(int fd, unsigned char *buf, int len);

extern void serial_flush_rx(int serfd);
extern void serial_flush_tx(int serfd);

#define GET_PORT_STATE(fd, state) \
if (tcgetattr(fd, state) < 0) { \
	DRIVER_ERROR("tcgetattr() %d %s\n", errno, strerror(errno)); \
	close(fd); \
	fd = -1; \
	return -EPERM; \
}

#define SET_PORT_STATE(fd, state) \
if (tcsetattr(fd, TCSANOW, state) < 0) { \
	DRIVER_ERROR("tcsetattr() %d %s\n", errno, strerror(errno)); \
	close(fd); \
	fd = -1; \
	return -EPERM; \
}

#endif
