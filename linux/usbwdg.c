#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "hidapi.h"

enum {
	WATCHDOG_UNKNOWN = 0,
	WATCHDOG_SERIAL,
	WATCHDOG_HID,
};

int active = 1;
int watchdog_type = WATCHDOG_UNKNOWN;
char* serial_device = NULL;
int serial_handle = -1;
hid_device* hid_handle = NULL;

int serial_connect();
void serial_close();
int serial_setup();

void serial_scan()
{
	if (serial_device != NULL)
	{
		free(serial_device);
		serial_device = NULL;
	}

	int n;
	struct dirent **namelist;
	const char* sysdir = "/sys/class/tty/";
	n = scandir(sysdir, &namelist, NULL, NULL);
	if (n < 0)
		return;

	while (n--)
	{
		if ((serial_device != NULL) ||
			(strncmp(namelist[n]->d_name, "ttyUSB", 6) != 0))
		{
			free(namelist[n]);
			continue;
		}

		char device[512];
		snprintf(device, sizeof(device), "%s%s/device/driver", sysdir, namelist[n]->d_name);
		char* driver = realpath(device, NULL);
		if (strcmp(driver, "/sys/bus/usb-serial/drivers/ch341-uart") == 0)
		{
			int len = strlen(namelist[n]->d_name);
			serial_device = malloc(6 + len);
			strcpy(serial_device, "/dev/");
			strcat(serial_device, namelist[n]->d_name);

			if (serial_setup() < 0)
			{
				serial_close();
				free(serial_device);
				serial_device = NULL;
			}
		}
		free(driver);
		free(namelist[n]);
	}
	free(namelist);

	if (serial_device != NULL)
	{
		watchdog_type = WATCHDOG_SERIAL;
	}
}

int serial_connect(void)
{
	serial_handle = open(serial_device, O_RDWR | O_NOCTTY | O_SYNC);
	return (serial_handle < 0) ? -1 : 0;
}

void serial_close(void)
{
	if (serial_handle >= 0)
	{
		close(serial_handle);
		serial_handle = -1;
	}
}

int serial_setup()
{
	serial_connect();

	if (serial_handle < 0)
	{
		return -1;
	}

	struct termios serial;

	if (tcgetattr(serial_handle, &serial) != 0)
	{
		return -1;
	}

	cfsetispeed(&serial, B9600);
	cfsetospeed(&serial, B9600);

	serial.c_cflag &= ~(PARENB | PARODD);
	serial.c_cflag &= ~CSTOPB;
	serial.c_cflag &= ~CSIZE;
	serial.c_cflag |= CS8;
	serial.c_cflag |= CRTSCTS;
	serial.c_cflag |= CREAD;
	serial.c_cflag |= CLOCAL;

	serial.c_iflag &= ~(IXON | IXOFF | IXANY);
	serial.c_iflag &= ~IGNBRK;

	serial.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	serial.c_oflag &= ~OPOST;

	serial.c_cc[VMIN] = 0;
	serial.c_cc[VTIME] = 10;

	if (tcsetattr(serial_handle, TCSANOW, &serial) != 0)
	{
		return -1;
	}

	tcflush(serial_handle, TCIFLUSH);

	int ret;
	unsigned char buf[3];
	buf[0] = 0x80;
	ret = write(serial_handle, buf, 1);
	if (ret != 1)
	{
		return -1;
	}

	ret = read(serial_handle, buf, 3);
	if ((ret != 3) && (buf[0] != 0x81))
	{
		return -1;
	}

	return 0;
}

int hidwdg_connect(void)
{
	static int init = 0;
	if (!init)
	{
		// Initialize the hidapi library
		hid_init();
		init = 1;
	}

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	hid_handle = hid_open(0x5131, 0x2007, NULL);

	if (hid_handle != NULL)
	{
		watchdog_type = WATCHDOG_HID;
		return 0;
	}
	else
	{
		return -1;
	}
}

void hidwdg_close(void)
{
	if (hid_handle != NULL)
	{
		hid_close(hid_handle);
		hid_handle = NULL;
	}
}

int watchdog_find(void)
{
	if (watchdog_type != WATCHDOG_UNKNOWN)
		return 0;

	hidwdg_connect();
	hidwdg_close();
	if (watchdog_type == WATCHDOG_HID)
		return 0;

	serial_scan();
	serial_close();
	if (watchdog_type == WATCHDOG_SERIAL)
		return 0;

	return -1;
}

int watchdog_connect(void)
{
	if (watchdog_type == WATCHDOG_HID)
	{
		return hidwdg_connect();
	}
	else if (watchdog_type == WATCHDOG_SERIAL)
	{
		return serial_connect();
	}
}

void watchdog_close(void)
{
	if (watchdog_type == WATCHDOG_HID)
	{
		hidwdg_close();
	}
	else if (watchdog_type == WATCHDOG_SERIAL)
	{
		serial_close();
	}
}

void watchdog_exit(void)
{
	watchdog_close();

	hid_exit();

	if (serial_device != NULL)
	{
		free(serial_device);
		serial_device = NULL;
	}

	watchdog_type = WATCHDOG_UNKNOWN;
}

void watchdog_signal(int seconds)
{
	if (watchdog_type == WATCHDOG_HID)
	{
		unsigned char buf[65] = {0, };
		if (seconds > 1270)
			seconds = 1270;
		buf[0] = seconds / 10;
		buf[1] = 0;
		hid_write(hid_handle, buf, 2);
		hid_read(hid_handle, buf, 2);
	}
	else if (watchdog_type == WATCHDOG_SERIAL)
	{
		unsigned char buf[1];
		if (seconds > 1270)
			seconds = 1270;
		buf[0] = seconds / 10;
		write(serial_handle, buf, 1);
	}
}

void signal_handler(int signal)
{
	switch (signal)
	{
		case SIGTERM:
			active = 0;
			break;
	}
}

void daemonize()
{
	pid_t pid, sid;

	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);
	if (pid > 0)
		exit(EXIT_SUCCESS);

	sid = setsid();
	if (sid < 0)
		exit(EXIT_FAILURE);

	umask(027);
	if (chdir("/") < 0)
		exit(EXIT_FAILURE);

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	signal(SIGCHLD, SIG_IGN); /* ignore child */
	signal(SIGTSTP, SIG_IGN); /* ignore tty signals */
	signal(SIGTTOU, SIG_IGN);
	signal(SIGTTIN, SIG_IGN);
	signal(SIGHUP, signal_handler); /* catch hangup signal */
	signal(SIGTERM, signal_handler); /* catch kill signal */
}

int main(void)
{
	if (geteuid() != 0)
	{
		fprintf(stderr, "must be root\n");
		exit(EXIT_FAILURE);
	}

	daemonize();

	while (active)
	{
		if (watchdog_find() != 0)
		{
			sleep(5);
			continue;
		}

		int t;
		while (active)
		{
			if (watchdog_connect() != 0)
			{
				break;
			}

			watchdog_signal(180);
			watchdog_close();

			for (t = 60; active && t; t--)
			{
				if (!active)
					break;
				sleep(1);
			}
		}
	}

	watchdog_exit();

	return 0;
}
