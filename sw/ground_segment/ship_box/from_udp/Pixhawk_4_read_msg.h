#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <sys/time.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef PIXHAWK4COMM_H
#define PIXHAWK4COMM_H

#define BAUDRATE_PIXHAWK_4 57600

#define SHIP_INFO_MSG_ID 190

#define SHIP_INFO_MSG "SHIP_INFO_MSG"

#define AC_ID_DEST "ground"

#define MESSAGE_ON_TX_FREQUENCY_CALCULATION 3

#define MSG_OUT_TX_FREQUENCY 10

#define PPRZ_STX 0x99

#define VERBOSE

#endif
