/* ========================================================================== */
/*   gps.c                                                                    */
/*                                                                            */
/*   i2c bit-banging code for ublox on Pi A/A+/B/B+                           */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/*   12/10/14: Modified for the UBlox Max8 on the B+ board                    */
/*   19/12/14: Rewritten to use wiringPi library                              */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */
// Version 0.1 7/9/2012
// * removed a line of debug code

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <wiringPi.h>
#include "misc.h"
#include "gps.h"

struct i2c_info {
	uint8_t address; // 7 bit address
	uint8_t sda; // pin used for sda coresponds to gpio
	uint8_t scl; // clock
	uint32_t clock_delay; // proporional to bus speed
	uint32_t timeout; //
	int Failed;
};

void delayMilliseconds (unsigned int millis)
{
	struct timespec sleeper, dummy ;

	sleeper.tv_sec  = (time_t)(millis / 1000) ;
	sleeper.tv_nsec = (long)(millis % 1000) * 1000000 ;
	nanosleep (&sleeper, &dummy) ;
}

float FixPosition(float Position)
{
	float Minutes, Seconds;

	Position = Position / 100;

	Minutes = trunc(Position);
	Seconds = fmod(Position, 1);

	return Minutes + Seconds * 5 / 3;
}

int GPSChecksumOK(unsigned char *Buffer, int Count)
{
	unsigned char XOR,i,c;

	XOR = 0;
	for (i = 1; i < (Count-4); i++)
	{
		c = Buffer[i];
		XOR ^= c;
	}

	return (Buffer[Count-4] == '*') && (Buffer[Count-3] == Hex(XOR >> 4)) && (Buffer[Count-2] == Hex(XOR & 15));
}

void SendUBX(int fileptr, unsigned char *MSG, int len)
{
	if (fileptr > 0)
	{
		int count = write(fileptr, MSG, len);
		if  (count < 0)
		{
			printf("UART TX error\n");
		}
	}
}

void SetFlightMode(int filePtr)
{
    // Send navigation configuration command
    unsigned char setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
    SendUBX(filePtr, setNav, sizeof(setNav));
	printf ("Setting flight mode\n");
}

void ProcessLine(struct i2c_info *bb, struct TGPS *GPS, char *Buffer, int Count, int filePtr)
{
	static SystemTimeHasBeenSet=0;
	
    float utc_time, latitude, longitude, hdop, altitude;
	int lock, satellites;
	char active, ns, ew, units, timestring[16], speedstring[16], *course, *date, restofline[80], *ptr;
	long Hours, Minutes, Seconds;

	if (GPSChecksumOK(Buffer, Count))
	{
		satellites = 0;

		if (strncmp(Buffer+3, "GGA", 3) == 0)
		{
			if (sscanf(Buffer+7, "%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &latitude, &ns, &longitude, &ew, &lock, &satellites, &hdop, &altitude, &units) >= 1)
			{
				// $GPGGA,124943.00,5157.01557,N,00232.66381,W,1,09,1.01,149.3,M,48.6,M,,*42
				if (satellites >= 4)
				{
					GPS->Time = utc_time;
					Hours = GPS->Time / 10000;
					Minutes = (GPS->Time / 100) % 100;
					Seconds = GPS->Time % 100;
					GPS->Seconds = Hours * 3600 + Minutes * 60 + Seconds;					
					GPS->Latitude = FixPosition(latitude);
					if (ns == 'S') GPS->Latitude = -GPS->Latitude;
					GPS->Longitude = FixPosition(longitude);
					if (ew == 'W') GPS->Longitude = -GPS->Longitude;
					GPS->Altitude = altitude;
				}
				GPS->Satellites = satellites;
			}
			if (Config.EnableGPSLogging)
			{
				WriteLog("gps.txt", Buffer);
			}
		}
		else if (strncmp(Buffer+3, "RMC", 3) == 0)
		{
			speedstring[0] = '\0';
			if (sscanf(Buffer+7, "%[^,],%c,%f,%c,%f,%c,%[^,],%s", timestring, &active, &latitude, &ns, &longitude, &ew, speedstring, restofline) >= 7)
			{			
				// $GPRMC,124943.00,A,5157.01557,N,00232.66381,W,0.039,,200314,,,A*6C

				ptr = restofline;
				
				course = strsep(&ptr, ",");

				date = strsep(&ptr, ",");
				
				GPS->Speed = (int)atof(speedstring);
				GPS->Direction = (int)atof(course);

				if ((atof(timestring) > 0) && !SystemTimeHasBeenSet)
				{
					struct tm tm;
					char timedatestring[32];
					time_t t;

					// Now create a tm structure from our date and time
					memset(&tm, 0, sizeof(struct tm));
					sprintf(timedatestring, "%c%c-%c%c-20%c%c %c%c:%c%c:%c%c",
											date[0], date[1], date[2], date[3], date[4], date[5],
											timestring[0], timestring[1], timestring[2], timestring[3], timestring[4], timestring[5]);
					strptime(timedatestring, "%d-%m-%Y %H:%M:%S", &tm);
				
					t = mktime(&tm);
					if (stime(&t) == -1)
					{
						printf("Failed to set system time\n");
					}
					else
					{
						SystemTimeHasBeenSet = 1;
					}
				}
			}

			if (Config.EnableGPSLogging)
			{
				WriteLog("gps.txt", Buffer);
			}
		}
		else if (strncmp(Buffer+3, "GSV", 3) == 0)
        {
            // Disable GSV
            printf("Disabling GSV\r\n");
            unsigned char setGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
            SendUBX(filePtr, setGSV, sizeof(setGSV));
        }
		else if (strncmp(Buffer+3, "GLL", 3) == 0)
        {
            // Disable GLL
            printf("Disabling GLL\r\n");
            unsigned char setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
            SendUBX(filePtr, setGLL, sizeof(setGLL));
        }
		else if (strncmp(Buffer+3, "GSA", 3) == 0)
        {
            // Disable GSA
            printf("Disabling GSA\r\n");
            unsigned char setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
            SendUBX(filePtr, setGSA, sizeof(setGSA));
        }
		else if (strncmp(Buffer+3, "VTG", 3) == 0)
        {
            // Disable VTG
            printf("Disabling VTG\r\n");
            unsigned char setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
            SendUBX(filePtr, setVTG, sizeof(setVTG));
        }
        else
        {
            printf("Unknown NMEA sentence: %s\n", Buffer);
        }
    }
    else
    {
       printf("Bad checksum\r\n");
	}
}

void *GPSLoop(void *some_void_ptr)
{
	int uart0_filestream = -1;

	unsigned char Line[100];
	int id, Length;
	struct i2c_info bb;
	struct TGPS *GPS;

	GPS = (struct TGPS *)some_void_ptr;

	Length = 0;

	uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart0_filestream == -1)
	{
		printf("Error - unable to open UART");
	}

	struct termios options;

	tcgetattr(uart0_filestream, &options);

	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	SetFlightMode(uart0_filestream);

	int i;
	unsigned char Character;

	while (1)
	{
		i = read(uart0_filestream,(void*)&Character,1);
		
		if ( i > 0 )
		{
			if (Character == 0xFF)
			{
				delayMilliseconds (100);
			}
			else if (Character == '$')
			{
				Line[0] = Character;
				Length = 1;
			}
			else if (Length > 90)
			{
				Length = 0;
			}
			else if ((Length > 0) && (Character != '\r'))
			{
				Line[Length++] = Character;
				if (Character == '\n')
				{
					Line[Length] = '\0';
					ProcessLine(&bb, GPS, Line, Length, uart0_filestream);
					delayMilliseconds (100);
					Length = 0;
				}
			}
		}
	}
}


