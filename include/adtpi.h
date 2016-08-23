/*
*
*
 */
#ifndef ADTPI
#define ADTPI

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define AVG_COUNT 32
//#define SAMPLE_READ_RATE_US 1000*3
#define CONV_TIMEOUT_US 10000
#define CH0 0
#define CH1 1
#define V_REF 4.096
#define COUNTS_MAX 16777216 //2^24
#define M_BIT 24
#define V_OFFS 2.048
#define INPUT_GAIN 2.68

#define LOG10_2 0.30103

//#define CONVERT_COUNTS 1 //uncomment to convert counts to physical value 

#define SY_ERR_PIN 7 
#define RDY_PIN 5 //paraleli SPI DOUT

//SERVER related definitions:
#define LISTEN_PORT 7176
#define CLIENT_MSG_LENGTH 70

typedef struct _stats_type
{
	double avg;
	double std;
	double enob;
}stats_type;

//delta_Vin = -Vref...+Vref --> delta_Vin_pp = 2*Vref
//counts = (Vref + delta_Vin)*(2^24)/(2*Vref)
//delta_Vin = (2*counts/(2^24) - 1)*Vref
//delta_Vin = (Vin - Voffs)*Ginput
//Vin = delta_Vin/Ginput + Voffs
#endif 