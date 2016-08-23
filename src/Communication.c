#include "adtpi.h"
#include "Communication.h"

int fd_spi; 

void pabort(const char *s)
{
	perror(s);
	abort();
}

//variable "uint8_t slave_select_id" declared just to be compatible with AD7176 library. 
////////////////////////////////////////////////////////////////////////////////
int32_t SPI_Write(uint8_t slave_select_id, uint8_t* buf, uint32_t buflength)
{
	int32_t ret;
	
	 ret = spiWrite(fd_spi, (char*)buf, buflength);
	 if (ret < 1)
		pabort("can't send spi message");
	
	return ret;
}

////////////////////////////////////////////////////////////////////////////////
int32_t SPI_Read(uint8_t slave_select_id, uint8_t* buf, uint32_t buflength)
{
	int32_t ret;

	 ret = spiXfer(fd_spi, (char*)buf, (char*)buf, buflength);
	 if (ret < 1)
		pabort("can't send spi message");
	
	return ret;
}
