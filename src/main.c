#include "adtpi.h"
#include "Communication.h"
#include "ad717x.h"
#include "math.h"

#define AD7176_2_INIT
#include "ad7176_2_regs.h"

#include <pigpio.h>

#include<stdio.h>
#include<string.h>    //strlen
#include<stdlib.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<unistd.h>    //write
#include<pthread.h> //for threading , link with lpthread

struct ad717x_device ad7176_2;
ad717x_st_reg ad7176_2_regs_active[ARRAY_SIZE(ad7176_2_regs)];

extern int fd_spi; 

//the thread function
void *connection_handler(void *);

pthread_mutex_t lock;
//Local functions
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int set_ad7176_gpio(struct ad717x_device *device, uint8_t set_clear)
{
	int ret;
	ad717x_st_reg *ad7176_2_GPIO;
	
	ad7176_2_GPIO = AD717X_GetReg(device, AD717X_GPIOCON_REG);
	
	if(set_clear > 0)
		ad7176_2_GPIO->value = ad7176_2_GPIO->value|AD717X_GPIOCON_REG_DATA1;
	else
		ad7176_2_GPIO->value = ad7176_2_GPIO->value&(~AD717X_GPIOCON_REG_DATA1);

		ret = AD717X_WriteRegister(device, ad7176_2_GPIO->addr);
		return ret;
}
//////////////////////////////////////////////////////////////////////////////////
int set_ad7176_channel(struct ad717x_device *device, uint8_t channel)
{
	int ret;
	ad717x_st_reg *ad7176_2_CH;
	
	if(channel == 0)
	{
		ad7176_2_CH = AD717X_GetReg(device, AD717X_CHMAP0_REG);//enable ch0
		ad7176_2_CH->value = ad7176_2_CH->value|AD717X_CHMAP_REG_CH_EN;
		ret = AD717X_WriteRegister(device, ad7176_2_CH->addr);
		if(ret < 0)
			return ret;
		
		ad7176_2_CH = AD717X_GetReg(device, AD717X_CHMAP1_REG);//disable ch1
		ad7176_2_CH->value = ad7176_2_CH->value&(~AD717X_CHMAP_REG_CH_EN);
		ret = AD717X_WriteRegister(device, ad7176_2_CH->addr);
	}
	else //if 1 or other number, set to 1
	{
		ad7176_2_CH = AD717X_GetReg(device, AD717X_CHMAP1_REG);//enable ch1
		ad7176_2_CH->value = ad7176_2_CH->value|AD717X_CHMAP_REG_CH_EN;
		ret = AD717X_WriteRegister(device, ad7176_2_CH->addr);
		if(ret < 0)
			return ret;
		
		ad7176_2_CH = AD717X_GetReg(device, AD717X_CHMAP0_REG);//disable ch0
		ad7176_2_CH->value = ad7176_2_CH->value&(~AD717X_CHMAP_REG_CH_EN);
		ret = AD717X_WriteRegister(device, ad7176_2_CH->addr);
	}

		return ret;
}

//////////////////////////////////////////////////////////////////////////////////
stats_type claculate_stats(double* input_buffer, int size)
{
	int i;
	stats_type result_statistics;

	     //result average
		result_statistics.avg = 0;		
		for (i = 0; i<size; ++i)
			result_statistics.avg += input_buffer[i];	
		result_statistics.avg = result_statistics.avg/size;
		
		//result standard dev
		result_statistics.std = 0;
		for (i = 0; i<size; ++i)
			result_statistics.std += (input_buffer[i] - result_statistics.avg)*(input_buffer[i] - result_statistics.avg);	
		result_statistics.std = sqrt(result_statistics.std / size);
	
		result_statistics.enob = M_BIT - log10(result_statistics.std)/LOG10_2;//http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.555.2763&rep=rep1&type=pdf
   
		//result_statistics.avg = abs(result_statistics.avg - COUNTS_MAX);//this in case if detector is inverting
       
	return 	result_statistics;
}
//////////////////////////////////////////////////////////////////////////////////
double convert_counts(unsigned int adc_counts)
{
	double delta_Vin, Vin;
		
		//table interpolation will be better
		
		//see header file for constants
		delta_Vin = (2*(double)adc_counts/COUNTS_MAX - 1)*V_REF;
	    Vin = V_OFFS + delta_Vin/INPUT_GAIN;
		
		return Vin;
}
//////////////////////////////////////////////////////////////////////////////////
void start_ad717x_conversion(void)
{
	gpioWrite(SY_ERR_PIN, 0);
	usleep(1);
	gpioWrite(SY_ERR_PIN, 1);//lo-hi edge initiates conv.
}
//////////////////////////////////////////////////////////////////////////////////
int get_adc_data(struct ad717x_device *device, uint8_t channel, double* input_buffer, int size)
{
	unsigned int adc_result, data_reg, data_status, timeout;
	int i, ret = 0;
	
	set_ad7176_channel(device, channel);
	
	for(i = 0; i<size; i++)
	{
		start_ad717x_conversion();
		timeout = CONV_TIMEOUT_US;
		while(gpioRead(RDY_PIN) > 0)//RDY pin high-lo signalizes conv. finish
		{
			usleep(1);
			timeout--;
			if(timeout == 0)
			{
				printf("ADC conversion timeout\n");
				break;
			}		
		}
			
		//usleep(SAMPLE_READ_RATE_US);
		
		ret = AD717X_ReadData(device, (int32_t*)&data_reg);			
		
		data_status = data_reg & 0xff;//adc status reg not further used/returned yet... maybe in future. see datasheet for status reg bit meanings
		adc_result = (data_reg>>8) & 0xffffff;
		
		#ifdef CONVERT_COUNTS
			input_buffer[i] = convert_counts(adc_result);
		#else
			input_buffer[i] = (double)adc_result;	
		#endif
		
		if(ret < 0)
		break;
	}
	
	return ret;
}
//////////////////////////////////////////////////////////////////////////////////
int init_gpio(void)
{
	int ret = 0;
	
	//init GPIO
	ret = gpioInitialise();
	if (ret < 0)
	return ret;
	
	ret = gpioSetMode(SY_ERR_PIN, PI_OUTPUT); // Set as output.	
	if (ret < 0)
	return ret;

	ret = gpioSetPullUpDown(SY_ERR_PIN, PI_PUD_UP);//pullup	
	if (ret < 0)
	return ret;	

	gpioWrite(SY_ERR_PIN, 0);
	
	ret = gpioSetMode(RDY_PIN, PI_INPUT); // Set as input.	

	return ret;
}
//////////////////////////////////////////////////////////////////////////////////
int init_ad7176(void)
{
	int ret = 0, i;
	ad717x_st_reg *ad7176_2_ID;
	ad717x_st_reg *ad7176_2_ADC_MODE;
	
	//need to do these things for re-initialization to work correctly
	//-->
	for(i = 0; i<ARRAY_SIZE(ad7176_2_regs); i++)
		ad7176_2_regs_active[i] = ad7176_2_regs[i];
	
	ad7176_2.slave_select_id = 0;
	ad7176_2.regs = NULL;
	ad7176_2.num_regs = 0;
	ad7176_2.useCRC = AD717X_DISABLE;
	//<--
	
	ret = (int)AD717X_Setup(&ad7176_2, AD7176_2_SLAVE_ID, ad7176_2_regs_active, ARRAY_SIZE(ad7176_2_regs_active));	
	if(ret < 0)
	{
		printf("AD717X_Setup failed. error code: %d\n", ret);
		return ret;
	}
	
	//enable 2.5V reference for diff drivers common mode voltage
	ad7176_2_ADC_MODE = AD717X_GetReg(&ad7176_2, AD717X_ADCMODE_REG);
	ad7176_2_ADC_MODE->value = ad7176_2_ADC_MODE->value|AD717X_ADCMODE_REG_REF_EN;
	AD717X_WriteRegister(&ad7176_2, ad7176_2_ADC_MODE->addr);
	
	//reading ADC status register value. Should be 0x0c94 for AD7172-2
	AD717X_ReadRegister(&ad7176_2, AD717X_ID_REG);

	ad7176_2_ID = AD717X_GetReg(&ad7176_2, AD717X_ID_REG);
	printf("ADC ID: %#04x <--", ad7176_2_ID->value);
	if(ad7176_2_ID->value == 0x0c94)
		printf("OK (AD7172-2)\n");
	else
		printf("Wrong!\n");
	
	unsigned int data_reg;
	ret = AD717X_ReadData(&ad7176_2, (int32_t*)&data_reg);//dummy read
	if(ret < 0)
		printf("dummy AD717X_ReadData failed. error code: %d\n", ret);
	
	return ret;
}
//////////////////////////////////////////////////////////////////////////////////
int init_periph(void)
{
	int ret = 0;
	
	ret = init_gpio();
	if (ret < 0)
	{
	   printf("pigpio initialisation failed\n");
	   return ret;
	}
	
	//spi init only after init_gpio()
	fd_spi = spiOpen(SPI_CH, SPI_SPEED, SPI_B(0)|0|0|SPI_N(0)|0|0|SPI_UX(0)|SPI_PX(0)|SPI_MODE(0));
	ret = fd_spi;

	if(ret < 0)
	{
		printf("SPI initialization failed. error code: %d\n", ret);
		return ret;
	}
			
	ret = init_ad7176();
	if (ret < 0)
	{
	   printf("ad7176 initialisation failed\n");
	   return ret;
	}
	
	printf("Initialisation successful\n");
	return ret;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
	int global_crc_error_counter = 0;
	
int handle_ret(int ret)//very imortant to have. Somethimes comunications can crash (for example in case of psu impulses etc)
{
	int i;
	
	if(ret < 0)
	{
		if(ret ==  COMM_ERR)
		{
			printf("CRC_ERR\n");
			global_crc_error_counter++;
			
			if(global_crc_error_counter >= 10)//if N crc errors in row...
			{
				printf("CRC errors too often. Initiating again...\n");
				global_crc_error_counter = 0;
				return init_periph();
			}
		}			
		else if(ret ==  INVALID_VAL)
			printf("INVALID_VAL\n");
		else if(ret ==  COMM_ERR)
			printf("TIMEOUT\n");
		else
			printf("error code: %d\n", ret);		
	}

	return ret;
}
//////////////////////////////////////////////////////////////////////////////////
double adc_buffer[AVG_COUNT];
double *adc_buffer_ptr = adc_buffer;
//////////////////////////////////////////////////////////////////////////////////

//Start of MAIN()
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{	
	int debug;
	int ret = 0;
	int i;
	
	stats_type result_statistics;
	
	    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        printf("\n mutex init failed\n");
        return 1;
    }
	
	//ADC part.......................................................................
	if(init_periph() < 0)
	{
		printf("Init failed\n");
		return -1;
	}
		
   for(i = 0; i<1; i++)
   {
		usleep(500*1000);
		
	    set_ad7176_gpio(&ad7176_2, 1);//turn on gpio led
		
		ret = get_adc_data(&ad7176_2, CH0, adc_buffer_ptr, AVG_COUNT);
		handle_ret(ret);
		result_statistics = claculate_stats(adc_buffer_ptr, AVG_COUNT);
		printf("\nCH0 avg: %.8f\n", result_statistics.avg);
		printf("CH0 std: %.8f\n", result_statistics.std);
		printf("CH0 ENOB: %.8f\n", result_statistics.enob);
		
		ret = get_adc_data(&ad7176_2, CH1, adc_buffer_ptr, AVG_COUNT);
		handle_ret(ret);
		result_statistics = claculate_stats(adc_buffer_ptr, AVG_COUNT);
		printf("\nCH1 avg: %.8f\n", result_statistics.avg);
		printf("CH1 std: %.8f\n", result_statistics.std);
		printf("CH1 ENOB: %.8f\n", result_statistics.enob);
		
		set_ad7176_gpio(&ad7176_2, 0);//turn off gpio led
	}
	
	printf(".............................\n");
	//SERVER part..................................................................
	int socket_desc , client_sock , c;
    struct sockaddr_in server , client;
	
	char clntName[INET_ADDRSTRLEN];

    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
     
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(LISTEN_PORT);
     
    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");
     
    //Listen
    listen(socket_desc , 3);//3 slots

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
	pthread_t thread_id;
	
    while( (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) ) //<--blocking call here
    {
		puts("Connection accepted");
		
		if(inet_ntop(AF_INET,&client.sin_addr.s_addr,clntName,sizeof(clntName))!=NULL)
		   printf("client IP address and assigned port: %s%c%d\n", clntName,'/',ntohs(client.sin_port));
		else 
		   printf("Unable to get client address\n");

        if( pthread_create( &thread_id , NULL ,  connection_handler , (void*) &client_sock) < 0)
        {
            perror("could not create thread");
            return 1;
        }
         
        //Now join the thread , so that we dont terminate before the thread
        //pthread_join( thread_id , NULL);
		pthread_detach(thread_id); /* don't track it */
        puts("Handler assigned");
    }
     
    if (client_sock < 0)
    {
        perror("accept failed");
        return 1;
    }
	
	terminate:
	shutdown(socket_desc, SHUT_RDWR);
	spiClose(fd_spi);
	gpioTerminate();
	pthread_mutex_destroy(&lock);
	
	return 0;
}//END MAIN()

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
 * This will handle connection for each client
 * */
void *connection_handler(void *socket_desc)
{
    //Get the socket descriptor
    int sock = *(int*)socket_desc;
    int read_size;
    char *message , client_message[CLIENT_MSG_LENGTH];
     stats_type stats_ch0, stats_ch1;
	
	char double_string_buf[15];
	
	unsigned int timeout;
	int ret;
    //Send some messages to the client
    //message = "\nAD7176 ready. Send tpm to get tp reading\n\n";
    //write(sock , message , strlen(message));
	
    //Receive a message from client
    while( (read_size = recv(sock , client_message , CLIENT_MSG_LENGTH , 0)) > 0 )
    {
        //end of string marker
		client_message[read_size] = '\0';
		
		//Send the message back to client
		if(strcmp(client_message, "tpm") == 0)
		{			
			pthread_mutex_lock(&lock);//prevent other threads executing following code until unlocked 
				
				set_ad7176_gpio(&ad7176_2, 1);//turn on gpio led <-no spi reads in this function
				
				
				ret = get_adc_data(&ad7176_2, CH0, adc_buffer_ptr, AVG_COUNT);					
				handle_ret(ret);
				
				if(ret >= 0) //if ret was <0, send last correct value	
				{
					stats_ch0 = claculate_stats(adc_buffer_ptr, AVG_COUNT);
					
					if(global_crc_error_counter > 0)
					printf("at ch0: crc error count: %d. clearing\n", global_crc_error_counter);
				
					global_crc_error_counter = 0;
				}								
								
				sprintf(double_string_buf, "%u", (unsigned int)stats_ch0.avg);
				strcpy(client_message, double_string_buf);
				strcat(client_message, " ");
				sprintf(double_string_buf, "%u", (unsigned int)stats_ch0.std);
				strcat(client_message, double_string_buf);
				
				strcat(client_message, " ");
				
				ret = get_adc_data(&ad7176_2, CH1, adc_buffer_ptr, AVG_COUNT);
				handle_ret(ret);
				
				if(ret >= 0)
				{					
					stats_ch1 = claculate_stats(adc_buffer_ptr, AVG_COUNT);
					
					if(global_crc_error_counter > 0)
					printf("at ch1: crc error count: %d. clearing\n", global_crc_error_counter);
				
					global_crc_error_counter = 0;
				}
				
				sprintf(double_string_buf, "%u", (unsigned int)stats_ch1.avg);
				strcat(client_message, double_string_buf);
				strcat(client_message, " ");
				sprintf(double_string_buf, "%u", (unsigned int)stats_ch1.std);
				strcat(client_message, double_string_buf);
				
				set_ad7176_gpio(&ad7176_2, 0);//turn off gpio led	
				
			pthread_mutex_unlock(&lock);
			
			sent_response:
			write(sock , client_message , strlen(client_message) + 1);
			//clear the message buffer
			memset(client_message, 0, CLIENT_MSG_LENGTH);
		}
    }
     
    if(read_size == 0)
    {
        puts("Client disconnected");
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }
         
    return 0;
} 
