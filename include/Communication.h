#ifndef COMMUNICATION
#define COMMUNICATION

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define SPI_SPEED 500000
#define SPI_CH 0

#define SPI_MODE(x)  (((x) & 0b11) << 0)
#define SPI_PX(x)  (((x) & 0b111) << 2)
#define SPI_UX(x)  (((x) & 0b111) << 5)
#define SPI_A         (1 << 8)
#define SPI_W         (1 << 9)
#define SPI_N(x)  (((x) & 0b1111) << 10)
#define SPI_T        (1 << 14)
#define SPI_R        (1 << 15)
#define SPI_B(x)  (((x) & 0b111111) << 16)

extern int fd_spi;

void pabort(const char *s);
int32_t SPI_Read(uint8_t slave_select_id, uint8_t* buf, uint32_t buflength);
int32_t SPI_Write(uint8_t slave_select_id, uint8_t* buf, uint32_t buflength);

#endif 