#ifndef GPS_H
#define GPS_H

void InitGps(void);
void USART0WriteChar(unsigned char data);

uint8_t CheckGps(void);
void GetGpsTime(unsigned char ARRAY[]);
void GetGpsDate(unsigned char ARRAY[]);
void GetGpsLatitude(unsigned char ARRAY[]);
void GetGpsLongitude(unsigned char ARRAY[]);

// void TempFunctionGpsHandle(void);

#endif // GPS_H