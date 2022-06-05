#ifndef GPS_H
#define GPS_H

void GpsInit(void);
void USART0WriteChar(unsigned char data);

uint8_t CheckGps(void);
void GetGpsTime(unsigned char ARRAY[]);
void GetGpsDate(unsigned char ARRAY[]);
void GetGpsLatitude(unsigned char ARRAY[]);
void GetGpsLongitude(unsigned char ARRAY[]);

#endif // GPS_H