#ifndef CONSTANTES_H
#define CONSTANTES_H

#define    IMU_ADAFRUIT               0x28
#define    MPU9250_ADDRESS            0x69
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G         0x00
#define    ACC_FULL_SCALE_4_G         0x08
#define    ACC_FULL_SCALE_8_G         0x10
#define    ACC_FULL_SCALE_16_G        0x18

#define VEXT_PIN 36

const int RX_PIN = 35;
const int TX_PIN = 12;

const int chipSelectPin = 26; // Pin para el Chip Select (CS) de la tarjeta SD
const int sckPin = 38;        // Pin para el Serial Clock (SCK)
const int misoPin = 39;       // Pin para el Master In Slave Out (MISO)
const int mosiPin = 18;       // Pin para el Master Out Slave In (MOSI)

//const int led = 25; // Pin para la luz azul
const int bluePin = 7; // Pin para la luz azul
const int greenPin = 6; // Pin para la luz verde
const int redPin = 5;   // Pin para la luz roja

const int datos = 10000; 


#endif
