#include "CONSTANTES.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Configuración del GPS
HardwareSerial gpsSerial(1); // Utilizar UART1
TinyGPSPlus gps;

uint8_t Buf_BNO055[19];
uint8_t Buf_BNO055_TEM[1];
uint8_t quat[8];
uint8_t Buf_MPU[16];
uint8_t med[1];

//Buffer de lectura de identificación de IMUs
int16_t Buf_MPU_READ[10];
int16_t Buf_BNO055_READ[14];
int sampleCount = 0;
int fileCount = 0;

volatile int interruptCounter;
int totalInterruptCounter;

bool tic_int;
int OK_INT = 0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long previousMillis = 0;
const long interval = 5000;

float temp, hum;
float K_QUAT; 

int cont = 0;
int estadoComu = 0;

File dataFile; // Variable para el archivo de datos
int numero = 1; // Inicializar el número a escribir en el archivo CSV

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  OK_INT = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
}


//Funcion de lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}
// Funcion de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void ReadMPU9250() {
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf_MPU);

  // Convertir registros acelerometro
  Buf_MPU_READ[0] = (Buf_MPU[0] << 8 | Buf_MPU[1]); //ACC X
  Buf_MPU_READ[1] = (Buf_MPU[2] << 8 | Buf_MPU[3]); //ACC Y
  Buf_MPU_READ[2] = (Buf_MPU[4] << 8 | Buf_MPU[5]); //ACC Z

  // Convertir registros giroscopio
  Buf_MPU_READ[3] = (Buf_MPU[8] << 8 | Buf_MPU[9]) ;  //GYRO X
  Buf_MPU_READ[4] = (Buf_MPU[10] << 8 | Buf_MPU[11]); //GYRO Y
  Buf_MPU_READ[5] = (Buf_MPU[12] << 8 | Buf_MPU[13]); //GYRO Z

  // ---  Lectura del magnetometro ---
  //  Serial.print("aca voy");
  //  uint8_t ST1;
  //  do
  //  {
  //    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  //    //Serial.print(ST1);
  //  } while (!(ST1 & 0x01));
  //  Serial.print("abc");

  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Convertir registros magnetometro
  Buf_MPU_READ[6] = (Mag[1] << 8 | Mag[0]); // MAGNE X
  Buf_MPU_READ[7] = (Mag[3] << 8 | Mag[2]); // MAGNE Y
  Buf_MPU_READ[8] = (Mag[5] << 8 | Mag[4]); // MAGNE Z

  //Temperatura
  Buf_MPU_READ[9] = (Buf_MPU[6] << 8 | Buf_MPU[7]);
}

void ReadBNO055() {
  I2Cread(IMU_ADAFRUIT , 0x08, 18, Buf_BNO055);
  I2Cread(IMU_ADAFRUIT , 0x20, 8, quat);
  I2Cread(IMU_ADAFRUIT , 0x34, 1, Buf_BNO055_TEM);

  // Convertir registros acelerometro
  Buf_BNO055_READ[0] = (Buf_BNO055[1] << 8 | Buf_BNO055[0]);
  Buf_BNO055_READ[1] = (Buf_BNO055[3] << 8 | Buf_BNO055[2]);
  Buf_BNO055_READ[2] = Buf_BNO055[5] << 8 | Buf_BNO055[4];

  // ---  Lectura del magnetometro ---
  Buf_BNO055_READ[3] = (Buf_BNO055[7] << 8 | Buf_BNO055[6]);
  Buf_BNO055_READ[4] = (Buf_BNO055[9] << 8 | Buf_BNO055[8]);
  Buf_BNO055_READ[5] = (Buf_BNO055[11] << 8 | Buf_BNO055[10]);

  // Convertir registros giroscopio
  Buf_BNO055_READ[6] = (Buf_BNO055[13] << 8 | Buf_BNO055[12]);
  Buf_BNO055_READ[7] = (Buf_BNO055[15] << 8 | Buf_BNO055[14]);
  Buf_BNO055_READ[8] = (Buf_BNO055[17] << 8 | Buf_BNO055[16]);

  //Lectura Quaterniones
  Buf_BNO055_READ[9] = (quat[1] << 8 | quat[0]);
  Buf_BNO055_READ[10] = (quat[3] << 8 | quat[2]);
  Buf_BNO055_READ[11] = (quat[5] << 8 | quat[4]);
  Buf_BNO055_READ[12] = (quat[7] << 8 | quat[6]);

  //Lectura Temperatura
  Buf_BNO055_READ[13] = Buf_BNO055_TEM[0];
}

void setup() {
  // Inicializar la comunicación serial
  Wire.begin(48, 20); //Scan OLED's I2C address via I2C0
  Wire.setClock(100000);
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Configurar el puerto serial para el GPS

  // Inicializar la comunicación SPI
  SPI.begin(sckPin, misoPin, mosiPin, chipSelectPin);

  // Inicializar la tarjeta SD
  if (!SD.begin(chipSelectPin)) {
    Serial.println("Error al inicializar la tarjeta SD.");
    return;
  }

  Serial.println("Tarjeta SD inicializada correctamente.");

  // Abrir o crear un archivo CSV llamado "data.csv"
  dataFile = SD.open("/dataTest2.csv", FILE_WRITE);
  
  // Verificar si el archivo se abrió correctamente
  if (!dataFile) {
    Serial.println("Error al abrir el archivo data.csv");
    return;
  }

  Serial.println("Archivo data.csv abierto correctamente.");

    //******BNO055******
  //I2CwriteByte(IMU_ADAFRUIT , 0x3D, 0x07);
  I2CwriteByte(IMU_ADAFRUIT , 0x07, 0x01); // cambia map page 1
  I2CwriteByte(IMU_ADAFRUIT , 0x0B, 0x00);
  I2CwriteByte(IMU_ADAFRUIT , 0x0A, 0x38);
  I2CwriteByte(IMU_ADAFRUIT , 0x07, 0x00);
  I2CwriteByte(IMU_ADAFRUIT , 0x3D, 0x0C); //Configura modo de operacion
  I2CwriteByte(IMU_ADAFRUIT , 0x3E, 0x00); //Configuracion power mode
  I2CwriteByte(IMU_ADAFRUIT , 0x09, 0x7D); //Configuracion power mode magnetometro 0x07D o 0x06D

  //******MPU 9250******

  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);  //Habilita Interrupciones
  I2CwriteByte(MPU9250_ADDRESS, 0x38, 0x01);
  I2CwriteByte(MPU9250_ADDRESS, 0x1a, 0x01);

  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 0x1c, ACC_FULL_SCALE_2_G);
  // Configurar giroscopio
  I2CwriteByte(MPU9250_ADDRESS, 0x1B, GYRO_FULL_SCALE_2000_DPS);

  //0x09 = 100 Hz,0x13 = 50 Hz, 0x27 = 25 Hz, 0x31 = 20 Hz
  I2CwriteByte(MPU9250_ADDRESS, 0x19, 0x13);
  I2CwriteByte(MPU9250_ADDRESS, 0x6c, 0x00);   //Permite desactivar ejes del giroscopio y aceletometro
  I2CwriteByte(MPU9250_ADDRESS, 0x6b, 0x00);

  //CONFIGURACION MAGNETOMETRO
  I2CwriteByte(MAG_ADDRESS, 0x0B, 0x01);
  I2CwriteByte(MAG_ADDRESS, 0x0C, 0x40);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x12);
  I2CwriteByte(MAG_ADDRESS, 0x0F, 0x00);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

  pinMode(redPin, OUTPUT);   // Configurar el pin de la luz roja como salida
  pinMode(bluePin, OUTPUT);   // Configurar el pin de la luz azul como salida
  pinMode(greenPin, OUTPUT); 
  
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  
}

void loop() {
  if (OK_INT) {
   
    if (sampleCount < datos) {
      // Lecturas IMUs
      ReadBNO055();
      ReadMPU9250();
      
      // Escritura en SD
      // ACELERACION BNO055
      dataFile.print(Buf_BNO055_READ[0]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[1]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[2]);
      dataFile.print(",");

      // CAMPO MAGNETICO BNO055
      dataFile.print(Buf_BNO055_READ[3]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[4]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[5]);
      dataFile.print(",");

      // VELOCIDAD ANGULAR BNO055
      dataFile.print(Buf_BNO055_READ[6]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[7]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[8]);
      dataFile.print(",");

      // QUATERNIONES BNO055
      dataFile.print(Buf_BNO055_READ[9]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[10]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[11]);
      dataFile.print(",");
      dataFile.print(Buf_BNO055_READ[12]);
      dataFile.print(",");
     
      // ACELERACION MPU9250
      dataFile.print(Buf_MPU_READ[0]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[1]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[2]);
      dataFile.print(",");

      // GIROSCOPIO MPU9250
      dataFile.print(Buf_MPU_READ[3]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[4]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[5]);
      dataFile.print(",");

      // CAMPO MAGNETICO MPU9250
      dataFile.print(Buf_MPU_READ[6]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[7]);
      dataFile.print(",");
      dataFile.print(Buf_MPU_READ[8]);

      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, LOW);

//      Serial.print(sampleCount);
//      Serial.print(",");
//      printMPU9250();
      printBNO055();   
       
      while (gpsSerial.available() > 0) {
         gps.encode(gpsSerial.read());
      }
      
    if (gps.location.isUpdated()) {
      
      Serial.print("Latitud: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", Longitud: ");
      Serial.println(gps.location.lng(), 6);

      dataFile.print(",");
      dataFile.print(gps.location.lat(), 6);
      dataFile.print(",");
      dataFile.println(gps.location.lng(), 6);
    }
    else {
      dataFile.print(",");
      dataFile.print(0);
      dataFile.print(",");
      dataFile.println(0);
      }                                                                        
    }
    
    if (sampleCount == datos) {
      dataFile.close();
    }

    if (sampleCount > datos) {
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, HIGH);
    }
    
    sampleCount++;
    OK_INT = 0;
 }

}


//FUNCIONES
void printMPU9250() {
  // ACELEROMETRO
//      Serial.print(Buf_MPU_READ[0]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[1]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[2]);
//      Serial.print(",");

  // GIROSCOPIO
//      Serial.print(Buf_MPU_READ[3]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[4]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[5]);
//      Serial.print(",");

  // MAGNETOMETRO
//      Serial.print(Buf_MPU_READ[6]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[7]);
//      Serial.print(",");
//      Serial.print(Buf_MPU_READ[8]);
//      Serial.println("");
}

void printBNO055() {
      // ACELEROMETRO
//      Serial.print(Buf_BNO055_READ[0]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[1]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[2]);
//      Serial.print(",");
//    
//      // MAGNETOMETRO
//      Serial.print(Buf_BNO055_READ[3]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[4]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[5]);
//      Serial.print(",");
//    
//      // GIROSCOPIO
//      Serial.print(Buf_BNO055_READ[6]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[7]);
//      Serial.print(",");
//      Serial.print(Buf_BNO055_READ[8]);
//      Serial.print(",");
    
      //Quaterniones
      K_QUAT = sqrt(pow(Buf_BNO055_READ[ 9], 2) +
                    pow(Buf_BNO055_READ[10], 2) +
                    pow(Buf_BNO055_READ[11], 2) +
                    pow(Buf_BNO055_READ[12], 2));
              
      Serial.print(Buf_BNO055_READ[9] / K_QUAT);
      Serial.print(",");
      Serial.print(Buf_BNO055_READ[10] / K_QUAT);
      Serial.print(",");
      Serial.print(Buf_BNO055_READ[11] / K_QUAT);
      Serial.print(",");
      Serial.println(Buf_BNO055_READ[12] / K_QUAT);
}
