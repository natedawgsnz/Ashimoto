/**
 * Includes
 */
#include "MPU6050.h"



MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl) {
    this->setSleepMode(false);
    
    //Initializations:
    currentGyroRange = 0;
    currentAcceleroRange=0;
}

//--------------------------------------------------
//-------------------General------------------------
//--------------------------------------------------

//void MPU6050::reset(){
    
//}

void MPU6050::initialize(){
    deviceReset();
    setSleepMode(false);
    #define MPU6050_CLOCK_PLL_XGYRO 0x01
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setGyroRange(MPU6050_GYRO_RANGE_250);
    setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);

//    this->write(MPU6050_PWR_MGMT_1_REG, 0x21);    

}

bool MPU6050::deviceReset(void){
    this->write(MPU6050_PWR_MGMT_1_REG, 0x80);
    for(int i=0;i<50;i++){
        wait(0.1);
        if(!(this->read(MPU6050_PWR_MGMT_1_REG) & 0x80))return true;
    }
    return false;
}

char MPU6050::setClockSource(uint8_t clock){
    char temp;
    temp = this->read(MPU6050_PWR_MGMT_1_REG);
    
    temp &= 0xF8; //BitMask
    temp |= clock;
//    temp &= 0xAF; //Don't Sleep
    

    this->write(MPU6050_PWR_MGMT_1_REG, temp);    
    return temp;
}

bool MPU6050::write(char address, char data) {
    bool result = false; //unsuccess
    char temp[2];
    temp[0]=address;
    temp[1]=data;
    
    for(int i=0;i<10 && (result = connection.write(MPU6050_ADDRESS * 2,temp,2, true)); i++) wait(0.1);
    return result;
}

char MPU6050::read(char address) {
    char retval;
    connection.write(MPU6050_ADDRESS * 2, &address, 1, false);
    connection.read(MPU6050_ADDRESS * 2, &retval, 1, true);
    return retval;
}

int MPU6050::read(char address, char *data, int length) {
    int result;
    result = connection.write(MPU6050_ADDRESS * 2, &address, 1, false);
    if(result != 0) return 30;
    result = connection.read(MPU6050_ADDRESS * 2, data, length, true);
    if(result != 0) return 40;
}

char MPU6050::setSleepMode(bool state) {
    char temp;
    temp = this->read(MPU6050_PWR_MGMT_1_REG);
    if (state == true)
        temp |= 1<<MPU6050_SLP_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_SLP_BIT);
    if(this->write(MPU6050_PWR_MGMT_1_REG, temp)) return 'F';
    else return 'T';
}

bool MPU6050::testConnection( void ) {
    char temp;
    temp = this->read(MPU6050_WHO_AM_I_REG);
    return (temp == (MPU6050_ADDRESS & 0xFE));
}

void MPU6050::setBW(char BW) {
    char temp;
    BW=BW & 0x07;
    temp = this->read(MPU6050_CONFIG_REG);
    temp &= 0xF8;
    temp = temp + BW;
    this->write(MPU6050_CONFIG_REG, temp);
}

void MPU6050::setI2CBypass(bool state) {
    char temp;
    temp = this->read(MPU6050_INT_PIN_CFG);
    if (state == true)
        temp |= 1<<MPU6050_BYPASS_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_BYPASS_BIT);
    this->write(MPU6050_INT_PIN_CFG, temp);
}

//--------------------------------------------------
//----------------Accelerometer---------------------
//--------------------------------------------------
char MPU6050::mpu_set_accel_fsr(char range)
{
    char temp;
    range = range & 0x03;
    currentAcceleroRange = range;
    
    temp = this->read(MPU6050_ACCELERO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + (range<<3);
    
    char tempA[2];
    
    tempA[0]=MPU6050_ACCELERO_CONFIG_REG;
    tempA[1]=range;
    
    bool result = false; //unsuccess
    for(int i=0; (i<10 && (result = connection.write(MPU6050_ADDRESS*2, tempA, 2, true))); i++){
        wait_us(200);
    }
    //connection.write(MPU6050_ADDRESS*2, tempA, 2, true);
    return temp;
}

char MPU6050::setAcceleroRange( char range ) {
    char temp;
    range = range & 0x03;
    currentAcceleroRange = range;
    
    temp = this->read(MPU6050_ACCELERO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + (range<<3);
    this->write(MPU6050_ACCELERO_CONFIG_REG, temp);
    return temp;
}

int MPU6050::getAcceleroRawX( void ) {
    short retval;
    char data[2]={0x00,0x00};
    this->read(MPU6050_ACCEL_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) | data[1];
    return (int)retval;
}
    
int MPU6050::getAcceleroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getAcceleroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getAcceleroRaw( int *data ) {
    char temp[6];
    int result = 0;
    result = this->read(MPU6050_ACCEL_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]);
    data[1] = (int)(short)((temp[2]<<8) + temp[3]);
    data[2] = (int)(short)((temp[4]<<8) + temp[5]);
    return result;
}

bool MPU6050::getAccelero( float *data ) {
    int temp[3];
    if(this->getAcceleroRaw(temp))return true;
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_2G) {
        data[0]=(float)temp[0] / 16384.0 * 9.81;
        data[1]=(float)temp[1] / 16384.0 * 9.81;
        data[2]=(float)temp[2] / 16384.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_4G){
        data[0]=(float)temp[0] / 8192.0 * 9.81;
        data[1]=(float)temp[1] / 8192.0 * 9.81;
        data[2]=(float)temp[2] / 8192.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_8G){
        data[0]=(float)temp[0] / 4096.0 * 9.81;
        data[1]=(float)temp[1] / 4096.0 * 9.81;
        data[2]=(float)temp[2] / 4096.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_16G){
        data[0]=(float)temp[0] / 2048.0 * 9.81;
        data[1]=(float)temp[1] / 2048.0 * 9.81;
        data[2]=(float)temp[2] / 2048.0 * 9.81;
        }
    
    #ifdef DOUBLE_ACCELERO
        data[0]*=2;
        data[1]*=2;   
        data[2]*=2;
    #endif   
    return false;
}

//--------------------------------------------------
//------------------Gyroscope-----------------------
//--------------------------------------------------
char MPU6050::mpu_set_gyro_fsr(char range)
{
    char temp;
    range = range & 0x03;
    currentGyroRange = range;
    
    temp = this->read(MPU6050_GYRO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + (range<<3);
    
    char tempA[2];
    
    tempA[0]=MPU6050_GYRO_CONFIG_REG;
    tempA[1]=range;
    
    bool result = false;
    for(int i=0; (i<10 && (result = connection.write(MPU6050_ADDRESS*2, tempA, 2, true))); i++){
        wait_us(200);
    }
    //connection.write(MPU6050_ADDRESS*2, tempA, 2, true);
    return temp;
}

char MPU6050::setGyroRange( char range ) {
    char temp;
    currentGyroRange = range;
    range = range & 0x03;
    temp = this->read(MPU6050_GYRO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + range<<3;
    this->write(MPU6050_GYRO_CONFIG_REG, temp);
    return temp;
}

int MPU6050::getGyroRawX( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) | data[1];
    return (int)retval;
}
    
int MPU6050::getGyroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getGyroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

void MPU6050::getGyroRaw( int *data ) {
    char temp[6];
    this->read(MPU6050_GYRO_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]);
    data[1] = (int)(short)((temp[2]<<8) + temp[3]);
    data[2] = (int)(short)((temp[4]<<8) + temp[5]);
}

void MPU6050::getGyro( float *data ) {
    int temp[3];
    this->getGyroRaw(temp);
    if (currentGyroRange == MPU6050_GYRO_RANGE_250) {
        data[0]=(float)temp[0] / 7505.7;
        data[1]=(float)temp[1] / 7505.7;
        data[2]=(float)temp[2] / 7505.7;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_500){
        data[0]=(float)temp[0] / 3752.9;
        data[1]=(float)temp[1] / 3752.9;
        data[2]=(float)temp[2] / 3752.9;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_1000){
        data[0]=(float)temp[0] / 1879.3;;
        data[1]=(float)temp[1] / 1879.3;
        data[2]=(float)temp[2] / 1879.3;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_2000){
        data[0]=(float)temp[0] / 939.7;
        data[1]=(float)temp[1] / 939.7;
        data[2]=(float)temp[2] / 939.7;
        }
}
//--------------------------------------------------
//-------------------Temperature--------------------
//--------------------------------------------------
int MPU6050::getTempRaw( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_TEMP_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

float MPU6050::getTemp( void ) {
    float retval;
    retval=(float)this->getTempRaw();
    retval=(retval+521.0)/340.0+35.0;
    return retval;
}

