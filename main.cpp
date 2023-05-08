/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include "math.h"
#include "Kalman.h"

I2C i2c(p28, p27); //SDA = p9, SCL = p10 
RF24 NRF24L01(p5, p6, p7, p15, p8);     //PinName mosi, PinName miso, PinName sck, PinName _cepin, PinName _csnpin
Serial pc(USBTX, USBRX);
Ticker loops;
Ticker RF_loop;
Kalman KalmanX; // Create the Kalman instances
Kalman KalmanY;
PwmOut Motor1(p24);
PwmOut Motor2(p23);
PwmOut Motor3(p22);
PwmOut Motor4(p21);
Timer timer;

#define MPU9250_Addr0 0x68 << 1 //0b1101000
#define MPU9250_Addr1 0x69 << 1 //0b1101001
#define MPU9250_WHOAMI 0x75 //WHOAMI
#define MPU9250_ACCX_H 0x3B
#define MPU9250_ACCX_L 0x3C
#define MPU9250_ACCY_H 0x3D
#define MPU9250_ACCY_L 0x3E
#define MPU9250_ACCZ_H 0x3F
#define MPU9250_ACCZ_L 0x40
#define MPU9250_GYROX_H 0x43
#define MPU9250_GYROX_L 0x44
#define MPU9250_GYROY_H 0x45
#define MPU9250_GYROY_L 0x46
#define MPU9250_GYROZ_H 0x47
#define MPU9250_GYROZ_L 0x48
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_GYRO_SCALE_FACTOR 16.4 //Scale Factor +-2000
#define MPU9250_ACC_SCALE_FACTOR 2048 //Scale Factor +-16

#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define PWR_MGMT_2 0x6C
#define INT_PIN_CFG 0x37
#define PI 3.1415

const uint64_t pipe = 0x1212121212LL;
int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int16_t PITCH_rate = 0, ROLL_rate = 0, YAW_rate = 0, THROTTLE_rate = 0;
int16_t PITCH_erate = 0, ROLL_erate = 0, YAW_erate = 0, THROTTLE_erate = 0;
float ComFilter_Roll, ComFilter_Pitch;
int8_t ackData[30];
int8_t flip1 = 1;
int BUT1, BUT2;
int rf_fail_count = 0;
uint32_t tick = 0;
int loop_flag1 = 0, loop_flag2 = 0;
float PWM1_C = 0, PWM2_C = 0, PWM3_C = 0, PWM4_C = 0;
float PWM1_PD = 0, PWM2_PD = 0, PWM3_PD = 0, PWM4_PD = 0;
float PWM1_SUM = 0, PWM2_SUM = 0, PWM3_SUM = 0, PWM4_SUM = 0;
float Roll_gyro = 0, Pitch_gyro = 0, Yaw_gyro = 0;
uint8_t GYRO_CONFIG_DATA = 0b00011000; //FS_SEL = 3
uint8_t ACC_CONFIG_DATA = 0b00011000; //AFS_SEL = 3
uint8_t BANDWIDTH_GYRO = 0b00000011; //Bandwidth of gyro: 0x03: 41Hz 1kHz
int16_t ACC_X, ACC_Y, ACC_Z;
int16_t GYRO_X, GYRO_Y, GYRO_Z;
float ACCX, ACCY, ACCZ;
float GYROX, GYROY, GYROZ;
uint8_t rt = 0;
uint8_t high_byte = 0x22;
uint8_t low_byte = 0x34;
uint16_t result_tbyte;
float dt = 0.01;
float pre_time = 0, now_time = 0;
float Kal_Roll = 0, Kal_Pitch = 0;
float angle_error = 0, pre_error = 0, pre_angle = 0;
float target = 0;
float Bias_GYROX, Bias_GYROY, Bias_GYROZ;
float Bias_ACCX, Bias_ACCY, Bias_ACCZ;
float Offset_GYROX, Offset_GYROY, Offset_GYROZ;
float Offset_ACCX, Offset_ACCY, Offset_ACCZ;
float SUM_GYROX, SUM_GYROY, SUM_GYROZ, SUM_ACCX, SUM_ACCY, SUM_ACCZ;
float Roll_acc, Pitch_acc;
float Roll_acc_off, Pitch_acc_off;
float NewG_Roll, NewG_Pitch, NewA_Roll, NewA_Pitch; 
float OldG_Roll = 0, OldA_Pitch = 0, OldG_Pitch = 0, OldA_Roll = 0; 
float tau = 1.0/4.0;
int a = 0;

uint8_t readbyte(uint8_t Address, uint8_t reg_addr);
float PIDcontrol(float angle, float Kp, float Kd);
float scaler(float PWM, float MAX, float MIN);

int constrain_int16(int16_t x, int min, int max) {
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

void RF_READ() {
    if (NRF24L01.available())
    {
        NRF24L01.read(recv, 10);

        // 스케일링 다른데서 곱하기
        ROLL = *(int16_t*)(&recv[0])- 3; //ROLL = - ROLL;           offset = 0
        PITCH = *(int16_t*)(&recv[2])- 7;//flip pitch and roll      offset = 6
        YAW = *(int16_t*)(&recv[4]) - 0; //                         offset = 2
        THROTTLE = *(int16_t*)(&recv[6]) - 0; //                    offset = 0
        BUT1 = recv[8];
        BUT2 = recv[9]; //should hold value here
        pc.printf("\r RF_READ : %d, %d, %d, %d \n\r", (int)ROLL, (int)PITCH, (int)YAW, (int)THROTTLE);
        rf_fail_count = 0;
    }

    else
    {
        rf_fail_count = rf_fail_count + 1;
                
        //printf(" rf_fail_count : %d\n\r", rf_fail_count);
        
        if (rf_fail_count >= 20 && rf_fail_count < 100)
        {
            printf(" rf_fail_count : %d\n\r", rf_fail_count);
            THROTTLE = THROTTLE - 2;
            THROTTLE = constrain_int16(THROTTLE, 0, 1023);
        }
        if (rf_fail_count >= 50)
        {
            THROTTLE = 0;
        }
        if (rf_fail_count >= 100)
        {
            rf_fail_count = 100;
        }
    }
}

uint8_t readbyte(uint8_t Address, uint8_t reg_addr)  {
    char reg_buff[2];
    char recv_buff[2];
    uint8_t ret;

    reg_buff[0] = reg_addr;   //레지스터
    i2c.write(Address, reg_buff, 1, 0);
    i2c.read(Address, recv_buff, 1, 1);

    ret = recv_buff[0];
    return recv_buff[0];   //ret = recv_buff[0]; return ret;
}

void SetConfig() {
    char cmd[2];
    char receive[2];
    uint8_t high_byte = 0x22;
    uint8_t low_byte = 0x34;
    uint16_t result_tbyte;
    //Who am I
    cmd[0] = MPU9250_WHOAMI;
    i2c.write(MPU9250_Addr0, cmd, 1, 0);     // cmd[0] 저장된 데이터를 보낸다.
    rt = i2c.read(MPU9250_Addr0, receive, 1, 1);
    //Gyro scale factor
    cmd[0] = MPU9250_GYRO_CONFIG;
    cmd[1] = GYRO_CONFIG_DATA;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //Acc scale factor
    cmd[0] = MPU9250_ACCEL_CONFIG;
    cmd[1] = ACC_CONFIG_DATA;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //Acc2
    cmd[0] = MPU9250_ACCEL_CONFIG2;
    cmd[1] = 0x05;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //Bandwidth of gyro
    cmd[0] = CONFIG;
    cmd[1] = BANDWIDTH_GYRO;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //SMPLRT_DIV(Controll sample rate)
    cmd[0] = SMPLRT_DIV; // Sample rate = Internal SR/(1+SMPLRT_DIV) 
    cmd[1] = 0x04;       // When SMPLRT_DIV=4, Sample rate = 200Hz (Internal SR = 1kHz)
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //PWR_MGMT_2
    cmd[0] = PWR_MGMT_2;
    cmd[1] = 0x00;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    //INT_PIN_CFG
    cmd[0] = INT_PIN_CFG;
    cmd[1] = 0x22;
    i2c.write(MPU9250_Addr0, cmd, 2, 0);
    /*result_tbyte = (uint16_t)high_byte << 8
    result_tbyte += low_byte; or result_tbyte |= low_byte; ↓랑 결과 같음*/
    result_tbyte = high_byte; //result_tbyte = (uint16_t)high_byte << 8; -> high는 원래 1바이트 짜리지만 2바이트로 가정한뒤 <<8비트 밀어줘라
    result_tbyte = result_tbyte << 8;
    result_tbyte += low_byte;
}

void GetAccGyro() {
    //Acc_x
    ACC_X = readbyte(MPU9250_Addr0, MPU9250_ACCX_H);
    ACC_X = ACC_X << 8;
    ACC_X += readbyte(MPU9250_Addr0, MPU9250_ACCX_L);
    ACCX = (float)ACC_X/MPU9250_ACC_SCALE_FACTOR;
        
    //Acc_y
    ACC_Y = readbyte(MPU9250_Addr0, MPU9250_ACCY_H);
    ACC_Y = ACC_Y << 8;
    ACC_Y += readbyte(MPU9250_Addr0, MPU9250_ACCY_L);
    ACCY = (float)ACC_Y/MPU9250_ACC_SCALE_FACTOR  ;

    //Acc_z
    ACC_Z = readbyte(MPU9250_Addr0, MPU9250_ACCZ_H);
    ACC_Z = ACC_Z << 8;
    ACC_Z += readbyte(MPU9250_Addr0, MPU9250_ACCZ_L);
    ACCZ = (float)ACC_Z/MPU9250_ACC_SCALE_FACTOR;// AFS_SEL=3은 +-16G >> Sensitivity Scale Factor = 2,048 LSB/g      
            
    //Gyro_x
    GYRO_X = readbyte(MPU9250_Addr0, MPU9250_GYROX_H);
    GYRO_X = GYRO_X << 8;
    GYRO_X += readbyte(MPU9250_Addr0, MPU9250_GYROX_L);
    GYROX = (float)GYRO_X/MPU9250_GYRO_SCALE_FACTOR;

    //Gyro_y
    GYRO_Y = readbyte(MPU9250_Addr0, MPU9250_GYROY_H);
    GYRO_Y = GYRO_Y << 8;
    GYRO_Y += readbyte(MPU9250_Addr0, MPU9250_GYROY_L);
    GYROY = (float)GYRO_Y/MPU9250_GYRO_SCALE_FACTOR;

    //Gyro_z
    GYRO_Z = readbyte(MPU9250_Addr0, MPU9250_GYROZ_H);
    GYRO_Z = GYRO_Z << 8;
    GYRO_Z += readbyte(MPU9250_Addr0, MPU9250_GYROZ_L);
    GYROZ = (float)GYRO_Z/MPU9250_GYRO_SCALE_FACTOR;
}

void control_loop(void) {
    RF_READ();
}

void timer_1ms(void) {
    tick++;
}

float PIDcontrol(float angle, float gyro, float Kp, float Kd) {
    float Kp_term, Ki_term, Kd_term;   // p항, i항, d항
    float PID;
    float I_error, D_error;

    angle_error = target - angle;   // 오차 = 목표치-현재값 angle: filtered angle
    Kp_term = Kp * angle_error;         // p항 = Kp*오차

    //I_error += angle_error * dt;             // 오차적분 += 오차*dt
    //Ki_term = Ki * I_error;        // i항 = Ki*오차적분

    //D_error = (angle_error-pre_error)/dt;  // 오차미분 = (현재오차-이전오차)/dt float gyro,
    D_error = gyro;
    Kd_term = -Kd * D_error;      // d항 = Kd*오차미분
//미분항은 외력에 의한 변경이므로 setpoint(target)에 의한 내부적인 요소 제외(-추가)
    //pre_error = angle_error;   // 현재오차를 이전오차로 pre_angle = angle

    PID = Kp_term + Kd_term;  // 제어량 = p항+d항
    return PID;
}

float scaler(float PWM_S, float MAX, float MIN) {
    float PWM_Scale;

    PWM_Scale = (PWM_S - MIN)/(MAX - MIN);
    return PWM_Scale;
}

void pwm_motor(void) {
    float PD_roll, PD_pitch, PD_yaw;

    PD_roll = PIDcontrol(ComFilter_Roll, Offset_GYROY, 47.0, 28.0); //Angle, Kp, Offset_GYROXOffset_GYROY
    PD_pitch = PIDcontrol(ComFilter_Pitch, Offset_GYROX, 48.0, 28.0);
    PD_yaw = PIDcontrol(Yaw_gyro, Offset_GYROZ, 0, 15.0);

    if(THROTTLE > 10) {
         PWM1_C = (float)THROTTLE/1159 - (float)ROLL/251 + (float)PITCH/251 + (float)YAW/251; //throttle:0~1000 -> 실제 1159
         PWM2_C = (float)THROTTLE/1159 - (float)ROLL/251 - (float)PITCH/251 - (float)YAW/251; //pwm:0~100
         PWM3_C = (float)THROTTLE/1159 + (float)ROLL/251 - (float)PITCH/251 + (float)YAW/251; //roll,pitch,yaw: +-256 
         PWM4_C = (float)THROTTLE/1159 + (float)ROLL/251 + (float)PITCH/251 - (float)YAW/251; //-> 실제 +-251
        //1912=1159+(251*3)-> max throttle: 0.6 & 1410 -> max Throttle: 0.8
        PWM1_C = PWM1_C * 0.8;
        PWM2_C = PWM2_C * 0.8;
        PWM3_C = PWM3_C * 0.8;
        PWM4_C = PWM4_C * 0.8;

        PWM1_PD = (PD_roll + PD_pitch - PD_yaw)/1000; 
        PWM2_PD = (PD_roll - PD_pitch + PD_yaw)/1000; 
        PWM3_PD = (- PD_roll - PD_pitch - PD_yaw)/1000; 
        PWM4_PD = (- PD_roll + PD_pitch + PD_yaw)/1000;

        PWM1_PD = PWM1_PD * 0.2;
        PWM2_PD = PWM2_PD * 0.2;
        PWM3_PD = PWM3_PD * 0.2;
        PWM4_PD = PWM4_PD * 0.2;

        PWM1_SUM = PWM1_C + PWM1_PD;
        PWM2_SUM = PWM2_C + PWM2_PD;
        PWM3_SUM = PWM3_C + PWM3_PD;
        PWM4_SUM = PWM4_C + PWM4_PD;

        if(PWM1_SUM < 0.001) {PWM1_SUM = 0;}
        else if(PWM1_SUM > 1.0) {PWM1_SUM = 1;}
        if(PWM2_SUM < 0.001) {PWM2_SUM = 0;}
        else if(PWM2_SUM > 1.0) {PWM2_SUM = 1;}
        if(PWM3_SUM < 0.001) {PWM3_SUM = 0;}
        else if(PWM3_SUM > 1.0) {PWM3_SUM = 1;}
        if(PWM4_SUM < 0.001) {PWM4_SUM = 0;}
        else if(PWM4_SUM > 1.0) {PWM4_SUM = 1;}

        Motor1 = PWM1_SUM;
        Motor2 = PWM2_SUM;
        Motor3 = PWM3_SUM;
        Motor4 = PWM4_SUM;
        //printf("C: %f %f %f %f \r\n", PWM1_C, PWM2_C, PWM3_C, PWM4_C);
        //printf("PD: %f %f %f %f \r\n", PWM1_PD, PWM2_PD, PWM3_PD, PWM4_PD);
        pc.printf("SUM: %f %f %f %f \r\n", PWM1_SUM, PWM2_SUM, PWM3_SUM, PWM4_SUM);
    }
    else {
        Motor1 = 0;
        Motor2 = 0;
        Motor3 = 0;
        Motor4 = 0;
    }
}

int main()
{
    pc.baud(115200);
    i2c.frequency(400000);
    Motor1.period(0.0001); //10kHz
    Motor2.period(0.0001);
    Motor3.period(0.0001);
    Motor4.period(0.0001);
    SetConfig();
    Motor1 = 0;
    Motor2 = 0;
    Motor3 = 0;
    Motor4 = 0;
    int num = 1;

    // RF 모듈 초기화
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(50);  // set channel 10 20 30
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2, 4); //1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
    pc.printf("Code Start\r\n");
    loops.attach_us(&control_loop, 2500); // ticker 2500
    loops.attach_us(&timer_1ms, 1000);       // 1ms period
    RF_loop.attach_us(&RF_READ, 10000);
    while (true) {
        
        if(tick >= 10)      // 10ms loop  dt = 0.01 10Hz
        {
            //우리 코드
            GetAccGyro();
            //printf("Raw ACC =>  ACC_X: %f, ACC_Y: %f, ACC_Z: %f \r\n", ACCX, ACCY, ACCZ);    
            //printf("Raw GYRO =>  GYRO_X: %f, GYRO_Y: %f, GYRO_Z: %f \r\n", GYROX, GYROY, GYROZ);

            if(loop_flag1 == 0){
                //Offset control
                SUM_GYROX += GYROX;
                SUM_GYROY += GYROY;
                SUM_GYROZ += GYROZ;
                SUM_ACCX += ACCX;
                SUM_ACCY += ACCY;
                SUM_ACCZ += ACCZ;
                printf("Adding ACC Bias... %d \r\n", num++);
                //printf("Adding GYRO Bias...%f %f %f\r\n", SUM_GYROX, SUM_GYROY, SUM_GYROZ);
                printf("%f %f %f\r\n", SUM_ACCX, SUM_ACCY, SUM_ACCZ);
            
                if(num == 301) {
                    printf("Find offset!\n Output revised data\r\n");
                    Bias_GYROX = SUM_GYROX/300;
                    Bias_GYROY = SUM_GYROY/300;
                    Bias_GYROZ = SUM_GYROZ/300;
                    Bias_ACCX = SUM_ACCX/300;
                    Bias_ACCY = SUM_ACCY/300;
                    Bias_ACCZ = SUM_ACCZ/300;
                    loop_flag1 = 1; // finish initialize
                }
            }
            else {
                Offset_GYROX = (GYROX - Bias_GYROX)*(-1);
                Offset_GYROY = (GYROY - Bias_GYROY)*(-1);
                Offset_GYROZ = (GYROZ - Bias_GYROZ)*(-1);
                Offset_ACCX = ACCX - Bias_ACCX;
                Offset_ACCY = (ACCY - Bias_ACCY)*(-1);
                Offset_ACCZ = ACCZ - Bias_ACCZ + 1;
                //printf("Offset GYRO:  GYRO_X: %f, GYRO_Y: %f, GYRO_Z: %f \r\n", Offset_GYROX, Offset_GYROY, Offset_GYROZ);
                //printf("Offset ACC:  ACC_X: %f, ACC_Y: %f, ACC_Z: %f \r\n", Offset_ACCX, Offset_ACCY, Offset_ACCZ);
            
                //Calculate Euler Angle     
                Roll_gyro = Roll_gyro + Offset_GYROY*dt;
                Pitch_gyro = Pitch_gyro + Offset_GYROX*dt;
                Yaw_gyro = Yaw_gyro + Offset_GYROZ*dt;
                Roll_acc = atan(ACCX/sqrt(ACCY*ACCY+ACCZ*ACCZ))*180.0/PI; //asin(ACCX)*180.0/PI;
                Pitch_acc =  atan(ACCY/sqrt(ACCX*ACCX+ACCZ*ACCZ))*180.0/PI; //atan(ACCY/ACCZ)*180.0/PI;
                Roll_acc_off = atan(Offset_ACCX/sqrt(Offset_ACCY*Offset_ACCY+Offset_ACCZ*Offset_ACCZ))*180.0/PI;
                Pitch_acc_off = atan(Offset_ACCY/sqrt(Offset_ACCX*Offset_ACCX+Offset_ACCZ*Offset_ACCZ))*180.0/PI;
                //printf("Euler: Euler_X: %f, Euler_Y: %f \r\n", Roll_acc, Pitch_acc);
                //printf("Euler: Euler_X: %f, Euler_Y: %f \r\n", Roll_acc_off, Pitch_acc_off);
            
                //Complementary Filter
                NewG_Roll = (1-dt/tau)*OldG_Roll + dt*Offset_GYROX;
                NewA_Roll = (1-dt/tau)*OldA_Roll + dt/tau*Roll_acc_off;
                NewG_Pitch = (1-dt/tau)*OldG_Pitch + dt*Offset_GYROY;
                NewA_Pitch = (1-dt/tau)*OldA_Pitch + dt/tau*Pitch_acc_off;
        
                OldG_Roll = NewG_Roll;
                OldA_Roll = NewA_Roll;
                OldG_Pitch = NewG_Pitch;
                OldA_Pitch = NewA_Pitch;

                ComFilter_Roll = (OldG_Roll + OldA_Roll);
                ComFilter_Pitch = (OldG_Pitch + OldA_Pitch);
                //Kalman Filter
                Kal_Roll = KalmanX.getAngle(Roll_acc_off, Offset_GYROY, dt); // Calculate the angle using a Kalman filter
                Kal_Pitch = KalmanY.getAngle(Pitch_acc_off, Offset_GYROX, dt);
                pwm_motor();

                //printf("Roll: %f, Pitch: %f\r\n", OldG_Roll, OldA_Roll);
                //printf("Filterd Euler angle:  Roll: %f, Pitch: %f Yaw: %f\r\n", ComFilter_Roll, ComFilter_Pitch, Yaw_gyro);
                //control_loop();
                //printf("%f %f \r\n", Roll_gyro, Pitch_gyro);
                // 우리 코드 끝
                //printf("ROLL: %f PITCH: %f\r\n", ComFilter_Roll, ComFilter_Pitch);
            }
            tick = 0;
        }
    }
}