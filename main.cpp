#include "mbed.h"
#include "PS3.h"
// #include "QEI.h"
#include "PIDcontroller.h"
#include "iostream"
#include "sstream"
#include "iomanip"
#include "BNO055.h"
#include <cmath>
using namespace std;


#define LEFT_FRONT 0x22
#define RIGHT_FRONT 0x14
#define LEFT_BACK 0x42
#define RIGHT_BACK 0x60
#define FAN 0x40

RawCAN      can(PB_5, PB_6, 1000000); //robomas
RawCAN      can2(PA_11, PA_12, 1000000);
// AnalogIn    poten(A0);
// DigitalOut  led1(LED1);
//PS3         ps3(A0, A1);
PID Lscissor(25.0, 5.0, 0, 0.05);
PID Rscissor(25.0, 5.0, 0, 0.05);
CircularBuffer<CANMessage, 32> queue;
char RcvData[8] = {0x00};

char asimawari[8] = {0x80,0x80,0x80,0x80,0xFF,0x00,0x00,0x00};
char kubi[8] = {0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00};

CANMessage msg(0x010, asimawari, 8, CANData, CANStandard);


PS3 ps3(A0,A1);
I2C i2c(PB_9,PB_8);
BNO055 bno(D14, D15);
/*
QEI roller_lf(D5,D6,NC,2048,QEI::X2_ENCODING); //b,a
QEI roller_rf(D9,D10,NC,2048,QEI::X2_ENCODING);
QEI roller_lb(D3,D4,NC,2048,QEI::X2_ENCODING);
QEI roller_rb(D7,D8,NC,2048,QEI::X2_ENCODING);
*/
//QEI fan(D11,D12,NC,2048,QEI::X2_ENCODING);

PID PID_lf(5.2,  0.0,  2.5, 0.050);
PID PID_rf(5.2,  0.0,  2.5, 0.050);
PID PID_lb(5.2,  0.0,  2.5, 0.050);
PID PID_rb(5.2,  0.0,  2.5, 0.050);

DigitalOut  sig(D13);
void send(char add, char data);
void getdata(void);
//void motor(int F1, int F2, int B1, int B2); //0:stop, 1:forward, 2:reverse
void motor(char F1, char F2, char B1, char B2);
void get_rpm();
void fan_rpm();

void get_angles();

void fan_control();

int Ry, Rx, Ra, Ly, Lx, La;

bool L1, R1, L2, R2, ue, sita, migi, hidari, maru, sankaku, sikaku, batu;

bool maru_osi, R2_osi, sikaku_osi;

double yaw;

bool corner_mode; //コーナーで90度曲がるときにtrueにする
double corner_yaw; //コーナー前の角度を保存
double corner_minus; //yawと↑の差
double yaw_kizyun = 0.0; //直進する基準
double hosei_yaw; //本当の角度から基準を引いて、直進方向を０とする

double hosei = 3.0; //補正の閾値(度)
bool hosei_mode; //角度を補正する
bool low_magaru_mode;
bool high_magaru_mode;

bool high_mode; //速度アップ
bool slower_stop;

Ticker flip1;
Ticker flip2;

int input_pwm[4];
double puls_kotaiti[4] = { 155.5, 160.8, 152.8, 160.5};//4   169.45, 169.4, 174.3, 162.75
double minus_kotaiti[4] = { 140.5, 141.5, 149.2, 133.0};//2   152.1, 141.0, 147.2, 134.75
int output_pwm[4];

double slower_rpm[4];

int LF,RF,LB,RB; //目標値
int pulse[4]; // lf:0, rf, lb:2, rb:3
double rpm[4]; // ↑↑

int fan_pulse;
double rpm_fan;


bool fan_mode;
bool fan_kyousei;
char fan_speed = 128;

bool hiena_mode;


//robomas
void scissorChangeData();

int motor_torqu[2] = {0};

struct C610Data{
    unsigned ID;
    int32_t counts;
    int32_t rpm;
    int32_t current;
    int32_t targetRPM;
    int32_t averageRPM;
    int16_t torqu;
};

struct C610Data M1;
struct C610Data M2;

void canListen(){
    CANMessage Rcvmsg;
    if (can.read(Rcvmsg)){
        queue.push(Rcvmsg);
    }
}

void datachange(unsigned ID, struct C610Data *C610, CANMessage *msg){
    if(ID == msg->id){
        C610->counts = uint16_t((msg->data[0] << 8) | msg->data[1]);
        C610->rpm = int16_t((msg->data[2] << 8) | msg->data[3]);
        C610->current = int16_t((msg->data[4] << 8) | msg->data[5]);
        // printf("%d %d %d\n",C610->counts, C610->rpm, C610->current);
    }
}

void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower){
    *upper = (torqu >> 8) & 0xFF;
    *lower = torqu & 0XFF;
}

void sendData(const int32_t torqu0, const int32_t torqu1){
    int16_t t0,t1;
    if(torqu0>32000){
        t0 = 32000;
    }else if(torqu0<-32000){
        t0 = -32000;
    }else{
        t0 = torqu0;
    }
    if(torqu1>32000){
        t1 = 32000;
    }else if(torqu1<-32000){
        t1 = -32000;
    }else{
        t1 = torqu1;
    }

    CANMessage sendmsg;
    sendmsg.id = 0x200;
    TorqueToBytes(t0, &sendmsg.data[0], &sendmsg.data[1]);
    TorqueToBytes(t1, &sendmsg.data[2], &sendmsg.data[3]);
    for(int i=4; i<8; i++){
        sendmsg.data[i] = 0x00;
    }
    can.write(sendmsg);
}

void can_send(){
    msg.id = 0x010;
    for(uint8_t i = 0;i < 8;i++){
        msg.data[i] = asimawari[i];
    }
    can2.write(msg);
    msg.id = 0x000;
    for(uint8_t i = 0;i < 8;i++){
        msg.data[i] = kubi[i];
    }
    can2.write(msg);
}

int main(){ 
    sig = 0;

    char stop = 0x80;
    char forward = 0xa7; //正転 2割99
    char reverse = 0x80 - (forward - 0x80); //逆転 67

    char low_forward = round((forward - 0x80) / 64 * 100 + 0x80); //90
    char low_reverse = round(0x80 - (0x80 - reverse) / 64 * 100); //70

    char high_forward = 0xff; // 6割　cc
    char high_reverse = 0x80 - (high_forward - 0x80); //34

    char high_low_forward = round((high_forward - 0x80) / 64 * 100 + 0x80);//ba
    char high_low_reverse = round(0x80 - (0x80 - high_reverse) / 64 * 100);//46

    //robomas
    can.attach(&canListen, CAN::RxIrq);

    // flip1.attach(&scissorChangeData,50ms);

    float adc_val = 0.0;
    int16_t TargetRotorPosition[2] = {0};
    bool scissorMove = 0;

    M1.ID = 0x201;
    M2.ID = 0x202;

    uint32_t counter=0;

    CANMessage Rxmsg;


    bno.reset();
    while(!bno.check());
    printf("bno start.\n");
    bno.setmode(OPERATION_MODE_IMUPLUS);

    /*
    roller_lf.reset();
    roller_rf.reset();
    roller_lb.reset();
    roller_rb.reset();
    */
    //fan.reset();

    flip2.attach(&fan_control,200ms);
    
    while (true) {
        string joy_state;

        while(!queue.empty()){
            queue.pop(Rxmsg);
            // led1 = !led1;
            datachange(M1.ID, &M1, &Rxmsg);
            datachange(M2.ID, &M2, &Rxmsg);
        }

        getdata();
        get_angles();

        // debag
        printf("M1:%d %d %d %d  M2: %d %d %d %d \n",M1.torqu, M1.rpm, M1.counts, M1.current, M2.torqu, M2.rpm, M2.counts, M2.current);

        // PIDなし
        if(!R2)R2_osi = false;
        if(!maru)maru_osi = false;
        if(!sikaku)sikaku_osi = false;

        
        if(hidari && !migi && !ue && !sita){
            // M1.targetRPM = 100;
            // M2.targetRPM = -100;
            sendData(10000, 10000);
            Lscissor.reset_accError();
            Rscissor.reset_accError();
        }else if(migi && !hidari && !ue && !sita){
            // M1.targetRPM = -100;
            // M2.targetRPM = 100;
            sendData(-10000, -10000);
            Lscissor.reset_accError();
            Rscissor.reset_accError();
        }else if(sita && !ue && !migi && !hidari){
            sendData(10000,0);
        }else if(batu){
            sendData(0, 10000);
        }else{
            M1.targetRPM = 0;
            M2.targetRPM = 0;
            sendData(0, 0);
        }

        if(sikaku && hiena_mode == false && sikaku_osi == false ){
            hiena_mode = true;
            sikaku_osi = true;
            asimawari[4] = 255;
        }else if(sikaku && hiena_mode == true && sikaku_osi == false ){
            hiena_mode = false;
            sikaku_osi = true;
            asimawari[4] = 0;
        }

        if(Ra < 45 && Ra > -45 && !(Rx == 0)){
            kubi[0] = 0x80;
            kubi[1] = 0xff;
        }else if(Ra >= 45 && Ra < 135){
            kubi[0] = 0xff;
            kubi[1] = 0x80;
        }else if(Ra > -135 && Ra <= -45){
            kubi[0] = 0x00;
            kubi[1] = 0x80;
        }else if(Ra >= 135 || Ra <= -135){
            kubi[0] = 0x80;
            kubi[1] = 0x00;
        }else{
            kubi[0] = 0x80;
            kubi[1] = 0x80;
        }

        if(high_mode == false && corner_mode == false && hosei_mode == false && slower_stop == false){
            if(L1 || R1){ //回転モード
                if(L1 && R1){
                    joy_state = "error";
                }else if(L1){
                    joy_state = "left_turn";
                    motor(reverse,reverse,reverse,reverse);
                }else if(R1){
                    joy_state = "right_turn";
                    motor(forward,forward,forward,forward);
                }
            }else{ //通常移動
                if(La >= 135/2 && La < 225/2){
                    joy_state = "mae";
                    motor(forward,reverse,forward,reverse);
                }else if(La >= 225/2 && La < 315/2){
                    joy_state = "hidari_mae";
                    motor(stop,reverse,forward,stop);
                }else if(La >= 315/2 || La < -(315/2)){
                    joy_state = "hidari";
                    motor(reverse,reverse,forward,forward);
                }else if(La >= -(315/2) && La < -(225/2)){
                    joy_state = "hidari_usiro";
                    motor(reverse,stop,stop,reverse);
                }else if(La >= -(225/2) && La < -(135/2)){
                    joy_state = "usiro";
                    motor(reverse,forward,reverse,forward);
                }else if(La >= -(135/2) && La < -(45/2)){
                    joy_state = "migi_usiro";
                    motor(stop,forward,reverse,stop);
                }else if(La >= -(45/2) && La < 45/2 && !(Lx == 0)){
                    joy_state = "migi";
                    motor(forward,forward,reverse,reverse);
                }else if(La >= 45/2 && La < 135/2){
                    joy_state = "migi_mae";
                    motor(forward,stop,stop,reverse);
                }else{
                    joy_state = "none";
                    motor(stop,stop,stop,stop);        
                }
            }
        }

        if(ps3.getSTARTState()){
            bno.reset();
            bno.setmode(OPERATION_MODE_IMUPLUS);
            yaw_kizyun = 0.0;
        }
        if(maru && corner_mode == false && maru_osi == false){
            corner_mode = true;
            maru_osi = true;
        }else if(maru && corner_mode == true && maru_osi == false){
            corner_mode = false;
            maru_osi = true;
        }

        if(corner_mode == true){
            joy_state = "corner";
            motor(reverse,reverse,reverse,reverse);
            if(hosei_yaw < 280 && hosei_yaw > 220){ //90度曲がれた時にモード切替
                corner_mode = false;
                joy_state = "corner_finish";
                
                motor(stop,stop,stop,stop);
                yaw_kizyun = yaw_kizyun - 90.0;
                if(yaw_kizyun < 0.0){
                    yaw_kizyun = yaw_kizyun + 360.0;
                }
            }
        }
        
        
        
        if(sankaku){
            joy_state = "hosei";
            hosei_mode = true;
        }else{
            hosei_mode = false;
            low_magaru_mode = false;
            high_magaru_mode = false;
        }
        if(ue && !sita && !migi && !hidari){
            high_mode = true;
        }else if(high_mode == true){
            slower_stop = true;
            slower_rpm[0] = 255;slower_rpm[1] = 0;slower_rpm[2] = 255;slower_rpm[3] = 0;
            high_mode = false;
        }else{
            high_mode = false;
        }

        if(slower_stop == true){
            motor(round(slower_rpm[0]), round(slower_rpm[1]), round(slower_rpm[2]), round(slower_rpm[3]));
            if(slower_rpm[0] == 128 && slower_rpm[1] == 128 && slower_rpm[2] == 128 && slower_rpm[3] == 128){
                slower_stop = false;
            }
        }

        if(corner_mode == false && high_mode == true && hosei_mode == false){
            joy_state = "high_nohosei";
            motor(high_forward,high_reverse,high_forward,high_reverse);
        }else if(corner_mode == false && high_mode == true && hosei_mode == true){
            if((hosei_yaw < hosei || hosei_yaw > (360 - hosei)) && high_magaru_mode == false){
                motor(high_forward,high_reverse,high_forward,high_reverse);
            }else{
                high_magaru_mode = true;
            }
        }


        if(La >= 135/2 && La < 225/2 && hosei_mode == true && high_mode == false){
            if((hosei_yaw < hosei || hosei_yaw > (360 - hosei)) && low_magaru_mode == false){
                motor(forward,reverse,forward,reverse);
            }else{
                low_magaru_mode = true;
            }
        }
        if(corner_mode == false && hosei_mode == true){
            if(low_magaru_mode == true){
                if(hosei_yaw > 1.0 && 180.0 > hosei_yaw){
                    motor(low_forward,reverse,low_forward,reverse);
                }else if(359.0 > hosei_yaw && hosei_yaw >= 180.0){
                    motor(forward,low_reverse,forward,low_reverse);
                }else{
                    low_magaru_mode = false;
                    high_magaru_mode = false;
                }
            }else if(high_magaru_mode == true && high_mode == true){
                if(hosei_yaw > 1.0 && 180.0 > hosei_yaw){
                    motor(high_low_forward,high_reverse,high_low_forward,high_reverse);
                }else if(359.0 > hosei_yaw && hosei_yaw >= 180.0){
                    motor(high_forward,high_low_reverse,high_forward,high_low_reverse);
                }else{
                    low_magaru_mode = false;
                    high_magaru_mode = false;
                }
            }
        }

        if(R2 && fan_mode == false && R2_osi == false){
            fan_kyousei = false;
            fan_mode = true;
            R2_osi = true;
        }else if(R2 && fan_mode == true && R2_osi == false){
            fan_mode = false;
            R2_osi = true;
        }
        if(L2){
            fan_kyousei = true;
            fan_mode = false;
            fan_speed = 128;
        }
        send(FAN,fan_speed);
        can_send();
        // printf("yaw: %f, ",yaw);
        // printf("hoyaw: %f, ",hosei_yaw);
        /*
        for(int n = 0;n < 4;n++){
            printf("%f, ",rpm[n]);
        }
        */
        //printf("fan: %f ,",rpm_fan);

        // cout << ", 足： " << joy_state << endl;
        ThisThread::sleep_for(1ms);
    } 
} 

void send(char add, char data){ //I2C 送信
    wait_us(30);
    i2c.start(); 
    i2c.write(add); 
    i2c.write(data); 
    i2c.stop(); 
} 

void getdata(void){ //PS3　情報受取
    //ジョイスティック
    Ry = ps3.getRightJoystickYaxis();
    Rx = ps3.getRightJoystickXaxis();
    Ra = ps3.getRightJoystickAngle();
    Ly = ps3.getLeftJoystickYaxis();
    Lx = ps3.getLeftJoystickXaxis();
    La = ps3.getLeftJoystickAngle();

    L1 = ps3.getButtonState(PS3::L1);
    R1 = ps3.getButtonState(PS3::R1);
    L2 = ps3.getButtonState(PS3::L2);
    R2 = ps3.getButtonState(PS3::R2);

    ue = ps3.getButtonState(PS3::ue);
    sita = ps3.getButtonState(PS3::sita);
    migi = ps3.getButtonState(PS3::migi);
    hidari = ps3.getButtonState(PS3::hidari);

    maru = ps3.getButtonState(PS3::maru);
    batu = ps3.getButtonState(PS3::batu);
    sankaku = ps3.getButtonState(PS3::sankaku);
    sikaku = ps3.getButtonState(PS3::sikaku);
}


void motor(char F1, char F2, char B1, char B2){ //足回り制御_PIDなし (LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK)
    input_pwm[0] = F1; input_pwm[1] = F2; input_pwm[2] = B1; input_pwm[3] = B2;
    for(int n = 0;n < 4;n++){
        input_pwm[n] = input_pwm[n] - 128;
        if(input_pwm[n] > 0){//基準４
            output_pwm[n] = round(minus_kotaiti[3] / puls_kotaiti[n] * (double)input_pwm[n]);
            output_pwm[n] = output_pwm[n] + 128;
        }else if(input_pwm[n] < 0){//基準２
            output_pwm[n] = round(minus_kotaiti[3] / minus_kotaiti[n] * (double)abs(input_pwm[n]));
            output_pwm[n] = 128 - output_pwm[n];
        }else{
            output_pwm[n] = 128;
        }
    }
    send( LEFT_FRONT, output_pwm[0]);
    send( RIGHT_FRONT, output_pwm[1]);
    send( LEFT_BACK, output_pwm[2]);
    send( RIGHT_BACK, output_pwm[3]);
}


void get_rpm(){
    /*
    pulse[0] = roller_lf.getPulses();
    pulse[1] = roller_rf.getPulses();
    pulse[2] = roller_lb.getPulses();
    pulse[3] = roller_rb.getPulses();
    roller_lf.reset();
    roller_rf.reset();
    roller_lb.reset();
    roller_rb.reset();
    */
    for(int i = 0;i < 4;i++){
        rpm[i] = (60 * 5 * (double)pulse[i]) / (2048 * 2);
    }
    //PID_asi();
}

void fan_rpm(){
    //fan_pulse = fan.getPulses();
    //fan.reset();
    //rpm_fan = (60 * 5 * (double)fan_pulse) / (2048 * 2);
}

void get_angles(){
    bno.get_angles();
    yaw = bno.euler.yaw;
    hosei_yaw = yaw - yaw_kizyun;
    if(hosei_yaw < 0.0){
        hosei_yaw = hosei_yaw + 360.0;
    }
}

void fan_control(){
    if(fan_kyousei == false && fan_mode == true && fan_speed > 0){
        fan_speed = fan_speed - 1;
    }else if(fan_kyousei == false && fan_mode == false && fan_speed < 128){
        fan_speed = fan_speed + 1;
    }
    if(slower_stop == true){
        slower_rpm[0] = slower_rpm[0] - 15.875;
        slower_rpm[1] = slower_rpm[1] + 16.0;
        slower_rpm[2] = slower_rpm[2] - 15.875;
        slower_rpm[3] = slower_rpm[3] + 16.0;
        if(slower_rpm[0] <= 128){
            slower_rpm[0] = 128;
        }
        if(slower_rpm[1] >= 128){
            slower_rpm[1] = 128;
        }
        if(slower_rpm[2] <= 128){
            slower_rpm[2] = 128;
        }
        if(slower_rpm[3] >= 128){
            slower_rpm[3] = 128;
        }
    }
}




int Lscissor_PID(C610Data *M){
    Lscissor.setInputLimits(-18000, 18000);
    Lscissor.setOutputLimits(0, 10000);

    // M->averageRPM = M->averageRPM * (6.0/7.0) + M->rpm/7.0;

    Lscissor.setSetPoint(M->targetRPM);
    Lscissor.setProcessValue(M->rpm);

    if(M->rpm <= M->targetRPM){
        M->torqu = Lscissor.compute();
    }else{
        M->torqu = -1 * Lscissor.compute();
    }
        
    return 0;
}

int Rscissor_PID(C610Data *M){
    Lscissor.setInputLimits(-18000, 18000);
    Lscissor.setOutputLimits(0, 10000);

    // M->averageRPM = M->averageRPM * (6.0/7.0) + M->rpm/7.0;

    Rscissor.setSetPoint(M->targetRPM);
    Rscissor.setProcessValue(M->rpm);

    if(M->rpm <= M->targetRPM){
        M->torqu = Rscissor.compute();
    }else{
        M->torqu = -1 * Rscissor.compute();
    }

    return 0;
}

void scissorChangeData(){
    Lscissor_PID(&M1);
    Rscissor_PID(&M2);
}