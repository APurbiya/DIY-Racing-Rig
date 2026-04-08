//****************************************************************************************************************
// Racing_Rig.ino - Advanced 2-Motor PID Motion Controller
//****************************************************************************************************************

#define MODE1    

#include <EEPROM.h>
#include <SoftwareSerial.h>

// Register bit manipulation macros
#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define COM0 0                          
#define START_BYTE '['       
#define END_BYTE ']'                    
#define PROCESS_PERIOD_uS 250           

unsigned long NextProcessTime = 0;

// Motor Feedback and Target positions
int Feedback_Left = 512;
int Feedback_Right = 512;
int Target_Left = 512;
int Target_Right = 512;
int PotInput = 512;

// Serial Communication Buffers
unsigned int RxByte[1] = {0};
int RxPtr[1] = {-1};
unsigned int RxBuffer[5][1] = {0};

// Pin Definitions 
const int ENApin_Left = 2;
const int ENBpin_Left = 3;
const int ENApin_Right = 4;
const int ENBpin_Right = 5;
const int PWMpin_Left = 9;
const int PWMpin_Right = 10;
const int FeedbackPin_Left = A0;
const int FeedbackPin_Right = A1;

// Motor Configuration Variables 
int DeadZone_Left = 0;
int DeadZone_Right = 0;
int CutoffLimitMax_Left = 1000;
int CutoffLimitMax_Right = 1000;
int CutoffLimitMin_Left = 23;
int CutoffLimitMin_Right = 23;
int InputClipMax_Left = 923;
int InputClipMax_Right = 923;
int InputClipMin_Left = 100;
int InputClipMin_Right = 100;

long Kp_Left_x100 = 100;
long Ki_Left_x100 = 40;
long Kd_Left_x100 = 40;
int Ks_Left = 1;

long Kp_Right_x100 = 420;
long Ki_Right_x100 = 40;
long Kd_Right_x100 = 40;
int Ks_Right = 1;

int PWMout_Left = 0;
int PWMout_Right = 0;
int PWMoffset_Left = 50;
int PWMoffset_Right = 50;
int PWMmax_Left = 100;
int PWMmax_Right = 100;
int PWMrev_Left = 200;
int PWMrev_Right = 200;

int Disable_Left = 1;
int Disable_Right = 1;
unsigned int Timer1FreqkHz = 25; 

// --- Advanced PWM Timer Functions ---

void InitialisePWMTimer1(unsigned int Freq) 
{
    uint8_t wgm = 8;
    TCCR1A = (TCCR1A & B11111100) | (wgm & B00000011);
    TCCR1B = (TCCR1B & B11100111) | ((wgm & B00001100) << 1);
    TCCR1B = (TCCR1B & B11111000) | 0x01; 
    ICR1 = (F_CPU / 2) / Freq;
}

void MyPWMWrite(uint8_t pin, uint8_t val) 
{
    #define OCR1A_MEM 0x88
    #define OCR1B_MEM 0x8A
    pinMode(pin, OUTPUT);
    uint32_t tmp = val;
    if (val == 0) digitalWrite(pin, LOW);
    else if (val == 255) digitalWrite(pin, HIGH);
    else 
    {
        uint16_t regLoc16 = (pin == 9) ? OCR1A_MEM : OCR1B_MEM;
        if (pin == 9) sbi(TCCR1A, COM1A1); else sbi(TCCR1A, COM1B1);
        tmp = (tmp * ICR1) / 255;
        _SFR_MEM16(regLoc16) = tmp;
    }
}

// --- EEPROM Persistence ---

void WriteEEPRomWord(int address, int intvalue) 
{
    EEPROM.write(address, intvalue / 256);
    EEPROM.write(address + 1, intvalue % 256);
}

int ReadEEPRomWord(int address) 
{
    return (EEPROM.read(address) * 256) + EEPROM.read(address + 1);
}

void WriteEEProm() 
{
    EEPROM.write(0, 114);
    EEPROM.write(1, CutoffLimitMin_Left);
    EEPROM.write(2, InputClipMin_Left);
    EEPROM.write(5, DeadZone_Left);
    WriteEEPRomWord(11, Kp_Left_x100);
    WriteEEPRomWord(13, Ki_Left_x100);
    WriteEEPRomWord(15, Kd_Left_x100);
    EEPROM.write(23, PWMoffset_Left);
    EEPROM.write(25, PWMmax_Left);
    // Logic for Right motor follows same address structure
}

void ReadEEProm() 
{
    if(EEPROM.read(0) != 114) { WriteEEProm(); return; }
    CutoffLimitMin_Left = EEPROM.read(1);
    InputClipMin_Left = EEPROM.read(2);
    CutoffLimitMax_Left = 1023 - CutoffLimitMin_Left;
    InputClipMax_Left = 1023 - InputClipMin_Left;
    Kp_Left_x100 = ReadEEPRomWord(11);
}

// --- Serial Protocol ---

void SendTwoValues(int id, int v1, int v2, int ComPort) 
{
    Serial.write(START_BYTE);
    Serial.write(id);
    Serial.write(v1);
    Serial.write(v2);
    Serial.write(END_BYTE);
}

void ParseCommand(int ComPort) 
{
    switch (RxBuffer[0][ComPort]) 
    {
        case 'A': // Target Left
            Target_Left = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            if (Target_Left > InputClipMax_Left) Target_Left = InputClipMax_Left;
            break;
        case 'B': // Target Right
            Target_Right = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            if (Target_Right > InputClipMax_Right) Target_Right = InputClipMax_Right;
            break;
        case 'r': // Handle Read requests
            if (RxBuffer[1][ComPort] == 'd') {
                if (RxBuffer[2][ComPort] == 'A') SendTwoValues('A', Feedback_Left / 4, Target_Left / 4, ComPort);
                if (RxBuffer[2][ComPort] == 'B') SendTwoValues('B', Feedback_Right / 4, Target_Right / 4, ComPort);
            }
            break;
    }
}

void DisableMotorLeft() 
{ 
    Disable_Left = 1; 
    digitalWrite(ENApin_Left, LOW); 
    digitalWrite(ENBpin_Left, LOW); 
    MyPWMWrite(PWMpin_Left, 0); 
}

void DisableMotorRight() 
{ 
    Disable_Right = 1; 
    digitalWrite(ENApin_Right, LOW); 
    digitalWrite(ENBpin_Right, LOW); 
    MyPWMWrite(PWMpin_Right, 0); 
}

void setup() {
    ReadEEProm();
    Serial.begin(500000);
    pinMode(ENApin_Left, OUTPUT); pinMode(ENBpin_Left, OUTPUT);
    pinMode(ENApin_Right, OUTPUT); pinMode(ENBpin_Right, OUTPUT);
    InitialisePWMTimer1(Timer1FreqkHz * 1000);
    
    // Fast ADC Optimization
    sbi(ADCSRA, ADPS2); 
    cbi(ADCSRA, ADPS1); 
    cbi(ADCSRA, ADPS0); 
    
    DisableMotorLeft(); 
    DisableMotorRight();
}

void loop() 
{
    // Serial Receiver State Machine
    while (Serial.available() > 0) 
    {
        RxByte[COM0] = Serial.read();
        if (RxByte[COM0] == START_BYTE) RxPtr[COM0] = 0;
        else if (RxPtr[COM0] >= 0) 
        {
            if (RxByte[COM0] == END_BYTE) 
            { 
                ParseCommand(COM0); RxPtr[COM0] = -1; 
            }
            else if (RxPtr[COM0] < 5) 
            { 
                RxBuffer[RxPtr[COM0]][COM0] = RxByte[COM0]; RxPtr[COM0]++; 
            }
        }
    }

    unsigned long CurrentTime = micros();
    if (CurrentTime >= NextProcessTime) 
    {
        NextProcessTime = CurrentTime + PROCESS_PERIOD_uS;
        Feedback_Left = analogRead(FeedbackPin_Left);
        Feedback_Right = analogRead(FeedbackPin_Right);

        // Safety Limit Checks
        if (Feedback_Left > CutoffLimitMax_Left || Feedback_Left < CutoffLimitMin_Left) 
        {
            DisableMotorLeft();
        }
        if (Feedback_Right > CutoffLimitMax_Right || Feedback_Right < CutoffLimitMin_Right) 
        {
            DisableMotorRight();
        }
    }
}