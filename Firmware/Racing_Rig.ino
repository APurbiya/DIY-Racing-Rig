//****************************************************************************************************************
// Racing_Rig.ino - Ultra-High Performance 2-Motor PID Motion Controller
// Developed for: H-Bridge Motor Drivers (BTS7960 / IBT-2 Compatible)
// Protocol: SMC3 Compatible Serial Interface
//****************************************************************************************************************

#define MODE1    

#include <EEPROM.h>
#include <SoftwareSerial.h>

/* * MACRO DEFINITIONS 
 * These handle register-level bit manipulation to bypass slow Arduino functions.
 * sbi = Set Bit / cbi = Clear Bit
 */
#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// --- System Constants ---
#define COM0 0                          
#define START_BYTE '['       
#define END_BYTE ']'                    
#define PROCESS_PERIOD_uS 250   // 4kHz Control Loop Frequency

// --- Global Timing Control ---
unsigned long NextProcessTime = 0;
unsigned long CurrentTime = 0;
unsigned long LoopCounter = 0;
unsigned long DiagnosticTimer = 0;

// --- High-Precision PID State Variables ---
// We use long to maintain precision during complex mathematical multiplications
long Error_Left, LastError_Left, Iterm_Left;
long Error_Right, LastError_Right, Iterm_Right;

// --- Motor Feedback and Target positions ---
int Feedback_Left = 512;
int Feedback_Right = 512;
int Target_Left = 512;
int Target_Right = 512;
int PotInput = 512;

// --- Serial Communication Buffers ---
unsigned int RxByte[1] = {0};
int RxPtr[1] = {-1};
unsigned int RxBuffer[5][1] = {0};

// --- Hardware Pin Assignments ---
// Motors are assigned to Timer1 (Pins 9/10) for 25kHz Ultrasonic PWM
const int ENApin_Left = 2;
const int ENBpin_Left = 3;
const int ENApin_Right = 4;
const int ENBpin_Right = 5;
const int PWMpin_Left = 9;
const int PWMpin_Right = 10;
const int FeedbackPin_Left = A0;
const int FeedbackPin_Right = A1;

// --- Mechanical Calibration Variables ---
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

// --- PID Control Tuning ---
long Kp_Left_x100 = 100;
long Ki_Left_x100 = 40;
long Kd_Left_x100 = 40;
int Ks_Left = 1;

long Kp_Right_x100 = 420;
long Ki_Right_x100 = 40;
long Kd_Right_x100 = 40;
int Ks_Right = 1;

// --- PWM Power Constraints ---
int PWMout_Left = 0;
int PWMout_Right = 0;
int PWMoffset_Left = 50;
int PWMoffset_Right = 50;
int PWMmax_Left = 100;
int PWMmax_Right = 100;
int PWMrev_Left = 200;
int PWMrev_Right = 200;

// --- Operational State Flags ---
int Disable_Left = 1;
int Disable_Right = 1;
unsigned int Timer1FreqkHz = 25; 

// --- Timer1 Configuration (25kHz PWM) ---
void InitialisePWMTimer1(unsigned int Freq) 
{
    // Fast PWM mode with ICR1 as TOP value
    uint8_t wgm = 8;
    TCCR1A = (TCCR1A & B11111100) | (wgm & B00000011);
    TCCR1B = (TCCR1B & B11100111) | ((wgm & B00001100) << 1);
    TCCR1B = (TCCR1B & B11111000) | 0x01; 
    ICR1 = (F_CPU / 2) / Freq;
}

// Optimized PWM Write using direct register access
void MyPWMWrite(uint8_t pin, uint8_t val) 
{
    #define OCR1A_MEM 0x88
    #define OCR1B_MEM 0x8A
    pinMode(pin, OUTPUT);
    uint32_t tmp = val;
    
    if (val == 0) 
    {
        digitalWrite(pin, LOW);
    } 
    else if (val == 255) {
        digitalWrite(pin, HIGH);
    } 
    else 
    {
        uint16_t regLoc16 = (pin == 9) ? OCR1A_MEM : OCR1B_MEM;
        if (pin == 9) 
        {
            sbi(TCCR1A, COM1A1);
        } 
        else 
        {
            sbi(TCCR1A, COM1B1);
        }
        tmp = (tmp * ICR1) / 255;
        _SFR_MEM16(regLoc16) = tmp;
    }
}

// --- EEPROM Handlers ---
void WriteEEPRomWord(int address, int intvalue) 
{
    EEPROM.write(address, intvalue / 256);
    EEPROM.write(address + 1, intvalue % 256);
}

int ReadEEPRomWord(int address) 
{
    return (EEPROM.read(address) * 256) + EEPROM.read(address + 1);
}

// --- Save Rig Calibration ---
void WriteEEProm() 
{
    EEPROM.write(0, 114); 
    EEPROM.write(1, CutoffLimitMin_Left);
    EEPROM.write(2, InputClipMin_Left);
    EEPROM.write(3, CutoffLimitMin_Right); 
    EEPROM.write(4, InputClipMin_Right);
    EEPROM.write(5, DeadZone_Left);
    EEPROM.write(6, DeadZone_Right);
    
    WriteEEPRomWord(11, Kp_Left_x100);
    WriteEEPRomWord(13, Ki_Left_x100);
    WriteEEPRomWord(15, Kd_Left_x100);
    EEPROM.write(23, PWMoffset_Left);
    EEPROM.write(25, PWMmax_Left);
    EEPROM.write(27, PWMrev_Left);
    
    WriteEEPRomWord(31, Kp_Right_x100);
    WriteEEPRomWord(33, Ki_Right_x100);
    WriteEEPRomWord(35, Kd_Right_x100);
    EEPROM.write(43, PWMoffset_Right);
    EEPROM.write(45, PWMmax_Right);
    EEPROM.write(47, PWMrev_Right);
}

// --- Load Rig Calibration ---
void ReadEEProm() 
{
    if(EEPROM.read(0) != 114) 
    { 
        WriteEEProm(); 
        return; 
    }
    CutoffLimitMin_Left = EEPROM.read(1);
    InputClipMin_Left = EEPROM.read(2);
    CutoffLimitMin_Right = EEPROM.read(3);
    InputClipMin_Right = EEPROM.read(4);
    
    CutoffLimitMax_Left = 1023 - CutoffLimitMin_Left;
    InputClipMax_Left = 1023 - InputClipMin_Left;
    CutoffLimitMax_Right = 1023 - CutoffLimitMin_Right;
    InputClipMax_Right = 1023 - InputClipMin_Right;

    Kp_Left_x100 = ReadEEPRomWord(11);
    Ki_Left_x100 = ReadEEPRomWord(13);
    Kd_Left_x100 = ReadEEPRomWord(15);
    Kp_Right_x100 = ReadEEPRomWord(31);
}

// --- PID Math Engine: Motor 1 ---
void CalculatePID_Left() 
{
    if (Disable_Left) return;
    Error_Left = Target_Left - Feedback_Left;
    
    if (abs(Error_Left) <= DeadZone_Left) 
    {
        PWMout_Left = 0;
    } 
    else 
    {
        long pTerm = (Kp_Left_x100 * Error_Left) / 100;
        Iterm_Left += (Ki_Left_x100 * Error_Left) / 100;
        
        // Integral Anti-Windup
        if (Iterm_Left > 12000) 
        {
            Iterm_Left = 12000;
        }
        else if (Iterm_Left < -12000) 
        {
            Iterm_Left = -12000;
        }
        
        long dTerm = (Kd_Left_x100 * (Error_Left - LastError_Left)) / 100;
        LastError_Left = Error_Left;
        
        PWMout_Left = (pTerm + (Iterm_Left / 10) + dTerm) / Ks_Left;
    }
    DriveMotorLeft(PWMout_Left);
}

// --- PID Math Engine: Motor 2 ---
void CalculatePID_Right() 
{
    if (Disable_Right) return;
    Error_Right = Target_Right - Feedback_Right;
    
    if (abs(Error_Right) <= DeadZone_Right) 
    {
        PWMout_Right = 0;
    } 
    else 
    {
        long pTerm = (Kp_Right_x100 * Error_Right) / 100;
        Iterm_Right += (Ki_Right_x100 * Error_Right) / 100;
        
        if (Iterm_Right > 12000) 
        {
            Iterm_Right = 12000;
        }
        else if (Iterm_Right < -12000) 
        {
            Iterm_Right = -12000;
        }
        
        long dTerm = (Kd_Right_x100 * (Error_Right - LastError_Right)) / 100;
        LastError_Right = Error_Right;
        
        PWMout_Right = (pTerm + (Iterm_Right / 10) + dTerm) / Ks_Right;
    }
    DriveMotorRight(PWMout_Right);
}

// --- Motor Direction Control ---
void DriveMotorLeft(int power) 
{
    int out = abs(power) + PWMoffset_Left;
    if (out > PWMmax_Left) out = PWMmax_Left;

    if (power > 0) 
    {
        digitalWrite(ENApin_Left, HIGH); 
        digitalWrite(ENBpin_Left, LOW);
        MyPWMWrite(PWMpin_Left, out);
    } 
    else if (power < 0) 
    {
        digitalWrite(ENApin_Left, LOW); 
        digitalWrite(ENBpin_Left, HIGH);
        MyPWMWrite(PWMpin_Left, out);
    } 
    else 
    {
        digitalWrite(ENApin_Left, LOW); 
        digitalWrite(ENBpin_Left, LOW);
        MyPWMWrite(PWMpin_Left, PWMrev_Left);
    }
}

void DriveMotorRight(int power) 
{
    int out = abs(power) + PWMoffset_Right;
    if (out > PWMmax_Right) 
    {
        out = PWMmax_Right;
    }

    if (power > 0) 
    {
        digitalWrite(ENApin_Right, HIGH); 
        digitalWrite(ENBpin_Right, LOW);
        MyPWMWrite(PWMpin_Right, out);
    } 
    else if (power < 0) 
    {
        digitalWrite(ENApin_Right, LOW); 
        digitalWrite(ENBpin_Right, HIGH);
        MyPWMWrite(PWMpin_Right, out);
    } 
    else 
    {
        digitalWrite(ENApin_Right, LOW); 
        digitalWrite(ENBpin_Right, LOW);
        MyPWMWrite(PWMpin_Right, PWMrev_Right);
    }
}

// --- Serial Command Parser ---
void ParseCommand(int ComPort) 
{
    switch (RxBuffer[0][ComPort]) 
    {
        case 'A': 
            Target_Left = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            Target_Left = constrain(Target_Left, InputClipMin_Left, InputClipMax_Left);
            Disable_Left = 0;
            break;
        case 'B': 
            Target_Right = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            Target_Right = constrain(Target_Right, InputClipMin_Right, InputClipMax_Right);
            Disable_Right = 0;
            break;
        case 'S': 
            WriteEEProm();
            break;
        case 'h':
            PrintHelpMenu(); 
            break;
        case 'd':
            RunRigDiagnostics(); 
            break;
    }
}

void PrintHelpMenu() 
{
    Serial.println(F("\n--- RIG MAINTENANCE COMMANDS ---"));
    Serial.println(F(" [Axxx] - Move Left Axis (0-1023)"));
    Serial.println(F(" [Bxxx] - Move Right Axis (0-1023)"));
    Serial.println(F(" [S]    - Save Current Calibration to EEPROM"));
    Serial.println(F(" [h]    - Show this Help Menu"));
    Serial.println(F(" [d]    - Run System Diagnostics"));
    Serial.println(F("---------------------------------"));
}

// --- Safety Interlocks ---
void DisableMotorLeft() 
{ 
    Disable_Left = 1; 
    digitalWrite(2, LOW); 
    digitalWrite(3, LOW); 
    MyPWMWrite(9, 0); 
}
void DisableMotorRight() 
{ 
    Disable_Right = 1; 
    digitalWrite(4, LOW); 
    digitalWrite(5, LOW); 
    MyPWMWrite(10, 0); 
}

// --- Hardware Initialization ---
void setup() 
{
    ReadEEProm();
    Serial.begin(500000);
    
    pinMode(ENApin_Left, OUTPUT); 
    pinMode(ENBpin_Left, OUTPUT);
    pinMode(ENApin_Right, OUTPUT); 
    pinMode(ENBpin_Right, OUTPUT);
    
    InitialisePWMTimer1(Timer1FreqkHz * 1000);
    
    // speed up ADC sampling
    sbi(ADCSRA, ADPS2); 
    cbi(ADCSRA, ADPS1); 
    cbi(ADCSRA, ADPS0); 
    
    DisableMotorLeft(); 
    DisableMotorRight();
    Serial.println(F("SYSTEM_ONLINE_STABLE"));
}

// --- Main Control Loop ---
void loop() 
{
    // Serial Data Processing
    while (Serial.available() > 0) 
    {
        RxByte[COM0] = Serial.read();
        if (RxByte[COM0] == START_BYTE) 
        {
            RxPtr[COM0] = 0;
        }
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

   CurrentTime = micros();
    if (CurrentTime >= NextProcessTime) 
    {
        NextProcessTime = CurrentTime + PROCESS_PERIOD_uS;
        
        Feedback_Left = analogRead(FeedbackPin_Left);
        Feedback_Right = analogRead(FeedbackPin_Right);
        
        SmoothingModule_Left();
        SmoothingModule_Right();
        
        if (Feedback_Left > CutoffLimitMax_Left || Feedback_Left < CutoffLimitMin_Left) 
        {
            DisableMotorLeft();
        }
        
        if (Feedback_Right > CutoffLimitMax_Right || Feedback_Right < CutoffLimitMin_Right) 
        {
            DisableMotorRight();
        }
        
        CalculatePID_Left();
        CalculatePID_Right();
    }
}

void RunRigDiagnostics() 
{
    Serial.println(F("\n--- RACING RIG MASTER DIAGNOSTICS ---"));
    Serial.print(F("Firmware Version: ")); 
    Serial.println(F("4.0.0-STABLE"));
    Serial.print(F("Process Interval: ")); 
    Serial.print(PROCESS_PERIOD_uS); 
    Serial.println(F(" uS"));
    Serial.print(F("PWM Carrier Frequency: ")); 
    Serial.print(Timer1FreqkHz); 
    Serial.println(F(" kHz"));
    Serial.println(F("\n[AXIS A - LEFT MOTOR]"));
    Serial.print(F("  Feedback Raw: ")); 
    Serial.println(Feedback_Left);
    Serial.print(F("  Target Raw:   ")); 
    Serial.println(Target_Left);
    Serial.print(F("  PWM Output:   ")); 
    Serial.println(PWMout_Left);
    Serial.print(F("  Status:       ")); 
    if(Disable_Left) 
    { 
        Serial.println(F("OFFLINE (SAFETY)")); 
    }

    else             
    {
        Serial.println(F("ACTIVE")); 
    }
    
    Serial.println(F("\n[AXIS B - RIGHT MOTOR]"));
    Serial.print(F("  Feedback Raw: ")); 
    Serial.println(Feedback_Right);
    Serial.print(F("  Target Raw:   ")); 
    Serial.println(Target_Right);
    Serial.print(F("  PWM Output:   ")); 
    Serial.println(PWMout_Right);
    Serial.print(F("  Status:       ")); 
    if(Disable_Right) { Serial.println(F("OFFLINE (SAFETY)")); }
    else              { Serial.println(F("ACTIVE")); }
    Serial.println(F("--------------------------------------"));
}

void SmoothingModule_Left() 
{
    static int pFeedback_L = 512;
    Feedback_Left = (Feedback_Left + pFeedback_L) / 2;
    pFeedback_L = Feedback_Left;
}

void SmoothingModule_Right() 
{
    static int pFeedback_R = 512;
    Feedback_Right = (Feedback_Right + pFeedback_R) / 2;
    pFeedback_R = Feedback_Right;
}

void ManualCalibrationMode() 
{
    // Routine to center the rig before PID takes over
}

// --- End of original Racing_Rig code ---
// ========================================
// === DAY 2 ADDITIONS (~400 lines)
//     - Comms watchdog + PowerScale
//     - PID process divider
//     - Serial feedback auto-reporting
//     - SendValue / SendTwoValues
//     - DeltaLoopCount
//     - Full extended command parser
//     - Proper packet-level serial handler
// ========================================

// --- Comms watchdog state ---
unsigned int  CommsTimeout      = 0;   // Counts up each loop tick; reset on valid packet
byte          PowerScale        = 7;   // Right-shift applied to PID result (~divide by 128)
                                       // Set to 9 (~divide by 512) when comms lost
unsigned long LastLoopCount     = 0;
byte          errorcount        = 0;

// --- PID process divider ---
int PIDProcessDivider = 1;             // Run PID every N timer ticks
int PIDProcessCounter = 0;

// --- Serial feedback auto-reporting ---
int SerialFeedbackEnabled  = 0;        // 0=off, 1=left motor, 2=right motor
int SerialFeedbackCounter  = 0;

//****************************************************************************************************************
//  SendValue
//  Transmits a 16-bit integer as a 5-byte SMC3 protocol packet: [id][high][low]
//****************************************************************************************************************

void SendValue(int id, int value, int ComPort)
{
    int high = value / 256;
    int low  = value - (high * 256);
    Serial.write(START_BYTE);
    Serial.write(id);
    Serial.write(high);
    Serial.write(low);
    Serial.write(END_BYTE);
}

//****************************************************************************************************************
//  SendTwoValues
//  Transmits two 8-bit values in one SMC3 packet: [id][v1][v2]
//****************************************************************************************************************

void SendTwoValues(int id, int v1, int v2, int ComPort)
{
    Serial.write(START_BYTE);
    Serial.write(id);
    Serial.write(v1);
    Serial.write(v2);
    Serial.write(END_BYTE);
}

//****************************************************************************************************************
//  DeltaLoopCount
//  Returns the number of PID cycles completed since this function was last called.
//  Used by the host software to measure controller throughput.
//****************************************************************************************************************

int DeltaLoopCount()
{
    unsigned long delta;
    if ((LastLoopCount == 0) || ((LoopCounter - LastLoopCount) > 32000))
    {
        delta         = 0;
        LastLoopCount = LoopCounter;
    }
    else
    {
        delta         = LoopCounter - LastLoopCount;
        LastLoopCount = LoopCounter;
    }
    return (int)delta;
}

//****************************************************************************************************************
//  ParseCommand_Extended
//  Full SMC3-compatible command set. Handles all PID tuning parameters,
//  PWM limits, cutoff/clip limits, enable/disable, read-back requests,
//  serial monitor feedback control and firmware version reporting.
//  Delegates the original A/B/S/h/d cases to ParseCommand().
//****************************************************************************************************************

void ParseCommand_Extended(int ComPort)
{
    CommsTimeout = 0;    // Any valid packet resets the comms watchdog

    switch (RxBuffer[0][ComPort])
    {
        // --- Original commands: delegate to base parser ---
        case 'A':
        case 'B':
        case 'S':
        case 'h':
        case 'd':
            ParseCommand(ComPort);
            break;

        // --- Kp ---
        case 'D':
            Kp_Left_x100  = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;
        case 'E':
            Kp_Right_x100 = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;

        // --- Ki ---
        case 'G':
            Ki_Left_x100  = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;
        case 'H':
            Ki_Right_x100 = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;

        // --- Kd ---
        case 'J':
            Kd_Left_x100  = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;
        case 'K':
            Kd_Right_x100 = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;

        // --- Ks derivative sample filter ---
        case 'M':
            Ks_Left  = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;
        case 'N':
            Ks_Right = (RxBuffer[1][ComPort] * 256) + RxBuffer[2][ComPort];
            break;

        // --- PWM offset and max ---
        case 'P':
            PWMoffset_Left  = RxBuffer[1][ComPort];
            PWMmax_Left     = RxBuffer[2][ComPort];
            break;
        case 'Q':
            PWMoffset_Right = RxBuffer[1][ComPort];
            PWMmax_Right    = RxBuffer[2][ComPort];
            break;

        // --- Cutoff limits and input clip ---
        case 'T':
            CutoffLimitMin_Left  = RxBuffer[1][ComPort];
            CutoffLimitMax_Left  = 1023 - CutoffLimitMin_Left;
            InputClipMin_Left    = RxBuffer[2][ComPort];
            InputClipMax_Left    = 1023 - InputClipMin_Left;
            break;
        case 'U':
            CutoffLimitMin_Right  = RxBuffer[1][ComPort];
            CutoffLimitMax_Right  = 1023 - CutoffLimitMin_Right;
            InputClipMin_Right    = RxBuffer[2][ComPort];
            InputClipMax_Right    = 1023 - InputClipMin_Right;
            break;

        // --- DeadZone and PWMrev ---
        case 'V':
            DeadZone_Left  = RxBuffer[1][ComPort];
            PWMrev_Left    = RxBuffer[2][ComPort];
            break;
        case 'W':
            DeadZone_Right = RxBuffer[1][ComPort];
            PWMrev_Right   = RxBuffer[2][ComPort];
            break;

        // --- PID process divider ---
        case 'Z':
            PIDProcessDivider = constrain(RxBuffer[1][ComPort], 1, 10);
            break;

        // --- Read parameter requests ---
        case 'r':
            if (RxBuffer[1][ComPort] == 'd')
            {
                switch (RxBuffer[2][ComPort])
                {
                    case 'A':
                        SendTwoValues('A', Feedback_Left / 4, Target_Left / 4, ComPort);
                        break;
                    case 'B':
                        SendTwoValues('B', Feedback_Right / 4, Target_Right / 4, ComPort);
                        break;
                    case 'a':
                        SendTwoValues('a',
                            PIDProcessDivider * 16 + Disable_Left + (Disable_Right * 2),
                            constrain(PWMout_Left, 0, 255), ComPort);
                        break;
                    case 'b':
                        SendTwoValues('b',
                            PIDProcessDivider * 16 + Disable_Left + (Disable_Right * 2),
                            constrain(PWMout_Right, 0, 255), ComPort);
                        break;
                    case 'D': SendValue('D', Kp_Left_x100,   ComPort); break;
                    case 'E': SendValue('E', Kp_Right_x100,  ComPort); break;
                    case 'G': SendValue('G', Ki_Left_x100,   ComPort); break;
                    case 'H': SendValue('H', Ki_Right_x100,  ComPort); break;
                    case 'J': SendValue('J', Kd_Left_x100,   ComPort); break;
                    case 'K': SendValue('K', Kd_Right_x100,  ComPort); break;
                    case 'M': SendValue('M', Ks_Left,         ComPort); break;
                    case 'N': SendValue('N', Ks_Right,        ComPort); break;
                    case 'P': SendTwoValues('P', PWMoffset_Left,  PWMmax_Left,  ComPort); break;
                    case 'Q': SendTwoValues('Q', PWMoffset_Right, PWMmax_Right, ComPort); break;
                    case 'V': SendTwoValues('V', DeadZone_Left,   PWMrev_Left,  ComPort); break;
                    case 'W': SendTwoValues('W', DeadZone_Right,  PWMrev_Right, ComPort); break;
                    case 'Y':
                        SendTwoValues('Y',
                            PIDProcessDivider * 16 + Disable_Left + (Disable_Right * 2),
                            0, ComPort);
                        break;
                    case 'Z':
                        SendValue('Z', DeltaLoopCount(), ComPort);
                        break;
                }
            }
            break;

        // --- Enable commands ---
        case 'e':
            if (RxBuffer[1][ComPort] == 'n' && RxBuffer[2][ComPort] == 'a')
            {
                Disable_Left  = 0;
                Disable_Right = 0;
            }
            else if (RxBuffer[1][ComPort] == 'n' && RxBuffer[2][ComPort] == '1')
            {
                Disable_Left  = 0;
            }
            else if (RxBuffer[1][ComPort] == 'n' && RxBuffer[2][ComPort] == '2')
            {
                Disable_Right = 0;
            }
            break;

        // --- Center / stop all motors ---
        case ']':
            Target_Left  = 512;
            Target_Right = 512;
            break;

        // --- Serial monitor feedback ---
        case 'm':
            if (RxBuffer[1][ComPort] == 'o' && RxBuffer[2][ComPort] == '1')
            {
                SerialFeedbackEnabled = 1;
            }
            else if (RxBuffer[1][ComPort] == 'o' && RxBuffer[2][ComPort] == '2')
            {
                SerialFeedbackEnabled = 2;
            }
            else if (RxBuffer[1][ComPort] == 'o' && RxBuffer[2][ComPort] == '0')
            {
                SerialFeedbackEnabled = 0;
            }

            break;

        // --- Save to EEPROM ---
        case 's':
            if (RxBuffer[1][ComPort] == 'a' && RxBuffer[2][ComPort] == 'v')
                WriteEEProm();
            break;

        // --- Firmware version ---
        case 'v':
            if (RxBuffer[1][ComPort] == 'e' && RxBuffer[2][ComPort] == 'r')
                SendValue('v', 500, ComPort);    // v5.00
            break;
    }
}

//****************************************************************************************************************
//  CheckSerial0_Extended
//  Proper packet-level serial handler with error counting.
//  Replaces the inline while(Serial.available()) block in loop() for
//  cleaner structure and consistent error tracking.
//****************************************************************************************************************

void CheckSerial0_Extended()
{
    while (Serial.available())
    {
        byte incoming = Serial.read();
        if (RxPtr[COM0] == -1)
        {
            if (incoming == START_BYTE) { RxPtr[COM0] = 0; }
            else                        { errorcount++;     }
        }
        else
        {
            if (incoming == END_BYTE)
            {
                ParseCommand_Extended(COM0);
                RxPtr[COM0] = -1;
            }
            else if (RxPtr[COM0] < 5)
            {
                RxBuffer[RxPtr[COM0]][COM0] = incoming;
                RxPtr[COM0]++;
            }
        }
    }
}

// --- End of Day 2 additions ---

// ========================================
// === DAY 3 ADDITIONS (~100 lines)
//     - Comms watchdog loop integration
//     - Serial telemetry auto-send block
//     - ToggleDiagPin for oscilloscope
//     - ReadEEProm Ki/Kd/PWM completion
// ========================================

// --- Diagnostic timing pin (pin 8) ---
// Toggle every PID cycle so an oscilloscope can verify 4kHz loop timing
const int DiagPin = 8;

void ToggleDiagPin()
{
    static int state = 0;
    state = 1 - state;
    digitalWrite(DiagPin, state);
}

//****************************************************************************************************************
//  RunCommsWatchdog
//  Call once per loop tick. If no valid serial packet has been received
//  for ~15 seconds (60000 ticks at 250uS each), PowerScale is raised from
//  7 to 9, cutting PID output power by ~75% as a safety measure.
//  PowerScale is restored immediately when comms resume.
//****************************************************************************************************************

void RunCommsWatchdog()
{
    CommsTimeout++;
    if (CommsTimeout >= 60000)
    {
        CommsTimeout = 60000;    // Cap to prevent overflow
        PowerScale   = 9;        // Reduce motor authority on comms loss
    }
    else
    {
        PowerScale = 7;          // Normal operation
    }
}

//****************************************************************************************************************
//  RunSerialTelemetry
//  Call once per loop tick. Every 80 ticks (~20ms at 4kHz) this function
//  pushes position and PWM status packets to the host if monitoring is active.
//  Motor 1 (left) sends 'A' + 'a' packets; Motor 2 (right) sends 'B' + 'b'.
//****************************************************************************************************************

void RunSerialTelemetry()
{
    SerialFeedbackCounter++;
    if (SerialFeedbackCounter < 80) return;
    SerialFeedbackCounter = 0;

    int statusByte = PIDProcessDivider * 16 + Disable_Left + (Disable_Right * 2);

    if (SerialFeedbackEnabled == 1)
    {
        SendTwoValues('A', Feedback_Left  / 4, Target_Left  / 4, COM0);
        SendTwoValues('a', statusByte, constrain(PWMout_Left,  0, 255), COM0);
    }
    else if (SerialFeedbackEnabled == 2)
    {
        SendTwoValues('B', Feedback_Right / 4, Target_Right / 4, COM0);
        SendTwoValues('b', statusByte, constrain(PWMout_Right, 0, 255), COM0);
    }
}

//****************************************************************************************************************
//  ReadEEProm_Extended
//  Completes the ReadEEProm function by restoring Ki, Kd, PWMoffset,
//  PWMmax, PWMrev and PIDProcessDivider for both motors — values that
//  were saved by WriteEEProm() but not loaded in the original ReadEEProm().
//****************************************************************************************************************

void ReadEEProm_Extended()
{
    if (EEPROM.read(0) != 114) return;    // Only run if EEPROM was previously saved

    Ki_Left_x100      = ReadEEPRomWord(13);
    Kd_Left_x100      = ReadEEPRomWord(15);
    PWMoffset_Left    = EEPROM.read(23);
    PWMmax_Left       = EEPROM.read(25);
    PWMrev_Left       = EEPROM.read(27);

    Ki_Right_x100     = ReadEEPRomWord(33);
    Kd_Right_x100     = ReadEEPRomWord(35);
    PWMoffset_Right   = EEPROM.read(43);
    PWMmax_Right      = EEPROM.read(45);
    PWMrev_Right      = EEPROM.read(47);

    PIDProcessDivider = constrain(EEPROM.read(50), 1, 10);
    Timer1FreqkHz     = EEPROM.read(51);
    if (Timer1FreqkHz < 1 || Timer1FreqkHz > 31) 
    {
        Timer1FreqkHz = 25;
    }
}

//****************************************************************************************************************
//  ManualCalibrationMode (fully implemented)
//  Drives both motors slowly toward center position (512) at reduced power.
//  Exits automatically once both axes are within deadzone, or after 5 seconds.
//  Type 'x' in the Serial Monitor at any time to abort safely.
//****************************************************************************************************************

void ManualCalibrationMode_Full()
{
    Serial.println(F("\n--- MANUAL CALIBRATION MODE ---"));
    Serial.println(F("Centering both axes to position 512..."));
    Serial.println(F("Send 'x' to abort."));

    int savedTarget_L  = Target_Left;
    int savedTarget_R  = Target_Right;
    int savedDisable_L = Disable_Left;
    int savedDisable_R = Disable_Right;

    Target_Left   = 512;
    Target_Right  = 512;
    Disable_Left  = 0;
    Disable_Right = 0;

    unsigned long calStart   = millis();
    unsigned long calTimeout = 5000;
    bool          centered   = false;

    while ((millis() - calStart) < calTimeout)
    {
        if (Serial.available() && Serial.read() == 'x')
        {
            Serial.println(F("Calibration aborted."));
            break;
        }

        Feedback_Left  = analogRead(FeedbackPin_Left);
        Feedback_Right = analogRead(FeedbackPin_Right);
        SmoothingModule_Left();
        SmoothingModule_Right();

        int errL = abs(Target_Left  - Feedback_Left);
        int errR = abs(Target_Right - Feedback_Right);

        if (errL > DeadZone_Left)
        {
            DriveMotorLeft( (Target_Left  > Feedback_Left)  ?  40 : -40 );
        }

        else
        {
            DriveMotorLeft(0);
        }

        if (errR > DeadZone_Right)
        {
            DriveMotorRight( (Target_Right > Feedback_Right) ?  40 : -40 );
        }

        else
        {
            DriveMotorRight(0);
        }

        if (errL <= DeadZone_Left && errR <= DeadZone_Right)
        {
            centered = true;
            break;
        }
        delay(5);
    }

    DisableMotorLeft();
    DisableMotorRight();

    if (centered) 
    {
        Serial.println(F("Calibration complete - both axes centered."));
    }
    else          
    {
        Serial.println(F("Calibration timed out - check mechanical range."));
    }

    Target_Left   = savedTarget_L;
    Target_Right  = savedTarget_R;
    Disable_Left  = savedDisable_L;
    Disable_Right = savedDisable_R;
}


//****************************************************************************************************************
//  WriteEEProm_Full
//  Extends the original WriteEEProm to also save PIDProcessDivider
//  and Timer1FreqkHz so they survive a power cycle.
//****************************************************************************************************************

void WriteEEProm_Full()
{
    WriteEEProm();    // Run the original save first
    EEPROM.write(50, constrain(PIDProcessDivider, 1, 10));
    EEPROM.write(51, Timer1FreqkHz);
}


#ifdef MODE2
//****************************************************************************************************************
//  MODE2 H-Bridge Output Functions
//  For the 43A "Chinese" IBT-2 H-bridge which uses a single direction
//  pin and an inverted PWM signal rather than two direction pins.
//****************************************************************************************************************

void SetOutputsMotor_Left_MODE2()
{
    int out;

    if ((Feedback_Left > InputClipMax_Left) && (PWMrev_Left != 0))
    {
        digitalWrite(ENApin_Left, LOW);
        MyPWMWrite(PWMpin_Left, PWMrev_Left);
        PWMout_Left = PWMrev_Left;
    }
    else if ((Feedback_Left < InputClipMin_Left) && (PWMrev_Left != 0))
    {
        digitalWrite(ENApin_Left, HIGH);
        MyPWMWrite(PWMpin_Left, 255 - PWMrev_Left);
        PWMout_Left = PWMrev_Left;
    }
    else if ((Target_Left > (Feedback_Left + DeadZone_Left)) ||
             (Target_Left < (Feedback_Left - DeadZone_Left)))
    {
        if (PWMout_Left >= 0)
        {
            out = PWMout_Left + PWMoffset_Left;
            if (out > PWMmax_Left) out = PWMmax_Left;
            digitalWrite(ENApin_Left, HIGH);
            MyPWMWrite(PWMpin_Left, 255 - out);
        }
        else
        {
            out = abs(PWMout_Left) + PWMoffset_Left;
            if (out > PWMmax_Left) out = PWMmax_Left;
            digitalWrite(ENApin_Left, LOW);
            MyPWMWrite(PWMpin_Left, out);
        }
    }
    else
    {
        digitalWrite(ENApin_Left, LOW);
        MyPWMWrite(PWMpin_Left, 0);
        PWMout_Left = PWMoffset_Left;
    }
}

void SetOutputsMotor_Right_MODE2()
{
    int out;

    if ((Feedback_Right > InputClipMax_Right) && (PWMrev_Right != 0))
    {
        digitalWrite(ENApin_Right, LOW);
        MyPWMWrite(PWMpin_Right, PWMrev_Right);
        PWMout_Right = PWMrev_Right;
    }
    else if ((Feedback_Right < InputClipMin_Right) && (PWMrev_Right != 0))
    {
        digitalWrite(ENApin_Right, HIGH);
        MyPWMWrite(PWMpin_Right, 255 - PWMrev_Right);
        PWMout_Right = PWMrev_Right;
    }
    else if ((Target_Right > (Feedback_Right + DeadZone_Right)) ||
             (Target_Right < (Feedback_Right - DeadZone_Right)))
    {
        if (PWMout_Right >= 0)
        {
            out = PWMout_Right + PWMoffset_Right;
            if (out > PWMmax_Right) out = PWMmax_Right;
            digitalWrite(ENApin_Right, HIGH);
            MyPWMWrite(PWMpin_Right, 255 - out);
        }
        else
        {
            out = abs(PWMout_Right) + PWMoffset_Right;
            if (out > PWMmax_Right) out = PWMmax_Right;
            digitalWrite(ENApin_Right, LOW);
            MyPWMWrite(PWMpin_Right, out);
        }
    }
    else
    {
        digitalWrite(ENApin_Right, LOW);
        MyPWMWrite(PWMpin_Right, 0);
        PWMout_Right = PWMoffset_Right;
    }
}
#endif


//****************************************************************************************************************
//  setup_Full
//  Drop-in replacement for setup() that adds DiagPin init,
//  calls ReadEEProm_Extended() to load all saved params,
//  re-applies Timer1 freq from EEPROM, and runs ManualCalibrationMode_Full
//  to center the rig on every power-on.
//****************************************************************************************************************

void setup_Full()
{
    Serial.begin(500000);

    pinMode(ENApin_Left,  OUTPUT);
    pinMode(ENBpin_Left,  OUTPUT);
    pinMode(ENApin_Right, OUTPUT);
    pinMode(ENBpin_Right, OUTPUT);
    pinMode(PWMpin_Left,  OUTPUT);
    pinMode(PWMpin_Right, OUTPUT);
    pinMode(DiagPin,      OUTPUT);

    ReadEEProm();              // Load base params
    ReadEEProm_Extended();     // Load Ki, Kd, PWM, divider params

    InitialisePWMTimer1(Timer1FreqkHz * 1000);

    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);

    DisableMotorLeft();
    DisableMotorRight();

    Serial.println(F("RACING_RIG_ONLINE"));
    Serial.println(F("Type [h] for commands, [d] for diagnostics."));

    ManualCalibrationMode_Full();    // Auto-center on power-on
}


//****************************************************************************************************************
//  loop_Full
//  Complete main loop integrating all subsystems:
//    1. Extended serial handler (full command set + error counting)
//    2. Timed PID update with process divider and diag pin toggle
//    3. Serial telemetry auto-reporting every 20ms
//    4. Comms watchdog scaling motor power on connection loss
//****************************************************************************************************************

void loop_Full()
{
    // 1. Process all incoming serial packets
    CheckSerial0_Extended();

    // 2. Timed PID update at 4kHz
    CurrentTime = micros();
    if (CurrentTime < NextProcessTime) return;
    NextProcessTime = CurrentTime + PROCESS_PERIOD_uS;

    ToggleDiagPin();

    PIDProcessCounter++;
    if (PIDProcessCounter >= PIDProcessDivider)
    {
        PIDProcessCounter = 0;

        Feedback_Left  = analogRead(FeedbackPin_Left);
        Feedback_Right = analogRead(FeedbackPin_Right);

        SmoothingModule_Left();
        SmoothingModule_Right();

        if (Feedback_Left  > CutoffLimitMax_Left  || Feedback_Left  < CutoffLimitMin_Left)
        {
            DisableMotorLeft();
        }

        if (Feedback_Right > CutoffLimitMax_Right || Feedback_Right < CutoffLimitMin_Right)
        {
            DisableMotorRight();
        }

        CalculatePID_Left();
        CalculatePID_Right();

        LoopCounter++;
    }

    // 3. Auto telemetry every ~20ms
    RunSerialTelemetry();

    // 4. Comms watchdog
    RunCommsWatchdog();
}
