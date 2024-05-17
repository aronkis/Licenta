#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>
#include <avr/io.h>

#define FALSE 0
#define TRUE  (!FALSE)

#define SYSTEM_CLOCK_FREQUENCY 8000000
#define BASE_PWM_FREQUENCY 20000
#define PWM_TOP_VALUE (SYSTEM_CLOCK_FREQUENCY / BASE_PWM_FREQUENCY / 2) //200
#define PWM_MIN_VALUE  90
#define PWM_START_VALUE 130
#define PWM_MAX_VALUE   200
#define DELAY_MULTIPLIER 100

#define AH PB5
#define BH PB4
#define CH PB3
#define AL PB2
#define BL PB1
#define CL PB0

#define PWM_PIN PD5

#define AH_BL ((SET_BIT(AH)) | SET_BIT(BL))
#define AH_CL ((SET_BIT(AH)) | SET_BIT(CL))
#define BH_CL ((SET_BIT(BH)) | SET_BIT(CL))
#define BH_AL ((SET_BIT(BH)) | SET_BIT(AL))
#define CH_AL ((SET_BIT(CH)) | SET_BIT(AL))
#define CH_BL ((SET_BIT(CH)) | SET_BIT(BL))

// ADC settings
/*  ZC_DETECTION_THRESHOLD = VIN * 255 / VREF; 
    VIN  = VBUS* / 2 = 1.3955 [V] = 1395.5 [mV], *  where VBUS is downscaled by VD/LPF
    VREF = 5000 [mV];
    ZC_DETECTION_THRESHOLD = 1395.5 * 255 / 5000 = 71
*/
// TODO: Might need to add an EXTERNAL_VOLTAGE_REFERENCE macro to compensate for small diviastions.
#define ZC_DETECTION_THRESHOLD 71

#define ADC_COIL_A_PIN  0x00
#define ADC_COIL_B_PIN  0x01
#define ADC_COIL_C_PIN  0x02
#define ADC_CURR_A_PIN  0x03
#define ADC_CURR_B_PIN  0x04
#define ADC_CURR_C_PIN  0x05
#define ADC_VBUS_PIN    0x06
#define ADC_SPD_REF_PIN 0x07
#define ADC_REF_SELECTION ((0 << REFS1) | (1 << REFS0)) // Using AVcc as the AREF
#define ADC_RES_ADJUST (1 << ADLAR) //8 bit precision
#define ADC_PRESCALER_8 ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)) // limit the ADC clock to 1 MHz
#define ADC_TRIGGER_SOURCE ((0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0)) // timer0 overflow to trigger a conversion; ADCSRB = 0
#define ADMUX_COIL_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_A_PIN)
#define ADMUX_COIL_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_B_PIN)
#define ADMUX_COIL_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_C_PIN)
#define ADMUX_CURR_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_A_PIN)
#define ADMUX_CURR_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_B_PIN)
#define ADMUX_CURR_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_C_PIN)
#define ADMUX_SPD_REF (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_SPD_REF_PIN)
#define ADMUX_VBUS    (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_VBUS_PIN)

#define SHUNT_RESISTANCE 20 //[mOhm]
#define ADC_RESOLUTION 256

#define TICKS_PER_SECOND 1000000UL
#define TICKS_PER_MINUTE (TICKS_PER_SECOND * 60)

#define RISING 0
#define FALLING 1

#define DRIVE_PORT PORTB
#define DRIVE_REG  DDRB

#define START_UP_COMMS 8
#define NUMBER_OF_STEPS 6
#define STARTUP_DELAY 10000

#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_REGISTER(reg) (~reg)

uint8_t driveTable[NUMBER_OF_STEPS];
uint8_t ADMUXTable[NUMBER_OF_STEPS];
uint8_t CurrentTable[NUMBER_OF_STEPS];
uint16_t startupDelays[START_UP_COMMS];

extern volatile uint8_t zcFlag;
extern volatile uint8_t speedUpdated;
extern volatile uint8_t currentUpdated;
extern volatile uint8_t currentHighside;
extern volatile uint8_t nextStep;
extern volatile uint8_t nextPhase;
extern volatile uint8_t motorState;
extern volatile uint8_t zeroCrossPolarity;
extern volatile uint8_t vbusVoltage;
extern volatile uint8_t debugMode;
extern volatile uint8_t speedRef;
extern volatile uint8_t shuntVoltageCoilA;
extern volatile uint8_t shuntVoltageCoilB;
extern volatile uint8_t shuntVoltageCoilC;

extern volatile uint16_t filteredTimeSinceCommutation;

void initPorts(void);
void initTimers(void);
void initADC(void);
void initComparator(void);
void enableWatchdogTimer(void);
void startupDelay(uint16_t time);
void generateTables(void);
void startMotor(void);
void changeChannel(uint8_t adcChannel);
uint8_t readChannel(uint8_t adcChannel);


inline uint8_t map(uint16_t input, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//inline functions TODO: ADD volatile to functions
inline void startADCConversion()
{
    ADCSRA |= SET_BIT(ADSC);
    return;
}

inline void waitForADCConversion()
{
    while(!ADCSRA & (1 << ADIF)) {}
}

inline void checkZeroCrossPolarity()
{
    zeroCrossPolarity = nextPhase & 0x01;
}

inline uint16_t zcDetectionHoldoffTime()
{
    return filteredTimeSinceCommutation / 2;
}

inline void setCompBTriggerValue(uint8_t timerValue)
{
    OCR0B = timerValue;
}

inline void setADCHoldoffInterrupt()
{
    TIMSK1 |= SET_BIT(OCIE1B);
} 

inline void setCommutationTimer()
{
    TIMSK1 |= SET_BIT(OCIE1A);
}

inline void disableTimer1Interrupts()
{
    TIMSK1 = 0;
}

inline void clearInterruptFlags(uint8_t reg)
{
    reg = reg;
}

inline void clearTimer1InterruptFlags()
{
    clearInterruptFlags(TIFR1);
}

inline void disableADCInterrupts()
{
    ADCSRA &= CLEAR_BIT(ADIE);
}

#endif