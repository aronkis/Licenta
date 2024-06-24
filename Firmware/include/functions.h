#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>
#include <avr/io.h>

#define FALSE 0
#define TRUE  (!FALSE)

#define SYSTEM_CLOCK_FREQUENCY 8000000
#define BASE_PWM_FREQUENCY 20000
#define PWM_TOP_VALUE (SYSTEM_CLOCK_FREQUENCY / BASE_PWM_FREQUENCY / 2) //200
#define PWM_MIN_VALUE  30
#define PWM_START_VALUE 90
#define PWM_MAX_VALUE   (PWM_TOP_VALUE - 1)
#define DELAY_MULTIPLIER 10

#define AH PB0
#define AL PB1
#define BH PB2
#define BL PB3
#define CH PB4
#define CL PB5

#define PWM_PIN PD5
#define LED_PIN PD4

// Commutation steps
#define AH_BL ((SET_BIT(AH)) | SET_BIT(BL))
#define AH_CL ((SET_BIT(AH)) | SET_BIT(CL))
#define BH_CL ((SET_BIT(BH)) | SET_BIT(CL))
#define BH_AL ((SET_BIT(BH)) | SET_BIT(AL))
#define CH_AL ((SET_BIT(CH)) | SET_BIT(AL))
#define CH_BL ((SET_BIT(CH)) | SET_BIT(BL))

// ADC settings
#define ZC_DETECTION_THRESHOLD (zcThresholdVoltage)
#define ADC_COIL_A_PIN  0x00
#define ADC_COIL_B_PIN  0x01
#define ADC_COIL_C_PIN  0x02
#define ADC_CURR_A_PIN  0x03
#define ADC_CURR_B_PIN  0x04
#define ADC_CURR_C_PIN  0x05
#define ADC_VBUS_PIN    0x06
#define ADC_SPD_REF_PIN 0x07
#define ADC_REF_SELECTION ((0 << REFS1) | (1 << REFS0)) // Using AVcc as the AREF
#define ADC_RES_ADJUST (1 << ADLAR) // 8 bit precision
#define ADC_PRESCALER_8 ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)) // limit the ADC clock to 1 MHz
#define ADC_TRIGGER_SOURCE ((0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0)) // Free running conversion;
#define ADMUX_COIL_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_A_PIN)
#define ADMUX_COIL_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_B_PIN)
#define ADMUX_COIL_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_C_PIN)
#define ADMUX_CURR_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_A_PIN)
#define ADMUX_CURR_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_B_PIN)
#define ADMUX_CURR_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_C_PIN)
#define ADMUX_SPD_REF (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_SPD_REF_PIN)
#define ADMUX_VBUS    (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_VBUS_PIN)

#define ADC_RESOLUTION 256
#define ADC_REFERENCE 5000  // [mV]
#define SHUNT_RESISTANCE 20 // [mOhm]
#define CURRENT_LIMIT 10000 // [mA]

#define RISING 1
#define FALLING 0

#define DRIVE_PORT PORTB
#define DRIVE_REG  DDRB

#define START_UP_COMMS 6
#define NUMBER_OF_STEPS 6
#define STARTUP_DELAY 10000

#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_REGISTER(reg) (reg &= ~reg)
#define CLEAR_INTERRUPT_FLAGS(reg) (reg = reg)
#define SET_COMPx_TRIGGER_VALUE(reg, value) (reg = value)
#define CHECK_ZERO_CROSS_POLARITY (zeroCrossPolarity = nextPhase & 0x01)
#define GREEN_LED (PORTD &= CLEAR_BIT(PD4))
#define RED_LED (PORTD |= SET_BIT(PD4))

volatile uint8_t nextStep;
volatile uint8_t nextPhase;
volatile uint8_t zeroCrossPolarity;
volatile uint8_t zcThresholdVoltage;
volatile uint8_t speedReference;
volatile uint8_t speedReferenceSave;
volatile uint8_t backEMFFound;
volatile uint8_t commutationState;
volatile uint8_t motorStarted;
volatile uint16_t thirtyDegreeTime;
volatile uint16_t motorStartupDelay;
volatile uint16_t motorStopCounter;
uint8_t programState;
uint8_t debugMode;

uint8_t driveTable[NUMBER_OF_STEPS];
uint8_t ADMUXTable[NUMBER_OF_STEPS];
uint8_t currentTable[NUMBER_OF_STEPS];
uint64_t startupDelays[START_UP_COMMS];

void initPorts(void);
void initTimers(void);
void initADC(void);
void initComparator(void);
void enableWatchdogTimer(void);
void startupDelay(uint64_t time);
void generateTables(void);
void startMotor(void);
void stopMotor(void);
void checkForMotorStop(void);
void checkForStartMotor(void);

#endif