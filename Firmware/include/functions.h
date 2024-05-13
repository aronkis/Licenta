#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
#define ADC_MEASURE

#include <stdint.h>

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
    VIN  = VBUS* / 2 = 1.3955, *  where VBUS is downscaled by VD/LPF
    VREF = 5000;
    ZC_DETECTION_THRESHOLD = 13955. * 255 / 5000 = 71
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
#define ADC_TRIGGER_SOURCE ((1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0)) // timer0 overflow to trigger a conversion
#define ADMUX_COIL_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_A_PIN)
#define ADMUX_COIL_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_B_PIN)
#define ADMUX_COIL_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_COIL_C_PIN)
#define ADMUX_CURR_A  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_A_PIN)
#define ADMUX_CURR_B  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_B_PIN)
#define ADMUX_CURR_C  (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_CURR_C_PIN)
#define ADMUX_SPD_REF (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_SPD_REF_PIN)
#define ADMUX_VBUS    (ADC_REF_SELECTION | ADC_RES_ADJUST | ADC_VBUS_PIN)
#define START_ADC_CONVERSION (ADCSRA |= SET_BIT(ADSC))
#define WAIT_FOR_ADC_CONVERSION (while (ADCSRA & (1 << ADSC)) {})
#define CHECK_ZERO_CROSS_POLARITY (zeroCrossPolarity = nextPhase & 0x01)

#define SHUNT_RESISTANCE 5 //tbd
#define ADC_RESOLUTION 256
#define TICKS_PER_MINUTE (1000000UL * 60)

// Comparator settings
#ifdef COMPARATOR_MEASURE
#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)
#endif

#ifdef ADC_MEASURE
#define RISING 0
#define FALLING 1
#endif 

#define DRIVE_PORT PORTB
#define DRIVE_REG  DDRB

#define START_UP_COMMS 8
#define NUMBER_OF_STEPS 6
#define STARTUP_DELAY 10000
#define ZC_DETECTION_HOLDOFF_TIME (filteredTimeSinceCommutation / 2) 
#define SET_COMPB_TRIGGER_VALUE(timerValue) (OCR0B = timerValue)


#define CLEAR_INTERRUPT_FLAGS(reg) (reg = reg)
#define SET_BIT(bitPos) (1 << bitPos)
#define CLEAR_BIT(bitPos) (~(1 << bitPos))
#define CLEAR_REGISTER(reg) (~reg)
#define SET_TIMER0_ZC_DETECTION_INT (TIMSK0 |= SET_BIT(TOIE0))
#define SET_TIMER1_HOLDOFF_INT (TIMSK1 |= SET_BIT(OCIE1B))
#define SET_TIMER1_COMMUTATE_INT (TIMSK1 |= SET_BIT(OCIE1A))
#define DISABLE_ALL_TIMER1_INTS (TIMSK1 = 0)
#define DISABLE_ALL_TIMER0_INTS (TIMSK0 = 0)
#define constrain(value, min, max) (value < min ? min : \
                                    value > max ? max : value)
#define map(input, in_min, in_max, \
                   out_min, out_max) ( (input - in_min) * (out_max - out_min) \
                                      /(in_max - in_min) + out_min)

uint8_t driveTable[NUMBER_OF_STEPS];
uint8_t ADMUXTable[NUMBER_OF_STEPS];
uint16_t startupDelays[START_UP_COMMS];
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
uint8_t readChannel(uint8_t adcChannel);

#endif