// ================================================================================
// ===
// === (c) 2021 Avio Sports Limited
// ===
// === v20.3 Working :)
// ===
// ================================================================================
 
// #define DKM_DEBUG_ON 1

/*
Changes to version v20.3 GOLD v0.1
Date				Author          Change Description
17/02/21     tsr			      Add patch to ignore incorrect high spikes in Power  with 20ms
17/02/21     tsr            Revert the timer interval to 20ms
22/01/21     tsr            change the timer interval to 30ms
			

Changes to version v15.19
--Added if (power < 0) power=0; line number = 1708
--Added Rowing Force read from line number#1533 
--coppied all version 15-17 
By: Wasil Ali
15/12/2017
*/
 
#include <stdint.h>
#include <string.h>
#include "math.h"
#include "nrf_drv_clock.h"
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "ble_nus.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ant_parameters.h"
#include "ant_interface.h"
#include "bsp.h"
#include "peer_manager.h"
#include "pstorage.h"
#include "pstorage.h"
#include "ble_conn_state.h"
#include "ant_error.h"
#include "ant_stack_config.h"
#include "ant_key_manager.h"
#include "ant_bpwr.h"
#include "ant_bpwr_simulator.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//#include "pstorage.h"
#include "nrf_error.h"
#include "nrf_delay.h"


#include "SEGGER_RTT.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

//#define WAKEUP_BUTTON_ID                0                                            /**< Button used to wake up the application. */
//#define BOND_DELETE_ALL_BUTTON_ID       1                                            /**< Button used for deleting all bonded centrals during startup. */

#define DEVICE_NAME                     "AvioPowerSense"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                        /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                800                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 333 ms). */
//#define APP_ADV_TIMEOUT_IN_SECONDS    180                                          /**< The advertising timeout in units of seconds. */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                          /**< The advertising timeout in units of seconds. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                            /**< Whether or not to include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device */
#define APP_TIMER_PRESCALER             0                                            /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                            /**< Size of timer operation queues. */

#define SECOND_1_25_MS_UNITS            300                                          /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                          /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 2)                   /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS)                       /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define CENTRAL_LINK_COUNT              0                                            /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                            /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define ANT_HRMRX_ANT_CHANNEL           0                                            /**< Default ANT Channel. */
#define ANT_HRMRX_DEVICE_NUMBER         0                                            /**< Device Number. */
#define ANT_HRMRX_TRANS_TYPE            0                                            /**< Transmission Type. */
#define ANTPLUS_NETWORK_NUMBER          0                                            /**< Network number. */

#define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
#define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
#define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0


//volatile uint8_t state = 1;

unsigned long millis(void);
void init_watchdog(void);
void kickTheDog(void);
void saveSystemData(void);
void toggleRedLED(void);
int16_t readADXL345(void);
int16_t readADXL345_2(void);
volatile int averagedADCReading=0;
double readTemp(void);
void initialiseFlashForFirstTime(void);

char deviceName[20] ={0};
nrf_saadc_value_t batteryADC;
volatile int bufferIsFree=1;
volatile int flashOperationComplete=1;
volatile int triggerCalibrateOnPowerUp = 3;
double rawAngle0360;
nrf_saadc_value_t lastADCReadingNTC=0;
volatile int zeroOffsetSameCounter=0;
volatile int zeroLoopCounter=0;
volatile int deltaZeroOffsetTemp=0;
volatile int doNotTurnOff=0;
volatile uint8_t reportBatteryNotPower=0;
volatile uint8_t debugOutputEnabled=0;

// ================================================================================
// ================================================================================
// ================================================================================
//#define CURRENT_BLOCK_START 0

//volatile int currentBlock=CURRENT_BLOCK_START;	
volatile double calculatedAngle=0.0, angularVelocity=0.0;
volatile double avgADCReadingNTC=0.0;

const uint16_t MCP4561_ADDR = 0x2E; //A0  to GND = 0x5C

// Register memory address
#define MCP4561_WIPER0 0x00
#define MCP4561_WIPER0_NV 0x02
#define MCP4561_TCON 0x04
#define MCP4561_STATUS 0x05

// Command Operation
#define MCP4561_WRITE 0x00
#define MCP4561_READ (0x03 << 2)
#define MCP4561_INC (0x01 << 2)
#define MCP4561_DEC (0x02 << 2)

// TCON Register Bits
#define MCP4561_TCON_GCEN (1 << 8) /* General Call Enable bit*/
#define MCP4561_TCON_R0HW (1 << 3) /* Hardware configuration control bit*/
#define MCP4561_TCON_R0A (1 << 2) /* Terminal A connect control bit*/
#define MCP4561_TCON_R0W (1 << 1) /* Wiper connect control bit*/
#define MCP4561_TCON_R0B (1 << 0) /* Terminal B connect control bit */

#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)
    #error Unsupported value of MODIFICATION_TYPE.
#endif

#ifndef SENSOR_TYPE // can be provided as preprocesor global symbol
    #define SENSOR_TYPE (TORQUE_NONE)
#endif

#define BPWR_CHANNEL_NUMBER         0x00 /**< Channel number assigned to Bicycle Power profile. */
#define ANTPLUS_NETWORK_NUMBER      0       /**< Network number. */
//#define CALIBRATION_DATA            0x55AAu /**< General calibration data value. */
#define CALIBRATION_DATA            0x1234u /**< General calibration data value. */

#define TIME_TAKEN_FOR_PERIPHERALS_TO_SETTLE_DOWN 250

// --------------------------------------------------------------------------------
// Memory locations
// --------------------------------------------------------------------------------

//#define FLASH_MEMORY_MAGIC_KEY_ADDRESS 10
//#define FLASH_MEMORY_CALIBRATION_ADDRESS 20

// --------------------------------------------------------------------------------
// Hardware connections
// --------------------------------------------------------------------------------

#define ANALOG1 5
#define ANALOG2 3
#define RED_LED 19      
#define GREEN_LED 20    
#define POWER_DOWN_BOARD 13
#define POWER_DOWN_PERIPHERAL 7

#define MCP4561_DESIRED_VALUE 200
#define MARGIN_OF_ADC_ERROR_ALLOWED 25
#define DEFAULT_10KG_CALIBRATION_ADC_VALUE 200.0

#define	ASSIGN_ANT_ID	1
#define DEADBEEF 0xDEADBEEF

uint8_t	nrfSwitch = 31;
uint8_t oneShotDone = 4;
// ================================================================================
// ================================================================================
// ================================================================================

void init_watchdog(void);
void kickTheDog(void);
void saveSystemData(void);
void goToSleep(void);	
// --------------------------------------------------------------------------------
// General variables
// --------------------------------------------------------------------------------
pstorage_handle_t handle;
pstorage_module_param_t param;

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

//char s[200];
volatile int power=0;
int calibratedPower = 0;
int stroke=0;
int triggerCalibration=0;
volatile int triggerSave=0;
volatile int enablePowerMeasurement=0;
volatile double	cadence=0.0;
volatile long millisCounter=0;

volatile int rtcCounter=0;
volatile uint32_t retval=0;
volatile int oldCalc[11]={0};

static int32_t temperature=21.0;
volatile double temperatureCompensation=1.0;

//volatile int oldPower1=-1, oldPower2=-1;

// ==========================================================================================	
// === Measure power
// ==========================================================================================	

volatile int rawAngle;
volatile int angle=0, angleX=0, angleY=0,angleZ=0;

volatile int rest1=0,oldPower=0,numberOfForceReadings=0,strokes=0;
volatile double force1=0.0;
volatile double avg1=0, avTorque=0.0;
//volatile long cadencePeriod=0, timeStartOfStrokeTimer1=0, timeStartOfStrokeMillis=0;
volatile double cadencePeriod=0;
//volatile long cadencePeriod=0, timeStartOfStrokeMillis=0;
volatile double batteryFilter=-1;
volatile int measureBattery =3000;  // set high so it goes into measure battery loop in first pass
double restingAngle=0;
nrf_saadc_value_t k=0;
volatile int connected=0;

volatile int dkmPower=0;
volatile int dkmCadence=0;
volatile uint8_t doNotGoToSleep=0;

/*Spike Removal Variables*/
volatile int lastValidPower = 0;
volatile int spikesOccurence = 0;
// ------------------------------------------------------------------------------------

volatile double energy=0.0;
volatile int energyStoredForDebug=0;
volatile int stopped=1;
volatile int bleSendState=0;

volatile double smoothedRawADCNTC =0.0;
volatile double smoothedRawADCLessRestingNTC =0.0;
int inPowerUpCheck = 1;
int oldADCValues[51] = {0}; 

//volatile int sendCount = 0;
volatile int sendBLEMessage=0;
//volatile long timeSinceLastTransition=0;
volatile int angleGoneBetweenLimits=0;
volatile int batteryReporting =0;
#define ANGLE_CHANGE_TIME 25
volatile int oldAngles[ANGLE_CHANGE_TIME+1]={0};
volatile int angleStoppedCount=0;
volatile int batteryPercentage = 75;
int CheckLEDStatus = 0;
int counter=150;
// --------------------------------------------------------------------------------
// pStorage variables (ie flash)
// --------------------------------------------------------------------------------

uint8_t   sourceData[8] = {0xDE,0xAD,0xBE,0xEF,0,0,0,0};

typedef struct {
	uint8_t deadBeef1;
	uint8_t deadBeef2;
	uint8_t deadBeef3;
	uint8_t deadBeef4;
	int calibrationValueNTC;
	int startingDigitalPotValue;
	int restingForceNTC;	
	int newANTNumber;

	volatile int powerModifier;	
	volatile int calibrationTemperature;	
	volatile int zeroOffsetTemperature;
	char name[32];
} systemData_t;

systemData_t systemData __attribute__((aligned(4)));

// --------------------------------------------------------------------------------
// TWI variables
// --------------------------------------------------------------------------------
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static volatile bool m_xfer_done = false;
//static uint8_t m_sample;

// --------------------------------------------------------------------------------
// RTC variables
// --------------------------------------------------------------------------------

#define LOGGING_SPEED_10MS 2 //tsr changes
//volatile int loggingSpeed=5; 
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); //< Declaring an instance of nrf_drv_rtc for RTC0. 

// --------------------------------------------------------------------------------
// ADC variables
// --------------------------------------------------------------------------------

#define SAMPLES_IN_BUFFER 5
//static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
//static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
//static nrf_ppi_channel_t     m_ppi_channel;
//static uint32_t              m_adc_evt_counter;

static volatile uint16_t                m_conn_handle = BLE_CONN_HANDLE_INVALID;     /**< Handle of the current connection. */
static ble_gap_adv_params_t             m_adv_params;                                /**< Parameters to be passed to the stack when starting advertising. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
char debugMessage[100]={0};

/** @snippet [ANT BPWR TX Instance] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);
//volatile int force=0;
volatile double force=0.0;
//nrf_saadc_value_t rawForceFromAmp1=0;
//volatile int averagedADCReading=0;
//nrf_saadc_value_t tempRawForceFromAmp1Old1=0;
//nrf_saadc_value_t tempRawForceFromAmp1Old2=0;

BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUMBER,
                             CHAN_ID_TRANS_TYPE,
                             CHAN_ID_DEV_NUM,
                             ANTPLUS_NETWORK_NUMBER);
BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                            (ant_bpwr_torque_t)(SENSOR_TYPE),
                            ant_bpwr_calib_handler,
                            ant_bpwr_evt_handler);

ant_bpwr_profile_t m_ant_bpwr;
/** @snippet [ANT BPWR TX Instance] */

ant_bpwr_simulator_t m_ant_bpwr_simulator;    /**< Simulator used to simulate profile data. */

/*
BPWR_DISP_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUMBER,
                             WILDCARD_TRANSMISSION_TYPE,
                             WILDCARD_DEVICE_NUMBER,
                             ANTPLUS_NETWORK_NUMBER);
BPWR_DISP_PROFILE_CONFIG_DEF(m_ant_bpwr,
                             ant_bpwr_evt_handler);

*/
// --------------------------------------------------------------------------------
// Debug routines
// --------------------------------------------------------------------------------
char s[200] = {0};

#define Dbg1_AlwaysOn(a,b)     	{ sprintf(s,a,b);   	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }
#define Dbg2_AlwaysOn(a,b,c)   	{ sprintf(s,a,b,c); 	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }

#define Dbg(a)     	   	{ if (debugOutputEnabled==1) {sprintf(s,a);   	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg1(a,b)     	{ if (debugOutputEnabled==1) {sprintf(s,a,b);   	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg2(a,b,c)   	{ if (debugOutputEnabled==1) {sprintf(s,a,b,c); 	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg3(a,b,c,d) 	{ if (debugOutputEnabled==1) {sprintf(s,a,b,c,d); 	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg4(a,b,c,d,e) 	{ if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e); 	SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg5(a,b,c,d,e,f) { if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e,f); SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg6(a,b,c,d,e,f,g) { if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e,f,g); SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg7(a,b,c,d,e,f,g,h) { if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e,f,g,h); SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg8(a,b,c,d,e,f,g,h,i) { if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e,f,g,h,i); SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define Dbg9(a,b,c,d,e,f,g,h,i,j) { if (debugOutputEnabled==1) {sprintf(s,a,b,c,d,e,f,g,h,i,j); SEGGER_RTT_WriteString(0, s); SEGGER_RTT_WriteString(0, "\r\n"); }}
#define SEND_DEBUG(a)    	{ if (debugOutputEnabled==1) {SEGGER_RTT_WriteString(0, a); 	}}
#define DbgNoCRLF(a)     	   	{ if (debugOutputEnabled==1) {sprintf(s,a);   	SEGGER_RTT_WriteString(0, s); }}

// ================================================================================
// ================================================================================
// ================================================================================

//volatile int triggerEraseAll=0;
//char startTimeStr[15]={0};
//char note[15]={0};
char name[20];
char nameToUpdate[20]={0};

void redLEDOff(void);
void redLEDOn(void);

void endTimingWithLED(void);
void startTimingWithLED(void);
void greenLEDOff(void);
void greenLEDOn(void);
void flashRedLED(void);
void flashRedLED1(int);
double getCurrentForceInNewton(void);
uint32_t changeAntID(int id);

void flashRedLED(void)
{
	redLEDOn();
	nrf_delay_ms(10);
	redLEDOff();
}

void	flashRedLED1(int a){
	int i;
	nrf_gpio_cfg_output(19);
	nrf_gpio_cfg_output(20);
	kickTheDog();
	for (i=0;i<a;i++) {
	uint32_t gpio_state = NRF_GPIO->OUT;      
	NRF_GPIO->OUTSET = ((1 << 19) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << 19) & gpio_state);
	nrf_delay_ms(30); 
	NRF_GPIO->OUTSET = ((1 << 20) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << 20) & gpio_state);
	nrf_delay_ms(30); 
	}
}
//unsigned long timeWhenCalibrationLastCalled=0;
//int numberOfTimesCalibrationCalledQuickly=0;
//int calbrationWith10KgDone=0;

// ================================================================================
// ================================================================================
// ================================================================================

uint16_t mcp4561_Read(uint8_t memAddr);
uint8_t mcp4561_Write(uint8_t memAddr,uint8_t cmd, uint16_t data);
//uint8_t initDigitalPotentiometer();

void setDigitalPotentiometerResistance(int val);

// ================================================================================
// ================================================================================
// ================================================================================

uint8_t mcp4561_Write(uint8_t memAddr, uint8_t cmd, uint16_t data2)
{
    uint8_t setVal = 0;
    uint8_t data[10];

	  //Dbg2("In MCP4561 write, mem=%u, val=%u", memAddr, data2);
	
    setVal = ((memAddr << 4) & 0xF0); // 4b Memory address      
    setVal |= cmd; // 2b Command operation      
    setVal |= (((data2 & 0x01FF) >> 8) & 0x03); // 2b Data

    data[0]=setVal;
    data[1]=data2;
		//ret_code_t retCode; dlm 20.7

		nrf_drv_twi_tx(&m_twi, MCP4561_ADDR, data, 2, true);
		//retCode = nrf_drv_twi_tx(&m_twi, MCP4561_ADDR, data, 2, true); dkm 20.7
		//APP_ERROR_CHECK(retCode);
//		Dbg1("Done %d",retCode);
		nrf_delay_ms(10);

	
    return 0;
}

// ================================================================================
// ================================================================================
// ================================================================================

static void setDigitalPotentiometerResistance(int val)
{
	mcp4561_Write(MCP4561_WIPER0, MCP4561_WRITE, val);
}

// ================================================================================
// ================================================================================
// ================================================================================

void reset()
{
	char s[30];
	nrf_delay_ms(100);
	sprintf(s,"FATAL-Reset caused\r\n");
    SEGGER_RTT_WriteString(0,s);	
	
	#define RED_LED 19
	#define POWER_DOWN_BOARD 13
	int i=0;
	for (i=0;i<5;i++) {
		nrf_gpio_cfg_output(RED_LED);
		NRF_GPIO->OUTSET = (1 << RED_LED);
		nrf_delay_ms(1000);
		NRF_GPIO->OUTCLR = (1 << RED_LED);
		nrf_delay_ms(250);
		}
	
	nrf_gpio_cfg_output(POWER_DOWN_BOARD);
	NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
	nrf_delay_ms(250);
	
	//nrf_gpio_cfg_output(POWER_DOWN_BOARD);
	//NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
	NVIC_SystemReset();
	
}


// ================================================================================
// ================================================================================
// ================================================================================

int loopCounter=0;
int MCP4561Value = 10;

void calibrationInTheLoop(){
	
	if (zeroLoopCounter++ < 10) return;
	zeroLoopCounter=0;
	
	kickTheDog();

	if(inPowerUpCheck == 1){
		if (systemData.startingDigitalPotValue==0) systemData.startingDigitalPotValue=128;
		MCP4561Value=systemData.startingDigitalPotValue;
		setDigitalPotentiometerResistance(MCP4561Value);
		inPowerUpCheck = 2;
	}

	int inAllowedZone=1;
	if (abs(MCP4561_DESIRED_VALUE - averagedADCReading) > MARGIN_OF_ADC_ERROR_ALLOWED) inAllowedZone=0;
	if (inAllowedZone==1) 
		zeroOffsetSameCounter++;
	else 
		zeroOffsetSameCounter=0;

	if (zeroOffsetSameCounter > 20) triggerCalibrateOnPowerUp = 2;             			    
	
	if (inAllowedZone==0) {
		if (averagedADCReading > MCP4561_DESIRED_VALUE) {
			MCP4561Value--;
			Dbg1("Gone down, value is %d",MCP4561Value);
			if (MCP4561Value < 1) {
				MCP4561Value = 0;
				Dbg("MCP4561 err, cant go low");
				reset();
				}
			setDigitalPotentiometerResistance(MCP4561Value);
			}

		if (averagedADCReading < MCP4561_DESIRED_VALUE) {
			MCP4561Value++;
			Dbg1("Gone up, value is %d",MCP4561Value);
			if (MCP4561Value >= 254) {
				MCP4561Value = 254;
				Dbg("MCP4561 err, cant go high");
				reset();
				}
			setDigitalPotentiometerResistance(MCP4561Value);
			}
		}

	Dbg5("Calib: Raw=%d, average=%d, Allowed=%u, Counter=%u, MCP=%u", lastADCReadingNTC, averagedADCReading, inAllowedZone, (int)(zeroOffsetSameCounter), MCP4561Value);
	
	if(triggerCalibrateOnPowerUp == 2){		
		systemData.startingDigitalPotValue = MCP4561Value;	
		systemData.restingForceNTC = averagedADCReading;
		if(systemData.restingForceNTC == 0){triggerCalibrateOnPowerUp = 1;}
		Dbg2("MCP4561 val%d, rest%d", MCP4561Value,systemData.restingForceNTC);
	}
}


// *************************************************************************************************************************************************************************
// *************************************************************************************************************************************************************************
// *************************************************************************************************************************************************************************

//int rawForceLessRestingTC=0;
volatile double rawADCLessRestingNTC=0.0;
int forceInN=0;
volatile double forceInNewtonsTC=0.0f;
volatile int lastGoodADCReading=0;
#define MAXIMUM_ALLOWED_DROP_PER_READING 150
volatile double d=0,e=200;

double getCurrentForceInNewton(void)
{

		long sumOfAllADCReadingsNTC = 0;
		int numberOfSamples, i;

	
		enablePowerMeasurement=0;

		nrf_gpio_cfg_output(POWER_DOWN_PERIPHERAL);	
	  NRF_GPIO->OUTSET = (1 << POWER_DOWN_PERIPHERAL);                         //Switch on Strain gauge
	
	  nrf_delay_us(400);
	
		// throw away the first readings
		nrf_drv_saadc_sample_convert(0, &lastADCReadingNTC);														//read opamp
		nrf_delay_us(25);
	/*@tsr changes for optimising the code */
		nrf_drv_saadc_sample_convert(0, &lastADCReadingNTC);														//read opamp
		nrf_delay_us(25);
		nrf_drv_saadc_sample_convert(0, &lastADCReadingNTC);														//read opamp
		nrf_delay_us(25);
		
    ret_code_t retCode;
//		int aa[21]={0};
		numberOfSamples = 0; 
		int tries=0;
		while ((numberOfSamples < 1) && (tries++ < 20)) {
			retCode=nrf_drv_saadc_sample_convert(0, &lastADCReadingNTC);														//read opamp
			
//			if (retCode == NRF_SUCCESS) {	
//			if (1==1) {	
//					aa[numberOfSamples]=lastADCReadingNTC;
					numberOfSamples++;
//					if (lastGoodADCReading ==0) lastGoodADCReading=lastADCReadingNTC;
//					if ((lastGoodADCReading - lastADCReadingNTC) < MAXIMUM_ALLOWED_DROP_PER_READING) 
	//						lastGoodADCReading = lastADCReadingNTC; 
	//				else 
	//						lastADCReadingNTC = lastGoodADCReading-1;
					sumOfAllADCReadingsNTC += lastADCReadingNTC;
	//				}
		}		
/*
		int j;
		for (j=0;j<numberOfSamples;j++) {
				Dbg3("Adding %d = %d, sum=%ld",j,aa[j],sumOfAllADCReadingsNTC);
				}
*/
		nrf_gpio_cfg_input(POWER_DOWN_PERIPHERAL,NRF_GPIO_PIN_PULLDOWN);
		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);												//Switch off Strain gauge

		if (numberOfSamples==0) numberOfSamples=1; // avoids div 0 error
		avgADCReadingNTC = sumOfAllADCReadingsNTC / numberOfSamples;
		
//---------------------For Zero Offset-------------------------------		
/*		unsigned long sumOfValues=0;
		#define NUMBER_OF_ADC_READINGS_TO_AVERAGE 20
				
		for(i = 0; i < NUMBER_OF_ADC_READINGS_TO_AVERAGE ; i++){
			sumOfValues = sumOfValues + oldADCValues[i];
			oldADCValues[i] = oldADCValues[i+1];
//			Dbg3("old %d = %d = %ld",i,oldADCValues[i],sumOfValues);
			}
		oldADCValues[NUMBER_OF_ADC_READINGS_TO_AVERAGE-1] = avgADCReadingNTC;
//		sumOfValues = sumOfValues + avgADCReadingNTC;

		for(i = 0; i < NUMBER_OF_ADC_READINGS_TO_AVERAGE ; i++){
//			Dbg3("old %d = %d = %ld",i,oldADCValues[i],sumOfValues);
			}
			
		averagedADCReading = sumOfValues/NUMBER_OF_ADC_READINGS_TO_AVERAGE;
Dbg4("Average=%d (last=%ld), sum=%ld, readings=%d",averagedADCReading,(long)avgADCReadingNTC, sumOfValues, NUMBER_OF_ADC_READINGS_TO_AVERAGE);
*/
//d = 1.0 * avgADCReadingNTC;
//e = e * 0.99 + 0.01 * d;
//averagedADCReading=(int)e;
averagedADCReading=avgADCReadingNTC;
//Dbg2("Average=%d (last=%ld)",averagedADCReading,(long)avgADCReadingNTC);
			
//-------------------------------------------------------------------		

		rawADCLessRestingNTC = avgADCReadingNTC - systemData.restingForceNTC - deltaZeroOffsetTemp;                      //this is for 10kg calibration only
		
		int deltaADC=0;
		deltaADC = avgADCReadingNTC - systemData.restingForceNTC;
		
		if (systemData.calibrationValueNTC == 0) return(0);
		//if (temperatureCompensation == 0) temperatureCompensation=100;

		double forceInNewtonsNTC = 0.0;
		//calibrationADCtoN = 	1.0 *  / temperatureCompensation;
		forceInNewtonsNTC = 98.1 * deltaADC / systemData.calibrationValueNTC;
		forceInNewtonsTC = forceInNewtonsNTC;// * temperatureCompensation;
			
/*
		Dbg8("avgADCReadingNTC=%d, sumOfValues=%ld, averagedADCReading=%d, restingForceNTC=%d, deltaADC=%d, calibrationValueNTC=%d, calibrationADCtoN=%d, ForceN=%d", 
				(int)avgADCReadingNTC,
				sumOfValues,
				averagedADCReading,
				systemData.restingForceNTC, 
				deltaADC,
				systemData.calibrationValueNTC,
				(int)calibrationADCtoN,
				(int)forceInNewtonsTC);
*/			
		smoothedRawADCLessRestingNTC   =  smoothedRawADCLessRestingNTC   * 0.96f + 0.04f * rawADCLessRestingNTC; // calib10kg
		smoothedRawADCNTC   =  smoothedRawADCNTC   * 0.9f + 0.1f * avgADCReadingNTC; // zero offset
			
		enablePowerMeasurement = 1;

		return(forceInNewtonsTC);
		
}



// *************************************************************************************************************************************************************************
// *************************************************************************************************************************************************************************
// *************************************************************************************************************************************************************************
	
/** @snippet [ANT BPWR calibration] */
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
//	Dbg1("***************Calib, count=%d******************",numberOfTimesCalibrationCalledQuickly);
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:	
						//systemData.restingForceNTC = smoothedRawADCNTC;
//						triggerSave=1;
						m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
						m_ant_bpwr.BPWR_PROFILE_general_calib_data = 200;
//						m_ant_bpwr.BPWR_PROFILE_general_calib_data = round(smoothedRawADCNTC);
						ant_bpwr_calib_response(&m_ant_bpwr);
						zeroOffsetSameCounter=0;
						triggerCalibrateOnPowerUp=1;
		//				Dbg1("ANT_BPWR_CALIB_ID_MANUAL Rest force reset %d",systemData.restingForceNTC);
						break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
    //        Dbg("ANT_BPWR_CALIB_ID_AUTO");
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
       //     Dbg("ANT_BPWR_CALIB_ID_CUSTOM_REQ");
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
     //       Dbg("ANT_BPWR_CALIB_ID_CUSTOM_UPDATE");
            break;

        default:
	//	Dbg1("Got unknown CALIB %d",p_page1->calibration_id);
            break;
    }
}


/** @snippet [ANT BPWR calibration] */
void nrf_power_gpregret_set(uint8_t);



static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
//		double d;
//	int end=0,i;
	
	Dbg4("%s data|%s|,strlen=%d%s",RTT_CTRL_BG_RED,(char *)p_data, strlen((char *)p_data),RTT_CTRL_BG_BLACK);
	unsigned int antID=0;
	int whatTheNewNumberIs;

	switch (p_data[0]) {
		
	case 'f':
		Dbg("ENG-c = Factory reset");
		zeroOffsetSameCounter=0;
		triggerCalibrateOnPowerUp=1;
		flashRedLED();
		break;

	
	case '1':
//		Dbg("ENG-1 = 10kg Calib(nb. numeral one)");
		systemData.calibrationValueNTC = smoothedRawADCLessRestingNTC;
		if ((systemData.calibrationValueNTC < 20) || (systemData.calibrationValueNTC > 120)) systemData.calibrationValueNTC = 78;
		systemData.calibrationTemperature = temperature;
		//Dbg3("RFLR=%d,tc=%d,sysCalib=%d",(int)rawADCLessRestingNTC, (int)(100.0 * temperatureCompensation), systemData.calibrationValueNTC);
		triggerSave=1;
		flashRedLED();
		break;
	
	case 'z':
		Dbg("ENG-z = Zero offset"); 
		zeroOffsetSameCounter=0;
		triggerCalibrateOnPowerUp=1;
//		systemData.restingForceNTC = smoothedRawADCNTC;
//		systemData.zeroOffsetTemperature = temperature;
//		triggerSave=1;
		flashRedLED();
		break;
	
	case 'm':
//		Dbg1("ENGINE - m = change power modifier to |%s|", (char * ) & p_data[1]);	
		systemData.powerModifier = (p_data[1] -'0')  * 100 + (p_data[2] -'0') * 10 + (p_data[3] -'0');
		Dbg1("ENG-m = chge pwr mod to int %d", systemData.powerModifier);
		triggerSave=1;
		break;
	
	case 'n':
		Dbg1("ENG-n = change name to |%s|",(char *)&p_data[1]);	
	
		if ((p_data[1]== 's') && 			(p_data[2]== 't') && 			(p_data[3]== 'a') && 			(p_data[4]== 'y') && 			(p_data[5]== ' ') && 			(p_data[6]== 'o') && 			(p_data[7]== 'n') ) {
				Dbg("Command: Do not turn unit off");
				doNotTurnOff=1;
				int i;
				for (i=0;i<150; i++) {
					flashRedLED();
					nrf_delay_ms(10);
					}
				return;
				}		
	
		//for (i=1;i<=16;i++) if (p_data[i]==';') end=i;
		if (strlen((char *)p_data) <=16) { 
			//p_data[0]='$';
			strcpy(nameToUpdate,(char *)&p_data[1]);
			}
		else {
			//Dbg("Name too long");
			return;
			}	
		flashRedLED();
		nrf_delay_ms(500);
		flashRedLED();
			
		break;

	case 'd':
		kickTheDog();
		sd_power_gpregret_set(0,1);
		Dbg("DFU Triggered");
		Dbg1("GPREGRET %d", NRF_POWER -> GPREGRET);
		NVIC_SystemReset();
		break;	
	
	case 'a':       //change ANT+ ID
		whatTheNewNumberIs = (p_data[1] -'0')  * 1000 + (p_data[2] -'0') * 100 + (p_data[3] -'0') * 10 + (p_data[4] -'0');
	
		if (whatTheNewNumberIs == 9191) {
				Dbg("Debug mode - sending battery ADC not power");
				reportBatteryNotPower=1;
				doNotGoToSleep=1;
				doNotTurnOff=1;
				redLEDOn();
				}
		
		if (whatTheNewNumberIs == 9292) {
				Dbg("Debug mode - turning off auto power down");
				doNotGoToSleep=1;
				doNotTurnOff=1;
				redLEDOn();
				}
		
		if (whatTheNewNumberIs == 9393) {
				Dbg("Debug mode - Flash reset");
				initialiseFlashForFirstTime();
				while (1) {};  // reset
				}
		
		if (whatTheNewNumberIs == 9494) {
				Dbg("Debug mode - turning debug output on");
				debugOutputEnabled=1;
				redLEDOn();
				return;
				}
		
		systemData.newANTNumber = whatTheNewNumberIs;
		nrf_delay_ms(20);
	
		antID = systemData.newANTNumber;
		nrf_delay_ms(20);
		changeAntID(antID);
		nrf_delay_ms(15);
		triggerSave = 1;
		break;
	
	default:
		flashRedLED();
		nrf_delay_ms(100);
		flashRedLED();
		nrf_delay_ms(100);
		flashRedLED();
//		Dbg("NUS Recvd, Unkwn");
		break;
	}
	
}


#define WATCHDOG_COUNTER        (32768 * 5)

void init_watchdog(void)
{
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = WATCHDOG_COUNTER;    
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
    NRF_WDT->TASKS_START = 1;
}

void kickTheDog(void)
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;  //Reload watchdog register 0
} 


unsigned long millis(void)
{
	return(millisCounter);
}

// ================================================================================
// ================================================================================
// ================================================================================

void redLEDOff()
{
	NRF_GPIO->OUTCLR = (1 << RED_LED);
}
void endTimingWithLED()
{
	NRF_GPIO->OUTCLR = (1 << RED_LED);
}
// ================================================================================
// ================================================================================
// ================================================================================

void greenLEDOff()
{
	NRF_GPIO->OUTCLR = (1 << GREEN_LED);
}
// ================================================================================
// ================================================================================
// ================================================================================

void redLEDOn()
{
	NRF_GPIO->OUTSET = (1 << RED_LED);
}
void startTimingWithLED()
{
	NRF_GPIO->OUTSET = (1 << RED_LED);
}
// ================================================================================
// ================================================================================
// ================================================================================

void greenLEDOn()
{
	NRF_GPIO->OUTSET = (1 << GREEN_LED);
}
// ================================================================================
// ================================================================================
// ================================================================================

void toggleRedLED()
{
	nrf_gpio_cfg_output(RED_LED);
	
	uint32_t gpio_state = NRF_GPIO->OUT;      
	NRF_GPIO->OUTSET = ((1 << RED_LED) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << RED_LED) & gpio_state);
}

// ================================================================================
// ================================================================================
// ================================================================================

void toggleGreenLED()
{
	nrf_gpio_cfg_output(GREEN_LED);
	
	uint32_t gpio_state = NRF_GPIO->OUT;      
	NRF_GPIO->OUTSET = ((1 << GREEN_LED) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << GREEN_LED) & gpio_state);
}

// ================================================================================
// ================================================================================
// ================================================================================

void startupFlashLEDs ()
{
	int i;
	nrf_gpio_cfg_output(19);
	nrf_gpio_cfg_output(20);
	kickTheDog();
	for (i=0;i<20;i++) {
	uint32_t gpio_state = NRF_GPIO->OUT;      
	NRF_GPIO->OUTSET = ((1 << 19) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << 19) & gpio_state);
	nrf_delay_ms(100); 
	NRF_GPIO->OUTSET = ((1 << 20) & ~gpio_state); 
	NRF_GPIO->OUTCLR = ((1 << 20) & gpio_state);
	nrf_delay_ms(100); 
	}
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEADBEEF, line_num, p_file_name);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }

}



/**@brief Attempt to both open the ant channel and start ble advertising.
*/
static void ant_and_adv_start(void)
{
    advertising_start();
//    ant_hrm_rx_start();
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */

static void timers_init(void)
{
    // Initialize timer module
    //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
	//PWM disable timers, to avoid device bricking, after ATO DFU
    // See: devzone.nordicsemi.com/.../
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_SHUTDOWN=1;
    NRF_TIMER1->EVENTS_COMPARE[0]=0;
    NRF_TIMER1->EVENTS_COMPARE[1]=0;
    NRF_TIMER1->EVENTS_COMPARE[2]=0;
    NRF_TIMER1->EVENTS_COMPARE[3]=0;
    NVIC_DisableIRQ(TIMER1_IRQn);

    // Initialize timer module.
   // APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false); //scheduler not used

}


/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

//!!! DKM 15.9    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
//    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
 


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */


static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

#define BLE_UUID_NUS_SERVICE 0x0001                      /**< The UUID of the Nordic UART Service. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
//#define BLE_UUID_TYPE_VENDOR_BEGIN  0x02 /**< Vendor UUID types start at this index (128-bit). */

//	ble_uuid_t                       adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

	
    ble_uuid_t adv_uuids[] = {
      {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
			{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE},
      {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
			};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialise advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

uint8_t p_data[100]={0x02,0x01,0x06,0x1b,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x67,0xF7,0xDB,0x34,0xC4,0x03,0x8E,0x5C,0x0B,0xAA,0x97,0x30,0X56,0xE6};
uint8_t sr_data[31] = {0};

void updateAdvertisingData(void)
{
	char t[20]={0};
	sprintf(t,"%0.6X", NRF_FICR->DEVICEADDR[0]);

	sr_data[0] = 0x13;  // ad field length = 19 bytes (0x13)
	sr_data[1] = 0x09;   // ad field type = 0x09 (Complete local name)
	
	sr_data[2] = 'A';  
	sr_data[3] = 'v';  
	sr_data[4] = 'i';  
	sr_data[5] = 'o';  
	sr_data[6] = ' ';  	

	if (deviceName[0]==0) {
		sr_data[7] = t[4];
		sr_data[8] = t[5];
		sr_data[9] = t[6];
		sr_data[10] = t[7];
		}
	else {
		int i;
		for (i=7;i<19;i++) sr_data[i]=0;
	//	Dbg3("%sChanged name to |%s|%s",RTT_CTRL_BG_GREEN,deviceName,RTT_CTRL_BG_BLACK);
		strcpy((char *)&sr_data[7],deviceName);
	}

	p_data[15]=batteryPercentage / 256; // power 
	p_data[16]=batteryPercentage % 256; // power  
	p_data[17]=(int)power / 256; // energy hi
	p_data[18]=(int)power % 256; // energy lo
	p_data[19]=strokes / 256; // strokes
	p_data[20]=strokes % 256; // strokes
	p_data[21]=(int)(averagedADCReading) / 256; // raw force
	p_data[22]=(int)(averagedADCReading) % 256; // raw force
	p_data[23]=(int)force / 256; // raw force
	p_data[24]=(int)force % 256; // raw force
//	p_data[25]=displayCadence; // rate
	p_data[25]=(int)(cadence); // rate
	p_data[26]=rawAngle / 256; // rate
	p_data[27]=rawAngle % 256; // rate
	p_data[28]++; // m

	//strcpy((char *)&sr_data[2],SENSOR_NAME);

//	int err_code = sd_ble_gap_adv_data_set(p_data, 31, sr_data, 31);
	//Dbg2("Errcode=%u %u",err_code,sizeof(advdata));

	//strcpy((char *)&sr_data[2],SENSOR_NAME);

	sd_ble_gap_adv_data_set(p_data, 31, sr_data, 31);
	//Dbg2("Errcode=%u %u",err_code,sizeof(advdata));
	
}

/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
	//m_nus.is_notification_enabled=true;  // added by dkm 17.3.17
    APP_ERROR_CHECK(err_code);	

}


/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */

/*
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

	Dbg("on_conn_params_evt");
	
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(err_code);
	}

}
*/

/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/* 
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
*/

/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
/*	
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

	Dbg1("Error BB was %d",err_code);
	nrf_delay_ms(100);
    err_code = ble_conn_params_init(&cp_init);
	Dbg1("Error AA was %d",err_code);
	nrf_delay_ms(100);
    APP_ERROR_CHECK(err_code);
*/	
}


/**@brief ANT CHANNEL_CLOSED event handler.
 */

static void on_ant_evt_channel_closed(void)
{
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
//        ant_hrm_rx_start();
			}
    else {
        ant_and_adv_start();
			}
}

/**@brief Application's Stack ANT event handler.
 *
 * @param[in]   p_ant_evt   Event received from the stack.
 */
/*
static void on_ant_evt(ant_evt_t * p_ant_evt)
{
    if (p_ant_evt->channel == ANT_HRMRX_ANT_CHANNEL)
    {
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
//                ant_hrm_disp_evt_handler(&m_ant_hrm, p_ant_evt);
                break;

            case EVENT_CHANNEL_CLOSED:
                on_ant_evt_channel_closed();
                break;

            default:
                // No implementation needed.
                break;
        }
    }
}
*/	
/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR simulator call] */

volatile long lastTimeSent0=0;
volatile long lastTimeSent1=0;

void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
//	Dbg1("Got event %d",event);
    switch (event)
    {
        case ANT_BPWR_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_16_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_17_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_18_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_80_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_81_UPDATED:
            /* fall through */
        case ANT_BPWR_PAGE_82_UPDATED:
            ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR simulator call] */


/**@brief Application's Stack BLE event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

//	Dbg1("BLE Event %d",p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_EVT_TX_COMPLETE:
	//Dbg("EVENT BLE_EVT_TX_COMPLETE");
						bufferIsFree=1;	
            break; 
	
        case BLE_GAP_EVT_CONNECTED:
            Dbg("EVENT conn");
						m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						connected = 1;
						greenLEDOn();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            Dbg("EVENT Disconn");
						m_conn_handle = BLE_CONN_HANDLE_INVALID;
						connected = 0;
						bleSendState=0;
						greenLEDOff();

            // Need to close the ANT channel to make it safe to write bonding information to flash
            err_code = sd_ant_channel_close(ANT_HRMRX_ANT_CHANNEL);
            APP_ERROR_CHECK(err_code);
						advertising_start();
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
            break;

        case BLE_GAP_EVT_TIMEOUT:
						Dbg("EVENT Tout");
	
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING) {
                // err_code = bsp_buttons_enable((1 << WAKEUP_BUTTON_ID) | (1 << BOND_DELETE_ALL_BUTTON_ID));
                // APP_ERROR_CHECK(err_code);
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
//								Dbg("R3");
								err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
								}
            break;

						#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
						//#define NRF_BLE_MAX_MTU_SIZE            30                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

					case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
						Dbg("EVENT MTU");
						err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
						APP_ERROR_CHECK(err_code);
						break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
						
					default:
//						Dbg1("BLE event other %d",p_ble_evt->header.evt_id);
            // No implementation needed.
            break;
    }
}


/**@brief Dispatches a stack event to all modules with a stack BLE event handler.
 *
 * @details This function is called from the Stack event interrupt handler after a stack BLE
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Stack Bluetooth event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
//                ant_hrm_disp_evt_handler(&m_ant_hrm, p_ant_evt);
                break;

            case EVENT_CHANNEL_CLOSED:
                on_ant_evt_channel_closed();
                break;

            default:
                // No implementation needed.
                break;
        }
    ant_bpwr_sens_evt_handler(&m_ant_bpwr, p_ant_evt);
}

/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void ble_ant_stack_init(void)
{
    uint32_t	err_code;
		Dbg("a1");
//		if(nrf_drv_clock_lfclk_is_running()){
//			Dbg("Clock ok");
//		}
		
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
		nrf_delay_ms(100);
//		Dbg("a2");
    // Initialize SoftDevice
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
		nrf_delay_ms(100);
		Dbg("a2a");

    // Initialize BLE stack
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);
//		Dbg("a3");
 
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
//		Dbg("a3a");
		nrf_delay_us(1000);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
//		Dbg("a3b");
		nrf_delay_us(100);
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
//		Dbg("a4");
	
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Subscribe for ANT events.
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);
		//Dbg("a4a");

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);

//		Dbg("a5");
		nrf_delay_ms(100);
}

//volatile int oldRawForceFromAmp1=0;
//volatile int oldOldRawForceFromAmp1=0;
/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code;

    // Wait for events
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void profile_setup(void)
{
/** @snippet [ANT BPWR TX Profile Setup] */
    uint32_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,BPWR_MANUFACTURER_ID,BPWR_MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,BPWR_SW_REVISION_MINOR,BPWR_SERIAL_NUMBER);

    // fill product's common data page.
    m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_GOOD);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);
//    err_code = ant_state_indicator_channel_opened();
//    APP_ERROR_CHECK(err_code);
/** @snippet [ANT BPWR TX Profile Setup] */
}

void simulator_setup(void)
{
    /** @snippet [ANT BPWR simulator init] */
    const ant_bpwr_simulator_cfg_t simulator_cfg =
    {
        .p_profile   = &m_ant_bpwr,
        .sensor_type = (ant_bpwr_torque_t)(SENSOR_TYPE),
    };

    /** @snippet [ANT BPWR simulator init] */

#if MODIFICATION_TYPE == MODIFICATION_TYPE_AUTO
    /** @snippet [ANT BPWR simulator auto init] */
    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, true);
    /** @snippet [ANT BPWR simulator auto init] */
#else
    /** @snippet [ANT BPWR simulator button init] */
    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, false);
    /** @snippet [ANT BPWR simulator button init] */
#endif
}
/**@brief Application main function.
 */



// ==========================================================================================	
// === Measure power
// ==========================================================================================	


void resetKeyVariables () 
{
	cadence=0.0;
	power=0.0;
	oldPower=0;
	energy=0.0;
	numberOfForceReadings=0;	
	angleGoneBetweenLimits=0;
	CheckLEDStatus = 1;
//	timeStartOfStrokeMillis = 0;
	
	m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = 0;
	m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=0;
	//m_ant_bpwr_simulator.p_profile->page_17.accumulated_torque=round(force);	
	
//	updateAdvertisingData();			
}	

volatile int oldTimer1Value=0;

#define BATTERY_AVERAGE_SIZE 15
volatile int batteryAverage[BATTERY_AVERAGE_SIZE + 1]={0};
void measurePower(void)
{
	//nrf_gpio_cfg_output(POWER_DOWN_PERIPHERAL);	
	//NRF_GPIO->OUTSET = (1 << POWER_DOWN_PERIPHERAL);
	
//		nrf_delay_us(600);
	
		// ------------------------------------------------------------------------------------
		// --- Housekeeping
		// ------------------------------------------------------------------------------------

		kickTheDog();
		
		// ------------------------------------------------------------------------------------
		// --- Battery and temperature
		// ------------------------------------------------------------------------------------
	
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
  	
//			if (measureBattery++ > 1000) {
//			int oldTemperature = temperature;
				/*             
				int a;
				sd_temp_get(&a);
				if (smoothedTemp == -40.0) {
					//Dbg1("Setting smoothed Temp for first time to %d",a);
					smoothedTemp=a;
					temperature=a/4;
					}
				if (a < (4*60)) { // sometimes the blooming thing reads 154 deg C, clearly wrong, just ignore it.
					smoothedTemp = 0.9 * smoothedTemp + 0.1 * a;
					temperature=smoothedTemp/4;		
					}
				*/
/*				
				temperature = readTemp(); 
				
				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  No Temp Compensation
				
				//temperatureCompensation = 1.0 + ((( - ) / systemData.calibrationTemperature) * 0.1);  
//				temperatureCompensation = 1.0;
				
				//((272.0/268.0) / 5.0) = 1.1765
				#define TEMPERATURE_COMPENSATION_PERCENTAGE 0.011765
				int deltaTemperature = temperature - systemData.zeroOffsetTemperature;
				temperatureCompensation = 1.0 + deltaTemperature * TEMPERATURE_COMPENSATION_PERCENTAGE;
				double d1;
				d1 = (7.0/9.0) * (temperature - systemData.zeroOffsetTemperature);
				deltaZeroOffsetTemp = round(d1);
				temperatureCompensation =1.0;
				
//				temperatureCompensatedRestingForce = systemData.restingForceNTC / temperatureCompensation;
	
				//Dbg4("temp=%d, ADC (a)=%d, smoothedTemp=%3.2f, TemperatureComp = %3.2f",temperature, a, smoothedTemp, temperatureCompensation);
				//nrf_gpio_cfg_input(battery, NRF_GPIO_PIN_PULLUP);	
				measureBattery=0;
				nrf_drv_saadc_sample_convert(1, &k);	
				batteryReporting=k;
				if (batteryFilter==-1.0) batteryFilter=k;
				batteryFilter = 0.99 * batteryFilter + 0.01 * k;
				batteryADC = batteryFilter / 4;
				Dbg2("Batt=%d, batFilt=%d",batteryADC, (int)batteryFilter);
				//batteryADC = batteryReporting / 4;

				if (batteryFilter < 210) {
					m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = 999;
					m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=0;
					power=999;
					cadence=0;
					updateAdvertisingData();
					return;
					}
				
			//	Dbg1("Battery %d", batteryADC);
				if ((batteryADC >= 210) && (batteryADC <= 219)) m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_CRITICAL);
				if ((batteryADC >= 220) && (batteryADC <= 229)) m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_LOW);
				if ((batteryADC >= 230) && (batteryADC <= 239)) m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_OK);
				if ((batteryADC >= 240) && (batteryADC <= 249)) m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_GOOD);
				if (batteryADC > 250) m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_NEW);
					
				batteryPercentage = (int)((100 * (batteryADC - 210)) / 45.0);
				if (batteryPercentage > 100) batteryADC = 100;
				if (batteryPercentage < 0) batteryADC =0;
				}
			//Dbg1("Battery1 %d", batteryADC);
			*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

			if (measureBattery++ > 1000) { // 1000 * 20mS = once every 20 seconds
				measureBattery=0;
//			int oldTemperature = temperature;
				/*             
				int a;
				sd_temp_get(&a);
				if (smoothedTemp == -40.0) {
					//Dbg1("Setting smoothed Temp for first time to %d",a);
					smoothedTemp=a;
					temperature=a/4;
					}
				if (a < (4*60)) { // sometimes the blooming thing reads 154 deg C, clearly wrong, just ignore it.
					smoothedTemp = 0.9 * smoothedTemp + 0.1 * a;
					temperature=smoothedTemp/4;		
					}
				*/
				temperature = readTemp(); 
				
				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  No Temp Compensation
				
				//temperatureCompensation = 1.0 + ((( - ) / systemData.calibrationTemperature) * 0.1);  
//				temperatureCompensation = 1.0;
				
				//((272.0/268.0) / 5.0) = 1.1765
				#define TEMPERATURE_COMPENSATION_PERCENTAGE 0.011765
				int deltaTemperature = temperature - systemData.zeroOffsetTemperature;
				temperatureCompensation = 1.0 + deltaTemperature * TEMPERATURE_COMPENSATION_PERCENTAGE;
				double d1;
				d1 = (7.0/9.0) * (temperature - systemData.zeroOffsetTemperature);
				deltaZeroOffsetTemp = round(d1);
				temperatureCompensation =1.0;
				
//				temperatureCompensatedRestingForce = systemData.restingForceNTC / temperatureCompensation;
	
				//Dbg4("temp=%d, ADC (a)=%d, smoothedTemp=%3.2f, TemperatureComp = %3.2f",temperature, a, smoothedTemp, temperatureCompensation);
				//nrf_gpio_cfg_input(battery, NRF_GPIO_PIN_PULLUP);	
						nrf_drv_saadc_sample_convert(1, &k);	
						batteryReporting=k;
						dkmPower= k/2;
						dkmCadence++;
						if (dkmCadence >=4) dkmCadence=0;
					//	Dbg2("BatteryReport raw=%d, halved=%d", batteryReporting, batteryReporting/2);					
						batteryReporting=batteryReporting/2;
			
						int i;
						// -------------------------------------------------------
						if (batteryAverage[0]==0) {
							// run for first time
							for (i=0;i<=BATTERY_AVERAGE_SIZE;i++) {
								batteryAverage[i]=batteryReporting;
								}
							}
						// -------------------------------------------------------
						unsigned long batteryTotal=0;								
						
						for (i=0;i<BATTERY_AVERAGE_SIZE;i++) {
								batteryTotal = batteryTotal + batteryAverage[i];
								batteryAverage[i] = batteryAverage[i+1];
								}
						
						#define BATTERY_MINIMUM_ACCEPTABLE_READING 200
						if (batteryReporting >= BATTERY_MINIMUM_ACCEPTABLE_READING) {
								batteryAverage[BATTERY_AVERAGE_SIZE] = batteryReporting;
								batteryTotal = batteryTotal + batteryReporting;
								}
						else { 
								batteryAverage[BATTERY_AVERAGE_SIZE] = batteryAverage[BATTERY_AVERAGE_SIZE-1];
								batteryTotal = batteryTotal + batteryAverage[BATTERY_AVERAGE_SIZE];
								}
						batteryTotal = batteryTotal / (BATTERY_AVERAGE_SIZE + 1);
						batteryADC = batteryTotal ;
								
						// -------------------------------------------------------				
								
						//Dbg1("Batt average=%d",batteryADC);

						if (batteryADC < 245) {
							m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = 999;
							m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=0;
							power=999;
							cadence=0;
							updateAdvertisingData();
							measureBattery=3000; // so we come into this loop again next time.
							m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_CRITICAL);
							return;
							}
						
						if((batteryADC >= 245) && (batteryADC <= 249)){ 
								m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_CRITICAL);
								}
						
						if((batteryADC >= 250) && (batteryADC <= 252)){ 
								m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_LOW);
								}
						
						if((batteryADC >= 253) && (batteryADC <= 276)){
								m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_OK);
								}
						
						if((batteryADC >= 277) && (batteryADC <= 299)){ 
								m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_GOOD);
								}
						
						if(batteryADC >= 300){ 
								m_ant_bpwr.page_82 = ANT_COMMON_page82(22,BATTERY_STATUS_NEW);
								}
						
						batteryPercentage = (int)((100 * (batteryADC - 245)) / 71.0);

						if(batteryPercentage > 100) batteryPercentage = 100;						
						if(batteryPercentage < 0) batteryPercentage = 0;
						
						Dbg1("Battery Percentage = %d", batteryPercentage);
					}

				
		// ------------------------------------------------------------------------------------
		// --- Filter force
		// ------------------------------------------------------------------------------------
					force = getCurrentForceInNewton();			

		//if (force>maxTorque) maxTorque=force1;
					
		//energy=energy + force;
		energy=energy + force;		
		numberOfForceReadings++;
		energyStoredForDebug = energy;		
	
		// ------------------------------------------------------------------------------------
		// --- Read the angle
		// ------------------------------------------------------------------------------------

		int oldAngle=angle;
//		if ((dkmCadence == 0) || (dkmCadence == 1)) 
//			angle=readADXL345();
//		else
//			Dbg("Got here 12");
			angle=readADXL345();
//			Dbg("Got here 14");

		// -------------------------------------------------------------------------
    // --- Check for auto power off
    // -------------------------------------------------------------------------
 
 // / * DKM 21 June 2019, uncommented this block
 
 
		if (doNotGoToSleep == 0) {
			if (millis() > 300000) {
				if (doNotTurnOff == 0) {
					Dbg("Sleep (1)\r\n");
					NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);
					NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
					NRF_GPIO->OUTCLR = (1 << nrfSwitch);
					while (1) {
						toggleRedLED();
						nrf_delay_ms(2000);
						toggleGreenLED();
						nrf_delay_ms(2000);
						NRF_GPIO->OUTCLR = (1 << nrfSwitch);
						NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
						}
					}
				}
			}
		
		
// * /  DKM 21 June 2019
		// ------------------------------------------------------------------------------------
    // --- Not pedalling ?
		// ------------------------------------------------------------------------------------

		int oldStopped=stopped;
		#define MAX_TIME_BETWEEN_STROKES 3000
		
		// --------------------------
		int i;
		int angleIsChanging=0;
		
		for (i=0;i<ANGLE_CHANGE_TIME;i++) {
			oldAngles[i]=oldAngles[i+1];
			}
		oldAngles[ANGLE_CHANGE_TIME-1]=angle;
		if (oldAngles[0] < (angle - 20)) angleIsChanging=1;
		if (oldAngles[0] > (angle + 20)) angleIsChanging=1;
			
		if (angleIsChanging==1) 
			angleStoppedCount=0;
		else {
			if (angleStoppedCount<250) angleStoppedCount++;
			}
		// --------------------------
		
		angleZ=angleStoppedCount;
//		Dbg4("A=%d, OA0=%d, ASC=%d, AIC=%d",angle,oldAngles[0],angleStoppedCount,angleIsChanging);
		
		if (angleStoppedCount>25) {
			// we've stopped	
			stopped=1;			
			resetKeyVariables();
//			return;
			}
		
/*			
		else {
			stopped=0;
			}
*/
		if ((oldStopped==1) && (stopped==0)) {
			//energy=0;
			resetKeyVariables();
			}

			
	// ------------------------------------------------------------------------------------------------------------------------

	// ------------------------------------------------------------------------------------
	// --- Check for a new stroke
	// ------------------------------------------------------------------------------------

		#define	ANGLE_THRESHOLD 270
		volatile int transitionOccurred=0;
		//if (timeSinceLastTransition	==0) timeSinceLastTransition=millis();
//		if ((angle > 20) && (angle < 100)) angleGoneBetweenLimits=1;	
		if (angle < 70) angleGoneBetweenLimits=1;	
			
		if ((oldAngle <= ANGLE_THRESHOLD) && ( angle > ANGLE_THRESHOLD) && (angleGoneBetweenLimits==1)) {
			//transition occurred
			if (millis() >= (60000/150)) { // 150 = max rpm
				transitionOccurred=1;
				angleGoneBetweenLimits=0;
				//timeSinceLastTransition=millis();
				}
			}		
		
	// ------------------------------------------------------------------------------------
	// --- Check for debug info
	// ------------------------------------------------------------------------------------

		if (reportBatteryNotPower == 1) {
				m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = dkmPower;	
				m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=round(dkmCadence);
				}
			
		if (transitionOccurred==1){
			
			//==================================================================================================================================	
			//==================================================================================================================================	
			//=== New stroke
			//==================================================================================================================================	
			//==================================================================================================================================	
			
			//Dbg("STROKE");
			strokes++;
			millisCounter=0;

			NRF_TIMER1->TASKS_CAPTURE[0] =1;			
			int timer1Value=NRF_TIMER1->CC[0];
			int strokeTime = timer1Value - oldTimer1Value;
			//Dbg2("Counter is %u, millis=%lu",strokeTime,millis());
			oldTimer1Value=timer1Value;
			
			cadencePeriod=1.0 * strokeTime * (20.0 / 627.0);  // timer1 fires 627 times in 20mS.
//			cadencePeriod=millis() - timeStartOfStroke;

			if (stopped == 1) {
				resetKeyVariables();
				//timeStartOfStrokeMillis=millis();
				stopped=0;				
				return;
				}
			//timeStartOfStrokeMillis=millis();		
			
			double dCadence; 
			if (cadencePeriod > 0) 
				dCadence = 60000.0/ cadencePeriod;
			else
				dCadence =0;

			cadence = round(dCadence);
//			cadence  = (dCadence + oldCadence1 ) / 2.0;
//			oldCadence1 = dCadence;			
			
			double dPower=0.0;
			double averageForceOverLastStroke=0.0;
			
			if  (numberOfForceReadings > 0) { 
				//dPower=1.0 * energy;
				averageForceOverLastStroke = energy / (1.0 * numberOfForceReadings);
				}
			else 
				averageForceOverLastStroke=0;

			// NB crank length = 172.5cm = 0.1725m
			#define LENGTH_OF_GOLDEN_CRANK (0.1725)
			avTorque = averageForceOverLastStroke * LENGTH_OF_GOLDEN_CRANK;
			
			double radiansPerSecond;
			radiansPerSecond = (2.0 * 3.14159265359) * (dCadence/60.0); 
			dPower = 2.0 * avTorque * radiansPerSecond; // the last 2 is because we're a left only crank

			// ----------------------------------------------------------------------------------
			
/*
			// Spike removal
			if (oldPower1==-1) oldPower1=dPower;
			if (oldPower2==-1) oldPower2=dPower;
			
			if ((dPower - oldPower1) > 150) {
				// Maybe a spike
				if (abs(oldPower1-oldPower2) < 50) {
					// we were steady state before, probably not a spike
					dPower= oldPower1;
					}
				else {
					// Nope, we were genuinely going up beforehand
					// just ignore this
					}
				}
*/			
			double d = 0.0;
			d = dPower * ((double)systemData.powerModifier / 100.0);
			power = round(d); 
			
			
//			oldPower2 = oldPower1;
//			oldPower1 = power;
						
			if (power < 0) power = 0;
			if (power > 2000) power = 2000;
			
			/*High Power Spikes Removal*/
      if ( power > 1500 ) /* Power exceeds 1500 */
			{
				if( spikesOccurence++ > 50 )
				{ 
					power = lastValidPower;
					reset();
				}
			  power = lastValidPower;
			}
			else /* case for the normal Power recorded*/
			{
				spikesOccurence = 0;
				lastValidPower = power;
			}
				
			

			energy=0.0;
			numberOfForceReadings=0;

			if (reportBatteryNotPower == 1) {
				m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = dkmPower;	
				m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=round(dkmCadence);
				}
			else {
				m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = power;	
				m_ant_bpwr_simulator._cb.cadence_sensorsim_state.current_val=round(cadence);
			//updateAdvertisingData();
				}
			

			
			//==================================================================================================================================	
			//==================================================================================================================================	
			//=== END OF NEW STROKE 
			//==================================================================================================================================	
			//==================================================================================================================================	
			} 
						
		if (connected==1) {
				if (sendBLEMessage==1)
						sendBLEMessage=0;
				else {
						sendBLEMessage=1;
						int a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p;
						a=(int)angle/2;
						b=(int)(cadence);
						c=(int)power; // int
						d=strokes % 256; 
						e=lastADCReadingNTC; // int
//						f=batteryADC;
						f=(int)batteryPercentage;
						g=temperature;
						h=round(force); // int
						i=energyStoredForDebug/2; //int // energyStoredForDebug
						//i=systemData.restingForceNTC; //int // 
						j=systemData.calibrationValueNTC;
						k=systemData.powerModifier;						
//						k=0;
						//l=avTorque;
//						l=(int)rawAngle0360/2.0;
						l=angleX/2;
						m=angleY/2;
						n=systemData.restingForceNTC;
						o=MCP4561Value;
						p=(int)(temperatureCompensation * 100.0);
						p=(int)(dkmPower - 200);
						if (a<0) a=0;
						if (b<0) b=0;
						if (c<0) c=0;
						if (d<0) d=0;
						if (e<0) e=0;
						if (f<0) f=0;
						if (g<0) g=0;
						//if (h<0) h=0;
						if (i<0) i=0;
						if (j<0) j=0;
						if (k<0) k=0;
						if (l<0) l=0;
						if (m<0) m=0;
						if (n<0) n=0;
						if (o<0) o=0;
						if (p<0) p=0;
						if (a>255) a=255;
						if (b>255) b=255;
						if (c>65535) c=65535;
						if (d>255) d=255;
						if (e>65535) e=65535;
						if (f>255) f=255;
						if (g>255) g=255;
						if (h>65535) h=65535;
						if (i>65535) i=65535;
						if (j>255) j=255;
						if (k>255) k=255;
						if (l>255) l=255;
//						if (l>65535) l=65535;
						if (m>255) m=255;
						if (n>255) n=255;
						if (o>255) o=255;
						if (p>255) p=255;
						uint8_t debugMessage2[22]={0};
						debugMessage2[0]=a;
						debugMessage2[1]=b;
						debugMessage2[2]=c/256;
						debugMessage2[3]=c%256;
						debugMessage2[4]=d;
						debugMessage2[5]=e/256;
						debugMessage2[6]=e%256;
						debugMessage2[7]=f;
						debugMessage2[8]=g;
						
						//-------------------------------------------------------------
						// Allows negative numbers to be transmitted
						union {
							int intValue;
							uint8_t byteValues[2];
						} bytesArray;
						bytesArray.intValue=h;						
						debugMessage2[9]=bytesArray.byteValues[0];
						debugMessage2[10]=bytesArray.byteValues[1];
						//-------------------------------------------------------------
						
						debugMessage2[11]=i/256;
						debugMessage2[12]=i%256;
						debugMessage2[13]=j;
						debugMessage2[14]=k;
						debugMessage2[15]=l;
						debugMessage2[16]=m;
						debugMessage2[17]=n;
						debugMessage2[18]=o;
						debugMessage2[19]=p;
						if (bufferIsFree==1) ble_nus_string_send(&m_nus, debugMessage2, 20);
		//        updateAdvertisingData();
				}
			}
			//oldForce=force;
		// ------------------------------------------------------------------------------------
		// --- Sample
		// ------------------------------------------------------------------------------------
	//		Dbg("Got here 15");

	//		Dbg("Got here 16");

//		nrf_gpio_cfg_input(POWER_DOWN_PERIPHERAL,NRF_GPIO_PIN_PULLDOWN);
//		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);

}

// ============================================================================================================
// ================================================Sleep=======================================================
// ============================================================================================================
void	goToSleep(){
			Dbg("Battery Too Low - Sleep\r\n");
			NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);
			NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
			NRF_GPIO->OUTCLR = (1 << nrfSwitch);
			while (1) {
				toggleRedLED();
				nrf_delay_ms(2000);
				toggleGreenLED();
				nrf_delay_ms(2000);
				NRF_GPIO->OUTCLR = (1 << nrfSwitch);
				NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
				}
}

// ============================================================================================================
// ============================================================================================================
// ============================================================================================================

uint8_t ledCounter=0;
//volatile int dkmLoopCounter=0;	
	
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	//    millisCounter = millisCounter + 1;
  //		millisCounter = millisCounter + nrf_drv_rtc_counter_get(&rtc);
    if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
				millisCounter = millisCounter + 20;
				if (millisCounter > 500000) millisCounter = 500000;
        //Dbg1("RTC %d",dkmLoopCounter++);
        nrf_drv_rtc_cc_set(&rtc,0,LOGGING_SPEED_10MS,true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        nrf_drv_rtc_counter_clear(&rtc);                          //Clear the RTC counter to start count from zero
        if (enablePowerMeasurement==1) {
					//startTimingWithLED();
          measurePower();
					//endTimingWithLED();

        }

				if ((batteryPercentage >= 15) && (batteryPercentage <= 35)){
					counter = 250;									// Flash LED twice every 5 seconds
				}
				else{
			  	counter = 100;                   // Flash LED once every 2 seconds
				}
				
				if (connected==1){ 
					counter = 25; 
				}
				/*		
				if ((ledCounter++ > counter && (CheckLEDStatus == 1))) {
          ledCounter=0;
					if((batteryPercentage >= 6) && (batteryPercentage <= 35)){
						greenLEDOff();
						flashRedLED1(3);
					}
					else if(batteryPercentage <= 5){
						greenLEDOff();
						flashRedLED1(7);
						goToSleep();
					}
					else{
						greenLEDOff();
						redLEDOn();						
					}
					CheckLEDStatus = 0;
			 }
			 else{
					redLEDOff();
				}
			 */
			 if	((ledCounter++ > counter && (CheckLEDStatus == 1) && (connected==0))) {
          ledCounter=0;
					if((batteryADC >= 266) && (batteryADC <= 270)){
						greenLEDOff();
						flashRedLED1(3);
					}
					else if((batteryADC <= 265)){
						greenLEDOff();
						flashRedLED1(7);
						goToSleep();
					}
					else{
						greenLEDOff();
						redLEDOn();						
					}
					CheckLEDStatus = 0;
			 }
			 else{
					redLEDOff();
				}
    }
}

void rtcInit()
{
		
	nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
	config.prescaler = 327; // 32768 / 327 = 10mS
	config.tick_latency=0;

	ret_code_t err_code2 = nrf_drv_rtc_init(&rtc, &config, rtc_handler);              //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the nrf_drv_config.h file.
    APP_ERROR_CHECK(err_code2);

    err_code2 = nrf_drv_rtc_cc_set(&rtc,0,LOGGING_SPEED_10MS,true);           //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC2_CONFIG_FREQUENCY constant in the nrf_drv_config.h file
    APP_ERROR_CHECK(err_code2);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);   
}

// ================================================================================
// ================================================================================
// ================================================================================

static int bufferIsFreeTimeout=0;
void saveSystemData()
{
	Dbg("Saving system data");
	uint32_t retval=pstorage_clear(&handle,64); 
//	Dbg1("Cleared, retval was %d",retval);	
//tsr	Dbg8("pStorage save, data is |%s|0x%X|0x%X|0x%X|0x%X| calib=%d, digitalPotStart=%d, restingForceNTC=%d",systemData.name,systemData.deadBeef1,systemData.deadBeef2,systemData.deadBeef3,systemData.deadBeef4,systemData.calibrationValueNTC,systemData.startingDigitalPotValue, systemData.restingForceNTC);	
	pstorage_store(&handle, &systemData.deadBeef1, sizeof(systemData), 0); //store test data in block 0
}	

void loadSystemData()
{
	uint32_t count=10;
	uint32_t retval;
	
	Dbg("Loadng sys data");
	
	systemData.deadBeef1 = 0x00;
	systemData.deadBeef2 = 0x00;
	systemData.deadBeef3 = 0x00;
	systemData.deadBeef4 = 0x00;
	systemData.calibrationValueNTC = 77;	 
	systemData.startingDigitalPotValue=80;
	systemData.restingForceNTC=200;
	systemData.calibrationTemperature=24;
	systemData.zeroOffsetTemperature=24;
	//systemData.restingForce22=70;
	systemData.powerModifier = 100;
	systemData.newANTNumber = 100;
	strcpy(systemData.name,"PS1");
	
	pstorage_access_status_get(&count);
	if (count != 0) {
		Dbg3("%sERR%s pStorage count pending ops =%d",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK,count);
		flashRedLED();
		nrf_delay_ms(10);
		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);
		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
		NRF_GPIO->OUTCLR = (1 << nrfSwitch);		
		return;
		}

	retval = pstorage_load(&systemData.deadBeef1, &handle, sizeof(systemData), 0); //try to load test data
	if (retval != NRF_SUCCESS) {
		Dbg2("%sERR%s Cant read flash",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK);
		flashRedLED();
		nrf_delay_ms(10);
		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);
		NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
		NRF_GPIO->OUTCLR = (1 << nrfSwitch);	
		return;
		}
	if ((systemData.deadBeef1 == 0xDE) && (systemData.deadBeef2 == 0xAD) && (systemData.deadBeef3 == 0xBE) && (systemData.deadBeef4 == 0xEF)) {
			Dbg4("Updating name to |%s%s%s| (%d)",RTT_CTRL_BG_GREEN,systemData.name,RTT_CTRL_BG_BLACK,	systemData.calibrationValueNTC);
			strcpy(deviceName,systemData.name);
			updateAdvertisingData();
		}
		else{
			flashRedLED();
			nrf_delay_ms(10);
			NRF_GPIO->OUTCLR = (1 << POWER_DOWN_PERIPHERAL);
			NRF_GPIO->OUTCLR = (1 << POWER_DOWN_BOARD);
			NRF_GPIO->OUTCLR = (1 << nrfSwitch);	
		}	
	//tsr Dbg8("pStorage load, data |%s|0x%X|0x%X|0x%X|0x%X| calib=%d, digitalPotStart=%d, restingForceNTC=%d",systemData.name,systemData.deadBeef1,systemData.deadBeef2,systemData.deadBeef3,systemData.deadBeef4,systemData.calibrationValueNTC,systemData.startingDigitalPotValue, systemData.restingForceNTC);	
}	

void initialiseFlashForFirstTime(void)
{
//	uint32_t retval;

	Dbg("Initialising flash for first time");	
	nrf_delay_ms(100);
	
	retval=pstorage_clear(&handle,64); 
	Dbg1("Cleared, retval %d",retval);	
	
	systemData.deadBeef1 = 0xDE;
	systemData.deadBeef2 = 0xAD;
	systemData.deadBeef3 = 0xBE;
	systemData.deadBeef4 = 0xEF;
	systemData.calibrationValueNTC=77;
	systemData.startingDigitalPotValue=128;
	systemData.calibrationTemperature=24;
	systemData.zeroOffsetTemperature=24;
	//systemData.restingForceNTC=1023-MCP4561_DESIRED_VALUE;
	systemData.restingForceNTC = 200;
	//systemData.restingForce22 = MCP4561_DESIRED_VALUE;
	char t[20];
	sprintf(t,"%0.6X", NRF_FICR->DEVICEADDR[0]);
	strcpy(systemData.name,t);
	systemData.powerModifier = 100;

	nrf_delay_ms(1000);
	saveSystemData();
	nrf_delay_ms(1000);

}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
//    fstorage_sys_event_handler(sys_evt);
//    fs_sys_event_handler(sys_evt);
}


static void cb_handler(pstorage_handle_t  * handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
  
  switch(op_code)
        {
           case PSTORAGE_CLEAR_OP_CODE:
               if (result == NRF_SUCCESS) {
//                  Dbg("pStorage - Clear operation successful in Callback");
	}
               else {
//                  Dbg("pStorage - Clear operation failed in Callback");
	}
               break;
 
     
          case PSTORAGE_LOAD_OP_CODE:
               if (result == NRF_SUCCESS) {
//                  Dbg("pStorage - Load operation successful in Callback");                
	}
               else {
//                  Dbg("pStorage - Load operation failed in Callback");
	}
               break;
     
          case PSTORAGE_STORE_OP_CODE:            
               if (result == NRF_SUCCESS) {
//                  Dbg("pStorage - Store operation successful in Callback");
	}
               else {
//                  Dbg("pStorage - Store operation failed in Callback");
	}               
               break;
        }
 
}

void pStorageInit()
{
	
//	uint32_t retval;
	
	softdevice_sys_evt_handler_set(sys_evt_dispatch);	
	
	Dbg("pStorage init");
	retval=pstorage_init(); 
	Dbg1("Initialised, retval was %d",retval);	
	
	param.block_size  = 64; 
	param.block_count = 1; 
	param.cb = cb_handler;
//	Dbg("pStorage register");
	retval = pstorage_register(&param, &handle); //register our pstorage and store store address in handle
	Dbg1("Regd, retval was %d",retval);	
	
}	

// ================================================================================
// ================================================================================
// ================================================================================

void saadc_callback2(nrf_drv_saadc_evt_t const * p_event)
{

}



void saadc_init(void)
{
 //   Dbg("Saadc init");
	ret_code_t err_code;
    nrf_saadc_channel_config_t channel_analog1_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(nrf_drv_saadc_gpio_to_ain(ANALOG1));
    nrf_saadc_channel_config_t channel_analog2_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(nrf_drv_saadc_gpio_to_ain(ANALOG2));
/*
	nrf_drv_saadc_config_t saadc_config;
	nrf_saadc_channel_config_t channel_config;

	//Configure SAADC
	saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
	saadc_config.oversample = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;       
	saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;    
	
    //err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback2);
*/
	err_code = nrf_drv_saadc_init(NULL, saadc_callback2);
    APP_ERROR_CHECK(err_code);

//	channel_analog1_config.gain=NRF_SAADC_GAIN1_2;
	channel_analog1_config.acq_time=NRF_SAADC_ACQTIME_10US;
  err_code = nrf_drv_saadc_channel_init(0, &channel_analog1_config);
  APP_ERROR_CHECK(err_code);
  
  channel_analog2_config.gain=NRF_SAADC_GAIN1_4;
	channel_analog2_config.acq_time=NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(1, &channel_analog2_config);
  APP_ERROR_CHECK(err_code);

}


int a=0;
// ================================================================================
// ================================================================================
// ================================================================================
// --------------------------
// filter variables
#define NZEROS 3
#define NPOLES 3
#define GAIN   2.105031112e+02
volatile static double xv[NZEROS+1], yv[NPOLES+1];
volatile uint8_t firstTimeReadingADXL = 1;
// --------------------------
//volatile static double xv[NZEROS+1], yv[NPOLES+1];

int16_t readADXL345()
{

    uint8_t rxData[20];
    int16_t x, y;
//    int16_t x, y, z;
    uint8_t addr = ADXL345_REG_DATAX0;
//    ret_code_t retCode;
//    int e;
//    uint8_t txData1[2] = {0x31, 11};
    //    Dbg1("In readADXL %d",a++);
    // Put the ADXL375 in Measurement Mode by writing 0x08 to the POWER_CTL Register
//!!    nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);
//    uint8_t txData[2] = {ADXL345_REG_POWER_CTL, 0x08};
    //    Dbg1("In readADXL %d",a++);
    // Put the ADXL375 in Measurement Mode by writing 0x08 to the POWER_CTL Register
//!!    nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData, sizeof(txData), false);
    //    int min=50000,max=0;
		
		if (firstTimeReadingADXL==1) {
				firstTimeReadingADXL=0;
				nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, &addr, 1, true);
				}
		
    nrf_drv_twi_rx(&m_twi, ADXL345_ADDRESS, rxData, 4);
				
    nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, &addr, 1, true);
//   if (retCode != NRF_SUCCESS) {
       //Dbg1("ADXL345 I2C Error, code = %d",retCode);
//			}
    //    nrf_delay_ms(10); 
//    retCode = nrf_drv_twi_rx(&m_twi, ADXL345_ADDRESS, rxData, 6);
    // ------------------------------------------------------------------------------------
    // --- Process angle to get new stroke
    // --- 6th order butterworth filter from here https://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
    // --- Lowpass, order=3, shoulder=2.5Hz, Freq=50Hz
    // ------------------------------------------------------------------------------------
    x = (rxData[1]<<8)+rxData[0];
    //    if (x<min) min=x;
    //    if (x>max) max=x;
    y = (rxData[3]<<8)+rxData[2];
//    z = (rxData[5]<<8)+rxData[4];
    angleX=x + 256;
    angleY=y + 256;
//    angleZ=z + 256;
 
//		angleZ= 1.0f * angleX * 360.0f/510.0f;
			rawAngle0360 = angleX;
//		Dbg1("Z%d", angleZ);
    calculatedAngle =(int)(atan2(-x,-y)*57.2957795+180);
//    Dbg4("Angles are %0.3d,%0.3d,%0.3d, %d",angleX,angleY,angleZ,(int)calculatedAngle);
    //    Dbg1("Leaving readADXL %d",e);
    int rawAngle; 
//    int rawAngle=abs(e-180); 
//    rawAngle=rawAngle; 
    rawAngle=angleX;  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; 
		xv[3] = rawAngle / GAIN;
		yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; 		
		yv[3] =   (xv[0] + xv[3]) + 3 * (xv[1] + xv[2])
								 + (  0.4683121112 * yv[0]) + ( -1.7564013818 * yv[1])
								 + (  2.2500850817 * yv[2]);
		
		double d;
		d=yv[3] * 360.0/510.0;

//		int angle;
//		angle=d;
		
///    angle    = rawAngle; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    return ((int)(d));  // this gives a saw tooth wave between 0 and 180 rather than a triangle wave of 0 .. 360
    //    return e;
}


// ================================================================================
// ================================================================================
// ================================================================================

const nrf_drv_twi_config_t twi_MCP4561_config = {
	 .scl                = 11,
	 .sda                = 14,
	 .frequency          = NRF_TWI_FREQ_400K,
	 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	 .clear_bus_init     = false
};

void twi_init (void)
{
//    ret_code_t err_code;

//	Dbg("TWI ok");
  nrf_drv_twi_init(&m_twi, &twi_MCP4561_config, NULL, NULL);	
  nrf_drv_twi_enable(&m_twi);
	Dbg("ok twi_init");
	nrf_delay_ms(10); 
}
// ================================================================================
// ================================================================================
// ================================================================================

void ADXL345_Init(){

	#define THRESH_TAP_ADDRESS  0x1D                                                 //Tap threshold register
	#define DUR_ADDRESS 				0x21                                                 //Tap duration register
	#define LATENT_ADDRESS 			0x22                                                 //Tap Latency register
	#define WINDOW_ADDRESS 			0x23                                                 //Tap window register
	#define TAP_AXES_ADDRESS 		0x2A                                                 //Axis control for single tap/double tap register
	#define INT_MAP_ADDRESS 		0x2F                                                 //Interrupt mapping control register
	#define INT_ENABLE_ADDRESS 	0x2E                                                 //Interrupt enable control register
	#define DATA_FORMAT_ADDRESS 0x31                                                 //Data format control register
	#define INT_SOURCE					0x30

	
	#define DUR 				0x0B                                                         //Tap duration value
	#define LATENT 			0xF0                                                         //Tap Latency value
	#define WINDOW 			0xFF                                                         //Tap window value
	#define THRESH_TAP 	0x40                                                         //Tap threshold value
	#define TAP_AXES 		0x07                                                     		 //Axis control for single tap/double tap value
	#define BW_RATE 		0x00                                                         //Data rate and power mode control value
	#define POWER_CTL 	0x09                                                      	 //Power-saving features control value
	#define INT_ENABLE 	0x40                                                     		 //Interrupt enable control value
	#define INT_MAP 		0x60                                                         //Interrupt mapping control value
	#define DATA_FORMAT 0x0A                                                    		 //Data format control value
//	#define FIFO_CTL 0                                                             //FIFO control value
  uint8_t rxData[5];	
	uint8_t txData1[2];
	uint8_t addr = INT_SOURCE;
//	uint8_t rxData[10];

	
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, &addr, 1, false);
	nrf_drv_twi_rx(&m_twi, ADXL345_ADDRESS, rxData, 1);
	
	txData1[0] =  THRESH_TAP_ADDRESS;
	txData1[1] =  THRESH_TAP;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);
	
	txData1[0] = 	DUR_ADDRESS;
	txData1[1] =  DUR;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);	

	txData1[0] = 	LATENT_ADDRESS;
	txData1[1] =  LATENT;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);	

	txData1[0] = 	WINDOW_ADDRESS;
	txData1[1] =  WINDOW;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);	
	
	txData1[0] = 	TAP_AXES_ADDRESS;
	txData1[1] =  TAP_AXES;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);
	
	txData1[0] = 	INT_MAP_ADDRESS;
	txData1[1] =  INT_MAP;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);

	txData1[0] = 	DATA_FORMAT_ADDRESS;
	txData1[1] =  DATA_FORMAT;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);

	txData1[0] = 	INT_ENABLE_ADDRESS;
	txData1[1] =  INT_ENABLE;
	nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);
}

// ================================================================================
// ================================================================================
// ================================================================================
void clearADXL345Int(){
//		uint8_t rxData[5];
//		uint8_t addr = INT_SOURCE;
		
//		nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, &addr, 1, false);
//		nrf_drv_twi_rx(&m_twi, ADXL345_ADDRESS, rxData, 1);
}
// ================================================================================
// ================================================================================
// ================================================================================
double readTemp(){
	
	#define	TMP102_ADDRESS	0x48           //Address of Temp prob
	#define	startByte				0x00           //Register address of temperature readings
	
	uint8_t addr = startByte;
	uint8_t rxData[2];
	int16_t digitalTemp;

	
	nrf_drv_twi_tx(&m_twi, TMP102_ADDRESS, &addr, 1, true);
	nrf_drv_twi_rx(&m_twi, TMP102_ADDRESS, rxData, 2);

	if(rxData[1]&0x01)	// 13 bit mode
  {
	// Combine bytes to create a signed int
    digitalTemp = ((rxData[0]) << 5) | (rxData[1] >> 3);// Temperature data can be + or -, if it should be negative,
	  // convert 13 bit to 16 bit and use the 2s compliment.
		if(digitalTemp > 0xFFF)
		{
			digitalTemp |= 0xE000;
			}
  }
  else	// 12 bit mode
  {
	// Combine bytes to create a signed int 
    digitalTemp = ((rxData[0]) << 4) | (rxData[1] >> 4);       // Temperature data can be + or -, if it should be negative,
	// convert 12 bit to 16 bit and use the 2s compliment.
		if(digitalTemp > 0x7FF)
		{
      digitalTemp |= 0xF000;
			}
  }
		
	digitalTemp *= 0.0625;
//	Dbg1("T%d", digitalTemp);
	return digitalTemp;
}

// ================================================================================
// ===============================Change ANT+ ID===================================
// ================================================================================

#define ANT_PLUS_NETWORK_KEY    {0xB9,0xA5,0x21,0xFB,0xBD,0x72,0xC3,0x45}            /**< The ANT+ network key. */
static uint8_t m_ant_plus_network_key[] = ANT_PLUS_NETWORK_KEY;

uint32_t changeAntID(int id){

//int device_number = id;
uint32_t err_code;
	
		err_code = sd_ant_channel_close(BPWR_CHANNEL_NUMBER);																								//close ANT+ channel
		VERIFY_SUCCESS(err_code);
		
		err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_ant_plus_network_key);							//set ANT+ address using ant+ keys
		VERIFY_SUCCESS(err_code);
		nrf_delay_ms(400);

		err_code = sd_ant_channel_id_set(BPWR_CHANNEL_NUMBER, id, BPWR_DEVICE_TYPE,5);											//set new ANT+ ID
		VERIFY_SUCCESS(err_code);
		
		err_code = sd_ant_channel_radio_freq_set(BPWR_CHANNEL_NUMBER, BPWR_ANTPLUS_RF_FREQ);								// assign ANT+ frequencey
		VERIFY_SUCCESS(err_code);

		err_code = sd_ant_channel_period_set(BPWR_CHANNEL_NUMBER, BPWR_MSG_PERIOD);                         //set period to 8182
		VERIFY_SUCCESS(err_code);
	
		err_code = sd_ant_channel_open(BPWR_CHANNEL_NUMBER);																								//open ANT+ connection again
		VERIFY_SUCCESS(err_code);
		
		return err_code;

}



// ==============================================================================================================
// ==============================================================================================================
// ==============================================================================================================

int main(void)
{

   uint32_t err_code;

	nrf_gpio_cfg_output(POWER_DOWN_BOARD);
	NRF_GPIO->OUTSET = (1 << POWER_DOWN_BOARD);
	
	sd_power_gpregret_clr(0,0);
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE); // NEW
	
	uint32_t reset_reason = NRF_POWER->RESETREAS;
	uint32_t reset_gret = NRF_POWER->GPREGRET;

	nrf_gpio_cfg_output(oneShotDone);
	NRF_GPIO->OUTSET = (1 << oneShotDone);
	
	nrf_gpio_cfg_output(nrfSwitch);
	NRF_GPIO->OUTSET = (1 << nrfSwitch);
	
	NRF_POWER->RESETREAS = 0xffffffff;
//	NRF_POWER->GPREGRET = 0xffffffff;
	
//	Dbg2("Reset cause =|0x%X,0x%X|",reset_reason,reset_gret );	
	sd_power_reset_reason_clr(0xffffffff);
	
	if (reset_reason == 2) {
		//Dbg("R1");
		nrf_delay_ms(10);
		reset();
		}

  Dbg2_AlwaysOn("%sVersion 20.4 Beta, (c) Avio Sports 2021.%s",RTT_CTRL_BG_BLUE,RTT_CTRL_BG_BLACK);
	nrf_delay_ms(10);
  flashRedLED();
	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);	
	//app_timer_start(0, 1, NULL);
	startupFlashLEDs();
	
  err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

	nrf_delay_ms(10);
  timers_init();
	nrf_delay_ms(10);
  ble_ant_stack_init();
	nrf_delay_ms(10);
  gap_params_init();
	nrf_delay_ms(10);
  advertising_init();
	nrf_delay_ms(10);
  services_init();
	nrf_delay_ms(10);
  conn_params_init();
	nrf_delay_ms(10);
	profile_setup();
	nrf_delay_ms(10);
	simulator_setup();
	nrf_delay_ms(10);
	rtcInit();
	nrf_delay_ms(10);
  ant_and_adv_start();
	nrf_delay_ms(10);
	init_watchdog();
	nrf_delay_ms(10);
	saadc_init();
 	nrf_delay_ms(15);
	twi_init();
	nrf_delay_ms(15);
	pStorageInit();	
//	nrf_delay_ms(10);
//	ADXL345_Init();
	nrf_delay_ms(10);
	
	loadSystemData();
	nrf_delay_ms(20);
	int antID = systemData.newANTNumber;
	nrf_delay_ms(20);
//initialiseFlashForFirstTime();

	if ((systemData.deadBeef1 != 0xDE) || (systemData.deadBeef2 != 0xAD) || (systemData.deadBeef3 != 0xBE) || (systemData.deadBeef4 != 0xEF)) {
		//Dbg("Init flash");
		initialiseFlashForFirstTime();
		systemData.deadBeef1 = 0xDE;
		systemData.deadBeef2 = 0xAD;
		systemData.deadBeef3 = 0xBE;
		systemData.deadBeef4 = 0xEF;
		systemData.calibrationValueNTC = DEFAULT_10KG_CALIBRATION_ADC_VALUE;
		systemData.startingDigitalPotValue=128;
		systemData.calibrationTemperature=24;
		systemData.zeroOffsetTemperature=24;
		systemData.restingForceNTC=200;
		systemData.newANTNumber = ASSIGN_ANT_ID;																	//It will assign ANT ID when programed for the very first time
		//systemData.restingForce22=MCP4561_DESIRED_VALUE;
		strcpy(deviceName,DEVICE_NAME); 
		systemData.powerModifier = 100;
		
		saveSystemData();
		nrf_delay_ms(3000);
		loadSystemData();
		
		if ((systemData.deadBeef1 != 0xDE) || (systemData.deadBeef2 != 0xAD) || (systemData.deadBeef3 != 0xBE) || (systemData.deadBeef4 != 0xEF)) {
			Dbg2("%sERR%s flash not init",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK);
			}	
		}

	nrf_delay_ms(10);
	changeAntID(antID);
		
	Dbg("Flash OK");
		
	if (systemData.newANTNumber == 9090) {
			Dbg2_AlwaysOn("%s9090 detected, turning off auto power down and reporting battery to ANT%s",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK);
			reportBatteryNotPower=1;
			doNotGoToSleep=1;
			doNotTurnOff=1;
			}
	
	if (systemData.newANTNumber == 9191) {
			Dbg2_AlwaysOn("%s9191 detected, turning off auto power down and reporting battery to ANT%s",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK);
			reportBatteryNotPower=1;
			doNotGoToSleep=1;
			doNotTurnOff=1;
			}
	
	if (systemData.newANTNumber == 9292) {
			Dbg2_AlwaysOn("%s9292 detected, turning off auto power down%s",RTT_CTRL_BG_RED,RTT_CTRL_BG_BLACK);
			doNotGoToSleep=1;
			doNotTurnOff=1;
			}
	
	m_ant_bpwr_simulator._cb.power_sensorsim_state.current_val = 0;	
	m_ant_bpwr_simulator.p_profile->page_17.accumulated_torque=0;	

	setDigitalPotentiometerResistance(systemData.startingDigitalPotValue);

	nrf_delay_ms(20);
	int c = 0;
  temperature = readTemp();
	
	greenLEDOff();
	enablePowerMeasurement=1;
  advertising_start();

	NRF_TIMER1->BITMODE = 03ul << 0ul;
	NRF_TIMER1->MODE = 0ul << 0ul;
	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->CC[0] = 1000;
	NRF_TIMER1->CC[1] = 100;
	NRF_TIMER1->PRESCALER = 9;
	NRF_TIMER1->INTENSET = 1ul << 16;
	NVIC_EnableIRQ(TIMER1_IRQn); 

	NRF_TIMER1->TASKS_START =1;
	NRF_TIMER1->TASKS_CAPTURE[1] =1;
	NVIC_SetPriority(TIMER1_IRQn, 3);
	NRF_TIMER1->TASKS_START =1;
	
//    uint8_t rxData[20];
//    int16_t x, y;
//    int16_t x, y, z;
//    uint8_t addr = ADXL345_REG_DATAX0;
//    ret_code_t retCode;
//    int e;
    uint8_t txData1[2] = {0x31, 11};
    //    Dbg1("In readADXL %d",a++);
    // Put the ADXL375 in Measurement Mode by writing 0x08 to the POWER_CTL Register
    nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData1, sizeof(txData1), false);
    uint8_t txData[2] = {ADXL345_REG_POWER_CTL, 0x08};
    //    Dbg1("In readADXL %d",a++);
    // Put the ADXL375 in Measurement Mode by writing 0x08 to the POWER_CTL Register
    nrf_drv_twi_tx(&m_twi, ADXL345_ADDRESS, txData, sizeof(txData), false);
    //    int min=50000,max=0;
	
	sd_ble_gap_tx_power_set(-4);
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE); // NEW

	if ((systemData.restingForceNTC < 100) || (systemData.restingForceNTC > 300)) systemData.restingForceNTC = 200;
	if ((systemData.calibrationValueNTC < 20) || (systemData.calibrationValueNTC > 120)) systemData.calibrationValueNTC = 78;

  while(1) {
			if (SEGGER_RTT_HasKey() != 0 ) {
					char dbgInput[200];
					SEGGER_RTT_Read(0,dbgInput,sizeof(dbgInput));
					Dbg1_AlwaysOn("Got input ******************************|%s|",dbgInput);

					if (dbgInput[0]=='d') {
						Dbg("Turning on debug");
						debugOutputEnabled=1;
						redLEDOn();
						}
					if (dbgInput[0]=='z') {
						Dbg("Zero offset");
						zeroOffsetSameCounter=0;
						triggerCalibrateOnPowerUp=1;
						redLEDOn();
						}
					}
		
		if (bufferIsFree==0) {
			if (bufferIsFreeTimeout++ > 200) {
				bufferIsFreeTimeout=0;
				bufferIsFree=1;	
				}
		}
	
		if (strlen(nameToUpdate) >0) {
			Dbg1("chg name |%s|",nameToUpdate);
			strcpy(systemData.name,nameToUpdate);
			strcpy(deviceName,nameToUpdate);
			saveSystemData();	
			Dbg1("name chgd |%s|",deviceName);
			flashRedLED();
			strcpy(nameToUpdate,"");
			}
		
		if (triggerSave==1) {
			saveSystemData();
			triggerSave=0;
			}
//		if ((triggerCalibrateOnPowerUp == 1) && (tapOn == 0)) {
		if (triggerCalibrateOnPowerUp == 1) {
			calibrationInTheLoop();	  
			if(triggerCalibrateOnPowerUp == 2 ){  //after calibration save data
				if ((systemData.restingForceNTC < 100) || (systemData.restingForceNTC > 300)) systemData.restingForceNTC = 200;
				if ((systemData.calibrationValueNTC < 20) || (systemData.calibrationValueNTC > 120)) systemData.calibrationValueNTC = 78;
				systemData.zeroOffsetTemperature = temperature;
				//triggerSave=1;
				saveSystemData();
				triggerCalibrateOnPowerUp = 0;
			}
		}
		
		if (c++ % 100==0) {
			char sConn[20];
			if (connected==1) 
				strcpy(sConn,"Conn");
			else
				strcpy(sConn,"");
			clearADXL345Int();
			NRF_TIMER1->TASKS_CAPTURE[0] =1;
			#if 0
			if (debugOutputEnabled==1) {
						sprintf(s,"Ang=%0.3d,AngZ=%0.3d,Force=%0.3dN,En=%0.3d,Raw=%0.4d,Strokes=%0.3d,Power=%0.3d,Rest=%0.3d,Cal10Kg=%0.3d,temp=%d,Ctemp=%d%%,batt=%d,count=%u,millis()=%lu,sleep=%u,%s", 
								angle, 
								(int)rawAngle0360, 
								(int)(round(force)), 
								(int)(round(energy)), 
								lastADCReadingNTC, 
								strokes, 
								power, 
								systemData.restingForceNTC,
								systemData.calibrationValueNTC,
								temperature,
								(int)(temperatureCompensation*100.0), 
								batteryADC,
								//app_timer_cnt_get(),
								NRF_TIMER1->CC[0],
								millis()/1000,
								doNotTurnOff,
								sConn); 
						SEGGER_RTT_WriteString(0, s); 
						SEGGER_RTT_WriteString(0, "\r\n");
				}
				#endif
			}
		power_manage();
		}
}

