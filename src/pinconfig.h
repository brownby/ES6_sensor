#ifndef pinconfig_h
#define pinconfig_h

    #define FRIMWARE_VERSION 3.4
    #define HARDWARE_VERSION 1
    
    #define FIRMWARE_URL    "http://s3.amazonaws.com/firmware.breatheioservers.com/firmware.json"
    //#define ERROR_LOG_URL   "http://api.breatheio.com/public/device/push/log"

    //FAN 
        //PINS 
        //#define PIN_FAN_CONTROL         14
        //#define PIN_FAN_SENSE           15

        //CONFIG
        #define FAN_FREQ                4900
        #define FAN_CHANNEL             0
        #define FAN_RESOLUTION          8

    //LCD - PINS
        #define TFT_RST                 4                                            
        #define TFT_DC                  5
        #define TFT_CS                  19  // First it was 16
        #define TFT_LED                 21

        //BRIGHTNESS
        #define LCD_FREQ                12000
        #define LCD_CHANNEL             4
        #define LCD_RESOLUTION          8

    // FanBoost
    #define FanBoostThreshold           30
    #define FanBoostSpeed               140
    #define FanMinimumSpeed             20

    //SENSORS

        //SENSORS
        //#define MQ_PIN                  32 // BreatheIO V6 - 35 PIN
        //#define DHT_PIN                 13

        //Configuration
        #define RatioMQ135CleanAir      3.6    

        #define PM_SET_PIN              22 
        //#define PM_RESET_PIN            12
        #define PM_SERIAL               Serial2
#endif