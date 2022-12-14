
// Define if BLE console is wanted
#ifdef BOARD_PCA10059
#define BLE_CONSOLE_AVAILABLE 
#endif

// Define if serial port is wanted on nRF52 board
// #define SERIAL_CONSOLE_AVAILABLE

// Define if BLE DIS is wanted
#define DEVICE_INFORMATION_SERVICE_AVAILABLE

// PWM port pin connections on nRF52 board
// Direction pin
#define MOTOR_DRIVER_APHASE 20
// PWM pin
#define MOTOR_DRIVER_AENBL 17
#define NRF_PWM_APHASE NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_APHASE)
#define NRF_PWM_AENBL NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_AENBL)

// Optional LED indicators for BLE states
#define BSP_LED_INDICATE_CONNECTED             BSP_BOARD_LED_2
#define BSP_LED_INDICATE_BONDING               BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_DIRECTED  BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_SLOW      BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_WHITELIST BSP_BOARD_LED_2
#define BSP_LED_INDICATE_INDICATE_ADVERTISING  BSP_BOARD_LED_2

