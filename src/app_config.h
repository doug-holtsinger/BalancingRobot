
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
// Direction pin
#define MOTOR_DRIVER_BPHASE 24
// PWM pin
#define MOTOR_DRIVER_BENBL 22 
#define NRF_PWM_APHASE NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_APHASE)
#define NRF_PWM_AENBL NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_AENBL)
#define NRF_PWM_BPHASE NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_BPHASE)
#define NRF_PWM_BENBL NRF_GPIO_PIN_MAP(0,MOTOR_DRIVER_BENBL)
// QDEC pins
#define NRF_QDEC_C1 NRF_GPIO_PIN_MAP(1, 15)
#define NRF_QDEC_C2 NRF_GPIO_PIN_MAP(0, 2)
#define NRF_QDEC_C3 NRF_GPIO_PIN_MAP(1, 10)
#define NRF_QDEC_C4 NRF_GPIO_PIN_MAP(1, 13)

// Optional LED indicators for BLE states
#define BSP_LED_INDICATE_CONNECTED             BSP_BOARD_LED_2
#define BSP_LED_INDICATE_BONDING               BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_DIRECTED  BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_SLOW      BSP_BOARD_LED_2
#define BSP_LED_INDICATE_ADVERTISING_WHITELIST BSP_BOARD_LED_2
#define BSP_LED_INDICATE_INDICATE_ADVERTISING  BSP_BOARD_LED_2

// Configuration for BalancingRobot
#define BALANCING_ROBOT_CONFIG

#define MOTOR_PID_RECORD_KEY 0x7011
#define SPEED_PID_RECORD_KEY 0x7012

#define MOTOR_PID_NUM 0
#define SPEED_PID_NUM 1

// IMU override to provide better sensitivity to changes in the acceleration
// vector at the expense of stability
#define NOISE_THRESHOLD_MULTIPLIER_ACCELEROMETER 0.0

