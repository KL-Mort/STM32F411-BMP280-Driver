#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_I2C_ADDR_VCC 	(0x77 <<1) /*Адреса I2C МОД якщо на живлення*/
#define BMP280_I2C_ADDR_GND 	(0x76 <<1)	/*Адреса I2C МОД якщо на землю*/
#define BMP280_REG_CALIB_00     0x88  /* Початок калібрувальних даних  */
#define BMP280_REG_CHIP_ID      0xD0  /* Регістр ідентифікатора чіпа */
#define BMP280_REG_RESET        0xE0  /* Регістр програмного перезавантаження */
#define BMP280_REG_STATUS       0xF3  /* Регістр статусу */
#define BMP280_REG_CTRL_MEAS    0xF4  /* Регістр налаштування вимірювання (режим, оверсемплінг) */
#define BMP280_REG_CONFIG       0xF5  /* Регістр конфігурації (IIR-фільтр, час standby) */
#define BMP280_REG_PRESS_MSB    0xF7  /* Початок блоку сирих даних вимірювання (6 байтів) */

#define BMP280_CHIP_ID_VAL      0x58  /* ID чіпа  */
#define BMP280_RESET_VALUE      0xB6  /* soft reset */

/* --- Маски бітів регістру статусу (0xF3) --- */
#define BMP280_STATUS_MEASURING (1 << 3) /* 1 - триває перетворення тиску/температури */
#define BMP280_STATUS_IM_UPDATE (1 << 0) /* 1 - триває копіювання даних з NVM у регістри */

/* --- Режими живлення/роботи (Power Modes) --- */
typedef enum
{
    BMP280_MODE_SLEEP  = 0x00, /* Режим сну (мінімальне споживання) */
    BMP280_MODE_FORCE = 0x01, /* Разове вимірювання за запитом і повернення в сон */
    BMP280_MODE_NORMAL = 0x03  /* Циклічне автоматичне оновлення даних */
} BMP280_Mode;

/* --- Коефіцієнти оверсемплінгу (Oversampling) --- */
typedef enum
{
    BMP280_OSRS_SKIPPED = 0x00, /* Вимірювання вимкнено */
    BMP280_OSRS_1X      = 0x01, /* 1 вибірка (16 біт) */
    BMP280_OSRS_2X      = 0x02, /* 2 вибірки (17 біт) */
    BMP280_OSRS_4X      = 0x03, /* 4 вибірки (18 біт) */
    BMP280_OSRS_8X      = 0x04, /* 8 вибірок (19 біт) */
    BMP280_OSRS_16X     = 0x05  /* 16 вибірок (20 біт) */
} BMP280_Oversampling;

/* --- Коефіцієнт IIR-фільтрації (подавлення коливань/шуму тиску) --- */
typedef enum
{
    BMP280_FILTER_OFF = 0x00,
    BMP280_FILTER_2   = 0x01,
    BMP280_FILTER_4   = 0x02,
    BMP280_FILTER_8   = 0x03,
    BMP280_FILTER_16  = 0x04
} BMP280_Filter;

/* --- Час очікування між вимірюваннями в Normal Mode (Standby Time) --- */
typedef enum
{
    BMP280_STANDBY_0_5_MS  = 0x00,
    BMP280_STANDBY_62_5_MS = 0x01,
    BMP280_STANDBY_125_MS  = 0x02,
    BMP280_STANDBY_250_MS  = 0x03,
    BMP280_STANDBY_500_MS  = 0x04,
    BMP280_STANDBY_1000_MS = 0x05,
    BMP280_STANDBY_2000_MS = 0x06,
    BMP280_STANDBY_4000_MS = 0x07
} BMP280_Standby;

/* --- Структура параметрів вимірювання --- */
typedef struct
{
    BMP280_Mode mode;
    BMP280_Oversampling osrs_t; /* Оверсемплінг температури */
    BMP280_Oversampling osrs_p; /* Оверсемплінг тиску */
    BMP280_Filter filter;       /* Налаштування IIR-фільтра */
    BMP280_Standby standby;     /* Пауза між циклами */
} BMP280_Config;
typedef enum {
	BMP280_OK 			=0,
	BMP280_ERR	 		=-1,
	BMP280_ERR_ID 		=-2,
	BMP280_ERR_ARGS		=-3

}BMP280_Status;


/* --- Вказівники на функції шини введення/виведення --- */
typedef int32_t (*BMP280_ReadFn)(void *context, uint8_t reg, uint8_t *data, uint16_t len);
typedef int32_t (*BMP280_WriteFn)(void *context, uint8_t reg, const uint8_t *data, uint16_t len);

typedef struct
{
    void *context;
    BMP280_ReadFn read;
    BMP280_WriteFn write;
} BMP280_Bus;

/* --- Основна структура дескриптора пристрою BMP280 --- */
typedef struct
{
    BMP280_Bus bus;

    // Калібрувальні коефіцієнти NVM (згідно з розділом 3.11.2 даташита)
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    int32_t  t_fine;
} BMP280;

/* --- Прототипи функцій драйвера --- */

int32_t BMP280_Init(BMP280 *dev);
int32_t BMP280_ReadID(BMP280 *dev, uint8_t *chip_id);
int32_t BMP280_GetStatus(BMP280 *dev, bool *is_measuring, bool *is_updating);
int32_t BMP280_SetConfig(BMP280 *dev, const BMP280_Config *config);
int32_t BMP280_SetMode(BMP280 *dev, BMP280_Mode mode);
int32_t BMP280_ReadData(BMP280 *dev, float *temperature, float *pressure);
int32_t BMP280_ReadDataForce(BMP280 *dev, float *tempurature, float *pressure);

#ifdef __cplusplus
}
#endif

#endif /* INC_BMP280_H_ */
