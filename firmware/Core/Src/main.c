/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Block 4.5 — hardware bring-up self test
  *
  *  Target : NUCLEO-F446RE @ 84 MHz
  *  Tested : I2C1 (MPU-9250 + SSD1306), USART1 (RPLIDAR C1),
  *           UART4 (HC-04 BLE), ADC1 ch10/11/12 (three potentiometers)
  *
  *  Wiring
  *    I2C1  SCL = PB8 (D15)   SDA = PB9 (D14)   -> MPU + OLED in parallel
  *    USART1 TX = PA9 (D8)    RX  = PA10 (D2)   -> lidar TX/RX crossed, 460800
  *    UART4  TX = PA0 (A0)    RX  = PA1 (A1)    -> BLE RXD/TXD crossed, 921600
  *    ADC1  IN10 = PC0        IN11 = PC1        IN12 = PC2   -> OT1/OT2/OT3
  *    LED   PA5 ; SWO printf on PB3
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "oled_font.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commands.h"
#include "nav.h"
#include "imu_heading.h"

/* ------------------------------------------------------------------ handles */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_uart4_tx;

/* ------------------------------------------------------------------- macros */
#define LIDAR_HALF_SIZE  500u
#define LIDAR_BUF_SIZE   (2u * LIDAR_HALF_SIZE)
#define IMU_ADDR7        0x68u
#define IMU_ADDR         (IMU_ADDR7 << 1)
#define OLED_ADDR        (0x3C << 1)
#define OLED_COL_OFFSET  2u
#define TELEM_BAUD       921600u
#ifndef PART2_ENABLE_MOTORS
#define PART2_ENABLE_MOTORS 0
#endif
/* 0: keep the lidar scan, but do not steer around or cancel a GOTO for an obstacle. */
#ifndef PART2_ENABLE_AVOIDANCE
#define PART2_ENABLE_AVOIDANCE 0
#endif
#ifndef PART2_LIDAR_FRONT_Q6
#define PART2_LIDAR_FRONT_Q6 (-1)
#endif
#ifndef PART2_WHEEL_MM
#define PART2_WHEEL_MM 65.0f
#define PART2_TRACK_MM 150.0f
#define PART2_CPR_L 1320.0f
#define PART2_CPR_R 1320.0f
#define PART2_ENCODER_L 1
#define PART2_ENCODER_R (-1)
#define PART2_MOTOR_L (-1)
#define PART2_MOTOR_R 1
#define PART2_IMU_SIGN 1
#endif

/* 左右轮是镜像安装的：同一个“前进”指令，右轮要反极性才和左轮同向。
   这里把两处符号一次定死，上层代码永远只用“正=前进”的语义：
     ENC_R_DIR   右轮编码器计数方向（前转计数变负）
     MOTOR_R_DIR 右轮电机 PWM 极性（正指令要驱动 BIN2 而不是 BIN1）
   如果哪天把右轮电机两根线对调了，把 MOTOR_R_DIR 改回 +1。 */
#define ENC_R_DIR        PART2_ENCODER_R
#define MOTOR_R_DIR      PART2_MOTOR_R

/* 整机极性：这块驱动板上 AIN1 高电平实际是“后退”，所以正指令要先翻一次。
   如果哪天把两个电机的线都反接了，把这里改回 +1。 */
#define MOTOR_POL        PART2_MOTOR_L

/* --------------------------- 底盘几何（可校准：转一圈看 odom 是否等于周长） */
#define WHEEL_MM         PART2_WHEEL_MM
#define TRACK_MM         PART2_TRACK_MM
#define MM_PER_TICK_L    (3.14159265f * WHEEL_MM / PART2_CPR_L)
#define MM_PER_TICK_R    (3.14159265f * WHEEL_MM / PART2_CPR_R)

/* --------------------------- 速度环增益（内环 PI，10 ms 一拍，单位 mm/s） */
#define KP_V             1.6f      /* 比例：每 1 mm/s 误差给多少 PWM */
#define KI_V             4.0f      /* 积分：每秒累积系数 */
#define V_MAX_MMS        250.0f    /* 直行最高线速度（电位器 OT1 满量程时） */
#define V_MIN_MMS        40.0f     /* 直行最低线速度（OT1 拧到 0 时） */
#define W_MAX_DPS        120.0f    /* 转向最高角速度（OT2 满量程时） */
#define W_MIN_DPS        20.0f     /* 转向最低角速度（OT2 拧到 0 时） */
#define DRIVE_WHEEL_LIMIT_MPS 0.12f /* 在实车制停距离验证前限制轮速目标 */

/* ---------------------------------------------------------------- variables */
static uint8_t  lidar_buf[LIDAR_BUF_SIZE];
static volatile uint32_t lidar_cb    = 0;
static volatile uint32_t lidar_err   = 0;
static volatile uint32_t lidar_lost_blocks = 0;
static volatile uint32_t lidar_queue_drop = 0;
static volatile uint32_t lidar_drop  = 0;
static volatile uint32_t lidar_scan_attempts = 0;
static volatile uint32_t lidar_restarts = 0;
static volatile uint32_t lidar_last_uart_error = 0;
static volatile uint8_t lidar_receiving = 0;

static uint8_t  oled_ok  = 0;
static uint8_t  imu_ok   = 0;
static uint8_t  oled_fb[1024];
static uint8_t  oled_line[129];

static uint8_t  i2c_found[8];
static uint8_t  i2c_count = 0;

/* ---------------------------------------------------------------- prototypes */
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void uart4_init(void);
static void uart4_rx_start(void);

static void adc_init(void);
static uint16_t adc_read(uint8_t ch);

static void i2c_bus_recover(void);
static HAL_StatusTypeDef imu_read(uint8_t reg, uint8_t *buf, uint16_t n);
static bool imu_configure(void);

static void oled_cmd(uint8_t c);
static void oled_init(void);
static void oled_clear(void);
static void oled_flush(void);
static void oled_char(uint8_t cx, uint8_t line, char c);
static void oled_str(uint8_t cx, uint8_t line, const char *s);

/* --------------------------------------------------------- RTOS objects ---- */
osMessageQueueId_t Q_Lidar;      /* DMA 半缓冲描述符 */
osMessageQueueId_t Q_Cmd;        /* 蓝牙指令 (x,y)/STOP */
osMessageQueueId_t Q_Path;       /* 路径点 */
osSemaphoreId_t    Sem_IMU;      /* TIM2 10 ms -> IMU 任务 */
osMutexId_t        Mtx_I2C;      /* MPU + OLED 共用 I2C1 */

typedef struct { uint32_t sequence; uint32_t tick; uint8_t half; } LidarChunk_t;
typedef struct { Cmd_t command; NavPath path; bool valid; } PathMessage;
typedef struct { uint32_t count, last, period, max; } TaskTiming;
enum { TIME_IMU, TIME_LIDAR, TIME_RX, TIME_PLAN, TIME_POSE,
       TIME_TRACK, TIME_MOTOR, TIME_OLED, TIME_TX, TIMING_COUNT };
enum { NAV_IDLE, NAV_RUNNING, NAV_DONE, NAV_STOPPED, NAV_FAULT, NAV_IMU_WAIT, NAV_BENCH, NAV_LIDAR_STOP };
enum { FAULT_NONE, FAULT_PATH, FAULT_CONTROL, FAULT_TIMEOUT, FAULT_IMU,
       FAULT_STALL_LEFT, FAULT_STALL_RIGHT, FAULT_STALL_BOTH, FAULT_OBSTACLE, FAULT_LIDAR_STALE };
typedef struct {
    Cmd_t command;
    uint8_t state, index, count, fault;
    bool imu_ready;
    uint8_t motion;
    int16_t ticks_l, ticks_r, pwm_l, pwm_r;
    NavLidarHit obstacle_stop;
} NavReport;
typedef struct {
    uint32_t run;
    int16_t reference_l, reference_r, actual_l, actual_r;
    int16_t gyro_yaw10, fused_yaw10, bias_mdps;
    bool braking;
} ControlReport;
static TaskTiming timings[TIMING_COUNT];
static volatile uint32_t cmd_drop, path_drop, imu_sem_full, imu_errors, i2c_wait_max, rx_drop;
static volatile uint32_t imu_bus_recoveries;
static volatile uint32_t imu_sample_us;
static volatile float imu_yaw = NAV_INITIAL_YAW_RAD;
static volatile float imu_bias_dps;
static volatile bool imu_calibrated;
static volatile uint32_t newest_command;
static NavPose pose = { .yaw_rad = NAV_INITIAL_YAW_RAD };
static PathMessage published_path;
static NavReport nav_report;
static ControlReport control_report;
static NavLidarStatus lidar_status;

/* 里程计（mm / 度），Task_Ctrl 每 10 ms 更新 */
static float odom_x, odom_y, odom_th_deg;

/* UART4 收蓝牙指令 */
static uint8_t rx_ring[256];
static volatile uint16_t rx_head, rx_tail;
static volatile bool rx_discard;
enum { TX_CAPACITY = 768u };
static uint8_t tx_lines[TX_CAPACITY];
static uint16_t tx_size;
static uint8_t ble_dma_payload[TX_CAPACITY];
#if !PART2_ENABLE_MOTORS
static uint8_t usb_dma_payload[TX_CAPACITY];
#endif

static volatile int16_t  s_ax, s_ay, s_az, s_gz;
static volatile uint16_t s_mv1, s_mv2, s_mv3;
static volatile uint32_t s_imu_frames, s_scan_frames, s_lidar_chunks;
static volatile uint32_t s_map_pkts;      /* 已发出的点云包数 */

static osThreadId_t hTaskIMU, hTaskLidar, hTaskCtrl, hTaskUI;
static osThreadId_t hTaskRx, hTaskPlan, hTaskTx;

static void Task_IMU(void *arg);
static void Task_Lidar(void *arg);
static void Task_Ctrl(void *arg);
static void Task_UI(void *arg);
static void Task_Rx(void *arg);
static void Task_Plan(void *arg);
static void Task_Tx(void *arg);
static void tim2_init_10ms(void);
static void motor_init(void);
static void motor_set(int16_t l, int16_t r);
static void motor_brake(void);


/* ================================================================ SSD1306 ==*/
static void oled_cmd(uint8_t c)
{
    uint8_t b[2];
    b[0] = 0x00;          /* control byte: command stream */
    b[1] = c;
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, b, 2, 20);
}

static void oled_init(void)
{
    static const uint8_t seq[] = {
        0xAE,             /* display off            */
        0xD5, 0x80,       /* clock divide           */
        0xA8, 0x3F,       /* multiplex 1/64         */
        0xD3, 0x00,       /* display offset         */
        0x40,             /* start line 0           */
        0x8D, 0x14,       /* charge pump on         */
        0x20, 0x02,       /* addressing: page mode (SSD1306 + SH1106) */
        0xA1,             /* segment remap          */
        0xC8,             /* COM scan dec           */
        0xDA, 0x12,       /* COM pins               */
        0x81, 0xCF,       /* contrast               */
        0xD9, 0xF1,       /* pre-charge             */
        0xDB, 0x40,       /* VCOMH                  */
        0xA4,             /* display follows RAM    */
        0xA6,             /* normal (not inverted)  */
        0x2E,             /* no scroll              */
        0xAF              /* display on             */
    };
    for (uint32_t i = 0; i < sizeof(seq); i++)
        oled_cmd(seq[i]);
}

static void oled_clear(void)
{
    memset(oled_fb, 0, sizeof(oled_fb));
}

static void oled_flush(void)
{
    oled_line[0] = 0x40;              /* control byte: data stream */
    for (uint8_t page = 0; page < 8u; page++) {
        oled_cmd((uint8_t)(0xB0u | page));
        /* SH1106 显存 132 列、可视区从第 2 列开始；SSD1306 会整体右移 2 px。
           用偏移 2 可以让两种控制器都显示正常（这也是市面上 0.96" 模块的通用写法）。 */
        oled_cmd((uint8_t)(0x00u | (OLED_COL_OFFSET & 0x0Fu)));
        oled_cmd((uint8_t)(0x10u | ((OLED_COL_OFFSET >> 4) & 0x0Fu)));
        memcpy(&oled_line[1], &oled_fb[page * 128u], 128u);
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, oled_line, 129, 40);
    }
}

static void oled_char(uint8_t cx, uint8_t line, char c)
{
    if (cx > 20u || line > 7u) return;
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = font6x8[(uint8_t)c - 32u];
    uint8_t *d = &oled_fb[line * 128u + cx * 6u];
    for (uint8_t i = 0; i < 6u; i++) d[i] = g[i];
}

static void oled_str(uint8_t cx, uint8_t line, const char *s)
{
    while (*s && cx < 21u) oled_char(cx++, line, *s++);
}

/* ==================================================================== ADC ==*/
static void adc_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);

    ADC->CCR   = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;  /* PCLK2/4 = 21 MHz */
    ADC1->CR1  = 0;
    ADC1->CR2  = 0;
    /* 84 cycles for channels 10/11/12 */
    ADC1->SMPR1 = (4u << 0) | (4u << 3) | (4u << 6);
    ADC1->SQR1  = 0;                    /* 1 conversion in the sequence */
    ADC1->CR2  |= ADC_CR2_ADON;
    HAL_Delay(2);
}

static uint16_t adc_read(uint8_t ch)
{
    uint32_t guard = 0;

    ADC1->SQR3 = (uint32_t)ch;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0u) {
        if (++guard > 200000u) return 0xFFFFu;
    }
    return (uint16_t)(ADC1->DR & 0x0FFFu);
}

/* =================================================================== I2C ===*/
static void i2c_scan(void)
{
    i2c_count = 0;
    for (uint8_t a = 0x08u; a < 0x78u; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 3) == HAL_OK) {
            if (i2c_count < sizeof(i2c_found)) i2c_found[i2c_count++] = a;
        }
    }
}

/* 总线被从机拉住时的恢复：9 个 SCL 时钟 + STOP，然后重新初始化 I2C1。
   只读一个字节能成功、读一串就失败、失败后一直 BUSY，就是这种情形。 */
static void i2c_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};
    volatile uint32_t d;
    uint32_t i;

    HAL_I2C_DeInit(&hi2c1);

    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      /* SDA 拉高 */

    for (i = 0u; i < 9u; i++) {                              /* 9 个时钟 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (d = 0; d < 80u; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (d = 0; d < 80u; d++) { }
    }

    /* STOP：SCL 高时 SDA 由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    for (d = 0; d < 80u; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (d = 0; d < 80u; d++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    MX_I2C1_Init();
    HAL_Delay(1);
}

/* At 100 kHz a 14-byte burst takes under 2 ms under normal conditions. */
static HAL_StatusTypeDef imu_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) return HAL_BUSY;
    return HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 4u);
}

static bool imu_configure(void)
{
    uint8_t who, wake = 1u, axes = 0u, verify[5];
    /* 100 Hz, gyro DLPF, +/-250 dps and +/-2 g match the conversion factors. */
    uint8_t settings[5] = {9u, 3u, 0u, 0u, 3u};
    return imu_read(0x75u, &who, 1u) == HAL_OK &&
           (who == 0x70u || who == 0x71u || who == 0x73u) &&
           HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6Bu, I2C_MEMADD_SIZE_8BIT,
                             &wake, 1u, 5u) == HAL_OK &&
           HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x6Cu, I2C_MEMADD_SIZE_8BIT,
                             &axes, 1u, 5u) == HAL_OK &&
           HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x19u, I2C_MEMADD_SIZE_8BIT,
                             settings, sizeof settings, 5u) == HAL_OK &&
           HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x19u, I2C_MEMADD_SIZE_8BIT,
                            verify, sizeof verify, 5u) == HAL_OK &&
           memcmp(settings, verify, sizeof settings) == 0;
}

/* =============================================================== UART4 ==== */
static void uart4_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &g);

    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = TELEM_BAUD;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();

    hdma_uart4_tx.Instance                 = DMA1_Stream4;
    hdma_uart4_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_uart4_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_uart4_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(&huart4, hdmatx, hdma_uart4_tx);
}

#if !PART2_ENABLE_MOTORS
static void uart2_bench_init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_2 | GPIO_PIN_3, .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = GPIO_AF7_USART2
    };
    HAL_GPIO_Init(GPIOA, &pins);
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + 57600u) / 115200u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    DMA1_Stream6->CR = DMA_CHANNEL_4 | DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE;
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    HAL_NVIC_SetPriority(USART2_IRQn, 6u, 0u);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}
#endif

static void uart4_rx_start(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    SET_BIT(UART4->CR1, USART_CR1_RXNEIE);
}

static void receive_byte(uint8_t ch, bool failed)
{
    const uint16_t head = rx_head;
    const uint16_t next = (head + 1u) & 255u;
    if (failed || next == rx_tail) {
        ++rx_drop;
        rx_discard = true;
    } else {
        rx_ring[head] = ch;
        __DMB();
        rx_head = next;
    }
    if (failed || ch == '\n' || ch == '\r' || rx_discard) {
        (void)osThreadFlagsSet(hTaskRx, 1u);
    }
}

/* ================================================================= TIM2 ==== */
/* 100 Hz (10 ms)：PSC=83 -> 1 MHz，ARR=9999 -> 100 Hz。不依赖 HAL TIM 模块。 */
static void tim2_init_10ms(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 84u - 1u;
    TIM2->ARR = 10000u - 1u;
    TIM2->CNT = 0u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR  = 0u;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1  = TIM_CR1_CEN;

    /* 必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY(默认 5)，否则 ISR 里
       调 FreeRTOS API 会触发 configASSERT。数值越大优先级越低。 */
    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;
        if (osSemaphoreRelease(Sem_IMU) != osOK) ++imu_sem_full;
    }
}

/* ================================================================ tasks ==== */

static void    enc_init(void);
static int16_t enc_read(void);
static void    enc2_init(void);
static int16_t enc2_read(void);

static const osThreadAttr_t attrIMU = {
    .name = "IMU", .stack_size = 1536u, .priority = osPriorityHigh1 };
static const osThreadAttr_t attrLidar = {
    .name = "LIDAR", .stack_size = 2048u, .priority = osPriorityNormal };
static const osThreadAttr_t attrCtrl = {
    .name = "CTRL", .stack_size = 3072u, .priority = osPriorityHigh2 };
static const osThreadAttr_t attrUI = {
    .name = "OLED", .stack_size = 1536u, .priority = osPriorityBelowNormal };
static const osThreadAttr_t attrRx = {
    .name = "BLE_RX", .stack_size = 1536u, .priority = osPriorityHigh3 };
static const osThreadAttr_t attrPlan = {
    .name = "PATH", .stack_size = 2048u, .priority = osPriorityAboveNormal };
static const osThreadAttr_t attrTx = {
    .name = "BLE_TX", .stack_size = 3072u, .priority = osPriorityBelowNormal1 };

static uint32_t microseconds(void) { return TIM5->CNT; }

static void init_measurement_clock(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->PSC = 83u;
    TIM5->ARR = UINT32_MAX;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;
}

static uint32_t timing_begin(unsigned task)
{
    uint32_t now = microseconds();
    taskENTER_CRITICAL();
    TaskTiming *entry = &timings[task];
    if (entry->count) entry->period = now - entry->last;
    entry->last = now;
    ++entry->count;
    taskEXIT_CRITICAL();
    return now;
}

static void timing_end(unsigned task, uint32_t start)
{
    const uint32_t elapsed = microseconds() - start;
    taskENTER_CRITICAL();
    if (elapsed > timings[task].max) timings[task].max = elapsed;
    taskEXIT_CRITICAL();
}

static void wait_period(uint32_t *deadline, uint32_t period)
{
    const uint32_t now = osKernelGetTickCount();
    *deadline += period;
    if ((int32_t)(*deadline - now) <= 0) *deadline = now + period;
    (void)osDelayUntil(*deadline);
}

/* ---- HAL 时基改用 TIM6：SysTick 已被 FreeRTOS 接管，若不换时基，
        HAL_GetTick() 永远不涨，所有带超时的 HAL 阻塞调用（I2C/UART）
        都会变成死循环。TIM6 中断只做 HAL_IncTick()。 ---- */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    (void)TickPriority;
    const uint32_t bus = HAL_RCC_GetPCLK1Freq();
    const uint32_t timer_clock = (RCC->CFGR & RCC_CFGR_PPRE1) == 0u ? bus : 2u * bus;
    TIM6->PSC = timer_clock / 1000000u - 1u;
    TIM6->ARR  = 1000u - 1u;     /* 1 kHz -> 1 ms */
    TIM6->CNT  = 0;
    TIM6->EGR  = TIM_EGR_UG;
    TIM6->SR   = 0;
    TIM6->DIER = TIM_DIER_UIE;
    TIM6->CR1  = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    return HAL_OK;
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;
        HAL_IncTick();
    }
}

/* IMU：100 Hz，由 TIM2 信号量驱动，读 MPU 并换算 */
static void Task_IMU(void *arg)
{
    (void)arg;
    uint8_t raw[14];
    unsigned failed = 0u;
    ImuHeading heading;
    ImuHeadingInit(&heading, NAV_INITIAL_YAW_RAD);
    int16_t last_l = enc_read(), last_r = enc2_read();
    uint32_t previous = 0u, next_probe = 0u;
    for (;;) {
        (void)osSemaphoreAcquire(Sem_IMU, osWaitForever);
        const uint32_t start = timing_begin(TIME_IMU);
        bool ok = false;
        if (!imu_ok && (int32_t)(start - next_probe) < 0) {
            timing_end(TIME_IMU, start);
            continue;
        }
        if (osMutexAcquire(Mtx_I2C, 5u) == osOK) {
            const uint32_t waited = microseconds() - start;
            if (waited > i2c_wait_max) i2c_wait_max = waited;
            if (!imu_ok) {
                /* Retry a stuck shared bus under the same mutex used by OLED. */
                i2c_bus_recover();
                ++imu_bus_recoveries;
                imu_ok = imu_configure();
                next_probe = start + 500000u;
            }
            ok = imu_ok && imu_read(0x3Bu, raw, sizeof raw) == HAL_OK;
            (void)osMutexRelease(Mtx_I2C);
        }
        if (ok) {
            failed = 0u;
            const uint32_t now = microseconds();
            const uint32_t dt = now - previous;
            previous = now;
            const int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
            const int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
            const int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
            const int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
            const int16_t now_l = enc_read(), now_r = enc2_read();
            const bool wheels_still = now_l == last_l && now_r == last_r;
            last_l = now_l;
            last_r = now_r;
            taskENTER_CRITICAL();
            const bool stopped = nav_report.state != NAV_RUNNING &&
                                 !control_report.braking &&
                                 nav_report.pwm_l == 0 && nav_report.pwm_r == 0;
            taskEXIT_CRITICAL();
            const float vx = (float)ax / 16384.0f;
            const float vy = (float)ay / 16384.0f;
            const float vz = (float)az / 16384.0f;
            ImuHeadingUpdate(&heading, (float)gz / 131.0f,
                             vx * vx + vy * vy + vz * vz, dt, stopped, wheels_still);
            taskENTER_CRITICAL();
            s_ax = ax;
            s_ay = ay;
            s_az = az;
            s_gz = gz;
            ++s_imu_frames;
            imu_calibrated = heading.calibrated;
            imu_bias_dps = heading.bias_dps;
            imu_yaw = NAV_INITIAL_YAW_RAD + (float)PART2_IMU_SIGN * (heading.yaw_rad - NAV_INITIAL_YAW_RAD);
            if (imu_yaw > 3.14159265f) imu_yaw -= 6.28318531f;
            if (imu_yaw < -3.14159265f) imu_yaw += 6.28318531f;
            imu_sample_us = now;
            taskEXIT_CRITICAL();
        } else {
            ++imu_errors;
            if (++failed >= 3u) {
                imu_ok = false;
                failed = 0u;
                next_probe = start + 500000u;
            }
        }
        timing_end(TIME_IMU, start);
    }
}
/* USART1 RX 使用循环 DMA。输入为空时保持监听，绝不循环发送 STOP/SCAN。 */
static bool lidar_start_receive(void)
{
    lidar_receiving = 0u;
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    const HAL_StatusTypeDef stopped = HAL_UART_AbortReceive(&huart1);
    if (stopped != HAL_OK || HAL_DMA_GetState(&hdma_usart1_rx) != HAL_DMA_STATE_READY) {
        if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK || HAL_UART_AbortReceive(&huart1) != HAL_OK) {
            return false;
        }
    }
    (void)osMessageQueueReset(Q_Lidar);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    lidar_receiving = 1u;
    if (HAL_UART_Receive_DMA(&huart1, lidar_buf, LIDAR_BUF_SIZE) != HAL_OK) {
        lidar_receiving = 0u;
        return false;
    }
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return true;
}

static void lidar_send_scan(void)
{
    uint8_t command[2] = {0xA5u, 0x20u};
    ++lidar_scan_attempts;
    (void)HAL_UART_Transmit(&huart1, command, sizeof command, 3u);
}

static bool lidar_dma_writes_half(uint8_t half)
{
    const uint32_t remaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    const uint8_t active_half = (remaining > LIDAR_HALF_SIZE) ? 0u : 1u;
    return remaining == 0u || active_half == half;
}

static bool lidar_copy_finished_half(LidarChunk_t chunk, uint8_t *out)
{
    const uint32_t now = osKernelGetTickCount();
    /* At 460800 8N1 a half takes at least 10.85 ms to fill. Reject pending
       descriptors and reject a copy crossing the next DMA half boundary. */
    if (!lidar_receiving || now - chunk.tick >= 10u ||
        chunk.sequence != lidar_cb || lidar_dma_writes_half(chunk.half)) {
        return false;
    }
    __DMB();
    memcpy(out, lidar_buf + chunk.half * LIDAR_HALF_SIZE, LIDAR_HALF_SIZE);
    __DMB();
    return lidar_receiving && osKernelGetTickCount() - chunk.tick < 10u &&
           chunk.sequence == lidar_cb && !lidar_dma_writes_half(chunk.half);
}

/* ================= 点云：解析 + 经蓝牙上发 =================
   5 字节一个样点：b0 = S/!S/quality，b1 = C+angle[14:8]，b2 = angle[7:0]，
   b3 = dist[7:0]，b4 = dist[15:8]（dist_q2，单位 1/4 mm）。
   按 5 度一个桶累积成 72 个角度桶，每两圈发布整圈快照：
     S <起始角度> <d0> ... <d11>     12 个桶一包，共 6 包
   每个桶发布点数最多、且至少 3 点的一簇回波，距离单位 mm，0 表示该角度无回波。
   ASCII 格式由遥测任务发送。 */
#define MAP_BINS      72
#define MAP_BIN_DEG   5
#define MAP_PKT_BINS  12

static const uint8_t MAP_DESC[7] = {0xA5u, 0x5Au, 0x05u, 0x00u, 0x00u, 0x40u, 0x81u};
static uint16_t map_bin[MAP_BINS];      /* 每个 5 度桶确认后的距离 mm，0 表示该方向没有成簇回波 */
static uint16_t map_latest[MAP_BINS];
static NavLidarHit map_nearest;         /* 本圈已确认回波里最近的一处，供页面标记 */
static bool     map_pending;
/* 同一扇区里距离相差 80 mm 以内算同一表面。至少 3 个点才发布，单点杂波不占桶。 */
#define MAP_CLUSTER_TOL_MM 80u
#define MAP_CLUSTER_MIN_HITS 3u
#define CONFIRMED_NEAR_MM 280u
#define CONFIRMED_AVOID_MM 800u
typedef struct {
    uint16_t mm[2];
    uint8_t hits[2];
} MapCluster;
static MapCluster map_cluster[MAP_BINS];
static bool     map_complete_revolution;
static volatile uint8_t map_lock;      /* 最近收到有效测距帧 */
static uint8_t  map_didx   = 0;         /* 描述符匹配进度 */
static uint8_t  map_smp[5];
static uint8_t  map_slen   = 0;
static uint8_t  map_skip   = 0;         /* 隔圈发送 */
static NavLidarGuard lidar_guard;

static void dma_send(USART_TypeDef *port, DMA_Stream_TypeDef *stream, uint8_t *owned, uint32_t flags)
{
    if (!tx_size || (stream->CR & DMA_SxCR_EN)) return;
    memcpy(owned, tx_lines, tx_size);
    DMA1->HIFCR = flags;
    stream->PAR = (uint32_t)&port->DR;
    stream->M0AR = (uint32_t)owned;
    stream->NDTR = tx_size;
    SET_BIT(port->CR3, USART_CR3_DMAT);
    SET_BIT(stream->CR, DMA_SxCR_EN);
}

/* Task_Tx is the sole transmitter. */
static void uart4_tx_bytes(const uint8_t *bytes, uint16_t size)
{
    if (size <= TX_CAPACITY - tx_size) {
        memcpy(tx_lines + tx_size, bytes, size);
        tx_size += size;
    }
}

static void flush_telemetry(void)
{
    dma_send(UART4, DMA1_Stream4, ble_dma_payload, 0x3Du);
#if !PART2_ENABLE_MOTORS
    dma_send(USART2, DMA1_Stream6, usb_dma_payload, 0x3Du << 16u);
#endif
    tx_size = 0u;
}

static void map_send_scan(void)
{
    uint16_t bins[MAP_BINS];
    char buf[96];
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool pending = map_pending;
    if (pending) {
        memcpy(bins, map_latest, sizeof bins);
        map_pending = false;
    }
    __set_PRIMASK(mask);
    if (!pending) {
        return;
    }

    for (uint8_t k = 0; k < (uint8_t)(MAP_BINS / MAP_PKT_BINS); k++) {
        int n = snprintf(buf, sizeof buf, "S %u", (unsigned)(k * MAP_PKT_BINS * MAP_BIN_DEG));

        for (uint8_t i = 0; i < (uint8_t)MAP_PKT_BINS; i++) {
            n += snprintf(buf + n, sizeof buf - (size_t)n, " %u",
                          (unsigned)bins[k * MAP_PKT_BINS + i]);
        }
        n += snprintf(buf + n, sizeof buf - (size_t)n, "\r\n");
        uart4_tx_bytes((const uint8_t *)buf, (uint16_t)n);
        s_map_pkts++;
    }
}

static void reset_lidar_parser(void)
{
    NavLidarResetSweep(&lidar_guard);
    map_lock = 0u;
    map_didx = 0u;
    map_slen = 0u;
    map_skip = 0u;
    map_complete_revolution = false;
    map_nearest = (NavLidarHit){0};
    memset(map_bin, 0, sizeof map_bin);
    memset(map_cluster, 0, sizeof map_cluster);
}

static NavLidarHit confirmed_hit(bool forward_corridor)
{
    uint16_t bins[MAP_BINS];
    NavLidarHit best = {0};
    taskENTER_CRITICAL();
    memcpy(bins, map_latest, sizeof bins);
    taskEXIT_CRITICAL();
    for (uint8_t i = 0u; i < MAP_BINS; ++i) {
        const uint16_t distance = bins[i];
        const uint16_t angle = (uint16_t)((i * MAP_BIN_DEG + MAP_BIN_DEG / 2u) * 64u);
        if (distance == 0u) continue;
        if (forward_corridor) {
            if (!NavLidarInForwardCorridor(angle, distance, PART2_LIDAR_FRONT_Q6, CONFIRMED_AVOID_MM)) continue;
        } else if (distance > CONFIRMED_NEAR_MM) {
            continue;
        }
        if (best.distance_mm == 0u || distance < best.distance_mm) {
            best.distance_mm = distance;
            best.angle_q6 = angle;
        }
    }
    return best;
}

static void map_cluster_add(uint8_t bin, uint16_t distance_mm)
{
    MapCluster *cluster = &map_cluster[bin];
    for (unsigned slot = 0u; slot < 2u; ++slot) {
        uint16_t delta;
        if (cluster->hits[slot] == 0u) continue;
        delta = distance_mm > cluster->mm[slot] ? (uint16_t)(distance_mm - cluster->mm[slot])
                                                : (uint16_t)(cluster->mm[slot] - distance_mm);
        if (delta > MAP_CLUSTER_TOL_MM) continue;
        if (cluster->hits[slot] < 255u) cluster->hits[slot]++;
        cluster->mm[slot] = (uint16_t)(((uint32_t)cluster->mm[slot] * 3u + distance_mm) / 4u);
        return;
    }
    {
        const unsigned weak = cluster->hits[1] < cluster->hits[0] ? 1u : 0u;
        if (cluster->hits[weak] < MAP_CLUSTER_MIN_HITS) {
            cluster->mm[weak] = distance_mm;
            cluster->hits[weak] = 1u;
        }
    }
}

static NavLidarHit map_finalize_bins(void)
{
    NavLidarHit nearest = {0};
    for (uint8_t i = 0u; i < MAP_BINS; ++i) {
        const MapCluster *cluster = &map_cluster[i];
        const unsigned best = cluster->hits[1] > cluster->hits[0] ? 1u : 0u;
        NavLidarHit hit;
        if (cluster->hits[best] < MAP_CLUSTER_MIN_HITS) {
            map_bin[i] = 0u;
            continue;
        }
        map_bin[i] = cluster->mm[best];
        hit.distance_mm = cluster->mm[best];
        hit.angle_q6 = (uint16_t)((i * MAP_BIN_DEG + MAP_BIN_DEG / 2u) * 64u);
        if (nearest.distance_mm == 0u || hit.distance_mm < nearest.distance_mm) nearest = hit;
    }
    return nearest;
}

static void publish_lidar_revolution(void)
{
    if (map_complete_revolution) {
        const NavLidarHit nearest = map_finalize_bins();
        map_skip ^= 1u;
        if (map_skip != 0u) {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            memcpy(map_latest, map_bin, sizeof map_latest);
            map_nearest = nearest;
            map_pending = true;
            __set_PRIMASK(mask);
            ++s_scan_frames;
        } else {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            map_nearest = nearest;
            __set_PRIMASK(mask);
        }
    }
    memset(map_bin, 0, sizeof map_bin);
    memset(map_cluster, 0, sizeof map_cluster);
    map_complete_revolution = true;
}

static void lidar_parse(const uint8_t *p, uint16_t n, uint32_t tick_ms)
{
    for (uint16_t i = 0; i < n; i++) {
        const uint8_t byte = p[i];
        if (byte == MAP_DESC[map_didx]) {
            if (++map_didx == sizeof MAP_DESC) {
                reset_lidar_parser();
                continue;
            }
        } else {
            map_didx = (byte == MAP_DESC[0]) ? 1u : 0u;
        }
        map_smp[map_slen++] = byte;
        if (map_slen < 5u) continue;

        {
            uint8_t  s0  = map_smp[0];
            uint16_t ang = (uint16_t)(((uint16_t)map_smp[2] << 7) | (uint16_t)(map_smp[1] >> 1));

            /* S/inverse-S complement, angle check bit and Q6 angle range. */
            const uint8_t sync = s0 & 0x03u;
            if ((sync != 1u && sync != 2u) || (map_smp[1] & 1u) == 0u || ang >= 23040u) {
                if (map_lock) {
                    reset_lidar_parser();
                }
                memmove(map_smp, map_smp + 1u, sizeof map_smp - 1u);
                map_slen = 4u;
                continue;
            }

            map_slen = 0u;
            map_lock = 1u;                            /* 已同步（诊断用） */

            if (sync == 1u) {
                publish_lidar_revolution();
            }

            {
                uint16_t dq2 = (uint16_t)(((uint16_t)map_smp[4] << 8) | (uint16_t)map_smp[3]);
                uint16_t dmm = (uint16_t)(dq2 >> 2);
                uint8_t  bin = (uint8_t)((ang / 64u) / MAP_BIN_DEG);

                const uint8_t quality = (uint8_t)(s0 >> 2);
                NavLidarObserve(&lidar_guard, ang, dmm, quality, sync == 1u, tick_ms, PART2_LIDAR_FRONT_Q6);
                if (bin >= MAP_BINS) bin = (uint8_t)(MAP_BINS - 1);
                if (quality != 0u && dmm >= 50u && dmm <= 12000u &&
                    !NavLidarBodyShell(ang, dmm, PART2_LIDAR_FRONT_Q6)) {
                    map_cluster_add(bin, dmm);
                }
            }
        }
    }
    taskENTER_CRITICAL();
    lidar_status = lidar_guard.status;
    taskEXIT_CRITICAL();
}


static void Task_Lidar(void *arg)
{
    (void)arg;
    uint8_t copy[LIDAR_HALF_SIZE];
    uint32_t last_sequence = 0u;
    uint32_t backoff_ticks = 250u;
    uint32_t last_scan_tick = osKernelGetTickCount();
    bool have_sequence = false;
    bool have_stream = false;
    const uint32_t first_scan_count = s_scan_frames;
    LidarChunk_t chunk;

    /* Match DMA priority so HAL's UART error path and DMA handler do not nest. */
    HAL_NVIC_SetPriority(USART1_IRQn, 6u, 0u);

    for (;;) {
        if (!lidar_receiving) {
            reset_lidar_parser();
            have_sequence = false;
            if (!lidar_start_receive()) {
                ++lidar_restarts;
                (void)osDelay(backoff_ticks);
                if (backoff_ticks < 2000u) backoff_ticks *= 2u;
                continue;
            }
        }

        if (osMessageQueueGet(Q_Lidar, &chunk, NULL, 100u) == osOK) {
            const uint32_t started = timing_begin(TIME_LIDAR);
            if (!lidar_copy_finished_half(chunk, copy)) {
                ++lidar_lost_blocks;
                have_sequence = false;
                reset_lidar_parser();
            } else {
                if (have_sequence && chunk.sequence != last_sequence + 1u) {
                    reset_lidar_parser();
                }
                lidar_parse(copy, sizeof copy, chunk.tick);
                last_sequence = chunk.sequence;
                have_sequence = true;
                ++s_lidar_chunks;
                if (s_scan_frames != first_scan_count) {
                    have_stream = true;
                }
                if (map_lock) {
                    backoff_ticks = 250u;
                }
            }
            timing_end(TIME_LIDAR, started);
        }
        lidar_drop = lidar_queue_drop + lidar_lost_blocks;

        if (!lidar_receiving) {
            /* Recover the receiver after UART/DMA errors without moving the motor. */
            ++lidar_restarts;
            (void)osDelay(backoff_ticks);
            if (backoff_ticks < 2000u) backoff_ticks *= 2u;
            continue;
        }

        const uint32_t now = osKernelGetTickCount();
        /* First listen for an active scan. One extra SCAN is allowed after a
           six-second startup grace. A silent port then stays listening. */
        if (!have_stream &&
            ((lidar_scan_attempts == 0u && now - last_scan_tick >= 400u) ||
             (lidar_scan_attempts == 1u && now - last_scan_tick >= 6000u))) {
            last_scan_tick = now;
            lidar_send_scan();
        }
    }
}


static void submit_command(Cmd_t cmd)
{
    taskENTER_CRITICAL();
    cmd.id = ++newest_command;
    taskEXIT_CRITICAL();
    if (osMessageQueuePut(Q_Cmd, &cmd, 0u, 0u) != osOK) {
        ++cmd_drop;
        (void)osMessageQueueReset(Q_Cmd);
        (void)osMessageQueuePut(Q_Cmd, &cmd, 0u, 0u);
    }
}

static void Task_Rx(void *arg)
{
    (void)arg;
    uart4_rx_start();
#if !PART2_ENABLE_MOTORS
    SET_BIT(USART2->CR1, USART_CR1_RXNEIE);
#endif
    char line[40];
    size_t length = 0u;
    bool discard = false;
    for (;;) {
        (void)osThreadFlagsWait(1u, osFlagsWaitAny, osWaitForever);
        const uint32_t start = timing_begin(TIME_RX);
        for (;;) {
            taskENTER_CRITICAL();
            if (rx_discard) {
                rx_tail = rx_head;
                rx_discard = false;
                discard = true;
                length = 0u;
            }
            const uint16_t tail = rx_tail;
            if (tail == rx_head) {
                taskEXIT_CRITICAL();
                break;
            }
            const uint8_t byte = rx_ring[tail];
            rx_tail = (tail + 1u) & 255u;
            taskEXIT_CRITICAL();
            if (byte == '\r' || byte == '\n') {
                if (!discard && length != 0u) {
                    Cmd_t parsed;
                    line[length] = '\0';
                    if (ParseVehicleCommand(line, &parsed)) submit_command(parsed);
                }
                discard = false;
                length = 0u;
            } else if (byte == 0u || length + 1u >= sizeof line) {
                discard = true;
            } else if (!discard) {
                line[length++] = (char)byte;
            }
        }
        timing_end(TIME_RX, start);
    }
}

static void Task_Plan(void *arg)
{
    (void)arg;
    PathMessage next;
    for (;;) {
        if (osMessageQueueGet(Q_Cmd, &next.command, NULL, osWaitForever) != osOK) continue;
        const uint32_t started = timing_begin(TIME_PLAN);
        taskENTER_CRITICAL();
        const NavPose start = pose;
        const bool latest = next.command.id == newest_command;
        taskEXIT_CRITICAL();
        if (latest) {
            memset(&next.path, 0, sizeof next.path);
            next.valid = next.command.kind != 'G' ||
                         NavGeneratePath(&next.path, &start, 0.001f * next.command.a,
                                         0.001f * next.command.b, 0.10f);
            if (osMessageQueuePut(Q_Path, &next, 0u, 0u) != osOK) {
                ++path_drop;
                (void)osMessageQueueReset(Q_Path);
                (void)osMessageQueuePut(Q_Path, &next, 0u, 0u);
            }
        }
        timing_end(TIME_PLAN, started);
    }
}

static float limit_magnitude(float value, float bound)
{
    return fminf(bound, fmaxf(-bound, value));
}

static NavMotion drive_motion(float left, float right)
{
    const float forward = left + right;
    if (forward > 0.005f) return NAV_MOTION_FORWARD;
    if (forward < -0.005f) return NAV_MOTION_REVERSE;
    if (right > left) return NAV_MOTION_LEFT;
    if (left > right) return NAV_MOTION_RIGHT;
    return NAV_MOTION_IDLE;
}

/* One encoder count is about 13 mm/s at the 10 ms period. A single noisy
   sample must not select the opposite LiDAR corridor while coasting. */
static NavMotion confirmed_wheel_motion(float left, float right,
                                        NavMotion *held, uint8_t *samples)
{
    const bool moving = fabsf(left) >= 0.040f || fabsf(right) >= 0.040f;
    const NavMotion instant = moving ? drive_motion(left, right) : NAV_MOTION_IDLE;
    if (instant == *held) {
        if (*samples < 3u) ++*samples;
    } else {
        *held = instant;
        *samples = 1u;
    }
    return *samples >= 3u ? *held : NAV_MOTION_IDLE;
}

static void Task_Ctrl(void *arg)
{
    (void)arg;
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous = microseconds() - 10000u;
    uint32_t started = 0u, selected = 0u;
    unsigned heartbeat = 0u;
    int16_t last_l = enc_read(), last_r = enc2_read();
    NavPose estimate = { .yaw_rad = NAV_INITIAL_YAW_RAD };
    NavTracker tracker;
    NavPid l_pid, r_pid;
    NavResetTracker(&tracker);
    NavResetPid(&l_pid);
    NavResetPid(&r_pid);
    PathMessage active = {0};
    PathMessage next;
    NavReport report = {0};
    float manual_done = 0.0f;
    bool had_imu = false;
    bool was_driving = false, braking = false;
    uint32_t brake_started = 0u;
    uint32_t left_motion = microseconds(), right_motion = left_motion;
    NavMotion sensed_motion = NAV_MOTION_IDLE;
    uint8_t sensed_samples = 0u;
    int8_t avoid_side = 0;
    uint32_t avoid_since = 0u;
    uint32_t avoid_clear_since = 0u;

    for (;;) {
        wait_period(&tick, 10u);
        const uint32_t now = timing_begin(TIME_POSE);
        const float dt = (float)(now - previous) * 1.0e-6f;
        previous = now;
        const int16_t now_l = enc_read(), now_r = enc2_read();
        const float dl = (float)(int16_t)(now_l - last_l) * MM_PER_TICK_L * PART2_ENCODER_L * 0.001f;
        const float dr = (float)(int16_t)(now_r - last_r) * MM_PER_TICK_R * ENC_R_DIR * 0.001f;
        last_l = now_l;
        last_r = now_r;
        taskENTER_CRITICAL();
        const float yaw = imu_yaw;
        const float bias = imu_bias_dps;
        const bool imu_ready = imu_calibrated && (microseconds() - imu_sample_us <= 50000u);
        const uint32_t latest = newest_command;
        taskEXIT_CRITICAL();
        const float track = TRACK_MM * 0.001f;
        const bool valid_dt = dt >= 0.001f && dt <= 0.04f;
        const float actual_l = valid_dt ? dl / dt : 0.0f;
        const float actual_r = valid_dt ? dr / dt : 0.0f;
        const bool pose_ok = NavUpdatePose(&estimate, dl, dr, track, yaw, imu_ready);
        taskENTER_CRITICAL();
        pose = estimate;
        odom_x = estimate.x_m * 1000.0f;
        odom_y = estimate.y_m * 1000.0f;
        odom_th_deg = estimate.yaw_rad * 57.2957795f;
        taskEXIT_CRITICAL();
        timing_end(TIME_POSE, now);

        const uint32_t track_start = timing_begin(TIME_TRACK);
        if (selected != latest) {
            selected = latest;
            active.path.count = 0u;
            active.command.kind = 0;
            report = (NavReport){ .command.id = latest, .state = NAV_IDLE };
            NavResetPid(&l_pid);
            NavResetPid(&r_pid);
            avoid_side = 0;
            avoid_since = 0u;
            avoid_clear_since = 0u;
        }
        while (osMessageQueueGet(Q_Path, &next, NULL, 0u) == osOK) {
            if (next.command.id != selected) continue;
            active = next;
            report = (NavReport){ .command = next.command, .count = (uint8_t)next.path.count };
            if (next.command.kind == 'S') {
                report.state = NAV_STOPPED;
            } else if (!next.valid) {
                report.state = NAV_FAULT;
                report.fault = FAULT_PATH;
            } else {
                report.state = PART2_ENABLE_MOTORS ? NAV_RUNNING : NAV_BENCH;
            }
            started = now;
            left_motion = right_motion = now;
            had_imu = false;
            manual_done = 0.0f;
            NavResetTracker(&tracker);
            taskENTER_CRITICAL();
            published_path = next;
            taskEXIT_CRITICAL();
        }
        float left = 0.0f, right = 0.0f;
        if (report.state == NAV_RUNNING || report.state == NAV_IMU_WAIT) {
            if (!pose_ok || dt > 0.04f || dt < 0.001f) {
                report.state = NAV_FAULT;
                report.fault = FAULT_CONTROL;
            } else if (now - started > 45000000u) {
                report.state = NAV_FAULT;
                report.fault = FAULT_TIMEOUT;
            } else if (active.command.kind == 'G') {
                if (!imu_ready) {
                    report.state = had_imu ? NAV_FAULT : NAV_IMU_WAIT;
                    report.fault = had_imu ? FAULT_IMU : FAULT_NONE;
                } else {
                    if (!had_imu) {
                        /* Calibration waits precede motion and must not count as a stall. */
                        left_motion = right_motion = now;
                        started = now;
                        had_imu = true;
                    }
                    const NavTrackStatus result = NavTrackStep(&tracker, &active.path, &estimate,
                                                               track, dt, &left, &right);
                    report.index = (uint8_t)tracker.waypoint_index;
                    report.state = result == NAV_TRACK_RUNNING ? NAV_RUNNING :
                                   (result == NAV_TRACK_ARRIVED ? NAV_DONE : NAV_FAULT);
                    if (result == NAV_TRACK_INVALID) report.fault = FAULT_CONTROL;
                }
            } else {
                const char kind = active.command.kind;
                const bool linear = kind == 'F' || kind == 'B';
                const float signed_target = (float)active.command.a * ((kind == 'B' || kind == 'R') ? -1.0f : 1.0f);
                manual_done += linear ? (dl + dr) * 500.0f : (dr - dl) * 57.2957795f / track;
                const float error = signed_target - manual_done;
                if (fabsf(error) < (linear ? 3.0f : 2.0f)) {
                    report.state = NAV_DONE;
                } else if (linear) {
                    const float cap = V_MIN_MMS + (float)s_mv1 * (V_MAX_MMS - V_MIN_MMS) / 3300.0f;
                    left = right = limit_magnitude(0.0015f * error, 0.001f * cap);
                } else {
                    const float cap = W_MIN_DPS + (float)s_mv2 * (W_MAX_DPS - W_MIN_DPS) / 3300.0f;
                    right = limit_magnitude(3.0f * error, cap) * track * 0.00872664626f;
                    left = -right;
                }
            }
        }
        if (PART2_ENABLE_MOTORS && report.state == NAV_RUNNING) {
            const bool reverse_command = active.command.kind == 'B';
            report.motion = (uint8_t)drive_motion(left, right);
            taskENTER_CRITICAL();
            const NavLidarStatus scan = lidar_status;
            taskEXIT_CRITICAL();
            const NavMotion sensed = confirmed_wheel_motion(actual_l, actual_r,
                                                           &sensed_motion, &sensed_samples);
            const bool reversing = reverse_command || report.motion == NAV_MOTION_REVERSE ||
                                   sensed == NAV_MOTION_REVERSE;
            float avoid_yaw = 0.0f;
            const NavLidarHit corridor = confirmed_hit(true);
            if (PART2_ENABLE_AVOIDANCE && !reversing) {
                avoid_yaw = NavAvoidYawRate(&corridor, PART2_LIDAR_FRONT_Q6, &avoid_side);
                if (avoid_yaw != 0.0f) {
                    avoid_clear_since = 0u;
                    if (avoid_since == 0u) avoid_since = now;
                } else if (avoid_side != 0) {
                    if (avoid_clear_since == 0u) avoid_clear_since = now;
                    if (now - avoid_clear_since >= 500000u) {
                        avoid_side = 0;
                        avoid_since = 0u;
                        avoid_clear_since = 0u;
                    } else {
                        avoid_yaw = avoid_side > 0 ? 0.40f : -0.40f;
                    }
                }
            }
            if (avoid_yaw != 0.0f) {
                /* Keep both wheels rolling forward so a pass cannot look like reverse. */
                const float forward = 0.050f;
                float turn = avoid_yaw * 0.5f * track;
                const float turn_limit = forward - 0.010f;
                if (turn > turn_limit) turn = turn_limit;
                if (turn < -turn_limit) turn = -turn_limit;
                left = forward - turn;
                right = forward + turn;
                report.motion = (uint8_t)drive_motion(left, right);
            }
            NavLidarHit obstacle = confirmed_hit(false);
            if (reversing && scan.hazards[LIDAR_REAR].distance_mm != 0u &&
                (obstacle.distance_mm == 0u ||
                 scan.hazards[LIDAR_REAR].distance_mm < obstacle.distance_mm)) {
                obstacle = scan.hazards[LIDAR_REAR];
            }
            const bool passing = avoid_side != 0 && !reversing;
            const bool blocked = NavLidarBlocksPass(&obstacle, PART2_LIDAR_FRONT_Q6, passing);
            const bool avoid_too_long = avoid_since != 0u && now - avoid_since > 8000000u;
            if (!lidar_receiving || !NavLidarFresh(&scan, osKernelGetTickCount())) {
                report.state = NAV_LIDAR_STOP;
                report.fault = FAULT_LIDAR_STALE;
            } else if (PART2_ENABLE_AVOIDANCE && (blocked || avoid_too_long)) {
                /* A new command is required even after a clear sweep. */
                report.state = NAV_LIDAR_STOP;
                report.fault = FAULT_OBSTACLE;
                report.obstacle_stop = obstacle.distance_mm != 0u ? obstacle : corridor;
                avoid_side = 0;
                avoid_since = 0u;
                avoid_clear_since = 0u;
            }
        }
        if (PART2_ENABLE_MOTORS && report.state == NAV_RUNNING) {
            const float fastest = fmaxf(fabsf(left), fabsf(right));
            if (fastest > DRIVE_WHEEL_LIMIT_MPS) {
                const float scale = DRIVE_WHEEL_LIMIT_MPS / fastest;
                left *= scale;
                right *= scale;
            }
            if (dl != 0.0f || fabsf(left) < 0.040f) left_motion = now;
            if (dr != 0.0f || fabsf(right) < 0.040f) right_motion = now;
            const bool left_stalled = now - left_motion > 800000u;
            const bool right_stalled = now - right_motion > 800000u;
            if (left_stalled || right_stalled) {
                report.state = NAV_FAULT;
                report.fault = left_stalled ?
                    (right_stalled ? FAULT_STALL_BOTH : FAULT_STALL_LEFT) : FAULT_STALL_RIGHT;
            }
        }
        timing_end(TIME_TRACK, track_start);

        const uint32_t motor_start = timing_begin(TIME_MOTOR);
        report.imu_ready = imu_ready;
        report.ticks_l = now_l;
        report.ticks_r = now_r;
        report.pwm_l = 0;
        report.pwm_r = 0;
        const float u_l = report.state == NAV_RUNNING ? NavMotorStep(&l_pid, left, actual_l, dt) : 0.0f;
        const float u_r = report.state == NAV_RUNNING ? NavMotorStep(&r_pid, right, actual_r, dt) : 0.0f;
        taskENTER_CRITICAL();
        const bool driving = report.state == NAV_RUNNING && newest_command == selected;
        if (!driving) {
            if (was_driving) {
                braking = true;
                brake_started = motor_start;
            }
            was_driving = false;
            braking = braking && motor_start - brake_started < 250000u;
            if (braking) motor_brake();
            else motor_set(0, 0);
            NavResetPid(&l_pid);
            NavResetPid(&r_pid);
        } else {
            braking = false;
            report.pwm_l = (int16_t)(1000.0f * u_l);
            report.pwm_r = (int16_t)(1000.0f * u_r);
            motor_set(report.pwm_l, report.pwm_r);
            was_driving = report.pwm_l != 0 || report.pwm_r != 0;
        }
        control_report = (ControlReport){
            .run = report.command.id,
            .reference_l = driving ? (int16_t)(1000.0f * left) : 0,
            .reference_r = driving ? (int16_t)(1000.0f * right) : 0,
            .actual_l = (int16_t)limit_magnitude(1000.0f * actual_l, 32000.0f),
            .actual_r = (int16_t)limit_magnitude(1000.0f * actual_r, 32000.0f),
            .gyro_yaw10 = (int16_t)(572.957795f * yaw),
            .fused_yaw10 = (int16_t)(572.957795f * estimate.yaw_rad),
            .bias_mdps = (int16_t)(1000.0f * bias),
            .braking = braking,
        };
        nav_report = report;
        taskEXIT_CRITICAL();
        timing_end(TIME_MOTOR, motor_start);
        if (++heartbeat >= 50u) {
            heartbeat = 0u;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}

static uint16_t read_pot(uint8_t channel)
{
    const uint16_t value = adc_read(channel);
    return value <= 4095u ? (uint16_t)(((uint32_t)value * 3300u) / 4095u) : 0u;
}

enum { OLED_SEGMENT_BYTES = 32u, OLED_SEGMENTS_PER_PAGE = 128u / OLED_SEGMENT_BYTES };
static volatile uint32_t oled_segments_written;
static volatile uint32_t oled_write_failures;

static bool oled_wait_bus_idle(void)
{
    const uint32_t started = microseconds();
    while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        if (microseconds() - started >= 300u) return false;
    }
    return true;
}

static bool oled_flush_page(uint8_t page, uint8_t segment)
{
    const uint8_t offset = (uint8_t)(segment * OLED_SEGMENT_BYTES);
    const uint8_t column = (uint8_t)(offset + OLED_COL_OFFSET);
    uint8_t cursor[] = {0u, (uint8_t)(0xB0u | page),
                        column & 15u, (uint8_t)(0x10u | (column >> 4))};
    if (!oled_wait_bus_idle() ||
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, cursor, sizeof cursor, 2u) != HAL_OK ||
        !oled_wait_bus_idle()) {
        return false;
    }
    /* HAL issues STOP but may return before the hardware clears BUSY. */
    oled_line[0] = 0x40u;
    memcpy(&oled_line[1], &oled_fb[(size_t)page * 128u + offset], OLED_SEGMENT_BYTES);
    const HAL_StatusTypeDef result =
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, oled_line, OLED_SEGMENT_BYTES + 1u, 5u);
    const bool idle = oled_wait_bus_idle();
    return result == HAL_OK && idle;
}

static void Task_UI(void *arg)
{
    (void)arg;
    uint32_t time = osKernelGetTickCount();
    uint8_t row = 0u, segment = 0u;
    for (;;) {
        wait_period(&time, 50u);
        const uint32_t start = timing_begin(TIME_OLED);
        s_mv1 = read_pot(10u);
        s_mv2 = read_pot(11u);
        s_mv3 = read_pot(12u);
        if (oled_ok && segment == 0u) {
            char line[32];
            taskENTER_CRITICAL();
            const NavReport status = nav_report;
            const int x = (int)odom_x, y = (int)odom_y, yaw = (int)odom_th_deg;
            taskEXIT_CRITICAL();
            memset(oled_fb + (size_t)row * 128u, 0, 128u);
            switch (row) {
                case 0: snprintf(line, sizeof line, "PART 2 %s", PART2_ENABLE_MOTORS ? "DRIVE" : "BENCH"); break;
                case 1: snprintf(line, sizeof line, "X %d Y %d", x, y); break;
                case 2: snprintf(line, sizeof line, "YAW %d", yaw); break;
                case 3: snprintf(line, sizeof line, "GO %d %d", status.command.a, status.command.b); break;
                case 4:
                    if (status.state == NAV_LIDAR_STOP) {
                        snprintf(line, sizeof line, "%s", status.fault == FAULT_OBSTACLE ?
                                 "OBSTACLE STOP" : "LIDAR LOST STOP");
                    } else {
                        snprintf(line, sizeof line, "STATE %u WP %u/%u", (unsigned)status.state,
                                 (unsigned)status.index, (unsigned)status.count);
                    }
                    break;
                case 5: snprintf(line, sizeof line, "IMU %lu", (unsigned long)s_imu_frames); break;
                case 6: snprintf(line, sizeof line, "LIDAR %lu", (unsigned long)s_lidar_chunks); break;
                default: snprintf(line, sizeof line, "EL %d ER %d", (int)enc_read(), (int)enc2_read()); break;
            }
            oled_str(0, row, line);
        }
        if (oled_ok && osMutexAcquire(Mtx_I2C, 2u) == osOK) {
            if (oled_flush_page(row, segment)) {
                ++oled_segments_written;
                if (++segment == OLED_SEGMENTS_PER_PAGE) {
                    segment = 0u;
                    row = (row + 1u) & 7u;
                }
            } else {
                ++oled_write_failures;
            }
            (void)osMutexRelease(Mtx_I2C);
        }
        timing_end(TIME_OLED, start);
    }
}

static void transmit_line(const char *line, int length, size_t capacity)
{
    if (length > 0 && (size_t)length < capacity) uart4_tx_bytes((const uint8_t *)line, (uint16_t)length);
}

static void Task_Tx(void *arg)
{
    (void)arg;
    static const char *const task_names[TIMING_COUNT] = {
        "imu", "lidar", "rx", "plan", "pose", "track", "motor", "oled", "tx" };
    uint32_t time = osKernelGetTickCount(), sequence = 0u, path_run = 0u;
    unsigned stat = 0u, cursor = 0u;
    for (;;) {
        wait_period(&time, 50u);
        const uint32_t start = timing_begin(TIME_TX);
        taskENTER_CRITICAL();
        const NavReport report = nav_report;
        const ControlReport control = control_report;
        const NavLidarStatus scan = lidar_status;
        const NavLidarHit confirmed = map_nearest;
        const int x = (int)odom_x, y = (int)odom_y, heading = (int)(odom_th_deg * 10.0f);
        const uint32_t run = published_path.command.id;
        taskEXIT_CRITICAL();
        char line[152];
        int n = snprintf(line, sizeof line, "P %d %d %d\r\n", x, y, heading);
        transmit_line(line, n, sizeof line);
        n = snprintf(line, sizeof line, "N %lu %u %d %d %u %u\r\n", (unsigned long)report.command.id,
                     (unsigned)report.state, report.command.a, report.command.b,
                     (unsigned)report.index, (unsigned)report.count);
        transmit_line(line, n, sizeof line);
        if (sequence % 4u == 0u) {
            n = snprintf(line, sizeof line, "A %lu %u %u %u %d %d %d %d\r\n",
                         (unsigned long)report.command.id, (unsigned)PART2_ENABLE_MOTORS,
                         (unsigned)report.fault, (unsigned)report.imu_ready,
                         report.ticks_l, report.ticks_r, report.pwm_l, report.pwm_r);
            transmit_line(line, n, sizeof line);
            n = snprintf(line, sizeof line, "C %lu %d %d %d %d %d %d %d %u\r\n",
                         (unsigned long)control.run, control.reference_l, control.reference_r,
                         control.actual_l, control.actual_r, control.gyro_yaw10,
                         control.fused_yaw10, control.bias_mdps, (unsigned)control.braking);
            transmit_line(line, n, sizeof line);
            const bool fresh = lidar_receiving && NavLidarFresh(&scan, osKernelGetTickCount());
            const NavMotion motion = (NavMotion)report.motion;
            const NavLidarHit ahead = confirmed_hit(true);
            const NavLidarHit near = confirmed_hit(false);
            n = snprintf(line, sizeof line, "O %u %u %u %u %d %d %u %d\r\n", (unsigned)fresh,
                         (unsigned)(fresh && (ahead.distance_mm != 0u || near.distance_mm != 0u)),
                         fresh ? (unsigned)confirmed.distance_mm : 0u,
                         (unsigned)report.obstacle_stop.distance_mm,
                         fresh && confirmed.distance_mm ? (int)confirmed.angle_q6 : -1,
                         report.obstacle_stop.distance_mm ? (int)report.obstacle_stop.angle_q6 : -1,
                         (unsigned)motion, (int)PART2_LIDAR_FRONT_Q6);
            transmit_line(line, n, sizeof line);
        }
        if (path_run != run) {
            cursor = 0u;
            path_run = run;
        }
        for (unsigned i = 0u; i < 2u; ++i) {
            taskENTER_CRITICAL();
            const unsigned count = published_path.path.count;
            const bool available = published_path.command.id == run && cursor < count;
            const NavPoint point = available ? published_path.path.points[cursor] : (NavPoint){0};
            taskEXIT_CRITICAL();
            if (!available) break;
            n = snprintf(line, sizeof line, "W %lu %u %ld %ld\r\n", (unsigned long)run, cursor,
                         lroundf(point.x_m * 1000.0f), lroundf(point.y_m * 1000.0f));
            transmit_line(line, n, sizeof line);
            if (++cursor == count) { cursor = 0u; break; }
        }
        if ((sequence & 1u) == 0u) {
            taskENTER_CRITICAL();
            const TaskTiming entry = timings[stat];
            taskEXIT_CRITICAL();
            n = snprintf(line, sizeof line, "T %s %lu %lu %lu\r\n", task_names[stat],
                         (unsigned long)entry.count, (unsigned long)entry.period, (unsigned long)entry.max);
            transmit_line(line, n, sizeof line);
            stat = (stat + 1u) % TIMING_COUNT;
        }
        if (sequence % 4u == 3u) map_send_scan();
        if (sequence % 20u == 0u) {
            n = snprintf(line, sizeof line, "RT ax=%d az=%d gz=%d eL=%d eR=%d oX=%d oY=%d oT=%d o1=%u o2=%u im=%lu cal=%u lk=%u pk=%lu\r\n",
                         (int)s_ax, (int)s_az, (int)s_gz, (int)enc_read(), (int)(ENC_R_DIR * enc2_read()),
                         x, y, heading / 10, (unsigned)s_mv1, (unsigned)s_mv2,
                         (unsigned long)s_imu_frames, (unsigned)imu_calibrated,
                         (unsigned)map_lock, (unsigned long)s_map_pkts);
            transmit_line(line, n, sizeof line);
            n = snprintf(line, sizeof line, "D %lu %lu %lu %lu %lu %lu %lu\r\n",
                         (unsigned long)lidar_drop, (unsigned long)cmd_drop, (unsigned long)path_drop,
                         (unsigned long)imu_sem_full, (unsigned long)imu_errors,
                         (unsigned long)i2c_wait_max, (unsigned long)rx_drop);
            transmit_line(line, n, sizeof line);
        }
        flush_telemetry();
        ++sequence;
        timing_end(TIME_TX, start);
    }
}

/* 编码器：TIM3 编码器模式，PA6=CH1 / PA7=CH2（AF2），四倍频计数。
   D157B 的 E1A/E1B 是开漏输出，所以这里开内部上拉。 */
static void enc_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &g);

    TIM3->CR1   = 0;
    TIM3->PSC   = 0;
    TIM3->ARR   = 0xFFFFu;
    TIM3->CCMR1 = (1u << 0) | (1u << 8);        /* CC1S=01(TI1) CC2S=01(TI2) */
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->SMCR  = (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1); /* SMS=011 -> 四倍频 */
    TIM3->CNT   = 0;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;
}

static int16_t enc_read(void)
{
    return (int16_t)TIM3->CNT;
}

/* 右轮编码器：TIM8 CH1=PC6 / CH2=PC7（AF3），同样四倍频。 */
static void enc2_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &g);

    TIM8->CR1   = 0;
    TIM8->PSC   = 0;
    TIM8->ARR   = 0xFFFFu;
    TIM8->CCMR1 = (1u << 0) | (1u << 8);
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM8->SMCR  = (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM8->CNT   = 0;
    TIM8->EGR   = TIM_EGR_UG;
    TIM8->CR1  |= TIM_CR1_CEN;
}

static int16_t enc2_read(void)
{
    return (int16_t)TIM8->CNT;
}

/* 电机：D157B 逻辑输入 AIN1/AIN2（左）、BIN1/BIN2（右）。
   AIN1=PB6 (TIM4_CH1)  AIN2=PB7 (TIM4_CH2)
   BIN1=PB14(TIM12_CH1) BIN2=PB15(TIM12_CH2)
   两路都是 20 kHz PWM（84 MHz / 4200）。
   IN1=PWM,IN2=0 正转；IN1=0,IN2=PWM 反转；都 0 滑行；都满 刹车。 */
static void motor_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;      /* AIN1 / AIN2 */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin       = GPIO_PIN_14 | GPIO_PIN_15;    /* BIN1 / BIN2 */
    g.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(GPIOB, &g);

    /* PWM mode 1 在 CH1/CH2：OCxM = 110 */
    TIM4->PSC   = 0u;
    TIM4->ARR   = 4199u;                        /* 84 MHz / 4200 = 20 kHz */
    TIM4->CCMR1 = (6u << 4) | (6u << 12);
    TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4->CCR1  = 0u;
    TIM4->CCR2  = 0u;
    TIM4->CR1   = TIM_CR1_ARPE;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->CR1  |= TIM_CR1_CEN;

    TIM12->PSC   = 0u;
    TIM12->ARR   = 4199u;
    TIM12->CCMR1 = (6u << 4) | (6u << 12);
    TIM12->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM12->CCR1  = 0u;
    TIM12->CCR2  = 0u;
    TIM12->CR1   = TIM_CR1_ARPE;
    TIM12->EGR   = TIM_EGR_UG;
    TIM12->CR1  |= TIM_CR1_CEN;
}

/* cmd: -1000..1000（正=前进），0=滑行；内部处理右轮镜像 */
static void motor_set(int16_t l, int16_t r)
{
    if (l >  1000) l =  1000;
    if (l < -1000) l = -1000;
    if (r >  1000) r =  1000;
    if (r < -1000) r = -1000;

#if !PART2_ENABLE_MOTORS
    l = 0;
    r = 0;
#endif
    r = (int16_t)(MOTOR_R_DIR * r);             /* 保留队友原左右轮极性 */
    l = (int16_t)(MOTOR_POL * l);                /* 左轮只翻整机极性 */

    if (l >= 0) {
        TIM4->CCR1 = (uint32_t)l * 4199u / 1000u;
        TIM4->CCR2 = 0u;
    } else {
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = (uint32_t)(-l) * 4199u / 1000u;
    }

    if (r >= 0) {
        TIM12->CCR1 = (uint32_t)r * 4199u / 1000u;
        TIM12->CCR2 = 0u;
    } else {
        TIM12->CCR1 = 0u;
        TIM12->CCR2 = (uint32_t)(-r) * 4199u / 1000u;
    }
}

static void motor_brake(void)
{
#if PART2_ENABLE_MOTORS
    /* AT8236 IN1=IN2=1 selects dynamic braking. CCR=ARR+1 is continuously high. */
    TIM4->CCR1 = TIM4->ARR + 1u;
    TIM4->CCR2 = TIM4->ARR + 1u;
    TIM12->CCR1 = TIM12->ARR + 1u;
    TIM12->CCR2 = TIM12->ARR + 1u;
#else
    motor_set(0, 0);
#endif
}

/* ================================================================== main === */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    init_measurement_clock();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    uart4_init();
#if !PART2_ENABLE_MOTORS
    uart2_bench_init();
#endif
    adc_init();
    enc_init();
    enc2_init();
    motor_init();       /* 上电两路都为 0，电机不转 */

    printf("\r\n========================================\r\n");
    printf("[BOOT] Block4.5 RTOS application\r\n");
    printf("[BOOT] SYSCLK = %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());

    /* ---- I2C bus ---- */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        g.Mode = GPIO_MODE_AF_OD;
        g.Pull = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &g);
    }
    i2c_bus_recover();
    i2c_scan();
    if (i2c_count == 0u) { i2c_bus_recover(); HAL_Delay(5); i2c_scan(); }

    printf("[I2C] found %u device(s):", (unsigned)i2c_count);
    for (uint8_t i = 0; i < i2c_count; i++) printf(" 0x%02X", i2c_found[i]);
    printf("\r\n");

    {
        uint8_t who = 0;
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x75,
                                                I2C_MEMADD_SIZE_8BIT, &who, 1, 30);
        imu_ok = st == HAL_OK && imu_configure();
        printf("[IMU] WHO_AM_I=0x%02X st=%d -> %s\r\n", who, (int)st,
               imu_ok ? "OK" : "FAIL");
    }

    oled_ok = 0;
    for (uint8_t i = 0; i < i2c_count; i++)
        if (i2c_found[i] == 0x3Cu) oled_ok = 1;

    oled_clear();
    HAL_Delay(100);
    if (oled_ok) {
        oled_init();
        HAL_Delay(20);
        oled_str(0, 0, "RTOS BOOT");
        oled_flush();
    }
    printf("[OLED] addr 0x3C -> %s\r\n", oled_ok ? "OK" : "NOT FOUND");

    /* ---- 雷达在 RTOS 对象建好之后再启动，避免 DMA 中断早于队列创建 ---- */

    /* ---- RTOS 对象 ---- */
    printf("[RTOS] kernel init...\r\n");
    if (osKernelInitialize() != osOK) Error_Handler();
    printf("[RTOS] kernel ok\r\n");

    const osMessageQueueAttr_t lidar = { .name = "Q_Lidar" };
    const osMessageQueueAttr_t commands = { .name = "Q_Cmd" };
    const osMessageQueueAttr_t paths = { .name = "Q_Path" };
    const osSemaphoreAttr_t imu_trigger = { .name = "TIM2_IMU" };
    const osMutexAttr_t i2c = { .name = "I2C1", .attr_bits = osMutexPrioInherit };
    Q_Lidar = osMessageQueueNew(4u, sizeof(LidarChunk_t), &lidar);
    Q_Cmd = osMessageQueueNew(8u, sizeof(Cmd_t), &commands);
    Q_Path = osMessageQueueNew(2u, sizeof(PathMessage), &paths);
    Sem_IMU = osSemaphoreNew(1u, 0u, &imu_trigger);
    Mtx_I2C = osMutexNew(&i2c);
    printf("[RTOS] objects q=%p/%p/%p sem=%p mtx=%p\r\n",
           (void *)Q_Lidar, (void *)Q_Cmd, (void *)Q_Path,
           (void *)Sem_IMU, (void *)Mtx_I2C);

    if (!Q_Lidar || !Q_Cmd || !Q_Path || !Sem_IMU || !Mtx_I2C) {
        printf("[RTOS] object create FAILED\r\n");
        Error_Handler();
    }

    hTaskIMU   = osThreadNew(Task_IMU,   NULL, &attrIMU);
    hTaskLidar = osThreadNew(Task_Lidar, NULL, &attrLidar);
    hTaskCtrl  = osThreadNew(Task_Ctrl,  NULL, &attrCtrl);
    hTaskUI    = osThreadNew(Task_UI,    NULL, &attrUI);
    hTaskRx    = osThreadNew(Task_Rx,    NULL, &attrRx);
    hTaskPlan  = osThreadNew(Task_Plan,  NULL, &attrPlan);
    hTaskTx    = osThreadNew(Task_Tx,    NULL, &attrTx);
    printf("[RTOS] tasks %p/%p/%p/%p\r\n",
           (void *)hTaskIMU, (void *)hTaskLidar,
           (void *)hTaskCtrl, (void *)hTaskUI);
    if (!hTaskIMU || !hTaskLidar || !hTaskCtrl || !hTaskUI || !hTaskRx || !hTaskPlan || !hTaskTx) {
        printf("[RTOS] thread create FAILED\r\n");
        Error_Handler();
    }

    /* ---- 雷达改为在 Task_Lidar 里启动（见任务实现），
            避免在调度器启动前操作 UART/DMA ---- */

    tim2_init_10ms();

    printf("[RTOS] start, tick = %lu Hz\r\n", (unsigned long)osKernelGetTickFreq());
    (void)osKernelStart();
    Error_Handler();
}

/* ============================================================ callbacks ==== */
static void lidar_queue_half(uint8_t half)
{
    if (lidar_receiving && Q_Lidar != NULL) {
        const LidarChunk_t chunk = {
            .sequence = ++lidar_cb,
            .tick = osKernelGetTickCount(),
            .half = half,
        };
        __DMB();
        if (osMessageQueuePut(Q_Lidar, &chunk, 0u, 0u) != osOK) {
            ++lidar_queue_drop;
        }
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        lidar_queue_half(0u);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        lidar_queue_half(1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        ++lidar_err;
        lidar_last_uart_error = huart->ErrorCode;
        lidar_receiving = 0u;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
        __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}

static void command_interrupt(USART_TypeDef *port)
{
    const uint32_t errors = USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE;
    const uint32_t status = port->SR;
    if ((status & (USART_SR_RXNE | errors)) != 0u) {
        const uint8_t byte = (uint8_t)port->DR;
        receive_byte(byte, (status & errors) != 0u);
    }
}

void UART4_IRQHandler(void) { command_interrupt(UART4); }
#if !PART2_ENABLE_MOTORS
void USART2_IRQHandler(void) { command_interrupt(USART2); }
#endif
void DMA1_Stream4_IRQHandler(void)   { HAL_DMA_IRQHandler(&hdma_uart4_tx); }
void I2C1_EV_IRQHandler(void)        { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void)        { HAL_I2C_ER_IRQHandler(&hi2c1); }

/* ================================================================ system ===*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 84;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 460800;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* 优先级必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY(默认 5)：
       DMA2_Stream2 的回调里会调 osMessageQueuePut()（FreeRTOS API），
       优先级 0 会让 configASSERT 直接死循环——之前"打完 wait 20ms 就死"就是这个。 */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);       /* USART1_RX */
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);              /* UART4 TC  */
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

/* printf over SWO: bounded wait, then drop the character */
int __io_putchar(int ch)
{
    uint32_t guard = 0u;
    while (ITM->PORT[0].u32 == 0u) {
        if (++guard > 3000u) return ch;
    }
    ITM->PORT[0].u8 = (uint8_t)ch;
    return ch;
}

void Error_Handler(void)
{
    __disable_irq();
    if (__HAL_RCC_TIM4_IS_CLK_ENABLED() && __HAL_RCC_TIM12_IS_CLK_ENABLED()) {
        motor_set(0, 0);
    }
    while (1) { }
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t handle, char *name)
{
    (void)handle;
    (void)name;
    Error_Handler();
}
