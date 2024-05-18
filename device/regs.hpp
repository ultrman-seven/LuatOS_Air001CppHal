#ifndef F07EFCA5_D4DF_40C3_9932_641DA1CE7F15
#define F07EFCA5_D4DF_40C3_9932_641DA1CE7F15

extern "C"
{
    typedef enum
    {
        /******  Cortex-M0+ Processor Exceptions Numbers ***************************************************************/
        NonMaskableInt_IRQn = -14, /*!< 2 Non Maskable Interrupt                                          */
        HardFault_IRQn = -13,      /*!< 3 Cortex-M Hard Fault Interrupt                                   */
        SVC_IRQn = -5,             /*!< 11 Cortex-M SV Call Interrupt                                     */
        PendSV_IRQn = -2,          /*!< 14 Cortex-M Pend SV Interrupt                                     */
        SysTick_IRQn = -1,         /*!< 15 Cortex-M System Tick Interrupt                                 */
        /******  AIR001 specific Interrupt Numbers *********************************************************************/
        WWDG_IRQn = 0,                 /*!< Window WatchDog Interrupt                                         */
        PVD_IRQn = 1,                  /*!< PVD through EXTI Line detection Interrupt(EXTI line 16)           */
        RTC_IRQn = 2,                  /*!< RTC interrupt through the EXTI line 19                            */
        FLASH_IRQn = 3,                /*!< FLASH global Interrupt                                            */
        RCC_IRQn = 4,                  /*!< RCC global Interrupt                                              */
        EXTI0_1_IRQn = 5,              /*!< EXTI 0 and 1 Interrupts                                           */
        EXTI2_3_IRQn = 6,              /*!< EXTI Line 2 and 3 Interrupts                                      */
        EXTI4_15_IRQn = 7,             /*!< EXTI Line 4 to 15 Interrupts                                      */
        DMA1_Channel1_IRQn = 9,        /*!< DMA1 Channel 1 Interrupt                                          */
        DMA1_Channel2_3_IRQn = 10,     /*!< DMA1 Channel 2 and Channel 3 Interrupts                           */
        ADC_COMP_IRQn = 12,            /*!< ADC&COMP Interrupts                                               */
        TIM1_BRK_UP_TRG_COM_IRQn = 13, /*!< TIM1 Break, Update, Trigger and Commutation Interrupts            */
        TIM1_CC_IRQn = 14,             /*!< TIM1 Capture Compare Interrupt                                    */
        TIM3_IRQn = 16,                /*!< TIM3 global Interrupt                                             */
        LPTIM1_IRQn = 17,              /*!< LPTIM1 global Interrupts                                          */
        TIM14_IRQn = 19,               /*!< TIM14 global Interrupt                                            */
        TIM16_IRQn = 21,               /*!< TIM16 global Interrupt                                            */
        TIM17_IRQn = 22,               /*!< TIM17 global Interrupt                                            */
        I2C1_IRQn = 23,                /*!< I2C1 Interrupt  (combined with EXTI 23)                           */
        SPI1_IRQn = 25,                /*!< SPI1 Interrupt                                                    */
        SPI2_IRQn = 26,                /*!< SPI2 Interrupt                                                    */
        USART1_IRQn = 27,              /*!< USART1 Interrupt                                                  */
        USART2_IRQn = 28,              /*!< USART2 Interrupt                                                  */
        LED_IRQn = 30,                 /*!< LED global Interrupt                                              */
    } IRQn_Type;
}

#define __CM0PLUS_REV 0          /*!< Core Revision r0p0                            */
#define __MPU_PRESENT 0          /*!< AIR001_Dev do not provide MPU                  */
#define __VTOR_PRESENT 1         /*!< Vector  Table  Register supported             */
#define __NVIC_PRIO_BITS 2       /*!< AIR001_Dev uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used  */

#include "stdint.h"
#include "core_cm0plus.h"

namespace ADC
{
    typedef struct
    {
        __IO uint32_t ISR;       /*!< ADC interrupt and status register,             Address offset: 0x00 */
        __IO uint32_t IER;       /*!< ADC interrupt enable register,                 Address offset: 0x04 */
        __IO uint32_t CR;        /*!< ADC control register,                          Address offset: 0x08 */
        __IO uint32_t CFGR1;     /*!< ADC configuration register 1,                  Address offset: 0x0C */
        __IO uint32_t CFGR2;     /*!< ADC configuration register 2,                  Address offset: 0x10 */
        __IO uint32_t SMPR;      /*!< ADC sampling time register,                    Address offset: 0x14 */
        uint32_t __RESERVED1[2]; /*!< Reserved,                                                      0x18-0x1C */
        __IO uint32_t TR;        /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
        uint32_t __RESERVED2;    /*!< Reserved,                                                      0x24 */
        __IO uint32_t CHSELR;    /*!< ADC group regular sequencer register,          Address offset: 0x28 */
        uint32_t __RESERVED3[5]; /*!< Reserved,                                                      0x2C */
        __IO uint32_t DR;        /*!< ADC group regular data register,               Address offset: 0x40 */
        __IO uint32_t CCSR;      /*!< ADC calibration configuration&status register  Address offset: 0x44 */
        uint32_t __RESERVED4[177];
        __IO uint32_t CCR; /*!< ADC common configuration register,             Address offset: 0x308 */
    } ADC_TypeDef;
}

namespace CRC
{
    typedef struct
    {
        __IO uint32_t DR;  /*!< CRC Data register,                         Address offset: 0x00 */
        __IO uint32_t IDR; /*!< CRC Independent data register,             Address offset: 0x04 */
        __IO uint32_t CR;  /*!< CRC Control register,                      Address offset: 0x08 */
    } CRC_TypeDef;
}

namespace COMP
{
    typedef struct
    {
        __IO uint32_t CSR; /*!< COMP control and status register,           Address offset: 0x00 */
        __IO uint32_t FR;  /*!< COMP filter register,                       Address offset: 0x04 */
    } COMP_TypeDef;
}

// typedef struct
// {
//     __IO uint32_t IDCODE; /*!< MCU device ID code,              Address offset: 0x00 */
//     __IO uint32_t CR;     /*!< Debug configuration register,    Address offset: 0x04 */
//     __IO uint32_t APBFZ1; /*!< Debug APB freeze register 1,     Address offset: 0x08 */
//     __IO uint32_t APBFZ2; /*!< Debug APB freeze register 2,     Address offset: 0x0C */
// } DBGMCU_TypeDef;

namespace DMA
{
    typedef struct
    {
        __IO uint32_t ISR;  /*!< DMA interrupt status register,                 Address offset: 0x00 */
        __IO uint32_t IFCR; /*!< DMA interrupt flag clear register,             Address offset: 0x04 */
    } DMA_TypeDef;

    typedef struct
    {
        __IO uint32_t CCR;   /*!< DMA channel x configuration register        */
        __IO uint32_t CNDTR; /*!< DMA channel x number of data register       */
        __IO uint32_t CPAR;  /*!< DMA channel x peripheral address register   */
        __IO uint32_t CMAR;  /*!< DMA channel x memory address register       */
    } DMA_Channel_TypeDef;
}

namespace FLASH
{
    typedef struct
    {
        __IO uint32_t ACR;       /*!< FLASH Access Control register,                     Address offset: 0x00 */
        uint32_t __RESERVED1;    /*!< Reserved1,                                         Address offset: 0x04 */
        __IO uint32_t KEYR;      /*!< FLASH Key register,                                Address offset: 0x08 */
        __IO uint32_t OPTKEYR;   /*!< FLASH Option Key register,                         Address offset: 0x0C */
        __IO uint32_t SR;        /*!< FLASH Status register,                             Address offset: 0x10 */
        __IO uint32_t CR;        /*!< FLASH Control register,                            Address offset: 0x14 */
        uint32_t __RESERVED2[2]; /*!< Reserved2,                                         Address offset: 0x18-0x1C */
        __IO uint32_t OPTR;      /*!< FLASH Option register,                             Address offset: 0x20 */
        __IO uint32_t SDKR;      /*!< FLASH SDK address register,                        Address offset: 0x24 */
        uint32_t __RESERVED3;    /*!< Reserved2,                                         Address offset: 0x28 */
        __IO uint32_t WRPR;      /*!< FLASH WRP address register,                        Address offset: 0x2C */
        uint32_t __RESERVED4[(0x90 - 0x2C) / 4 - 1];
        __IO uint32_t STCR; /*!< FLASH sleep time config register,                  Address offset: 0x90 */
        uint32_t __RESERVED5[(0x100 - 0x90) / 4 - 1];
        __IO uint32_t TS0;     /*!< FLASH TS0 register,                                Address offset: 0x100 */
        __IO uint32_t TS1;     /*!< FLASH TS1 register,                                Address offset: 0x104 */
        __IO uint32_t TS2P;    /*!< FLASH TS2P register,                               Address offset: 0x108 */
        __IO uint32_t TPS3;    /*!< FLASH TPS3 register,                               Address offset: 0x10C */
        __IO uint32_t TS3;     /*!< FLASH TS3 register,                                Address offset: 0x110 */
        __IO uint32_t PERTPE;  /*!< FLASH PERTPE register,                             Address offset: 0x114 */
        __IO uint32_t SMERTPE; /*!< FLASH SMERTPE register,                            Address offset: 0x118 */
        __IO uint32_t PRGTPE;  /*!< FLASH PRGTPE register,                             Address offset: 0x11C */
        __IO uint32_t PRETPE;  /*!< FLASH PRETPE register,                             Address offset: 0x120 */
    } FLASH_TypeDef;

    typedef struct
    {
        __IO uint8_t RDP;       /*!< FLASH option byte Read protection,             Address offset: 0x00 */
        __IO uint8_t USER;      /*!< FLASH option byte user options,                Address offset: 0x01 */
        __IO uint8_t nRDP;      /*!< Complemented FLASH option byte Read protection,Address offset: 0x02 */
        __IO uint8_t nUSER;     /*!< Complemented FLASH option byte user options,   Address offset: 0x03 */
        __IO uint8_t SDK_STRT;  /*!< SDK area start address(stored in SDK[4:0]),    Address offset: 0x04 */
        __IO uint8_t SDK_END;   /*!< SDK area end address(stored in SDK[12:8]),     Address offset: 0x05 */
        __IO uint8_t nSDK_STRT; /*!< Complemented SDK area start address,           Address offset: 0x06 */
        __IO uint8_t nSDK_END;  /*!< Complemented SDK area end address,             Address offset: 0x07 */
        uint32_t __RESERVED1;   /*!< RESERVED1,                                     Address offset: 0x08 */
        __IO uint16_t WRP;      /*!< FLASH option byte write protection,            Address offset: 0x0C */
        __IO uint16_t nWRP;     /*!< Complemented FLASH option byte write protection,Address offset: 0x0E */
    } OB_TypeDef;
}

namespace GPIO
{
    typedef struct
    {
        __IO uint32_t RTSR;       /*!< EXTI Rising Trigger Selection Register 1,        Address offset:   0x00 */
        __IO uint32_t FTSR;       /*!< EXTI Falling Trigger Selection Register 1,       Address offset:   0x04 */
        __IO uint32_t SWIER;      /*!< EXTI Software Interrupt event Register 1,        Address offset:   0x08 */
        __IO uint32_t PR;         /*!< EXTI Pending Register 1                          Address offset:   0x0C */
        uint32_t __RESERVED1[4];  /*!< Reserved 1,                                                0x10 -- 0x1C */
        uint32_t __RESERVED2[5];  /*!< Reserved 2,                                                0x20 -- 0x30 */
        uint32_t __RESERVED3[11]; /*!< Reserved 3,                                                0x34 -- 0x5C */
        __IO uint32_t EXTICR[3];  /*!< EXTI External Interrupt Configuration Register,            0x60 -- 0x68 */
        uint32_t __RESERVED4[5];  /*!< Reserved 5,                                                0x6C -- 0x7C */
        __IO uint32_t IMR;        /*!< EXTI Interrupt Mask Register ,                   Address offset:   0x80 */
        __IO uint32_t EMR;        /*!< EXTI Event Mask Register ,                       Address offset:   0x84 */
    } EXTI_TypeDef;

    typedef struct
    {
        __IO uint32_t MODER;   /*!< GPIO port mode register,               Address offset: 0x00      */
        __IO uint32_t OTYPER;  /*!< GPIO port output type register,        Address offset: 0x04      */
        __IO uint32_t OSPEEDR; /*!< GPIO port output speed register,       Address offset: 0x08      */
        __IO uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
        __IO uint32_t IDR;     /*!< GPIO port input data register,         Address offset: 0x10      */
        __IO uint32_t ODR;     /*!< GPIO port output data register,        Address offset: 0x14      */
        __IO uint32_t BSRR;    /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
        __IO uint32_t LCKR;    /*!< GPIO port configuration lock register, Address offset: 0x1C      */
        __IO uint32_t AFR[2];  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
        __IO uint32_t BRR;     /*!< GPIO Bit Reset register,               Address offset: 0x28      */
    } GPIO_TypeDef;
}

namespace IIC
{
    typedef struct
    {
        __IO uint32_t CR1;
        __IO uint32_t CR2;
        __IO uint32_t OAR1;
        __IO uint32_t OAR2;
        __IO uint32_t DR;
        __IO uint32_t SR1;
        __IO uint32_t SR2;
        __IO uint32_t CCR;
        __IO uint32_t TRISE;
    } I2C_TypeDef;
}

typedef struct
{
    __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
    __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
    __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
    __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
    __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
    __IO uint32_t __RESERVED1; /*!< RESERVED1,                                          Address offset: 0x14 */
    __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
    __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
} LPTIM_TypeDef;

namespace sys
{
    typedef struct
    {
        __IO uint32_t CR1;       /*!< PWR Power Control Register 1,                     Address offset: 0x00 */
        __IO uint32_t CR2;       /*!< PWR Power Control Register 2,                     Address offset: 0x04 */
        uint32_t __RESERVED1[3]; /*!< Reserved1,                                        Address offset: 0x08-0x10 */
        __IO uint32_t SR;        /*!< PWR Power Status Register,                        Address offset: 0x14 */
    } PWR_TypeDef;

    typedef struct
    {
        __IO uint32_t CR;          /*!< RCC Clock Sources Control Register,                                     Address offset: 0x00 */
        __IO uint32_t ICSCR;       /*!< RCC Internal Clock Sources Calibration Register,                        Address offset: 0x04 */
        __IO uint32_t CFGR;        /*!< RCC Regulated Domain Clocks Configuration Register,                     Address offset: 0x08 */
        __IO uint32_t PLLCFGR;     /*!< RCC System PLL configuration Register,                                  Address offset: 0x0C */
        __IO uint32_t ECSCR;       /*!< RCC External clock source control register,                             Address offset: 0x10 */
        __IO uint32_t __RESERVED1; /*!< Reserved,                                                               Address offset: 0x14 */
        __IO uint32_t CIER;        /*!< RCC Clock Interrupt Enable Register,                                    Address offset: 0x18 */
        __IO uint32_t CIFR;        /*!< RCC Clock Interrupt Flag Register,                                      Address offset: 0x1C */
        __IO uint32_t CICR;        /*!< RCC Clock Interrupt Clear Register,                                     Address offset: 0x20 */
        __IO uint32_t IOPRSTR;     /*!< RCC IO port reset register,                                             Address offset: 0x24 */
        __IO uint32_t AHBRSTR;     /*!< RCC AHB peripherals reset register,                                     Address offset: 0x28 */
        __IO uint32_t APBRSTR1;    /*!< RCC APB peripherals reset register 1,                                   Address offset: 0x2C */
        __IO uint32_t APBRSTR2;    /*!< RCC APB peripherals reset register 2,                                   Address offset: 0x30 */
        __IO uint32_t IOPENR;      /*!< RCC IO port enable register,                                            Address offset: 0x34 */
        __IO uint32_t AHBENR;      /*!< RCC AHB peripherals clock enable register,                              Address offset: 0x38 */
        __IO uint32_t APBENR1;     /*!< RCC APB peripherals clock enable register1,                             Address offset: 0x3C */
        __IO uint32_t APBENR2;     /*!< RCC APB peripherals clock enable register2,                             Address offset: 0x40 */
        uint32_t __RESERVED2[4];   /*!< Reserved,                                                               Address offset: 0x44-0x50 */
        __IO uint32_t CCIPR;       /*!< RCC Peripherals Independent Clocks Configuration Register,              Address offset: 0x54 */
        __IO uint32_t __RESERVED3; /*!< Reserved,                                                               Address offset: 0x58 */
        __IO uint32_t BDCR;        /*!< RCC Backup Domain Control Register,                                     Address offset: 0x5C */
        __IO uint32_t CSR;         /*!< RCC Unregulated Domain Clock Control and Status Register,               Address offset: 0x60 */
    } RCC_TypeDef;

    typedef struct
    {
        __IO uint32_t CFGR1;     /*!< SYSCFG configuration register 1,                   Address offset: 0x00 */
        uint32_t __RESERVED1[5]; /*!< Reserved,                                                   0x04 --0x14 */
        __IO uint32_t CFGR2;     /*!< SYSCFG configuration register 2,                   Address offset: 0x18 */
        __IO uint32_t CFGR3;     /*!< SYSCFG configuration register 3,                   Address offset: 0x1C */
    } SYSCFG_TypeDef;
}

namespace RTC
{
    typedef struct
    {
        __IO uint32_t CRH;
        __IO uint32_t CRL;
        __IO uint32_t PRLH;
        __IO uint32_t PRLL;
        __IO uint32_t DIVH;
        __IO uint32_t DIVL;
        __IO uint32_t CNTH;
        __IO uint32_t CNTL;
        __IO uint32_t ALRH;
        __IO uint32_t ALRL;
        uint32_t __RESERVED1;
        __IO uint32_t BKP_RTCCR;
    } RTC_TypeDef;
}

namespace SPI
{
    typedef struct
    {
        __IO uint32_t CR1; /*!< SPI Control register 1,                              Address offset: 0x00 */
        __IO uint32_t CR2; /*!< SPI Control register 2,                              Address offset: 0x04 */
        __IO uint32_t SR;  /*!< SPI Status register,                                 Address offset: 0x08 */
        __IO uint32_t DR;  /*!< SPI data register,                                   Address offset: 0x0C */
    } SPI_TypeDef;
}

namespace TIM
{
    typedef struct
    {
        __IO uint32_t CR1;    /*!< TIM control register 1,                   Address offset: 0x00 */
        __IO uint32_t CR2;    /*!< TIM control register 2,                   Address offset: 0x04 */
        __IO uint32_t SMCR;   /*!< TIM slave mode control register,          Address offset: 0x08 */
        __IO uint32_t DIER;   /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
        __IO uint32_t SR;     /*!< TIM status register,                      Address offset: 0x10 */
        __IO uint32_t EGR;    /*!< TIM event generation register,            Address offset: 0x14 */
        __IO uint32_t CCMR1;  /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
        __IO uint32_t CCMR2;  /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
        __IO uint32_t CCER;   /*!< TIM capture/compare enable register,      Address offset: 0x20 */
        __IO uint32_t CNT;    /*!< TIM counter register,                     Address offset: 0x24 */
        __IO uint32_t PSC;    /*!< TIM prescaler register,                   Address offset: 0x28 */
        __IO uint32_t ARR;    /*!< TIM auto-reload register,                 Address offset: 0x2C */
        __IO uint32_t RCR;    /*!< TIM repetition counter register,          Address offset: 0x30 */
        __IO uint32_t CCR[4]; /*!< TIM capture/compare register 1,           Address offset: 0x34 */
        __IO uint32_t BDTR;   /*!< TIM break and dead-time register,         Address offset: 0x44 */
        __IO uint32_t DCR;    /*!< TIM DMA control register,                 Address offset: 0x48 */
        __IO uint32_t DMAR;   /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
        __IO uint32_t OR;     /*!< TIM option register,                      Address offset: 0x50 */
    } TIM_TypeDef;
}

namespace USART
{
    typedef struct
    {
        __IO uint32_t SR;   /*!< USART     Status  register ,              Address offset: 0x00  */
        __IO uint32_t DR;   /*!< USART Data register,                      Address offset: 0x04  */
        __IO uint32_t BRR;  /*!< USART Baud rate register,                 Address offset: 0x08  */
        __IO uint32_t CR1;  /*!< USART     Control  register 1,            Address offset: 0x0C  */
        __IO uint32_t CR2;  /*!< USART     Control  register 2,            Address offset: 0x10  */
        __IO uint32_t CR3;  /*!< USART     Control  register 3,            Address offset: 0x14  */
        __IO uint32_t GTPR; /*!< USART Guard time and prescaler register,  Address offset: 0x18  */
    } USART_TypeDef;
}

namespace WDG
{
    typedef struct
    {
        __IO uint32_t CR;  /*!< WWDG Control register,       Address offset: 0x00 */
        __IO uint32_t CFR; /*!< WWDG Configuration register, Address offset: 0x04 */
        __IO uint32_t SR;  /*!< WWDG Status register,        Address offset: 0x08 */
    } WWDG_TypeDef;

    typedef struct
    {
        __IO uint32_t KR;  /*!< IWDG Key register,       Address offset: 0x00 */
        __IO uint32_t PR;  /*!< IWDG Prescaler register, Address offset: 0x04 */
        __IO uint32_t RLR; /*!< IWDG Reload register,    Address offset: 0x08 */
        __IO uint32_t SR;  /*!< IWDG Status register,    Address offset: 0x0C */
                           //__IO uint32_t WINR;        /*!< IWDG Window register,    Address offset: 0x10 */
    } IWDG_TypeDef;
}

// typedef struct
// {
//     __IO uint32_t CR;  /*!< LED Control register,        Address offset: 0x00 */
//     __IO uint32_t PR;  /*!< LED Prescaler register,      Address offset: 0x04 */
//     __IO uint32_t TR;  /*!< Time register,               Address offset: 0x08 */
//     __IO uint32_t DR0; /*!< Data0 register,              Address offset: 0x0C */
//     __IO uint32_t DR1; /*!< Data1 register,              Address offset: 0x10 */
//     __IO uint32_t DR2; /*!< Data2 register,              Address offset: 0x14 */
//     __IO uint32_t DR3; /*!< Data3 register,              Address offset: 0x18 */
//     __IO uint32_t IR;  /*!< Interrupt register,          Address offset: 0x1C */
// } LED_TypeDef;

#define FLASH_BASE (0x08000000UL) /*!< FLASH base address */
#define FLASH_END (0x08007FFFUL)  /*!< FLASH end address */
#define FLASH_SIZE (FLASH_END - FLASH_BASE + 1)
#define FLASH_PAGE_SIZE 0x00000080U /*!< FLASH Page Size, 128 Bytes */
#define FLASH_PAGE_NB (FLASH_SIZE / FLASH_PAGE_SIZE)
#define FLASH_SECTOR_SIZE 0x00001000U /*!< FLASH Sector Size, 4096 Bytes */
#define FLASH_SECTOR_NB (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define SRAM_BASE (0x20000000UL)   /*!< SRAM base address */
#define SRAM_END (0x20000FFFUL)    /*!< SRAM end address */
#define PERIPH_BASE (0x40000000UL) /*!< Peripheral base address */
#define IOPORT_BASE (0x50000000UL) /*!< IOPORT base address */

/*!< Peripheral memory map */
#define APBPERIPH_BASE (PERIPH_BASE)
#define AHBPERIPH_BASE (PERIPH_BASE + 0x00020000UL)

/*!< APB peripherals */
#define TIM3_BASE (APBPERIPH_BASE + 0x00000400UL)
#define TIM14_BASE (APBPERIPH_BASE + 0x00002000UL)
#define LED_BASE (APBPERIPH_BASE + 0x00002400UL)
#define RTC_BASE (APBPERIPH_BASE + 0x00002800UL)
#define WWDG_BASE (APBPERIPH_BASE + 0x00002C00UL)
#define IWDG_BASE (APBPERIPH_BASE + 0x00003000UL)
#define SPI2_BASE (APBPERIPH_BASE + 0x00003800UL)
#define USART2_BASE (APBPERIPH_BASE + 0x00004400UL)
#define I2C_BASE (APBPERIPH_BASE + 0x00005400UL)
#define PWR_BASE (APBPERIPH_BASE + 0x00007000UL)
#define LPTIM_BASE (APBPERIPH_BASE + 0x00007C00UL)
#define SYSCFG_BASE (APBPERIPH_BASE + 0x00010000UL)
#define COMP1_BASE (APBPERIPH_BASE + 0x00010200UL)
#define COMP2_BASE (APBPERIPH_BASE + 0x00010210UL)
#define ADC1_BASE (APBPERIPH_BASE + 0x00012400UL)
#define ADC_BASE (APBPERIPH_BASE + 0x00012708UL)
#define TIM1_BASE (APBPERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE (APBPERIPH_BASE + 0x00013000UL)
#define USART1_BASE (APBPERIPH_BASE + 0x00013800UL)
#define TIM16_BASE (APBPERIPH_BASE + 0x00014400UL)
#define TIM17_BASE (APBPERIPH_BASE + 0x00014800UL)
#define DBGMCU_BASE (APBPERIPH_BASE + 0x00015800UL)

/*!< AHB peripherals */
#define DMA1_BASE (AHBPERIPH_BASE + 0x00000000UL)
#define DMA1_Channel1_BASE (DMA1_BASE + 0x00000008UL)
#define DMA1_Channel2_BASE (DMA1_BASE + 0x0000001CUL)
#define DMA1_Channel3_BASE (DMA1_BASE + 0x00000030UL)
#define RCC_BASE (AHBPERIPH_BASE + 0x00001000UL)
#define EXTI_BASE (AHBPERIPH_BASE + 0x00001800UL)
#define FLASH_R_BASE (AHBPERIPH_BASE + 0x00002000UL) /*!< FLASH registers base address */
#define OB_BASE 0x1FFF0E80UL                         /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE 0x1FFF0FFCUL                  /*!< FLASH Size register base address */
#define UID_BASE 0x1FFF0E00UL                        /*!< Unique device ID register base address */
#define CRC_BASE (AHBPERIPH_BASE + 0x00003000UL)

/*!< IOPORT */
#define GPIOA_BASE (IOPORT_BASE + 0x00000000UL)
#define GPIOB_BASE (IOPORT_BASE + 0x00000400UL)
#define GPIOF_BASE (IOPORT_BASE + 0x00001400UL)

// #define LED ((LED_TypeDef *)LED_BASE)
#define RTC ((RTC_TypeDef *)RTC_BASE)
#define WWDG ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG ((IWDG_TypeDef *)IWDG_BASE)
#define I2C ((I2C_TypeDef *)I2C_BASE) /* Kept for legacy purpose */
#define PWR ((PWR_TypeDef *)PWR_BASE)
#define LPTIM ((LPTIM_TypeDef *)LPTIM_BASE) /* Kept for legacy purpose */
#define SYSCFG ((SYSCFG_TypeDef *)SYSCFG_BASE)
#define ADC ((ADC_Common_TypeDef *)ADC_BASE) /* Kept for legacy purpose */
#define DBGMCU ((DBGMCU_TypeDef *)DBGMCU_BASE)
#define DMA1 ((DMA_TypeDef *)DMA1_BASE)
#define RCC ((RCC_TypeDef *)RCC_BASE)
#define EXTI ((EXTI_TypeDef *)EXTI_BASE)
#define FLASH ((FLASH_TypeDef *)FLASH_R_BASE)
#define OB ((OB_TypeDef *)OB_BASE)
#define CRC ((CRC_TypeDef *)CRC_BASE)

#endif /* F07EFCA5_D4DF_40C3_9932_641DA1CE7F15 */
