#ifndef __CPPLIB_SYSTEM_H
#define __CPPLIB_SYSTEM_H

#include <stdint.h>
#include "nrf.h"
#include "hal//cl_hal_peripheral.h"
#include "util//ringbuffer.h"
#include "boards//cl_board.h"

namespace CppLib { 
    
#define LOG_MESSAGE_BUFFER_COUNT    8
#define LOG_MESSAGE_BUFFER_LENGTH   64
    
#define REG_MASK_REG_INDEX(a) ((a) / 32)
#define REG_MASK_BIT_MASK(a)  (1 << ((a) % 32))

enum LogSeverity {LS_DEBUG, LS_INFO, LS_WARNING, LS_ERROR, LS_FATAL};
extern const char *cpplibLogSeverityStrings[];

typedef void (*InterruptCallback)(void);
typedef void (*InterruptCallbackIndexed)(uint32_t index);

typedef void (*LogMessageHandler)(LogSeverity severity, char *module, uint32_t errorCode, char *message);

// UARTE
static NRF_UARTE_Type  *cpplibResourceUarte[]     = CPPLIB_BOARD_UART_INTERFACE_LIST;
static uint32_t         cpplibResourceUarteIrqN[] = CPPLIB_BOARD_UART_INTERFACE_IRQn;

// SPIM
static NRF_SPIM_Type   *cpplibResourceSpim[]      = CPPLIB_BOARD_SPIM_INTERFACE_LIST;
static uint32_t         cpplibResourceSpimIrqN[]  = CPPLIB_BOARD_SPIM_INTERFACE_IRQn;

// SPIS
static NRF_SPIS_Type   *cpplibResourceSpis[]      = CPPLIB_BOARD_SPIS_INTERFACE_LIST;
static uint32_t         cpplibResourceSpisIrqN[]  = CPPLIB_BOARD_SPIS_INTERFACE_IRQn;

// SPIS
static NRF_TWIM_Type   *cpplibResourceTwim[]      = CPPLIB_BOARD_TWIM_INTERFACE_LIST;
static uint32_t         cpplibResourceTwimIrqN[]  = CPPLIB_BOARD_TWIM_INTERFACE_IRQn;

// PWM
static NRF_PWM_Type    *cpplibResourcePwm[]     = CPPLIB_BOARD_PWM_INTERFACE_LIST;
static uint32_t         cpplibResourcePwmIrqN[] = CPPLIB_BOARD_PWM_INTERFACE_IRQn;

typedef struct
{
    LogSeverity     severity;
    char            *module;
    uint32_t        errorCode;
    char            message[LOG_MESSAGE_BUFFER_LENGTH];
}logMessageT;

class System
{
private:
    uint32_t mResourceMapPPIChannel;
    uint32_t mResourceMap[(CPPLIB_BOARD_PERIPHERAL_NUM + 31) / 32];
    uint32_t mResourceMapSdkConfig[(CPPLIB_BOARD_PERIPHERAL_NUM + 31) / 32];

    uint32_t mGpioMap[(CPPLIB_BOARD_GPIO_NUM + 31) / 32];

    LogMessageHandler mLogMessageHandler;
    LogSeverity mDefaultSeverity;

    logMessageT mNewLogMessage;
    RingBuffer<logMessageT, LOG_MESSAGE_BUFFER_COUNT> mLogMessageList;    

    bool resourceMapCheckAndSet(uint32_t perNum);
    void resourceMapClear(uint32_t perNum);
    
public:
    System();

    static char sprintBuf[128];
    void setLogHandler(LogMessageHandler handler) { mLogMessageHandler = handler; }
    void setLogDefaultSeverity(LogSeverity severity) { mDefaultSeverity = severity; }

    void registerError(LogSeverity severity, char *module, uint32_t errorCode, char *message);
    void pollLogMessages();
    // Resource registering methods

    void registerGpio(uint32_t gpioNum);
    void freeGpio(uint32_t gpioNum);
    
    static void sdkLogMessageHandler(LogSeverity severity, char *module, uint32_t errorCode, char *message);
    
    uint32_t allocPPIChannel(void);
    void deallocPPIChannel(uint32_t ppiChannelNum);

    NRF_UARTE_Type *allocUartE(uint32_t *uartIndex);
    void deallocUartE(volatile NRF_UARTE_Type *base);
    static void registerUartIrq(InterruptCallbackIndexed callback) { mIrqCallbackUart = callback; }
    static InterruptCallbackIndexed mIrqCallbackUart;
    
    // SPI Master management functions
    NRF_SPIM_Type *allocSpim(uint32_t *perIndex);
    void deallocSpim(volatile NRF_SPIM_Type *base);
    static void registerSpimIrq(InterruptCallbackIndexed callback) { mIrqCallbackSpim = callback; }
    static InterruptCallbackIndexed mIrqCallbackSpim;
    
    // SPI Slave management functions
    NRF_SPIS_Type *allocSpis(uint32_t *perIndex);
    void deallocSpis(volatile NRF_SPIS_Type *base);
    static void registerSpisIrq(InterruptCallbackIndexed callback) { mIrqCallbackSpis = callback; }
    static InterruptCallbackIndexed mIrqCallbackSpis;   

    // TWI Master management functions
    NRF_TWIM_Type *allocTwim(uint32_t *perIndex);
    void deallocTwim(volatile NRF_TWIM_Type *base);
    static void registerTwimIrq(InterruptCallbackIndexed callback) { mIrqCallbackTwim = callback; }
    static InterruptCallbackIndexed mIrqCallbackTwim;   
    
    // PWM management functions
    NRF_PWM_Type *allocPwm(uint32_t *perIndex);
    void deallocPwm(volatile NRF_PWM_Type *base);
    
    // GPIOTE interrupt functions
    static InterruptCallback mIrqCallbackGpiote;
};

extern System nrfSystem;

} // End of namespace CppLib

#endif
