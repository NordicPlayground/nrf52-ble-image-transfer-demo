#include "system//cl_system.h"
#include "boards//cl_board.h"
#include <string.h>
#include <stdio.h>
#include "sdk_config.h"

#if defined(NRF_LOG_ENABLED) && (NRF_LOG_ENABLED == 1)
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#endif

namespace CppLib {

System nrfSystem;    

const char *cpplibLogSeverityStrings[] = {"Debug", "Info", "Warning", "Error", "Fatal"};
char System::sprintBuf[128] = {0};

InterruptCallbackIndexed System::mIrqCallbackUart = 0;
InterruptCallbackIndexed System::mIrqCallbackSpim = 0;
InterruptCallbackIndexed System::mIrqCallbackSpis = 0;
InterruptCallbackIndexed System::mIrqCallbackTwim = 0;
InterruptCallback        System::mIrqCallbackGpiote = 0;

System::System()
{
    mDefaultSeverity = LS_INFO;
    mLogMessageHandler = 0;
    mResourceMapPPIChannel = 0;
    for(int i = 0; i < ((CPPLIB_BOARD_PERIPHERAL_NUM + 31) / 32); i++)
    {
        mResourceMap[i] = 0;
        mResourceMapSdkConfig[i] = 0;
    }
#if defined(NRF_LOG_ENABLED) && (NRF_LOG_ENABLED == 1)
   mLogMessageHandler = sdkLogMessageHandler;
#endif
#if (defined(SPI0_ENABLED) && (SPI0_ENABLED == 1)) || (defined(TWI0_ENABLED) && (TWI0_ENABLED == 1))
    mResourceMapSdkConfig[SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn / 32] |= (1 << (SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn % 32));
#endif
#if (defined(SPI1_ENABLED) && (SPI1_ENABLED == 1)) || (defined(TWI1_ENABLED) && (TWI1_ENABLED == 1))
    mResourceMapSdkConfig[SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn / 32] |= (1 << (SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn % 32));
#endif
#if defined(SPI2_ENABLED) && (SPI2_ENABLED == 1)
    mResourceMapSdkConfig[SPIM2_SPIS2_SPI2_IRQn / 32] |= (1 << (SPIM2_SPIS2_SPI2_IRQn % 32));
#endif
#if defined(UART0_ENABLED) && (UART0_ENABLED == 1)
    mResourceMapSdkConfig[UARTE0_UART0_IRQn / 32] |= (1 << (UARTE0_UART0_IRQn % 32));
#endif
#if defined(UART1_ENABLED) && (UART1_ENABLED == 1)
    mResourceMapSdkConfig[UARTE1_IRQn / 32] |= (1 << (UARTE1_IRQn % 32));
#endif    
}

void System::registerError(LogSeverity severity, char *module, uint32_t errorCode, char *message)
{
    if(severity >= mDefaultSeverity)
    {
        mNewLogMessage.severity = severity;
        mNewLogMessage.module = module;
        mNewLogMessage.errorCode = errorCode;
        if(strlen(message) > LOG_MESSAGE_BUFFER_LENGTH) message[LOG_MESSAGE_BUFFER_LENGTH - 1] = 0;
        strcpy(mNewLogMessage.message, message);
        if(!mLogMessageList.isFull())
            mLogMessageList.put(&mNewLogMessage);
        pollLogMessages();
    }
}

void System::pollLogMessages()
{
    if(mLogMessageHandler)
    {
        while(!mLogMessageList.isEmpty())
        {
            logMessageT newMessage;
            mLogMessageList.get(&newMessage);
            mLogMessageHandler(newMessage.severity, newMessage.module, newMessage.errorCode, newMessage.message);
        }
    }
}

void System::registerGpio(uint32_t gpioNum)
{
    if(mGpioMap[gpioNum / 32] & (1 << (gpioNum % 32)))
    {
        char message[32];
        sprintf(message, "Conflicting pin P%i.%i", (gpioNum / 32), (gpioNum % 32));
        registerError(LS_WARNING, "System", 0, message);
    }
    else
    {
        mGpioMap[gpioNum / 32] |= (1 << (gpioNum % 32));
    }
}

void System::freeGpio(uint32_t gpioNum)
{
    mGpioMap[gpioNum / 32] &= ~(1 << (gpioNum % 32));
}
        
uint32_t System::allocPPIChannel(Event event, Task task)
{
    return 0;
}

void System::deallocPPIChannel(uint32_t ppiChannelNum)
{
    
}

bool System::resourceMapCheckAndSet(uint32_t perNum)
{
    if((mResourceMap[perNum / 32] & (1 << (perNum % 32))) == 0 &&
       (mResourceMapSdkConfig[perNum / 32] & (1 << (perNum % 32))) == 0)
    {
        mResourceMap[perNum / 32] |= (1 << (perNum % 32));
        return true;
    }
    else return false;
}

void System::resourceMapClear(uint32_t perNum)
{
    mResourceMap[perNum / 32] &= ~(1 << (perNum % 32));
}

NRF_UARTE_Type *System::allocUartE(uint32_t *perIndex)
{
    for(int i = 0; i < CPPLIB_BOARD_UART_INTERFACE_NUM; i++)
        if(resourceMapCheckAndSet(cpplibResourceUarteIrqN[i]))
        {
            *perIndex = i;
            return cpplibResourceUarte[i];
        }
    return 0;
}

void System::deallocUartE(volatile NRF_UARTE_Type *base)
{
    for(int i = 0; i < CPPLIB_BOARD_UART_INTERFACE_NUM; i++)  
        if(cpplibResourceUarte[i] == base)
        {
            resourceMapClear(cpplibResourceUarteIrqN[i]);
            return;
        }
}

NRF_SPIM_Type *System::allocSpim(uint32_t *perIndex)
{
    for(int i = 0; i < CPPLIB_BOARD_SPIM_INTERFACE_NUM; i++)
        if(resourceMapCheckAndSet(cpplibResourceSpimIrqN[i]))
        {
            *perIndex = i;
            return cpplibResourceSpim[i];
        }
    return 0;
}

void System::deallocSpim(volatile NRF_SPIM_Type *base)
{
    for(int i = 0; i < CPPLIB_BOARD_SPIM_INTERFACE_NUM; i++)  
        if(cpplibResourceSpim[i] == base)
        {
            resourceMapClear(cpplibResourceSpimIrqN[i]);
            return;
        }
}

NRF_SPIS_Type *System::allocSpis(uint32_t *perIndex)
{
    for(int i = 0; i < CPPLIB_BOARD_SPIS_INTERFACE_NUM; i++)
        if(resourceMapCheckAndSet(cpplibResourceSpisIrqN[i]))
        {
            *perIndex = i;
            return cpplibResourceSpis[i];
        }
    return 0;
}

void System::deallocSpis(volatile NRF_SPIS_Type *base)
{
    for(int i = 0; i < CPPLIB_BOARD_SPIS_INTERFACE_NUM; i++)  
        if(cpplibResourceSpis[i] == base)
        {
            resourceMapClear(cpplibResourceSpisIrqN[i]);
            return;
        }
}

NRF_TWIM_Type *System::allocTwim(uint32_t *perIndex)
{
    for(int i = 0; i < CPPLIB_BOARD_TWIM_INTERFACE_NUM; i++)
        if(resourceMapCheckAndSet(cpplibResourceTwimIrqN[i]))
        {
            *perIndex = i;
            return cpplibResourceTwim[i];
        }
    return 0;
}

void System::deallocTwim(volatile NRF_TWIM_Type *base)
{
    for(int i = 0; i < CPPLIB_BOARD_TWIM_INTERFACE_NUM; i++)  
        if(cpplibResourceTwim[i] == base)
        {
            resourceMapClear(cpplibResourceTwimIrqN[i]);
            return;
        }
}

NRF_PWM_Type *System::allocPwm(uint32_t *perIndex)
{
    for(int i = 0; i < CPPLIB_BOARD_PWM_INTERFACE_NUM; i++)
        if(resourceMapCheckAndSet(cpplibResourcePwmIrqN[i]))
        {
            *perIndex = i;
            return cpplibResourcePwm[i];
        }
    return 0;
}

void System::deallocPwm(volatile NRF_PWM_Type *base)
{
    for(int i = 0; i < CPPLIB_BOARD_PWM_INTERFACE_NUM; i++)  
        if(cpplibResourcePwm[i] == base)
        {
            resourceMapClear(cpplibResourcePwmIrqN[i]);
            return;
        }
}

#if defined(NRF_LOG_ENABLED) && (NRF_LOG_ENABLED == 1)
void System::sdkLogMessageHandler(LogSeverity severity, char *module, uint32_t errorCode, char *message)
{
    switch(severity)
    {
        case LS_DEBUG:
            NRF_LOG_DEBUG("%s - %s\r\n", (uint32_t)module, (uint32_t)message);
            break;
        case LS_INFO:
            NRF_LOG_INFO("%s - %s\r\n", (uint32_t)module, (uint32_t)message);
            break;
        case LS_WARNING:
            NRF_LOG_WARNING("%s - %s\r\n", (uint32_t)module, (uint32_t)message);
            break;
        case LS_ERROR:
        case LS_FATAL:
            NRF_LOG_ERROR("%s - %s\r\n", (uint32_t)module, (uint32_t)message);
            break;
        default:
            break;
    }
}
#endif

// Interrupt vectors

extern "C" {
#if !defined(UART0_ENABLED) || (UART0_ENABLED == 0)
void UARTE0_UART0_IRQHandler(void)
{
    if(System::mIrqCallbackUart)
    {
        System::mIrqCallbackUart(0);
    }
}    
#endif

#if !defined(UART1_ENABLED) || (UART1_ENABLED == 0)
void UARTE1_IRQHandler(void)
{
    if(System::mIrqCallbackUart)
    {
        System::mIrqCallbackUart(1);
    }
} 
#endif

#if (!defined(SPI0_ENABLED) || (SPI0_ENABLED == 0))  && (!defined(TWI0_ENABLED) || (TWI0_ENABLED == 0))                   
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{
    if(System::mIrqCallbackSpim)
    {
        System::mIrqCallbackSpim(1);
    }
    if(System::mIrqCallbackSpis)
    {
        System::mIrqCallbackSpis(1);
    }
    if(System::mIrqCallbackTwim)
    {
        System::mIrqCallbackTwim(0);
    }
}
#endif

#if (!defined(SPI1_ENABLED) || (SPI1_ENABLED == 0)) && (!defined(TWI1_ENABLED) || (TWI1_ENABLED == 0)) 
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{
    if(System::mIrqCallbackSpim)
    {
        System::mIrqCallbackSpim(2);
    }
    if(System::mIrqCallbackSpis)
    {
        System::mIrqCallbackSpis(2);
    }
    if(System::mIrqCallbackTwim)
    {
        System::mIrqCallbackTwim(1);
    }    
}
#endif

#if !defined(SPI2_ENABLED) || (SPI2_ENABLED == 0)
void SPIM2_SPIS2_SPI2_IRQHandler(void)
{
    if(System::mIrqCallbackSpim)
    {
        System::mIrqCallbackSpim(0);
    }    
    if(System::mIrqCallbackSpis)
    {
        System::mIrqCallbackSpis(0);
    }    
}
#endif

#if !defined(GPIOTE_ENABLED) || (GPIOTE_ENABLED == 0)
void GPIOTE_IRQHandler(void)
{
    if(System::mIrqCallbackGpiote)
    {
        System::mIrqCallbackGpiote();
    }
}
#endif

} // End of extern "C"

} // End of namespace CppLib
