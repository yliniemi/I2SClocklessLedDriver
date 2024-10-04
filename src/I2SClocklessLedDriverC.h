/* library options
 *  ENABLE_HARDWARE_SCROLL : to enable the HARDWARE SCROLL. Attention wjhen enabled you can use the offset  but it could mean slow when using all the pins
 *  NUMSTRIPS add this before the #include of the library this will help with the speed of the buffer calculation
 *  USE_PIXELSLIB : to use tthe pixel lib library automatic functions
 */

 
#pragma once


#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <rom/ets_sys.h>
//#include "esp32-hal-log.h"
#include "esp_log.h"
#include "math.h"



#ifndef NUMSTRIPS
#define NUMSTRIPS 16
#endif

#define I2S_DEVICE 0

#define AA (0x00AA00AAL)
#define CC (0x0000CCCCL)
#define FF (0xF0F0F0F0L)
#define FF2 (0x0F0F0F0FL)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#ifdef COLOR_ORDER_GRBW
#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 4
#else
#ifdef COLOR_ORDER_RGB
#define _p_r 0
#define _p_g 1
#define _p_b 2
#define _nb_components 3
#else
#ifdef  COLOR_ORDER_RBG
#define _p_r 0
#define _p_g 2
#define _p_b 1
#define _nb_components 3
#else
#ifdef COLOR_ORDER_GBR
#define _p_r 2
#define _p_g 0
#define _p_b 1
#define _nb_components 3
#else
#ifdef COLOR_ORDER_BGR
#define _p_r 2
#define _p_g 1
#define _p_b 0
#define _nb_components 3
#else
#ifdef COLOR_ORDER_BRG
#define _p_r 1
#define _p_g 2
#define _p_b 0
#define _nb_components 3
#else
#ifdef COLOR_ORDER_GRB
#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 3
#else

#define _p_r 1
#define _p_g 0
#define _p_b 2
#define _nb_components 3
#endif
#endif
#endif
#endif
#endif
#endif
#endif


typedef union
{
    uint8_t bytes[16];
    uint32_t shorts[8];
    uint32_t raw[2];
} Lines;

struct OffsetDisplay
{
    int offsetx;
    int offsety;
    int panel_height;
    int panel_width;
};

static const char *TAG = "I2SClocklessLedDriver";
static void _I2SClocklessLedDriverinterruptHandler(void *arg);
static void transpose16x1_noinline2(unsigned char *A, uint16_t *B);
static void loadAndTranspose(struct I2SClocklessLedDriver *driver);

enum colorarrangment
{
    ORDER_GRBW,
    ORDER_RGB,
    ORDER_RBG,
    ORDER_GRB,
    ORDER_GBR,
    ORDER_BRG,
    ORDER_BGR,
};

enum displayMode
{
    NO_WAIT,
    WAIT,
    LOOP,
    LOOP_INTERUPT,
};

int MOD(int a, int b)
{
    if (a < 0)
    {
        if (-a % b == 0)
            return 0;
        else
            return b - (-a) % b;
    }
    else
        return a % b;
}

struct LedTiming
{

    //led timing
    uint32_t T0;
    uint32_t T1;
    uint32_t T2;

    //compileled
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
};

    struct I2SClocklessLedDriverDMABuffer
    {
        lldesc_t descriptor;
        uint8_t *buffer;
    };

struct I2SClocklessLedDriver
{



    int deviceBaseIndex[2];
    int deviceClockIndex[2];
    int deviceWordSelectIndex[2];
    periph_module_t deviceModule[2];

    i2s_dev_t *i2s;
    intr_handle_t _gI2SClocklessDriver_intr_handle;
    volatile xSemaphoreHandle I2SClocklessLedDriver_sem;
    volatile xSemaphoreHandle I2SClocklessLedDriver_semSync;
    volatile xSemaphoreHandle I2SClocklessLedDriver_semDisp;
    volatile xSemaphoreHandle I2SClocklessLedDriver_waitDisp;
    volatile int dmaBufferActive;
    volatile bool wait;
    enum displayMode __displayMode;
    volatile int ledToDisplay;
    struct OffsetDisplay _offsetDisplay, _defaultOffsetDisplay;
    // volatile int oo=0;
    uint8_t *leds,*saveleds;
    int startleds;
    int linewidth;
    int dmaBufferCount; //we use two buffers
    volatile bool transpose;

    volatile int num_strips;
    volatile int num_led_per_strip;
    volatile uint16_t total_leds;
    //int clock_pin;
    int p_r, p_g, p_b;
    int i2s_base_pin_index;
    int nb_components;
    int stripSize[16];
    volatile bool isDisplaying;
    volatile bool isWaiting;
    volatile bool framesync;
    volatile bool wasWaitingtofinish;
    volatile int counti;
    
    struct I2SClocklessLedDriverDMABuffer **DMABuffersTransposed;
    struct I2SClocklessLedDriverDMABuffer *DMABuffersTampon[4];
    
    // I2SClocklessLedDriver(){};
    
    // OffsetDisplay getDefaultOffset();
    // Pixel * strip(int stripNum);
    
    // void initled(I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int *sizes,  int num_strips);
    // void initled(I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip);






};


    void showPixels(struct I2SClocklessLedDriver *driver);
    void initDMABuffers(struct I2SClocklessLedDriver *driver);
    void initledSizes(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int *sizes,  int num_strips,colorarrangment cArr);
    void initled(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip,colorarrangment cArr);
    int maxLength(int *sizes,int num_strips);
    void i2sInit(struct I2SClocklessLedDriver *driver);
    void setPins(struct I2SClocklessLedDriver *driver, int *Pins);
    void __initled(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip);
    struct I2SClocklessLedDriverDMABuffer *allocateDMABuffer(int bytes);
    void i2sStop(struct I2SClocklessLedDriver *driver);
    void i2sReset_DMA();
    void i2sReset_FIFO();
    void putdefaultones(struct I2SClocklessLedDriver *driver, uint16_t *buffer);
    void i2sReset();
    void i2sStart(struct I2SClocklessLedDriver *driver, struct I2SClocklessLedDriverDMABuffer *startBuffer);

    
void I2SClocklessLedDriverConstruct(struct I2SClocklessLedDriver *driver)
{

    // struct I2SClocklessLedDriverDMABuffer
    // {
        // lldesc_t descriptor;
        // uint8_t *buffer;
    // };

    driver->deviceBaseIndex[0] = I2S0O_DATA_OUT0_IDX;
    driver->deviceBaseIndex[1] = I2S1O_DATA_OUT0_IDX;
    driver->deviceClockIndex[0] = I2S0O_BCK_OUT_IDX;
    driver->deviceClockIndex[1] = I2S1O_BCK_OUT_IDX;
    driver->deviceWordSelectIndex[0] = I2S0O_WS_OUT_IDX;
    driver->deviceWordSelectIndex[1] = I2S1O_WS_OUT_IDX;
    driver->deviceModule[0] = PERIPH_I2S0_MODULE;
    driver->deviceModule[1] = PERIPH_I2S1_MODULE;

    // driver->i2s;
    // driver->_gI2SClocklessDriver_intr_handle;
    driver->I2SClocklessLedDriver_sem = NULL;
    driver->I2SClocklessLedDriver_semSync = NULL;
    driver->I2SClocklessLedDriver_semDisp = NULL;
    driver->I2SClocklessLedDriver_waitDisp = NULL;
    driver->dmaBufferActive = 0;
    // driver->wait;
    // driver->__displayMode;
    // driver->ledToDisplay;
    // driver->_offsetDisplay;
    // driver->_defaultOffsetDisplay;
    // driver->oo=0;
    // driver->leds;
    // driver->saveleds;
    // driver->startleds;
    // driver->linewidth;
    driver->dmaBufferCount = 2; //we use two buffers
    driver->transpose = false;

    // driver->num_strips;
    // driver->num_led_per_strip;
    // driver->total_leds;
    // driver->clock_pin;
    // driver->p_r;
    // driver->p_g;
    // driver->p_b;
    // driver->i2s_base_pin_index;
    // driver->nb_components;
    // driver->stripSize[16];
    driver->isDisplaying = false;
    driver->isWaiting = false;
    driver->framesync = false;
    driver->wasWaitingtofinish = false;
    // driver->counti;
    
    driver->DMABuffersTransposed = NULL;
    // driver->DMABuffersTampon[4];
}


// I2SClocklessLedDriver::I2SClocklessLedDriver(){};

void setPins(struct I2SClocklessLedDriver *driver, int *Pins)
    {

        for (int i = 0; i < driver->num_strips; i++)
        {

            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[Pins[i]], PIN_FUNC_GPIO);
            gpio_set_direction((gpio_num_t)Pins[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
            gpio_matrix_out(Pins[i], driver->deviceBaseIndex[I2S_DEVICE] + i + 8, false, false);
        }
    }

    //Corrected = 255 * (Image/255)^(1/2.2).


    void i2sInit(struct I2SClocklessLedDriver *driver)
    {
        int interruptSource;
        if (I2S_DEVICE == 0)
        {
            driver->i2s = &I2S0;
            periph_module_enable(PERIPH_I2S0_MODULE);
            interruptSource = ETS_I2S0_INTR_SOURCE;
            driver->i2s_base_pin_index = I2S0O_DATA_OUT0_IDX;
        }
        else
        {
            driver->i2s = &I2S1;
            periph_module_enable(PERIPH_I2S1_MODULE);
            interruptSource = ETS_I2S1_INTR_SOURCE;
            driver->i2s_base_pin_index = I2S1O_DATA_OUT0_IDX;
        }

        i2s_dev_t *i2s = driver->i2s;
        i2sReset();
        i2sReset_DMA();
        i2sReset_FIFO();
        i2s->conf.tx_right_first = 0;

        // -- Set parallel mode
        i2s->conf2.val = 0;
        i2s->conf2.lcd_en = 1;
        i2s->conf2.lcd_tx_wrx2_en = 1; // 0 for 16 or 32 parallel output
        i2s->conf2.lcd_tx_sdx2_en = 0; // HN

        // -- Set up the clock rate and sampling
        i2s->sample_rate_conf.val = 0;
        i2s->sample_rate_conf.tx_bits_mod = 16; // Number of parallel bits/pins
        i2s->clkm_conf.val = 0;

        i2s->clkm_conf.clka_en = 0;

//add the capability of going a bit faster
        i2s->clkm_conf.clkm_div_a = 3;    // CLOCK_DIVIDER_A;
        i2s->clkm_conf.clkm_div_b = 1;    //CLOCK_DIVIDER_B;
        i2s->clkm_conf.clkm_div_num = 33; //CLOCK_DIVIDER_N;


        i2s->fifo_conf.val = 0;
        i2s->fifo_conf.tx_fifo_mod_force_en = 1;
        i2s->fifo_conf.tx_fifo_mod = 1;  // 16-bit single channel data
        i2s->fifo_conf.tx_data_num = 32; //32; // fifo length
        i2s->fifo_conf.dscr_en = 1;      // fifo will use dma
        i2s->sample_rate_conf.tx_bck_div_num = 1;
        i2s->conf1.val = 0;
        i2s->conf1.tx_stop_en = 0;
        i2s->conf1.tx_pcm_bypass = 1;

        i2s->conf_chan.val = 0;
        i2s->conf_chan.tx_chan_mod = 1; // Mono mode, with tx_msb_right = 1, everything goes to right-channel

        i2s->timing.val = 0;
        i2s->int_ena.val = 0;
        /*
        // -- Allocate i2s interrupt
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ENA_V,1, I2S_OUT_EOF_INT_ENA_S);
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ENA_V, 1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ENA_V, 1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
        */
        esp_err_t e = esp_intr_alloc(interruptSource, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &_I2SClocklessLedDriverinterruptHandler, driver, &driver->_gI2SClocklessDriver_intr_handle);

        // -- Create a semaphore to block execution until all the controllers are done

        if (driver->I2SClocklessLedDriver_sem == NULL)
        {
            driver->I2SClocklessLedDriver_sem = xSemaphoreCreateBinary();
        }

        if (driver->I2SClocklessLedDriver_semSync == NULL)
        {
            driver->I2SClocklessLedDriver_semSync = xSemaphoreCreateBinary();
        }
        if (driver->I2SClocklessLedDriver_semDisp == NULL)
        {
            driver->I2SClocklessLedDriver_semDisp = xSemaphoreCreateBinary();
        }
    }

    void initDMABuffers(struct I2SClocklessLedDriver *driver)
    {
        driver->DMABuffersTampon[0] = allocateDMABuffer(driver->nb_components * 8 * 2 * 3); //the buffers for the
        driver->DMABuffersTampon[1] = allocateDMABuffer(driver->nb_components * 8 * 2 * 3);
        driver->DMABuffersTampon[2] = allocateDMABuffer(driver->nb_components * 8 * 2 * 3);
        driver->DMABuffersTampon[3] = allocateDMABuffer(driver->nb_components * 8 * 2 * 3 * 4);

        putdefaultones(driver, (uint16_t *)driver->DMABuffersTampon[0]->buffer);
        putdefaultones(driver, (uint16_t *)driver->DMABuffersTampon[1]->buffer);

    }



/*
    OffsetDisplay I2SClocklessLedDriver::getDefaultOffset()
    {
        return _defaultOffsetDisplay;
    }
*/

    
    void showPixels(struct I2SClocklessLedDriver *driver)
    {

    if (driver->__displayMode == NO_WAIT && driver->isDisplaying == true)
            {
                //printf("deja display\n");
              //  ESP_LOGE(TAG, "already displaying... wait");
                //return;
                driver->wasWaitingtofinish = true;
                if(driver->I2SClocklessLedDriver_waitDisp==NULL)
                    driver->I2SClocklessLedDriver_waitDisp = xSemaphoreCreateCounting(10,0);
                    const TickType_t xDelay = 50 ; //to avoid full blocking
                xSemaphoreTake(driver->I2SClocklessLedDriver_waitDisp, portMAX_DELAY);
                //printf("one re\n");
            }

        if (driver->leds == NULL)
        {
            ESP_LOGE(TAG, "no leds buffer defined");
            return;
        }
        driver->ledToDisplay = 0;
        driver->transpose = true;
        driver->DMABuffersTampon[0]->descriptor.qe.stqe_next = &(driver->DMABuffersTampon[1]->descriptor);
        driver->DMABuffersTampon[1]->descriptor.qe.stqe_next = &(driver->DMABuffersTampon[0]->descriptor);
        driver->DMABuffersTampon[2]->descriptor.qe.stqe_next = &(driver->DMABuffersTampon[0]->descriptor);
        driver->DMABuffersTampon[3]->descriptor.qe.stqe_next = 0;
        driver->dmaBufferActive = 0;
        loadAndTranspose(driver);//leds, stripSize, num_strips, (uint16_t *)DMABuffersTampon[0]->buffer, ledToDisplay, __green_map, __red_map, __blue_map, __white_map, nb_components, p_g, p_r, p_b);
        driver->dmaBufferActive = 1;
        i2sStart(driver, driver->DMABuffersTampon[2]);
        driver->isDisplaying=true;
        if (driver->__displayMode == WAIT)
        {
            driver->isWaiting = true;
            if (driver->I2SClocklessLedDriver_sem==NULL)
            driver->I2SClocklessLedDriver_sem=xSemaphoreCreateBinary();
            xSemaphoreTake(driver->I2SClocklessLedDriver_sem, portMAX_DELAY);
        }
        else
        {
            driver->isWaiting = false;
        }
    }

/*
    Pixel * I2SClocklessLedDriver::strip(int stripNum)
    {
        Pixel * l =(Pixel *)leds;
        //Serial.printf(" strip %d\n",stripNum);

        for(int i=0;i< (stripNum % num_strips);i++)
        {
             //Serial.printf("     strip %d\n",stripSize[i]);
            l=l+stripSize[i];
        }
        return l;
    }
*/

    int maxLength(int *sizes,int num_strips)
    {
            int max=0;
            for(int i=0;i<num_strips;i++)
            {
                if(max<sizes[i])
                {
                    max=sizes[i];
                }
            }
            return max;
    }
/*
    void I2SClocklessLedDriver::initled(uint8_t *leds, int *Pinsq, int *sizes,  int num_strips)
    {
        total_leds=0;
        for(int i=0;i<num_strips;i++)
        {
            this->stripSize[i]=sizes[i];
            total_leds+=sizes[i];
        }
        int maximum= maxLength( sizes,num_strips);
        //Serial.printf("maximum %d\n",maximum);
         ESP_LOGE(TAG, "maximum %d\n",maximum);
        nb_components = _nb_components;
        p_r = _p_r;
        p_g = _p_g;
        p_b = _p_b;
          __initled(leds, Pinsq, num_strips, maximum);
    }
*/
/*
    void I2SClocklessLedDriver::initled(uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip)
    {
         for(int i=0;i<num_strips;i++)
        {
            this->stripSize[i]=num_led_per_strip;
        }
        initled(leds, Pinsq,this->stripSize, num_strips);
    }
*/

    void initledSizes(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int *sizes,  int num_strips, enum colorarrangment cArr)
    {
        driver->total_leds=0;
        for(int i=0;i<num_strips;i++)
        {
            driver->stripSize[i]=sizes[i];
            driver->total_leds+=sizes[i];
        }
        int maximum= maxLength( sizes,num_strips);

           
        switch (cArr)
        {
        case ORDER_RGB:
            driver->nb_components = 3;
            driver->p_r = 0;
            driver->p_g = 1;
            driver->p_b = 2;
            break;
        case ORDER_RBG:
            driver->nb_components = 3;
            driver->p_r = 0;
            driver->p_g = 2;
            driver->p_b = 1;
            break;
        case ORDER_GRB:
            driver->nb_components = 3;
            driver->p_r = 1;
            driver->p_g = 0;
            driver->p_b = 2;
            break;
        case ORDER_GBR:
            driver->nb_components = 3;
            driver->p_r = 2;
            driver->p_g = 0;
            driver->p_b = 1;
            break;
        case ORDER_BRG:
            driver->nb_components = 3;
            driver->p_r = 1;
            driver->p_g = 2;
            driver->p_b = 0;
            break;
        case ORDER_BGR:
            driver->nb_components = 3;
            driver->p_r = 2;
            driver->p_g = 1;
            driver->p_b = 0;
            break;
        case ORDER_GRBW:
            driver->nb_components = 4;
            driver->p_r = 1;
            driver->p_g = 0;
            driver->p_b = 2;
            break;
        }
          __initled(driver, leds, Pinsq, num_strips, maximum);
    }

    void initled(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip,colorarrangment cArr)
    {
         for(int i=0;i<num_strips;i++)
        {
            driver->stripSize[i]=num_led_per_strip;
        }
        initledSizes(driver, leds, Pinsq, driver->stripSize, num_strips,cArr);
    }


    void __initled(struct I2SClocklessLedDriver *driver, uint8_t *leds, int *Pinsq, int num_strips, int num_led_per_strip)
    {
        driver->startleds = 0;
        driver->dmaBufferCount = 2;
        driver->leds = leds;
        driver->saveleds = leds;
        driver->num_led_per_strip = num_led_per_strip;
        driver->_offsetDisplay.offsetx = 0;
        driver->_offsetDisplay.offsety = 0;
        driver->_offsetDisplay.panel_width = num_led_per_strip;
        driver->_offsetDisplay.panel_height = 9999;
        driver->_defaultOffsetDisplay = driver->_offsetDisplay;
        driver->linewidth = num_led_per_strip;
        driver->num_strips = num_strips;
        // driver->dmaBufferCount = dmaBufferCount;

        setPins(driver, Pinsq);
        i2sInit(driver);
        initDMABuffers(driver);
    }

    I2SClocklessLedDriverDMABuffer *allocateDMABuffer(int bytes)
    {
        I2SClocklessLedDriverDMABuffer *b = (I2SClocklessLedDriverDMABuffer *)heap_caps_malloc(sizeof(I2SClocklessLedDriverDMABuffer), MALLOC_CAP_DMA);
        if (!b)
        {
            ESP_LOGE(TAG, "No more memory\n");
            return NULL;
        }

        b->buffer = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
        if (!b->buffer)
        {
            ESP_LOGE(TAG, "No more memory\n");
            return NULL;
        }
        memset(b->buffer, 0, bytes);

        b->descriptor.length = bytes;
        b->descriptor.size = bytes;
        b->descriptor.owner = 1;
        b->descriptor.sosf = 1;
        b->descriptor.buf = b->buffer;
        b->descriptor.offset = 0;
        b->descriptor.empty = 0;
        b->descriptor.eof = 1;
        b->descriptor.qe.stqe_next = 0;

        return b;
    }

    void i2sReset_DMA()
    {

        (&I2S0)->lc_conf.out_rst = 1;
        (&I2S0)->lc_conf.out_rst = 0;
    }

    void i2sReset_FIFO()
    {

        (&I2S0)->conf.tx_fifo_reset = 1;
        (&I2S0)->conf.tx_fifo_reset = 0;
    }

    void IRAM_ATTR i2sStop(struct I2SClocklessLedDriver *driver)
    {

       
        esp_intr_disable(driver->_gI2SClocklessDriver_intr_handle);
       
ets_delay_us(16);
        (&I2S0)->conf.tx_start = 0;
        while( (&I2S0)->conf.tx_start ==1){}
         i2sReset();
         
             driver->isDisplaying =false;
    
        if( driver->__displayMode == NO_WAIT && driver->wasWaitingtofinish == true)
        {
               driver->wasWaitingtofinish = false;
                  xSemaphoreGive(driver->I2SClocklessLedDriver_waitDisp);
                 
        }
    
        
    }

    void putdefaultones(struct I2SClocklessLedDriver *driver, uint16_t *buffer)
    {
        /*order to push the data to the pins
         0:D7
         1:1
         2:1
         3:0
         4:0
         5:D6
         6:D5
         7:1
         8:1
         9:0
         10:0
         11:D4
         12:D3
         13:1
         14:1
         15:0
         16:0
         17:D2
         18:D1
         19:1
         20:1
         21:0
         22:0
         23:D0
         */
        for (int i = 0; i < driver->nb_components * 8 / 2; i++)
        {
            buffer[i * 6 + 1] = 0xffff;
            buffer[i * 6 + 2] = 0xffff;
        }
    }

    void i2sStart(struct I2SClocklessLedDriver *driver, I2SClocklessLedDriverDMABuffer *startBuffer)
    {

        i2sReset();
        driver->framesync = false;
        driver->counti = 0;

        (&I2S0)->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;

        (&I2S0)->out_link.addr = (uint32_t) & (startBuffer->descriptor);

        (&I2S0)->out_link.start = 1;

        (&I2S0)->int_clr.val = (&I2S0)->int_raw.val;

        (&I2S0)->int_clr.val = (&I2S0)->int_raw.val;
        (&I2S0)->int_ena.val = 0;

        /*
         If we do not use the regular showpixels, then no need to activate the interupt at the end of each pixels
         */
        //if(transpose)
        (&I2S0)->int_ena.out_eof = 1;

        (&I2S0)->int_ena.out_total_eof = 1;
        esp_intr_enable(driver->_gI2SClocklessDriver_intr_handle);

        //We start the I2S
        (&I2S0)->conf.tx_start = 1;

        //Set the mode to indicate that we've started
        driver->isDisplaying = true;
    }

    void IRAM_ATTR i2sReset()
    {
        const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
        (&I2S0)->lc_conf.val |= lc_conf_reset_flags;
        (&I2S0)->lc_conf.val &= ~lc_conf_reset_flags;
        const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
        (&I2S0)->conf.val |= conf_reset_flags;
        (&I2S0)->conf.val &= ~conf_reset_flags;
    }

    // static void IRAM_ATTR interruptHandler(void *arg);

static void IRAM_ATTR _I2SClocklessLedDriverinterruptHandler(void *arg)
{
    struct I2SClocklessLedDriver *cont = (struct I2SClocklessLedDriver *)arg;

    if (GET_PERI_REG_BITS(I2S_INT_ST_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ST_V, I2S_OUT_EOF_INT_ST_S))
    {
        cont->framesync = !cont->framesync;

        if (((I2SClocklessLedDriver *)arg)->transpose)
        {
            cont->ledToDisplay++;
            if (cont->ledToDisplay < cont->num_led_per_strip)
            {
               loadAndTranspose(cont);

               if (cont->ledToDisplay == cont->num_led_per_strip - 3) //here it's not -1 because it takes time top have the change into account and it reread the buufer
                {
                    cont->DMABuffersTampon[cont->dmaBufferActive]->descriptor.qe.stqe_next = &(cont->DMABuffersTampon[3]->descriptor);
                }
                cont->dmaBufferActive = (cont->dmaBufferActive + 1) % 2;
            }
        }
        else
        {
            if (cont->framesync)
            {
                portBASE_TYPE HPTaskAwoken = 0;
                xSemaphoreGiveFromISR(cont->I2SClocklessLedDriver_semSync, &HPTaskAwoken);
                if (HPTaskAwoken == pdTRUE)
                    portYIELD_FROM_ISR();
            }
        }
    }

    if (GET_PERI_REG_BITS(I2S_INT_ST_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ST_V, I2S_OUT_TOTAL_EOF_INT_ST_S))
    {

             
                 
        i2sStop((I2SClocklessLedDriver *)arg);
        if (cont->isWaiting)
        {
            portBASE_TYPE HPTaskAwoken = 0;
            xSemaphoreGiveFromISR(cont->I2SClocklessLedDriver_sem, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE)
                // portYIELD_FROM_ISR(HPTaskAwoken);   why did Yves write this so?
                portYIELD_FROM_ISR();
        }
    }
    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);
}

static void IRAM_ATTR transpose16x1_noinline2(unsigned char *A, uint16_t *B)
{

    uint32_t x, y, x1, y1, t;

    y = *(unsigned int *)(A);
#if NUMSTRIPS > 4
    x = *(unsigned int *)(A + 4);
#else
    x = 0;
#endif

#if NUMSTRIPS > 8
    y1 = *(unsigned int *)(A + 8);
#else
    y1 = 0;
#endif
#if NUMSTRIPS > 12
    x1 = *(unsigned int *)(A + 12);
#else
    x1 = 0;
#endif

    // pre-transform x
#if NUMSTRIPS > 4
    t = (x ^ (x >> 7)) & AA;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & CC;
    x = x ^ t ^ (t << 14);
#endif
#if NUMSTRIPS > 12
    t = (x1 ^ (x1 >> 7)) & AA;
    x1 = x1 ^ t ^ (t << 7);
    t = (x1 ^ (x1 >> 14)) & CC;
    x1 = x1 ^ t ^ (t << 14);
#endif
    // pre-transform y
    t = (y ^ (y >> 7)) & AA;
    y = y ^ t ^ (t << 7);
    t = (y ^ (y >> 14)) & CC;
    y = y ^ t ^ (t << 14);
#if NUMSTRIPS > 8
    t = (y1 ^ (y1 >> 7)) & AA;
    y1 = y1 ^ t ^ (t << 7);
    t = (y1 ^ (y1 >> 14)) & CC;
    y1 = y1 ^ t ^ (t << 14);
#endif
    // final transform
    t = (x & FF) | ((y >> 4) & FF2);
    y = ((x << 4) & FF) | (y & FF2);
    x = t;

    t = (x1 & FF) | ((y1 >> 4) & FF2);
    y1 = ((x1 << 4) & FF) | (y1 & FF2);
    x1 = t;

    *((uint16_t *)(B)) = (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 5)) = (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
    *((uint16_t *)(B + 6)) = (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 11)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
    *((uint16_t *)(B + 12)) = (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
    *((uint16_t *)(B + 17)) = (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
    *((uint16_t *)(B + 18)) = (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
    *((uint16_t *)(B + 23)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
}

static void IRAM_ATTR loadAndTranspose(struct I2SClocklessLedDriver *driver)
{

    int nbcomponents=driver->nb_components;
    Lines secondPixel[nbcomponents];
    uint16_t *buffer;
    if(driver->transpose)
    buffer=(uint16_t *)driver->DMABuffersTampon[driver->dmaBufferActive]->buffer;
    else
     buffer=(uint16_t *)driver->DMABuffersTransposed[driver->dmaBufferActive]->buffer;

    uint16_t led_tmp=driver->ledToDisplay;
    memset(secondPixel,0,sizeof(secondPixel));
    uint8_t *poli = driver->leds + driver->ledToDisplay * nbcomponents;
    for (int i = 0; i < driver->num_strips; i++)
    {

        if(driver->ledToDisplay < driver->stripSize[i])
        {
        secondPixel[driver->p_g].bytes[i] = poli[1];
        secondPixel[driver->p_r].bytes[i] = poli[0];
        secondPixel[driver->p_b].bytes[i] = poli[2];
        if (nbcomponents > 3)
            secondPixel[3].bytes[i] = poli[3];
        }
         poli += driver->stripSize[i]* nbcomponents;
    }

    transpose16x1_noinline2(secondPixel[0].bytes, (uint16_t *)buffer);
    transpose16x1_noinline2(secondPixel[1].bytes, (uint16_t *)buffer + 3 * 8);
    transpose16x1_noinline2(secondPixel[2].bytes, (uint16_t *)buffer + 2 * 3 * 8);
    if (nbcomponents > 3)
        transpose16x1_noinline2(secondPixel[3].bytes, (uint16_t *)buffer + 3 * 3 * 8);
}

