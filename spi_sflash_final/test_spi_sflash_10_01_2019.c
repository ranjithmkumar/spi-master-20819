#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_rand.h"
#include "wiced_timer.h"
#include "wiced_rtos.h"
#include "wiced_bt_stack.h"
#include "wiced_rtc.h"
#include "wiced_hal_sflash.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/* Threads defines */
#define THREAD_STACK_MIN_SIZE       (1024)          /* Sensible stack size for most threads */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/*Queue defines*/
#define MESSAGE_SIZE        (16)
#define QUEUE_LENGTH        (32)

/*SPI 1 defines*/
#define CLK_1                                 WICED_P15
#define MISO_1                                WICED_P14
#define MOSI_1                                WICED_P13
#define CS_1                                  WICED_P12
#define DEFAULT_FREQUENCY                   (1000000u)         /* 1 MHz */
#define GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1) ((((UINT32)CS_1&0xff)<<24)|((UINT32)CLK_1&0xff)<<16)|(((UINT32)MOSI_1&0xff)<<8)|((UINT32)MISO_1)
/******************************************************************************
 *                                Structures
 ******************************************************************************/
/*pSPI data packet structure*/
typedef struct
{
    uint8_t  header;
    uint16_t data;
} __attribute__((packed)) data_packet;

/*Temperature record structure stored in sflash*/
typedef struct
{
    uint16_t record_no;
    uint8_t dec_temp;
    uint8_t frac_temp;
    wiced_rtc_time_t timestamp;
}temperature_record;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static uint32_t             spi_master_gpio_cfg_1 = 0;
static wiced_thread_t *spi_1;
static wiced_thread_t *spi_2;
static wiced_queue_t * msg_q;
static wiced_semaphore_t *sem;
static uint16_t r_no=1;
static uint32_t curr_addr=0x00002000; //current address being written
static uint32_t rd_addr=0x00002000; // address where the user has to start reading from after button press
static uint32_t sect_old;
uint32_t diff=sizeof(temperature_record); // address increment after writing/reading
char string[96];

/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void             spi_master_driver_1(uint32_t arg);
static void             spi_master_driver_2(uint32_t arg);
void initialize_app( void );
void button_cback( void *data, uint8_t port_pin );
/******************************************************************************
 * Entry point to the application. Initialize the app
 *****************************************************************************/
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL );
}


wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
    /* BlueTooth stack enabled */
    case BTM_ENABLED_EVT:
        wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

        WICED_BT_TRACE("\r\nSample SPI Master Application\r\n");

        initialize_app();

        break;

    default:
        break;
    }
    return result;
}


/******************************************************************************
 * This functions initializes the SPI and GPIO as well as timer
 ******************************************************************************/
void initialize_app( void )
{
    int i;
    wiced_result_t result;

    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, NULL );
    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );

    wiced_hal_sflash_init();
    wiced_hal_sflash_erase(curr_addr,sizeof(temperature_record));
    sect_old=curr_addr/4096;
    WICED_BT_TRACE("\r\nsize of sflash %d\r\n", wiced_hal_sflash_get_size());

    wiced_rtc_init();

    wiced_hal_pspi_init(SPI1,
            SPI_MASTER,
            INPUT_PIN_PULL_UP,
            GPIO_CFG(CS_1,CLK_1,MOSI_1,MISO_1),
            DEFAULT_FREQUENCY,
            SPI_LSB_FIRST,
            SPI_SS_ACTIVE_LOW,
            SPI_MODE_0,
            CS_1);

    spi_1=wiced_rtos_create_thread();
    if ( WICED_SUCCESS == wiced_rtos_init_thread(spi_1,
            PRIORITY_MEDIUM,
            "SPI 1 instance",
            spi_master_driver_1,
            THREAD_STACK_MIN_SIZE,
            NULL ) )
    {
        WICED_BT_TRACE( "PSPI thread created\n\r" );
    }

    spi_2=wiced_rtos_create_thread();
    if(WICED_SUCCESS == wiced_rtos_init_thread(spi_2,
            PRIORITY_HIGH,
            "SPI 2 instance",
            spi_master_driver_2,
            THREAD_STACK_MIN_SIZE,
            NULL))
    {
        WICED_BT_TRACE( "SFLASH thread created\n\r" );
    }

    msg_q = wiced_rtos_create_queue();
    if(msg_q == NULL)
    {
        WICED_BT_TRACE("Queue creation error\r\n");
    }
    wiced_rtos_init_queue(msg_q,
            "queue",
            MESSAGE_SIZE,
            QUEUE_LENGTH);

    sem = wiced_rtos_create_semaphore();
    wiced_rtos_init_semaphore(sem);
}


void spi_master_driver_1( uint32_t arg )
{

    data_packet send_data;
    data_packet rec_data;
    uint8_t data[3]={0x01,0x02,0x03};
    uint8_t temp;
    wiced_result_t result;
    static int j=0;
    uint32_t count;
    temperature_record sampled_temp;
    while(1)
    {
        send_data.data = data[j];

        wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_LOW);
        wiced_hal_pspi_exchange_data(SPI1,
                sizeof(send_data.data),
                (uint8_t*)&send_data.data,
                (uint8_t*)&rec_data.data);
        wiced_hal_gpio_set_pin_output(CS_1, GPIO_PIN_OUTPUT_HIGH);
        if((rec_data.data == 0x000a && (j==0)) || j)
        {
            j++;
            if(j>2)
                j=2;
            temp=rec_data.data/100;
            sampled_temp.dec_temp=temp;
            temp=rec_data.data%100;
            sampled_temp.frac_temp=temp;
            wiced_rtc_get_time(&(sampled_temp.timestamp));
            sampled_temp.record_no=r_no;
            r_no++;
            result = wiced_rtos_push_to_queue(msg_q,
                    &sampled_temp,
                    WICED_NO_WAIT );
            if(result != WICED_SUCCESS)
            {
                WICED_BT_TRACE("Push failed \n\r");
            }

            wiced_rtos_get_queue_occupancy(msg_q, &count);
            if( count > (QUEUE_LENGTH/2))
            {
                wiced_rtos_set_semaphore(sem);
            }
        }
        wiced_rtos_delay_milliseconds(1000, ALLOW_THREAD_TO_SLEEP);

    }

}

void spi_master_driver_2( uint32_t arg )
{
    uint8_t dat;
    int i;
    uint32_t no_of_bytes_written;
    wiced_result_t result;
    temperature_record sampled_temperature;
    uint32_t sect_new;

    while(1)
    {
        sect_new=curr_addr/4096;
        if(sect_new-sect_old)
        {
            wiced_hal_sflash_erase(curr_addr,sizeof(temperature_record));
            sect_old=sect_new;
        }
        wiced_rtos_get_semaphore(sem,  WICED_WAIT_FOREVER);
        for(i=0; i < QUEUE_LENGTH/2; i++)
        {
            result = wiced_rtos_pop_from_queue( msg_q,
                    &(sampled_temperature),
                    WICED_WAIT_FOREVER );
            if(result == WICED_SUCCESS)
            {
                no_of_bytes_written = wiced_hal_sflash_write(curr_addr,sizeof(temperature_record),&sampled_temperature);
                if(no_of_bytes_written != sizeof(sampled_temperature))
                {
                    WICED_BT_TRACE("did not write \n\r");
                }
                else
                {
                    curr_addr+=diff;
                }
            }
            else
            {
                WICED_BT_TRACE("Pop failed due to error\n\r");
            }

        }
    }
}

void button_cback( void *data, uint8_t port_pin )
{
    uint32_t no_of_bytes_read;
    temperature_record stored_temperature;

    while(rd_addr<curr_addr)
    {
        no_of_bytes_read =  wiced_hal_sflash_read(rd_addr,sizeof(temperature_record),&stored_temperature);
        if(no_of_bytes_read != sizeof(stored_temperature))
        {
            WICED_BT_TRACE("did not read \n\r");
        }
        else
        {
            rd_addr+=diff;
            wiced_rtc_ctime(&(stored_temperature.timestamp),string);
            WICED_BT_TRACE("Record #: %d | Time stamp: %s | Temperature:%d.%d \n\r",stored_temperature.record_no,
                    string,
                    stored_temperature.dec_temp,
                    stored_temperature.frac_temp);
        }
    }
    WICED_BT_TRACE("No temperature records to be read \n\r");

}

