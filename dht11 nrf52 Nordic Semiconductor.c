/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
*
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "sdk_config.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "net_state.h"
#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "proxy.h"
#include "mesh_opt_gatt.h"
#include "mesh_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_on_off_client.h"
#include "generic_on_off_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "simple_pwm.h"

#define ONOFF_SERVER_0_LED              (BSP_LED_0)

static                                  generic_on_off_server_t m_server;
static                                  generic_on_off_client_t m_client;
static bool                             m_device_provisioned;
static bool                             m_led_flag = false;
static bool                             m_on_off_button_flag = false;

// TODO: Hands on 2.1 - Change the DEVICE_NAME to something unique
#define DEVICE_NAME                     "nRF5x Mesh Light"

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(25,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. was 250 */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. was 1000 */
#define GROUP_MSG_REPEAT_COUNT          (2)
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


unsigned char DHT11_READ_1_BYTES_DATA(void);
void DHT11_DIR_INPUT(void);
void DHT11_DIR_OUTPUT(void);
void DHT11_START_SEQUENCE(void);
char DHT11_READ_5_BYTES_DATA(void);
unsigned long int ip;

void Convert_Humidity_Temperature_Values(unsigned char byte1);
void DHT_task_callback(void);

unsigned char decimal_value3, decimal_value2,decimal_value1;
unsigned long int Count_micro_second = 0;
unsigned long int test_start_seq1;
unsigned long int test_start_seq2;
unsigned long int test_start_seq3;

unsigned char Humidity_Int;
unsigned char Humidity_DEC;
unsigned char TEMPERATURE_Int;
unsigned char TEMPERATURE_DEC;
unsigned char CHECKSUM;
unsigned char err;
unsigned int TEMPERATURE_VALUE;
unsigned int HUMIDITY_VALUE;

uint8_t BT_Buff[20],BT_Buff1[20], str[20] = "ADC 1234";
uint16_t bytes_to_send = 20;
int flag = 0;

#define level_0 0
#define level_1 1

//#define DHT11_ONE_WIRE_INPUT_PIN 4
long int j=0;
int timer_flag=0,k=0,K=0, hum = 0, temp = 0, count = 0;

uint32_t DHT11_ONE_WIRE_INPUT_PIN = 15;

static bool m_device_provisioned;

static void gap_params_init(void);
static void conn_params_init(void);

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

static bool on_off_server_get_cb(const generic_on_off_server_t * p_server)
{
  return m_led_flag;
}

static bool on_off_server_set_cb(const generic_on_off_server_t * p_server, bool value)
{
    // TODO: Hands on 2.3 - After initializing the PWM library in main(), change this function to use the PWM Driver instead of the hal_led_.. functions
    //                      Try to make the LED's fade in and out when the callback occurs, rather than having it set/cleared immediately
    uint32_t err_code;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET command to %u\n", value);
    if (value)
    {
        hal_led_pin_set(ONOFF_SERVER_0_LED, true);
        m_led_flag = true;
    }
    else
    {
        hal_led_pin_set(ONOFF_SERVER_0_LED, false);
        m_led_flag = false;
    }
    
    return value;
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);

    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static bool client_publication_configured(void)
{
    dsm_handle_t pub_addr_handle;
    
    if (access_model_publish_address_get(m_client.model_handle, &pub_addr_handle) == NRF_SUCCESS)
    {
        if (pub_addr_handle == DSM_HANDLE_INVALID)
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    // printf("inside public fn");
    return true;

}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    /*uint32_t status = client_publication_configured();
    printf("status: %u",status);*/
        if (client_publication_configured())
    {
        uint32_t status = NRF_SUCCESS;
         
        switch (button_number)
        {
            // printf("inside fn");
            /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
            the STATUS message to inform client about the state change. This is a demonstration of
            state change publication due to local event. */
            case 0:
            case 1:
                /* send a group message to the ODD group, with flip the current button flag value */
               // m_on_off_button_flag= false;
                 //status = generic_on_off_client_set(&m_client,m_on_off_button_flag);
                status = generic_on_off_client_set_unreliable(&m_client,
                                                           m_on_off_button_flag,
                                                         GROUP_MSG_REPEAT_COUNT);
                                                         
                break;
            
            /* Initiate node reset */
            case 3:
                /* Clear all the states to reset the node. */
                if (mesh_stack_is_device_provisioned())
                {
                    (void) proxy_stop();
                    mesh_stack_config_clear();
                    node_reset();
                }
                else
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
                }
                break;

            default:
                break;
        }  
    }
    else{
        //printf("Not inside fun");
    }
}

static void app_rtt_input_handler(int key)
{
    //printf("key %d", key);
    if (key >= 0 && key <= 4)
    {
        
        uint32_t button_number = key - 0;
       // printf("key %u", button_number);
        button_event_handler(button_number);
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    // TODO: Hands on 2.3 - Change the following code to pulse the LED's rather than blink
    //                      Use the LED_BLINK_CNT_PROV to decide the number of pulses
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void client_status_cb(const generic_on_off_client_t * p_self, generic_on_off_status_t status, uint16_t src)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "server status received \n");
    switch (status)
    {
        case GENERIC_ON_OFF_STATUS_ON:
            m_on_off_button_flag=1;   
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "set m_on_off_button_flag to 1 \n");
            break;

        case GENERIC_ON_OFF_STATUS_OFF:
            m_on_off_button_flag=0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "set m_on_off_button_flag to 0 \n");
            break;

        case GENERIC_ON_OFF_STATUS_ERROR_NO_REPLY:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "GENERIC_ON_OFF_STATUS_ERROR_NO_REPLY \n");
            break;

        case GENERIC_ON_OFF_STATUS_CANCELLED:
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unknown status \n");
            break;
    }
}
static void client_publish_timeout_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_server.get_cb = on_off_server_get_cb;
    m_server.set_cb = on_off_server_set_cb;
    ERROR_CHECK(generic_on_off_server_init(&m_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_server.model_handle));
    //Initialize client on model 1
    m_client.status_cb = client_status_cb;
    m_client.timeout_cb = client_publish_timeout_cb;
    ERROR_CHECK(generic_on_off_client_init(&m_client, 1));
    ERROR_CHECK(access_model_subscription_list_alloc(m_client.model_handle));
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = SERVER_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Proxy Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

#if defined S140 // todo remove that after S140 priority fixing
    softdevice_irq_priority_checker();
#endif

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);
   
     //app_rtt_input_handler(0);
     //unsigned int i=0;
    
    // TODO: Hands on 2.3 - Initialize the simple_pwm library to use the 4 LED's on the devkit
    //                      To verify that the PWM driver works, set one of the LED's to blink or pulse in a loop
    //                      Hint: Look at the comments in simple_pwm.h for examples of how to use the simple_pwm library
    //                      Hint2: The LED's on the board are defined by the BSP_LED_x defines, where x is 0-3

   /* for (;;)
    {
        (void)sd_app_evt_wait();
        i++;
        printf("NO = %d\n", i);
        if(i == 5){
        
        button_event_handler(button_num);

        }
        /*DHT_task_callback();  
        printf("Temp = %d\n",TEMPERATURE_Int);      */       
      // nrf_delay_ms(1000);
    //}
}

void DHT_task_callback(void)
{
                //uint8_t BT_Buff[20],BT_Buff1[20], str[20] = "ADC 1234";
                //uint16_t bytes_to_send = 20;
                ret_code_t err_code;
                //while(1)
                {
                        //while(ble_nus_string_send(&m_nus, str, &bytes_to_send) != NRF_SUCCESS);
                       // NRF_LOG_INFO("get first data\n");
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get first data\n");
                        DHT11_START_SEQUENCE();
                        err = DHT11_READ_5_BYTES_DATA();
                        if(err == 0)
                        {
                                Convert_Humidity_Temperature_Values(Humidity_Int);
                                //NRF_LOG_INFO("Humidity %d\n",(int)Humidity_Int);
                                decimal_value2 = ((decimal_value2&0x0f)|0x30);
                                decimal_value1 = ((decimal_value1&0x0f)|0x30);
                               // NRF_LOG_INFO("humidity = %c%c\n",decimal_value2,decimal_value1);
                                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "humidity = %c%c\n",decimal_value2,decimal_value1);
                    
                                   // sprintf((char *)BT_Buff,"HUM-%d",Humidity_Int);
                                //while(ble_nus_string_send(&m_nus, BT_Buff, &bytes_to_send) != NRF_SUCCESS);
        
                                //nop();
                                Convert_Humidity_Temperature_Values(TEMPERATURE_Int);
                                //NRF_LOG_INFO("Temperature = %d\n", (int)TEMPERATURE_Int);
                                decimal_value2 = ((decimal_value2&0x0f)|0x30);
                                decimal_value1 = ((decimal_value1&0x0f)|0x30);
                               // NRF_LOG_INFO("temp = %c%c\n",decimal_value2,decimal_value1);
                                //printf("temp = %c%c\n",decimal_value2,decimal_value1);
                               __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Temp = %d\n",TEMPERATURE_Int);
                                                                                                                    
                                //sprintf((char *)BT_Buff1,"Temp-%d",TEMPERATURE_Int);
                                //strcat(BT_Buff1, BT_Buff);
                                //while(ble_nus_string_send(&m_nus, BT_Buff1, &bytes_to_send) != NRF_SUCCESS);
                                // nop();
                                //ip++;
                                                                }

                     
 if(TEMPERATURE_Int >= 25){
       m_on_off_button_flag= true;
              uint32_t button_num = 0;
     button_event_handler(button_num);
     }
     else{
          m_on_off_button_flag= false;
              uint32_t button_num = 0;
     button_event_handler(button_num);
     
     }
                        //nrf_delay_ms(5000);
                }
}


void DHT11_START_SEQUENCE(void)
{
            DHT11_DIR_OUTPUT();

            nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN, level_1);
            //DHT11_ONE_WIRE_OUTPUT_PIN = 1;

            nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN, level_0);
            nrf_delay_ms(25);
            //ets_delay_us(22000);


            nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN, level_1);
            //  Delay_micro_second(30);
                nrf_delay_us(30);


            DHT11_DIR_INPUT();


            Count_micro_second = 0;
            //while(DHT11_ONE_WIRE_INPUT_PIN==0)
            while(nrf_gpio_pin_read(DHT11_ONE_WIRE_INPUT_PIN)==0)
            {

            //  printf("waiting for make line high by dht11\n");
                nrf_delay_us(1);
            }
            test_start_seq2 = Count_micro_second;

            Count_micro_second = 0;
            //  while(DHT11_ONE_WIRE_INPUT_PIN==1)
            while(nrf_gpio_pin_read(DHT11_ONE_WIRE_INPUT_PIN)==1)
            {
                    //printf("waiting for make line low by dht11\n");
                    nrf_delay_us(1);
            }
            test_start_seq3 = Count_micro_second;
}


char DHT11_READ_5_BYTES_DATA(void)
{
                //vTaskSuspendAll ();
                err = 0;
                Humidity_Int = DHT11_READ_1_BYTES_DATA();
                Humidity_DEC = DHT11_READ_1_BYTES_DATA();
                TEMPERATURE_Int = DHT11_READ_1_BYTES_DATA();
                TEMPERATURE_DEC = DHT11_READ_1_BYTES_DATA();
                CHECKSUM = DHT11_READ_1_BYTES_DATA();
//              printf("datas = %c %c\n",Humidity_Int,TEMPERATURE_Int);

                if (((Humidity_Int + Humidity_DEC + TEMPERATURE_Int + TEMPERATURE_DEC) & 0xff) != CHECKSUM) 
                {

                   // NRF_LOG_INFO("got wrong values\n");
                        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "got wrong values\n");
                    return 1;
                }
                else
                {
                    //NRF_LOG_INFO("got values\n");
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "got values\n");
                    return err;
                }
}

unsigned char DHT11_READ_1_BYTES_DATA(void)
{

            unsigned int i=0;
            unsigned char Return_Value=0;

            for(i=0;i<8;i++)
            {
                        Return_Value = Return_Value << 1;

                    //printf("inside low");
                    while((nrf_gpio_pin_read(DHT11_ONE_WIRE_INPUT_PIN)) == 0)
                    {
                                    //  printf("inside low");
                                    //NRF_LOG_INFO("inside low1");
                                        nrf_delay_us(5);
                    }
                    Count_micro_second = 0;

                    //printf("inside high");
                    //NRF_LOG_INFO("inside high");
                    while((nrf_gpio_pin_read(DHT11_ONE_WIRE_INPUT_PIN))==1)
                    {
                                Count_micro_second++;
                                nrf_delay_us(5);
                    }

                    if(Count_micro_second<7)
                    {   
                                    
                                    Return_Value = Return_Value | 0;
                    }
                    else
                    {
                                Return_Value = Return_Value | 1;
                    }       
            }
//      printf("return_value=%c\n",Return_Value);

    return Return_Value;

}

void DHT11_DIR_INPUT(void)
{
                nrf_gpio_cfg_input(DHT11_ONE_WIRE_INPUT_PIN, NRF_GPIO_PIN_PULLUP);
}

void DHT11_DIR_OUTPUT(void)
{
                nrf_gpio_cfg_output(DHT11_ONE_WIRE_INPUT_PIN);
                //DHT11_ONE_WIRE_DIR = 1;
}

void Convert_Humidity_Temperature_Values(unsigned char byte1)
{

            unsigned char intermediate_value1=0;                              //Varible required while converting the hex value to decimal value
                /*******************************************************************************
                Procedure to convert 8bit hex value to equivalent BCD value
                *******************************************************************************/
            decimal_value3=(byte1/100);
            intermediate_value1=byte1-(100*decimal_value3);
            decimal_value2=(intermediate_value1/10);
            decimal_value1=intermediate_value1-(10*decimal_value2);
}


int main(void)
{
   
    initialize();
    execution_start(start);
    
    while(1)
    {
        execution_start(DHT_task_callback);   
        (void)sd_app_evt_wait();
        nrf_delay_ms(5000);
    }
}
