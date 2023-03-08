/* Joseph Garro
// jmg289@uakron.edu
// 3/2/2023
// v1.1
// firmware for esp32 and xbee based smart power outlet
// resources used:
    https://github.com/theElementZero/ESP32-UART-interrupt-handling/blob/master/uart_interrupt.c
    https://github.com/analogdevicesinc/arduino/tree/master/Arduino%20Uno%20R3/libraries/ADE9153A
    https://github.com/nlohmann/json
//---------------------------------*/

// additional classes for functionality
#include "xbee_api.hpp"
#include "json.hpp"

// analog Decives code for ADE9153A
#include "ADE9153A.h"
#include "ADE9153AClass.h"

// esp32 classes
#include "stdio.h"                                      // standard io
#include "driver/gpio.h"                                // esp GPIO pin control
#include "driver/uart.h"                                // esp UART driver
#include "driver/spi_master.h"                          // esp SPI driver
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"                          // freeRTOS for multitasking
#include "freertos/task.h"                              // create and schedudle tasks
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "Arduino.h"

// c++ classes
#include <iostream>
#include <queue>
#include <ctime>
#include <time.h>
//-----------------------------------------------------------------------------------------------
// link json class
using json = nlohmann::json;

//------------------------------//
//      global definitions      //
//------------------------------//

#define BUFFER_SIZE (1024 * 4)                          // hold 4096 bytes in each buffer
const int MEASUREMENT_MULTIPLIER = 10000;               // multiplier to set sig figs in outlet measurements

//------------------------------//
//     ADE9153A definitions     //
//------------------------------//

ADE9153AClass topMeter;                                 // top ADE9153A
ADE9153AClass bottomMeter;                              // bottom ADE9153A

// structs to hold measurement data
struct EnergyRegs energyValsTop;
struct PowerRegs powerValsTop;
struct RMSRegs rmsValsTop;
struct PQRegs pqValsTop;
struct EnergyRegs energyValsBottom;
struct PowerRegs powerValsBottom;
struct RMSRegs rmsValsBottom;
struct PQRegs pqValsBottom;

//------------------------------//
//            queues            //
//------------------------------//

static QueueHandle_t xbee_queue;                        // queue to handle xbee events
// queue for pointers to incoming XBEE frames
std::queue<std::vector<uint8_t>> xbee_incoming;         // hold incoming XBEE frames
// queue for pointers to outgoing XBEE frames
std::queue<std::vector<uint8_t>> xbee_outgoing;         // hold outgoing xbee frames
// queues to store power measurement information
std::queue<json> collected_measurements;                // hold collected power measurements

//------------------------------//
//          semaphores          //
//------------------------------//

SemaphoreHandle_t measurements_lock = NULL;             // semaphore for measurement queue
SemaphoreHandle_t outgoing_lock = NULL;                 // sempahore for outgoing xbee queue
SemaphoreHandle_t incoming_lock = NULL;                 // semaphore for incoming data

//------------------------------//
//      outlet information      //
//------------------------------//

// global receptacle state
static int RECEPTACLE_STATE = 0;
// global maximum instantaneous power draw; shut off outlet if exceeded
static float MAX_INSTANTANEOUS_POWER_DRAW = -1;         // none by default
// global sustatined maximum power draw; shut off if exceeded
static float MAX_SUSTAINED_POWER_DRAW = -1;             // none by default
// global period for sustained power draw
static int SUSTAINED_POWER_DRAW_PERIOD = -1;            // none by default

//------------------------------//
//     GPIO pin definitions     //
//------------------------------//

// XBEE UART PINS
#define XBEE_UART (UART_NUM_2)                          // uart2 to communicate between xbee and esp32
#define XBEE_UART_RX (GPIO_NUM_18)                      // uart2 TX
#define XBEE_UART_TX (GPIO_NUM_19)                      // uart2 RX

// ADE9153A SPI Pins
#define SPI_SPEED 1000000                               // SPI speed
#define ADE9153A_MOSI (GPIO_NUM_32)                     // MOSI line => pin 29 on both ADE9153As
#define ADE9153A_MISO (GPIO_NUM_33)                     // MISO line => pin 30 on both ADE9153As
#define ADE9153A_SCLK (GPIO_NUM_26)                     // SCLK line => pin 31 on both ADE9153As
#define TOP_RESET (GPIO_NUM_17)                         // rst line => pin 28 on ADE9153A #1
#define BOTTOM_RESET (GPIO_NUM_4)                       // rst line => pin 28 on ADE9153A #2
#define TOP_CS (GPIO_NUM_16)                            // chipselect line for top ADE9153A => pin 32 on ADE9153A #1
#define BOTTOM_CS (GPIO_NUM_0)                          // chipselect line for bottom ADE9153A => pin 32 on ADE9153A #2

// receptacle control pins
#define TOP_CONTROL (GPIO_NUM_14)
#define BOTTOM_CONTROL (GPIO_NUM_12)

//------------------------------//
//         logging tags         //
//------------------------------//

static const char *XBEE_TAG = "xbee uart";
static const char *RECEPTACLE_TAG = "receptacle state";
static const char *MAX_INSTANTANEOUS_POWER_DRAW_TAG = "set max instantanous power draw";
static const char *MAX_SUSTAINED_POWER_DRAW_TAG = "set max sustained power draw";
static const char *SET_SYSTEM_TIME = "set system time";
static const char *PARSE_FRAME = "parse xbee frame";
static const char *SETUP = "setup";

//------------------------------//
//     function definitions     //
//------------------------------//

// configure UART connection to xbee module
static void xbee_uart_init(void){
    ESP_LOGI(XBEE_TAG, "configuring xbee uart connection");
    // UART configuration settings
    const uart_config_t xbee_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB};

    // install UART driver
    ESP_ERROR_CHECK(uart_driver_install(XBEE_UART, BUFFER_SIZE * 4, BUFFER_SIZE * 4, 1024, &xbee_queue, 0));        // install UART driver on pins connected to xbee, buffer of 2048 bytes, event queue enabled
    ESP_ERROR_CHECK(uart_param_config(XBEE_UART, &xbee_uart_config));                                               // write xbee_uart_config to xbee UART
    ESP_ERROR_CHECK(uart_set_pin(XBEE_UART, XBEE_UART_TX, XBEE_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));   // assign TX and RX pins to xbee UART
};

// UART event handler for XBEE
// synthesized from https://github.com/espressif/esp-idf/blob/49551cc48cb3cdd5563059028749616de313f0ec/examples/peripherals/uart/uart_events/main/uart_events_example_main.c
static void xbee_uart_event_task(void *pvParameters){
    uart_event_t xbee_event;                                                                // hold UART event
    uint8_t* dtmp = (uint8_t*) malloc(BUFFER_SIZE);                                         // temporary buffer
    for(;;){
        // activate when a UART event is detected
        if(xQueueReceive(xbee_queue, (void*)&xbee_event, (TickType_t)portMAX_DELAY)){
            size_t eventsize = xbee_event.size;
            bzero(dtmp, BUFFER_SIZE); 
            ESP_LOGI(XBEE_TAG, "uart[%d] event:", XBEE_UART);                               // zero buffer
            switch(xbee_event.type){                                                        // handle different UART events
                // read incoming UART data
                case UART_DATA:
                    ESP_LOGI(XBEE_TAG, "[UART DATA]: %d", eventsize);
                    uart_read_bytes(XBEE_UART, dtmp, eventsize, portMAX_DELAY);             // write data to dtmp
                    xSemaphoreTake(incoming_lock, portMAX_DELAY);                           // lock incoming frame queue
                    xbee_incoming.push(std::vector<uint8_t>(dtmp, dtmp + eventsize));       // push frame to incoming xbee queue
                    xSemaphoreGive(incoming_lock);                                          // release incoming frame queue
                    break;

                // HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(XBEE_TAG, "hw fifo overflow");
                    uart_flush_input(XBEE_UART);
                    xQueueReset(xbee_queue);
                    break;

                // UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(XBEE_TAG, "ring buffer full");
                    uart_flush_input(XBEE_UART);
                    xQueueReset(xbee_queue);
                    break;

                // UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(XBEE_TAG, "uart rx break");
                    break;

                // UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(XBEE_TAG, "uart parity error");
                    break;

                // UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(XBEE_TAG, "uart frame error");
                    break;

                default:
                    ESP_LOGI(XBEE_TAG, "xbee UART event: %d", xbee_event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
};

// toggle receptacle states based on control packet
void toggleReceptacles(json json_data){
    // make sure value exists
    if(!json_data["value"].is_null() && !json_data["value"].is_array() && json_data["value"].is_number()){
        switch(json_data["value"].get<int>()){
            // top off bottom off
            case 0:
                gpio_set_level(TOP_CONTROL, 0);
                gpio_set_level(BOTTOM_CONTROL, 0);
                RECEPTACLE_STATE = 0;
                ESP_LOGI(RECEPTACLE_TAG, "top off, bottom off");
                break;

            // top off bottom on
            case 1:
                gpio_set_level(TOP_CONTROL, 0);
                gpio_set_level(BOTTOM_CONTROL, 1);
                RECEPTACLE_STATE = 1;
                ESP_LOGI(RECEPTACLE_TAG, "top off, bottom on");
                break;

            // top on bottom off
            case 2:
                gpio_set_level(TOP_CONTROL, 1);
                gpio_set_level(BOTTOM_CONTROL, 0);
                RECEPTACLE_STATE = 2;
                ESP_LOGI(RECEPTACLE_TAG, "top on, bottom off");
                break;

            // top on bottom on
            case 3:
                gpio_set_level(TOP_CONTROL, 1);
                gpio_set_level(BOTTOM_CONTROL, 1);
                RECEPTACLE_STATE = 3;
                ESP_LOGI(RECEPTACLE_TAG, "top on, bottom on");
                break;

            // top on only
            case 4:
                gpio_set_level(TOP_CONTROL, 1);    
                RECEPTACLE_STATE = 4 & RECEPTACLE_STATE;
                ESP_LOGI(RECEPTACLE_TAG, "top on");
                break;

            // top off only
            case 5:
                gpio_set_level(TOP_CONTROL, 0);
                RECEPTACLE_STATE = 5 & RECEPTACLE_STATE;
                ESP_LOGI(RECEPTACLE_TAG, "top off");
                break;

            // bottom on only
            case 6:
                gpio_set_level(BOTTOM_CONTROL, 1);
                RECEPTACLE_STATE = 6 & RECEPTACLE_STATE;
                ESP_LOGI(RECEPTACLE_TAG, "bottom on");
                break;

            // bototm off only
            case 7:
                gpio_set_level(BOTTOM_CONTROL, 0);
                RECEPTACLE_STATE = 7 & RECEPTACLE_STATE;
                ESP_LOGI(RECEPTACLE_TAG, "bottom off");
                break;

            // invalid 
            default:
                ESP_LOGI(RECEPTACLE_TAG, "%d is an invalid value", json_data["value"].get<int>());
                break;
        }
    } // otherwise error
    else
        ESP_LOGI(RECEPTACLE_TAG, "error setting receptacle state");
}

// set maximum instantaneous power
void setMaximumInstantaneousPowerDraw(json json_data){
    // make sure value exists and is valid
    if(!json_data["value"].is_null() && !json_data["value"].is_array() && json_data.at("value").is_number()){
        // update maximum instaneous power
        MAX_INSTANTANEOUS_POWER_DRAW = json_data["value"].get<float>();
        ESP_LOGI(MAX_INSTANTANEOUS_POWER_DRAW_TAG, "set maximum instantanoues power draw to %f", MAX_INSTANTANEOUS_POWER_DRAW);
    }
    // otherwise error 
    else
        ESP_LOGI(MAX_INSTANTANEOUS_POWER_DRAW_TAG, "error setting maximum instantaneous power");
}

// set maximum power over a sustained period
void setMaximumSustainedPowerDraw(json json_data){
    // make sure value exists and is valid
    if((!json_data["value"].is_null() && !json_data["range"].is_null()) && (!json_data["value"].is_array() && !json_data["range"].is_array()) && (json_data.at("value").is_number() && json_data.at("range").is_number())){
        // update maximum instaneous power
        MAX_SUSTAINED_POWER_DRAW = json_data["value"].get<float>();
        ESP_LOGI(MAX_SUSTAINED_POWER_DRAW_TAG, "set maximum sustained power draw to %f", MAX_SUSTAINED_POWER_DRAW);
        SUSTAINED_POWER_DRAW_PERIOD = json_data["range"].get<int>();
        ESP_LOGI(MAX_SUSTAINED_POWER_DRAW_TAG, "set maximum sustained power draw period to %d days", SUSTAINED_POWER_DRAW_PERIOD);
    }
    // otherwise error 
    else
        ESP_LOGI(MAX_SUSTAINED_POWER_DRAW_TAG, "error setting maximum sustained power");
}

// set system time using time recieved from hub
void setSystemTime(json json_data){
 // make sure value exists and is valid
    if((!json_data["s"].is_null() && !json_data["us"].is_null() && !json_data["tz"].is_null()) && (!json_data["s"].is_array() && !json_data["us"].is_array() && !json_data["tz"].is_array()) && (json_data.at("s").is_number() && json_data.at("us").is_number() && json_data["tz"].is_string())){
        // update timezone
        setenv("TZ", (json_data["tz"].get<std::string>()).c_str(), 1);
        tzset();
        // set time
        timeval newtime;
        newtime.tv_sec = json_data["s"].get<int>();
        newtime.tv_usec = json_data["us"].get<int>();
        settimeofday(&newtime, NULL);
        ESP_LOGI(SET_SYSTEM_TIME, "set system time");
    }
    // otherwise error 
    else
        ESP_LOGI(SET_SYSTEM_TIME, "error setting system time");
}

// perform an action in the smart power outlet based on the contents of recieved frame
void performOutletAction(json json_payload){
    if(!json_payload["op"].is_null() && json_payload["op"].is_number()){
        //  get type of JSON
        int json_type = json_payload["op"].get<int>();
        // get data from json packet
        json json_data = json_payload["data"];
        switch(json_type){
             // outlet will not recieve measurement packets, ignore type == 0
             // handle changes in receptacle state
             case 1:
                toggleReceptacles(json_data);
                break;
                    
            //handle setting maximum instaneous power draw
            case 2:
                setMaximumInstantaneousPowerDraw(json_data);
                break;

            // handle maximum sustained power draw
            case 3:
                setMaximumSustainedPowerDraw(json_data);
                break;

            // handle time set
            case 4:
                setSystemTime(json_data);
                break;
            default:
                break;
       };       
    };
};

//----------------------------------------------------------
// sample data
//float test_v0 = 120.690;
//float test_i0 = 14.589;

float test_rp0 = 140.346;
float test_pf0 = 0.9999;

float test_rp1 = 5.524;
float test_pf1 = 0.8890;

unsigned char source = 1;

unsigned char type = 0;

// sample ADE9153A every second
static void sampleADE9153A(void* pvParameters){
    for(;;){
        time_t currTime;
        time(&currTime);
        xSemaphoreTake(measurements_lock, portMAX_DELAY);                                   // take measurements semaphore
        collected_measurements.push(json{
            {"s", currTime}, 
            {"t", {
                {"p", int(float(topMeter.SPI_Read_32(REG_AWATT)) * CAL_POWER_CC * MEASUREMENT_MULTIPLIER)},
                {"f", int(float(topMeter.SPI_Read_32(REG_APF))/(float)134217728 * MEASUREMENT_MULTIPLIER)}}}, 
            {"b",{
                {"p", int(MEASUREMENT_MULTIPLIER*test_rp1)},
                {"f", int(MEASUREMENT_MULTIPLIER*test_pf1)}}}
        });
        xSemaphoreGive(measurements_lock);
        vTaskDelay(1000/portTICK_PERIOD_MS);   
    }
}
/*
 * xbee frames arrive in the following format
        {
            "FRAME TYPE", X,                                    -- type of xbee frame, IE at command response, transmit request, etc
            "FRAME OVERHEAD", {                                 -- data relevant to xbee protocol, IE frame ID, destination, etc
                XXX, XXX
                .
                .
                .
            },
            "FRAME DATA", {                                     -- data to perform outlwt interactions with
                "data"{                                         -- data to act on, necessary data for operations will be found here in the expected key-value pairs
                    "value", X,
                    .
                    .
                }
                "op", x,                                        -- operation, IE toggle receptacles, set power limit, etc
            }                                                               
        }
*/
// determine action to take based on recieved XBEE frame
static void parseFrame(void *pvParameters){
    bool set = false;                                                                           // bool for if semaphore is stil set after loop
    for(;;){
        // work on existing xbee frames
        xSemaphoreTake(incoming_lock, portMAX_DELAY);                                           // take incoming semaphore
        set = true;                                                                             // semaphore taken
        while(!xbee_incoming.empty()){
            // get oldest xbee frame
            std::vector<uint8_t> xbee_frame = xbee_incoming.front();     
            // remove xbee frame from queue                   
            xbee_incoming.pop();
            xSemaphoreGive(incoming_lock);                                                      // release once front popped
            set = false;                                                                        // semaphore released
            // json with all information about xbee frame
            json j = readFrame(xbee_frame.data());
            //std::cout << "JSON Frame: " << j->dump() << std::endl;
            // handle error cases
            // unrecognized frame
            if(j == -1){
                ESP_LOGI(PARSE_FRAME, "recieved an unrecognized frame");
            }
            // invalid frame
            else if(j == -2){
                ESP_LOGI(PARSE_FRAME, "recieved an invalid frame");
            }
            // otherwise do something
            else{
                // get type of frame
                uint8_t frameType = j["FRAME TYPE"].get<int>();
                // should I change these to dynamiclly allocated vars????
                json frame_payload = j["FRAME DATA"];
                json frame_overhead = j["FRAME OVERHEAD"];
                // switch based on type of frame
                switch(frameType){
                    // rx response                                  -- transmit request will include data -> means this is an outlet action
                    case 0x90:                                      
                        performOutletAction(frame_payload);
                        break;
                    
                    // explicit rx response                         -- transmit request will include data -> means this is an outlet action
                    case 0x91:
                        performOutletAction(frame_payload);
                        break;

                    // all other cases                              -- other operations deal with XBee behavior -> do not need ESP32's attention
                    default:                                        
                        break;
                };
            }
            vTaskDelay(1);
        }
        if(set == true)                                                                         // release if not unset
            xSemaphoreGive(incoming_lock);                                                      // release semaphore lock
    vTaskDelay(100);       
    }
};

// send xbee frames stored in queue
static void sendFrame(void *pvParameters){
    bool set = false;                                                                           // bool for if semaphore is set
    for(;;){
        // work on existing xbee frames
        xSemaphoreTake(outgoing_lock, portMAX_DELAY);                                           // take incoming semaphore
        set = true;                                                                             // semaphore taken
        while(!xbee_outgoing.empty()){
            // get oldest xbee frame
            std::vector<uint8_t> xbee_frame = xbee_outgoing.front();                               
            // remove xbee frame from queue      
            xbee_outgoing.pop();
            xSemaphoreGive(outgoing_lock);                                                      // release semaphore
            set = false;                                                                        // semaphore released
            uart_write_bytes(XBEE_UART, xbee_frame.data(), xbee_frame.size());
            vTaskDelay(1);
        }
        if(set == true)                                                                         // release if not unset
            xSemaphoreGive(outgoing_lock);                                                      // release semaphore
        vTaskDelay(100);
    }
};

// test function to get current time
static void getTime2(void *pvParameters){
    for(;;){
        struct tm *loctime;
        time_t currtime;
        currtime = time(NULL);
        loctime = localtime(&currtime);
        std::cout << asctime(loctime) << std::endl;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    };
};

// get time from hub
void getTime(){
    // frame to ask hub for time
    std::vector<uint8_t> getTime = {0x7E, 0x00, 0x18, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0x22, 0x6F, 0x70, 0x22, 0x3A, 0x31, 0x30, 0x34, 0x7D, 0x05};
    //std::string time_op = "{op:104}";
    //std::vector <uint8_t> getTime = formTXFrame(time_op, 0, 0, NULL, NULL);
    // json to store hub response
    json hubResponse = {};
    // bufffer for response
    uint8_t* dtmp = (uint8_t*) malloc(BUFFER_SIZE);   
    bzero(dtmp, BUFFER_SIZE); 
    int length = 0;
    // get current time
    struct tm *loctime;                                                                     // tm
    time_t currtime;
    currtime = time(NULL);                                                                  // get time since epoch
    loctime = localtime(&currtime);                                                         // set local time
    uart_flush(XBEE_UART);
    while(loctime->tm_year < 123){                                                          // get years since epoch
        ESP_LOGI(SETUP, "time not valid");
        uart_write_bytes(XBEE_UART, getTime.data(), getTime.size());                        // send time request to hub
        vTaskDelay(5000/portTICK_PERIOD_MS);                                                // wait 2s for response
        ESP_ERROR_CHECK(uart_get_buffered_data_len(XBEE_UART, (size_t*)&length));
        if(length > 0){
            length = uart_read_bytes(XBEE_UART, dtmp, length, 100);                         // read in bytes
            uart_flush(XBEE_UART);                         
            hubResponse = readFrame(dtmp);
            std::cout << "DUMP: " << hubResponse.dump();
            // check response here
            setSystemTime(hubResponse["FRAME DATA"]["data"]);
            currtime = time(NULL);                                                          // get time since epoch
            loctime = localtime(&currtime);  
        }
    }   
    ESP_LOGI(SETUP, "valid time set");
}

// clear queues before continuing
// thank you David Rodríguez - dribeas : https://stackoverflow.com/questions/709146/how-do-i-clear-the-stdqueue-efficiently
void clearXbeeQueues(std::queue<std::vector<uint8_t>> &old_queue){
   std::queue<std::vector<uint8_t>> empty_queue;
   std::swap(old_queue, empty_queue);
}

// form frames containing outlet measurements
static void formMeasurementFrames(void *pvParameters){
    json frame;
    bool set = false;                                                                       // bool for if semaphore is set
    for(;;){
        xSemaphoreTake(measurements_lock, portMAX_DELAY);                                   // take semaphore
        set = true;                                                                         // semaphore taken
        // form frame while not empty
        while(!collected_measurements.empty()){
            frame = json{                                                        // create JSON
                {"op", 101},                                                                // hub OP for measurements = 101
                {"data", collected_measurements.front()}
            };
            collected_measurements.pop();                                                   // removed first element
            xSemaphoreGive(measurements_lock);                                              // release semaphore
            set = false;                                                                    // semaphore released
            xSemaphoreTake(outgoing_lock, portMAX_DELAY);                                   // take semaphore
            std::cout << frame.dump() << std::endl;
            xbee_outgoing.push(formTXFrame(frame.dump(), 0, 0, NULL, NULL));                // push measurement frame to outgoing XBEE frame queue
            xSemaphoreGive(outgoing_lock);                                                  // release semaphore
            vTaskDelay(1);
        }
        if(set == true)                                                                      // release if not unset
            xSemaphoreGive(measurements_lock);                                              // release semaphore
        vTaskDelay(100);
    }
}

static void printHeap(void* pvParameter){
    for(;;){
        std::cout << "Free Heap Size: " << int(esp_get_free_heap_size()) << std::endl;
        std::cout << "Largest block : " << int(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)) << std::endl;
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}

// main function
extern "C" void app_main() {
    //esp_log_level_set(XBEE_TAG, ESP_LOG_VERBOSE);                                           //uncomment to have more verbose logging

    // enable receptacle control pins
    gpio_reset_pin(TOP_CONTROL);
    gpio_reset_pin(BOTTOM_CONTROL);
    gpio_set_direction(TOP_CONTROL, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BOTTOM_CONTROL, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(TOP_CONTROL, 1);                                                         // enabled by default
    gpio_set_level(BOTTOM_CONTROL, 1);                                                      // enabled by default

    // set ADE9153A Pin directions
    gpio_set_direction(ADE9153A_SCLK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(ADE9153A_MOSI, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(ADE9153A_MISO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(TOP_CS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(TOP_RESET, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BOTTOM_CS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BOTTOM_RESET, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(BOTTOM_RESET, 1);
    gpio_set_level(TOP_RESET, 1);
    gpio_set_level(TOP_CS, 1);
    gpio_set_level(BOTTOM_CS, 1);
    // make sure xbee queues are empty
    clearXbeeQueues(xbee_incoming);
    clearXbeeQueues(xbee_outgoing);

    // initalize xbee UART connection
    xbee_uart_init();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    uart_flush_input(XBEE_UART);                                                            // flush xbee buffers after initalization
    uart_flush(XBEE_UART);          
    
    // set system time
    getTime();
    uart_flush_input(XBEE_UART);                                                            // flush xbee buffers after initalization
    uart_flush(XBEE_UART);          
    xQueueReset(xbee_queue);                                                                // flush xbee queue and buffers after initalization 
    vTaskDelay(1000/portTICK_PERIOD_MS);
    
    // initalize both ADE9153As
    bool topInit = topMeter.SPI_Init(SPI_SPEED, ADE9153A_SCLK, ADE9153A_MISO, ADE9153A_MOSI, TOP_CS, TOP_RESET);
    std::cout << "bool value: " << topInit << "\nhex value: " << std::hex << topMeter.SPI_Read_32(REG_VERSION_PRODUCT)<< std::endl;
    // initalize bottom here

    // semaphore initialization
    measurements_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(measurements_lock);                                                      // release measurements_lock
    incoming_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(incoming_lock);                                                          // release incoming lock
    outgoing_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(outgoing_lock);                                                          // release outgoing lock


    // begin multitasking
    xTaskCreatePinnedToCore(xbee_uart_event_task, "handle xbee", 4*2048, NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(parseFrame, "parse incoming frames", 2*32768, NULL, 13, NULL, 1);
    xTaskCreatePinnedToCore(sendFrame, "send frames in xbee queue", 32768, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(getTime2, "print time", 16384, NULL, 13, NULL, 1);
    xTaskCreatePinnedToCore(sampleADE9153A, "form measurements", 2*2048, NULL, 20, NULL, 1);  
    xTaskCreatePinnedToCore(formMeasurementFrames, "form measurements", 16384, NULL, 20, NULL, 1);  
    xTaskCreatePinnedToCore(printHeap, "print heap", 1024, NULL, 20, NULL, 1);  
}
