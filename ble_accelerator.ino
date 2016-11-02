/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <SPI.h>
#include <Wire.h>
#include <lib_aci.h>
#include <inttypes.h>
#include <aci_setup.h>
#include "uart_over_ble.h"
#include "services.h"
#include "bma250.h"

// Use following define to add timing support: works with iOS but will cause a 1 minute time-out/disconnection with Linux Bluez stack
//#define SUPPORT_TIMING_CHANGE

// Use the following define to use serial interface for debugging
#define DEBUG_USING_SERIAL

#define BMA_RANGE BMA250_range_2g

// All time below is in milliseconds. The READ_TIME_INTERVAL needs to be as close as the BMA update interval. MIN_BLE_XFER_INTERVAL needs
// to be < 3 x BMA_UPDATE_INTERVAL or else we will miss the read interval!
#define BMA_UPDATE_INTERVAL  BMA250_update_time_64ms
#define READ_TIME_INTERVAL 64
#define MIN_BLE_XFER_INTERVAL 200

#define MOTION_THRESHOLD 1    // mgs

#define LATE_THRESHOLD 8      // milliseconds

static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;

// Accelerator
BMA250 accel;

// Fifo
typedef struct {
  uint8_t buffer[6];
} buffer_type;

#define BURST_SIZE 3
#define MAX_FIFO_ELEMENTS (10*BURST_SIZE)
buffer_type fifo[MAX_FIFO_ELEMENTS];
int8_t fifo_read = 0;
int8_t fifo_write = 0;
int8_t nb_elements = 0;
unsigned long last_read_time = 0;
unsigned long last_sent_time = 0;

#define STATUS_OK 0xFF
#define OVERFLOW  0x00
#define TOO_LATE  0xFD
int8_t status = STATUS_OK;

int8_t x_p_lo, x_p_hi, y_p_lo, y_p_hi, z_p_lo, z_p_hi;
int8_t x_lo, x_hi, y_lo, y_hi, z_lo, z_hi;

// Store the setup for the nRF8001 in the flash of the AVR to save on RAM 
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

// aci_struct contain:
//     total initial credits
//     current credit
//     current state of the aci (setup/standby/active/sleep)
//     open remote pipe pending
//     close remote pipe pending
//     Current pipe available bitmap
//     Current pipe closed bitmap
//     Current connection interval, slave latency and link supervision timeout
//     Current State of the the GATT client (Service Discovery)
//     Status of the bond (R) Peer address
static struct aci_state_t aci_state;

// Temporary buffers for sending ACI commands
static hal_aci_evt_t  aci_data;

// Timing change state variable
static bool timing_change_done = false;

// Used to test the UART TX characteristic notification
static uart_over_ble_t uart_over_ble;
static uint8_t uart_buffer[20];
static uint8_t uart_buffer_len = 0;
static uint8_t dummychar = 0;

// Define how assert should function in the BLE library 
void __ble_assert(const char *file, uint16_t line)
{
#ifdef DEBUG_USING_SERIAL
     Serial.print("ERROR "); Serial.print(file); Serial.print(": "); Serial.print(line); Serial.print("\n");
#endif
     while(1);
}

void setup(void)
{
#ifdef DEBUG_USING_SERIAL
     Serial.begin(115200);
     Serial.println(F("Arduino setup"));
     Serial.println(F("Set line ending to newline to send data from the serial monitor"));
#endif

     // Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
     aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
     aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
     aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*) setup_msgs;
     aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

     // Tell the ACI library, the MCU to nRF8001 pin connections. The Active pin is optional and can be marked UNUSED
     aci_state.aci_pins.board_name = BOARD_DEFAULT; // See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
     aci_state.aci_pins.reqn_pin   = 10; // SS for Nordic board, 9 for REDBEARLAB_SHIEfsdfsdfzxZxLD_V1_1
     aci_state.aci_pins.rdyn_pin   = 2;  // 3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
     aci_state.aci_pins.mosi_pin   = MOSI;
     aci_state.aci_pins.miso_pin   = MISO;
     aci_state.aci_pins.sck_pin    = SCK;

     aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8; //SPI_CLOCK_DIV8  = 2MHz SPI speed, SPI_CLOCK_DIV16 = 1MHz SPI speed
  
     aci_state.aci_pins.reset_pin              = 9; // 4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
     aci_state.aci_pins.active_pin             = UNUSED;
     aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

     aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
     aci_state.aci_pins.interrupt_number       = 1;

    // We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
    // If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001, then we initialize the data structures required 
    // to setup the nRF80. The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
    lib_aci_init(&aci_state, false);
    delay(500);

    // Init Accelerator
    accel.begin(BMA_RANGE, BMA_UPDATE_INTERVAL);
    delay(READ_TIME_INTERVAL);
    
    // Read initial accelerator values
    accel.start_read(); // This function gets new data from the accelerometer
    x_p_lo = accel.read_next_val();
    x_p_hi = accel.read_next_val();
    y_p_lo = accel.read_next_val();
    y_p_hi = accel.read_next_val();
    z_p_lo = accel.read_next_val();
    z_p_hi = accel.read_next_val();
    last_read_time = millis();
    last_sent_time = millis();
}

void uart_over_ble_init(void)
{
    uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
     bool status = false;

     if ( (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX)) &&
          (aci_state.data_credit_available >= 1) ) {
          status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
          if (status)
              aci_state.data_credit_available--;
     }
     return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
     bool status = false;
#ifdef SUPPORT_TIMING_CHANGE
     aci_ll_conn_params_t *conn_params;
#endif
     if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX)) {
#ifdef DEBUG_USING_SERIAL
          Serial.println(*byte, HEX);
#endif
          switch(*byte) {
          // Queues a ACI Disconnect to the nRF8001 when this packet is received.
          // May cause some of the UART packets being sent to be dropped
          case UART_OVER_BLE_DISCONNECT:
               lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
               status = true;
               break;

          // Queues an ACI Change Timing to the nRF8001
          case UART_OVER_BLE_LINK_TIMING_REQ:
               // Parameters:
               // Connection interval min: 2 bytes
               // Connection interval max: 2 bytes
               // Slave latency:           2 bytes
               // Timeout:                 2 bytes
               // Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
               // Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
#ifdef SUPPORT_TIMING_CHANGE
              conn_params = (aci_ll_conn_params_t *)(byte+1);
              lib_aci_change_timing(conn_params->min_conn_interval, conn_params->max_conn_interval, conn_params->slave_latency,
                                    conn_params->timeout_mult);  
#endif
              status = true;
              break;
          // Clears the RTS of the UART over BLE
          case UART_OVER_BLE_TRANSMIT_STOP:
               uart_over_ble.uart_rts_local = false;
               status = true;
               break;
          // Set the RTS of the UART over BLE
          case UART_OVER_BLE_TRANSMIT_OK:
               uart_over_ble.uart_rts_local = true;
               status = true;
               break;
          }
     }
     return status;
}

bool bconnected = false;

void aci_loop()
{
     static bool setup_required = false;

     // We enter the if statement only when there is a ACI event available to be processed
     if (lib_aci_event_get(&aci_state, &aci_data)) {
          aci_evt_t * aci_evt;
          aci_evt = &aci_data.evt;

          switch(aci_evt->evt_opcode) {
            
          // As soon as you reset the nRF8001 you will get an ACI Device Started Event
          case ACI_EVT_DEVICE_STARTED:  {
               aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
               switch(aci_evt->params.device_started.device_mode) {
               // When the device is in the setup mode
               case ACI_DEVICE_SETUP:
#ifdef DEBUG_USING_SERIAL
                    Serial.println(F("Evt Device Started: Setup"));
#endif
                    setup_required = true;
                    break;
               case ACI_DEVICE_STANDBY:
#ifdef DEBUG_USING_SERIAL
                    Serial.println(F("Evt Device Started: Standby"));
#endif
                    // Looking for an iPhone by sending radio advertisements
                    // When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
                    if (aci_evt->params.device_started.hw_error)
                         delay(20); // Magic number used to make sure the HW error event is handled correctly.
                    else {
                         lib_aci_connect(0/* in seconds */, 0x0050 /* advertising interval 50ms*/);
#ifdef DEBUG_USING_SERIAL
                         Serial.println(F("Advertising started"));
#endif
                    }
                    break;
               }
          }
          break; // ACI Device Started Event

          // If an ACI command response event comes with an error -> stop
          case ACI_EVT_CMD_RSP:
               if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status) {
                    // ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
                    // TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
                    // all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
#ifdef DEBUG_USING_SERIAL
                    Serial.print(F("ACI Command "));
                    Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
                    Serial.print(F("Evt Cmd respone: Status "));
                    Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
#endif
               }
               if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode) {
                    // Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
                    lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                    (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
               }
               break;

          case ACI_EVT_CONNECTED:
#ifdef DEBUG_USING_SERIAL
                Serial.println(F("Evt Connected"));
#endif
                uart_over_ble_init();
                timing_change_done = false;
                aci_state.data_credit_available = aci_state.data_credit_total;
                     
                // Get the device version of the nRF8001 and store it in the Hardware Revision String
                lib_aci_device_version();
                bconnected = true;
                break;

          case ACI_EVT_PIPE_STATUS:
#ifdef DEBUG_USING_SERIAL
               Serial.println(F("Evt Pipe Status"));
#endif
#ifdef SUPPORT_TIMING_CHANGE
               if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done)) {
                    lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                                      // Used to increase or decrease bandwidth
                    timing_change_done = true;
               }
#endif
               break;

          case ACI_EVT_TIMING:
#ifdef DEBUG_USING_SERIAL
               Serial.println(F("Evt link connection interval changed"));
#endif
               lib_aci_set_local_data(&aci_state,
                                     PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                     (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                     PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
               break;

          case ACI_EVT_DISCONNECTED:
               bconnected = false;
#ifdef DEBUG_USING_SERIAL
              Serial.println(F("Evt Disconnected/Advertising timed out"));
#endif
              lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
#ifdef DEBUG_USING_SERIAL
              Serial.println(F("Advertising started"));
#endif
              break;

          case ACI_EVT_DATA_RECEIVED:
#ifdef DEBUG_USING_SERIAL
               Serial.print(F("Pipe Number: "));
               Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);

#endif
               if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number) {
                    for (int i=0; i<aci_evt->len - 2; i++) {
#ifdef DEBUG_USING_SERIAL
                         Serial.print((char)aci_evt->params.data_received.rx_data.aci_data[i]);
                         Serial.print(F(" "));
#endif
                         uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
                    }
                    uart_buffer_len = aci_evt->len - 2;
#ifdef DEBUG_USING_SERIAL
                    Serial.println(F(""));
#endif
                    if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX)) {
                    }
               }
               if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number) {
                    uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); // Subtract for Opcode and Pipe number
               }
               break;

          case ACI_EVT_DATA_CREDIT:
          aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
          break;

          case ACI_EVT_PIPE_ERROR:
               // See the appendix in the nRF8001 Product Specication for details on the error codes
#ifdef DEBUG_USING_SERIAL
               Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
               Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
               Serial.print(F("  Pipe Error Code: 0x"));
               Serial.println(aci_evt->params.pipe_error.error_code, HEX);
#endif
               // Increment the credit available as the data packet was not sent.
               // The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted for the credit.
               if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
                    aci_state.data_credit_available++;
               bconnected = false;
               break;

          case ACI_EVT_HW_ERROR:
#ifdef DEBUG_USING_SERIAL
               Serial.print(F("HW error: "));
               Serial.println(aci_evt->params.hw_error.line_num, DEC);
              for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++) {
                   Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
              }
              Serial.println();
              Serial.println(F("Advertising started"));
#endif
              lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
              break;
          }
      }
      else {
           // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
           // Arduino can go to sleep now
           // Wakeup from sleep from the RDYN line
      }

      // setup_required is set to true when the device starts up and enters setup mode. It indicates that do_aci_setup() should be called. 
      // The flag should be cleared if do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
     if (setup_required) {
          if (SETUP_SUCCESS == do_aci_setup(&aci_state))
               setup_required = false;
     }
}

bool bfirst_change = true;

bool bconnection_complete = false;
unsigned long connection_time = 0;

void loop() {

     // Process any ACI commands or events
     aci_loop();

     while ((millis() - last_read_time) < READ_TIME_INTERVAL);
     if ((millis() - last_read_time) > (READ_TIME_INTERVAL + LATE_THRESHOLD))
          status = 255 - (millis() - last_read_time - READ_TIME_INTERVAL);
     accel.start_read(); // This function gets new data from the accelerometer
     x_lo = accel.read_next_val();
     x_hi = accel.read_next_val();
     y_lo = accel.read_next_val();
     y_hi = accel.read_next_val();
     z_lo = accel.read_next_val();
     z_hi = accel.read_next_val();
     last_read_time = millis();

     if (bconnected && !bconnection_complete) {
          if (connection_time == 0)
               connection_time = millis();
          else if (connection_time - millis() > 1000) {
               bconnection_complete = true;   
#ifdef DEBUG_USING_SERIAL
               Serial.println(F("Connection complete!"));
#endif
          }
     }
     else if (bconnected) {
          bool bchange = false;
          int16_t delta_x = (((int16_t) x_lo) | ((int16_t) x_hi) * 0x100) - (((int16_t) x_p_lo) | ((int16_t) x_p_hi) * 0x100);
          if ( (delta_x > MOTION_THRESHOLD) || (delta_x < -MOTION_THRESHOLD) )
               bchange = true;
          int16_t delta_y = (((int16_t) y_lo) | ((int16_t) y_hi) * 0x100) - (((int16_t) y_p_lo) | ((int16_t) y_p_hi) * 0x100);
          if ( (delta_y > MOTION_THRESHOLD) || (delta_y < -MOTION_THRESHOLD) )
               bchange = true;
          int16_t delta_z = (((int16_t) z_lo) | ((int16_t) z_hi) * 0x100) - (((int16_t) z_p_lo) | ((int16_t) z_p_hi) * 0x100);
          if ( (delta_z > MOTION_THRESHOLD) || (delta_z < -MOTION_THRESHOLD) )
               bchange = true;

          if (bchange || (nb_elements > 0)) {
               if (bfirst_change) { // Save the still motion values as they are our reference
                    fifo[fifo_write].buffer[0] = x_p_lo;
                    fifo[fifo_write].buffer[1] = x_p_hi;
                    fifo[fifo_write].buffer[2] = y_p_lo;
                    fifo[fifo_write].buffer[3] = y_p_hi;
                    fifo[fifo_write].buffer[4] = z_p_lo;
                    fifo[fifo_write].buffer[5] = z_p_hi;
                    fifo_write++;
                    if (fifo_write == MAX_FIFO_ELEMENTS)
                         fifo_write = 0;
                    bfirst_change = false; 
               }
               if (bchange || ((nb_elements % BURST_SIZE) != 0)) {
                    fifo[fifo_write].buffer[0] = x_lo;
                    fifo[fifo_write].buffer[1] = x_hi;
                    fifo[fifo_write].buffer[2] = y_lo;
                    fifo[fifo_write].buffer[3] = y_hi;
                    fifo[fifo_write].buffer[4] = z_lo;
                    fifo[fifo_write].buffer[5] = z_hi;
                    fifo_write++;
                    if (fifo_write == MAX_FIFO_ELEMENTS)
                         fifo_write = 0;              
                    // Check for overflow here...
                    if (fifo_read == fifo_write)
                         status = OVERFLOW;
                    nb_elements++;
               }
               if ((nb_elements % BURST_SIZE) == 0) {
                     uint8_t sendBuffer[21];
                     int8_t sendLength = 0;
                     int8_t burst, data;
                     sendBuffer[sendLength++] = status;
                     for (burst = 0; burst < BURST_SIZE; burst++) {
                           for (data = 0; data < 6; data++)
                                sendBuffer[sendLength++] = fifo[fifo_read].buffer[data];
                           fifo_read++;
                           if (fifo_read == MAX_FIFO_ELEMENTS)
                                fifo_read = 0;
                     }
                     while ((millis() - last_sent_time) < MIN_BLE_XFER_INTERVAL);
                     if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*) sendBuffer, sendLength)) {
#ifdef DEBUG_USING_SERIAL
                          Serial.println(F("TX dropped!"));
#endif
                     }
                     nb_elements -= BURST_SIZE;
                     last_sent_time = millis();
                     status = STATUS_OK;
               }
          }
          else {
                bfirst_change = true;
          }
     }
     else {
          bconnection_complete = false;
          connection_time = 0;
          delay(READ_TIME_INTERVAL);
     }
     
     x_p_lo = x_lo;
     x_p_hi = x_hi;
     y_p_lo = y_lo;
     y_p_hi = y_hi;
     z_p_lo = z_lo;
     z_p_hi = z_hi;
}

