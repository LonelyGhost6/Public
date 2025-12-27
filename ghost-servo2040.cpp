/**
 * Copyright (c) 2023 Eddie Carrera
 * MIT License
 */

#include "main.h"
using namespace plasma;
using namespace servo;

/////////////* Global Variables */////////////
/* Create an array of servo pointers */
const int START_PIN = servo2040::SERVO_1;
const int END_PIN = servo2040::SERVO_18;
const int NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

/* Set up the shared analog inputs */
Analog sen_adc = Analog(servo2040::SHARED_ADC);
Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                        servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

/* Set up the analog multiplexer, including the pin for controlling pull-up/pull-down */
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
                          PIN_UNUSED, servo2040::SHARED_ADC);

/* Create the LED bar, using PIO 1 and State Machine 0 */
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

uint servoEnabled = false;

// Touch sensor LED's
LedPulse sensor_leds[4] = {
        {0.0f, 150, 100, 50, 0, false}, // TS1 → LED 0
        {1.0f, 50, 150, 100, 0, false}, // TS2 → LED 1
        {2.0f, 100, 50, 150, 0, false}, // TS3 → LED 2
        {3.0f, 150, 50, 100, 0, false}  // TS4 → LED 3
};

// RX = LED 5
// TX = LED 6
LedPulse rx_led = {4.0f, 150, 0, 0, 0, false};
LedPulse tx_led = {5.0f, 0, 150, 0, 0, false};

int main()
{
    /*******************************************************************************
     * Initializations
     ******************************************************************************/
    /* Initialize the servo cluster */
    servos.init();

    /* Initialize analog inputs with pull downs */
    for (auto i = 0u; i < servo2040::NUM_SENSORS; i++)
    {
        mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
    }

    /* Initialize A0,A1,A2 */
    gpio_init_mask(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK);
    gpio_set_dir_masked(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK,
                        GPIO_OUTPUT_MASK); // Set output
    gpio_put_masked(A0_GPIO_MASK | A1_GPIO_MASK | A3_GPIO_MASK,
                    GPIO_LOW_MASK); // Set LOW

    stdio_init_all();
    /* Wait for VCP/CDC connection */
    led_bar.start();
    multicore_launch_core1(core1_entry);
    while (!stdio_usb_connected()){pendingVCP_ledSequence();}
    led_bar.clear();


    /*******************************************************************************
     * Application
     ******************************************************************************/
    while (1)
    {
        /* Monitor and parse serial data */
        parse_and_command_task();
        monitor_analogPin();
    }
}

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void push_tx_event() {
    multicore_fifo_push_blocking(1); // 1 = TX event
}

void push_rx_event() {
    multicore_fifo_push_blocking(2); // 2 = RX event
}

void push_ts_event(uint sensor_idx) {
    multicore_fifo_push_blocking(sensor_idx); // TS event
}

void push_ts_off_event(uint sensor_idx) {
    multicore_fifo_push_blocking(sensor_idx);
}

void core1_entry() {
    const uint64_t PULSE_US = 2000; // 2 ms pulse

    while (true) {
        // process all events currently in FIFO
        while (multicore_fifo_rvalid()) {
            uint32_t evt = multicore_fifo_pop_blocking();
            if (evt == 1) { // TX
                blink(5.0f, tx_led.r, tx_led.g, tx_led.b);
                tx_led.end_us = time_us_64() + PULSE_US;
                tx_led.active = true;
            } else if (evt == 2) { // RX
                blink(4.0f, rx_led.r, rx_led.r, rx_led.r);
                rx_led.end_us = time_us_64() + PULSE_US;
                rx_led.active = true;
            } else if (evt >= 10 && evt <= 13) { // TS
                int idx = evt -10;
                blink((float)idx, sensor_leds[idx].r, sensor_leds[idx].g, sensor_leds[idx].b);
                sensor_leds[idx].end_us = time_us_64() + PULSE_US;
                sensor_leds[idx].active = true;
            } else if (evt >= 20 && evt <= 23) { // TS off
                int idx = evt -20;
                blink((float)idx, 0, 0,0);
                sensor_leds[idx].active = false;
            }
        }

        // turn off LEDs if pulse expired
        uint64_t now = time_us_64();
        if (tx_led.active && now >= tx_led.end_us) {
            blink(5.0f, 0,0,0);
            tx_led.active = false;
        }
        if (rx_led.active && now >= rx_led.end_us) {
            blink(4.0f, 0,0,0);
            rx_led.active = false;
        }
        tight_loop_contents();
    }
}
/*******************************************************************************
 * Core Functions
 ******************************************************************************/
void parse_and_command_task()
{
    cmdPkt curr_cmdPkt;
    uint value = 0;
    int input;

    input = getchar_timeout_us(GETC_TIMEOUT_US);

    while (input != PICO_ERROR_TIMEOUT)
    {
        monitor_analogPin();
        /***************************** START OF PARSING *************************************/
        // Check if command
        if (input & 0x80) // Only start parsing if command detected
        {
            curr_cmdPkt.startIdx = getchar_timeout_us(GETC_TIMEOUT_US);
            curr_cmdPkt.count = getchar_timeout_us(GETC_TIMEOUT_US);

            if (input == SET_CMD)
            {
                curr_cmdPkt.cmd = set;
                push_rx_event();
            }
            else if (input == GET_CMD)
            {
                curr_cmdPkt.cmd = get;
                push_rx_event();
            }
            else if (input == LED_CMD) {
                curr_cmdPkt.cmd = led;
                push_rx_event();
            }
            else {
                break; // xxx: BAD COMMAND, makes compiler happy to avoid uninitialized curr_cmdPkt.cmd>:(
            }

            if (curr_cmdPkt.cmd == set) {
                for (uint idx = 0; idx < curr_cmdPkt.count; idx++) {
                    value = 0;
                    value = getchar_timeout_us(GETC_TIMEOUT_US) & 0x7F;
                    value |= (getchar_timeout_us(GETC_TIMEOUT_US) & 0x7F) << 7;
                    curr_cmdPkt.valueBuff[idx] = value;
                }
            }
            /***************************** END OF PARSING *************************************/

            /* NOTE:
                Servos do not move at all until A0 is SET to to enable by sending a nonzero number
                to the pin. However, servo values are still saved even before they are enabled.
                This way the servos will go to PWM values they are set to right after being enabled.
                If no value was sent to the servo before being enabled, they will move to 1500.

                Servos can be disabled be sending SET ZERO to A0. This will make A0 LOW as well
                as disable the servoes in software by deasserting the PWM values.

            */
            /***************************** RUN COMMAND *************************************/
            if (curr_cmdPkt.cmd == set) {
                for (uint idx = 0; idx < curr_cmdPkt.count; idx++, curr_cmdPkt.startIdx++) {
                    // startIdx is servo
                    if (curr_cmdPkt.startIdx <= SERVO18) {
                        servos.pulse(cmdPin_to_hardwarePin((cmdPins) curr_cmdPkt.startIdx),
                                     curr_cmdPkt.valueBuff[idx], servoEnabled);
                    }
                        // startIdx is A0/A1/A2
                    else if (curr_cmdPkt.startIdx >= RELAY) {
                        bool enableState = curr_cmdPkt.valueBuff[idx] ? true : false;

                        // Set physical pins
                        gpio_put(cmdPin_to_hardwarePin((cmdPins) curr_cmdPkt.startIdx), enableState);

                        // Enable/disable PWM outputs
                        if (curr_cmdPkt.startIdx == RELAY) {
                            servoEnabled = enableState;
                            if (enableState) {
                                servos.enable_all();
                            } else {
                                servos.disable_all();
                            }
                        }
                    }
                }
            }
            else if (curr_cmdPkt.cmd == get) {
                uint tx[3] = {GET_CMD, curr_cmdPkt.startIdx, curr_cmdPkt.count};
                vcp_transmit(tx, 3);

                for (uint idx = 0; idx < curr_cmdPkt.count; idx++, curr_cmdPkt.startIdx++) {
                    // startIdx is servo
                    if (curr_cmdPkt.startIdx <= SERVO18) {
                        uint pwmValue = 0;
                        uint mappedPin = cmdPin_to_hardwarePin((cmdPins) curr_cmdPkt.startIdx);
                        pwmValue = servos.pulse(mappedPin);
                        tx[0] = pwmValue & 0x7F;
                        tx[1] = (pwmValue >> 7) & 0x7F;
                        vcp_transmit(tx, 2);
                    }
                        // startIdx is touch sensor
                    else if (curr_cmdPkt.startIdx <= TS6) {
                        uint mappedPin = cmdPin_to_hardwarePin((cmdPins) curr_cmdPkt.startIdx);
                        float sensor_voltage = read_analogPin(mappedPin);
                        uint voltage_binary = round(sensor_voltage * b1024_3_3V_RATIO);// only send request pin voltage
                        tx[0] = voltage_binary & 0x7F;
                        tx[1] = (voltage_binary >> 7) & 0x7F;
                        vcp_transmit(tx, 2);
                    } else if (curr_cmdPkt.startIdx == CURR) {
                        float current_f = read_current();
                        uint current_binary = round(current_f / CURR_LSb) + 512;
                        tx[0] = current_binary & 0x7F;
                        tx[1] = (current_binary >> 7) & 0x7F;
                        vcp_transmit(tx, 2);
                    } else if (curr_cmdPkt.startIdx == VOLT) {
                        float voltage_f = read_voltage();
                        uint voltage_binary = round(voltage_f * b1024_3_3V_RATIO);
                        tx[0] = voltage_binary & 0x7F;
                        tx[1] = (voltage_binary >> 7) & 0x7F;
                        vcp_transmit(tx, 2);
                    }
                }

                /***************************** COMMAND END *************************************/
            }
//            else if (curr_cmdPkt.cmd == led) {
//                for (uint idx = 0; idx < curr_cmdPkt.count; idx++) {
//                    if (idx == OFF){
//                        led_off();
//                    } else if ( idx == WHITE) {
//                        blink(255,255,255);
//                    } else if ( idx == RED) {
//                        blink(255,0,0);
//                    } else if ( idx == GREEN) {
//                        blink(0,255,0);
//                    } else if ( idx == BLUE) {
//                        blink(0,0,255);
//                    }
//                }
//            }
        }

        /***************************** CHECK IF MORE DATA *************************************/

        input = getchar_timeout_us(GETC_TIMEOUT_US);
    } // while (input != PICO_ERROR_TIMEOUT)
}
/*******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 * VCP/Parsing Support Functions
 ******************************************************************************/
uint cmdPin_to_hardwarePin(cmdPins cmdPin)
{
    return RP_hardwarePins_table[cmdPin];
}
/*******************************************************************************
 ******************************************************************************/
void vcp_transmit(uint *txbuff, uint size)
{
    for (uint byte = 0; byte < size; byte++)
    {
        putchar_raw(txbuff[byte]);
        push_tx_event();
    }
}

/*******************************************************************************
 * LED Support Functions
 ******************************************************************************/
void pendingVCP_ledSequence(void)
{
    static float offset = 0.0;
    const uint updates = 50;

    offset += 0.005;

    // Update all the LEDs
    for (auto i = 0u; i < servo2040::NUM_LEDS; i++)
    {
        float hue = (float)i / (float)servo2040::NUM_LEDS;
        led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
    }

    sleep_ms(1000 / updates);
}

void blink(float i, int r, int g, int b){
    led_bar.set_rgb(i, r, g, b);
}

/*******************************************************************************
 * Sensing Support Functions
 ******************************************************************************/
float read_current(void)
{
    mux.select(servo2040::CURRENT_SENSE_ADDR);
    return (cur_adc.read_current());
}
/*******************************************************************************
 ******************************************************************************/
float read_voltage(void)
{
    mux.select(servo2040::VOLTAGE_SENSE_ADDR);
    return (vol_adc.read_voltage());
}
/*******************************************************************************
 ******************************************************************************/
float read_analogPin(uint sensorAddress)
{
    mux.select(sensorAddress);
    return (sen_adc.read_voltage());
}
/*******************************************************************************
 ******************************************************************************/
void monitor_analogPin(void)
{
    for (uint sensor_index = 0; sensor_index < 4; sensor_index++)
    {
        float sensor_voltage = read_analogPin(sensor_index);  // <-- correct MUX channel
        uint voltage_binary = round(sensor_voltage * b1024_3_3V_RATIO);

        if (voltage_binary > 1000) {        // pressed
            push_ts_event(10 + sensor_index);
        }
        else if (voltage_binary < 100) {    // released
            push_ts_off_event(20 + sensor_index);
        }
    }
    tight_loop_contents();
}