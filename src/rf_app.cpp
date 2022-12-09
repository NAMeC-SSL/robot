/*
 * rf_app.cpp
 *
 *  Created on: 25 Apr 2019
 *      Author: jdelsol
 */
#include "rf_app.h"
#include "mbed.h"

namespace {
#define RF_FREQ_MAX 2525
#define RF_FREQ_MIN 2400
}

typedef enum {
    RX_EVENT = 1,
    TX_EVENT = 2,
} rf_com_events_t;

typedef struct {
    uint8_t id_app;
    bool print_process;
    bool process_running;
} rf_app_param_t;

static rf_app_param_t rf_app_param;

RF_app::RF_app(NRF24L01 *device,
        RFAppMode rf_mode,
        RFAppInterrupt enable_interrupt,
        uint16_t frequency,
        uint8_t *Tx_addr,
        uint8_t packet_size):
        _device(device)
{
    setup(rf_mode, enable_interrupt, frequency, Tx_addr, packet_size);
}

RF_app::RF_app(NRF24L01 *device,
        RFAppMode rf_mode,
        uint16_t frequency,
        uint8_t *Tx_addr,
        uint8_t packet_size):
        _device(device)
{
    setup(rf_mode, frequency, Tx_addr, packet_size);
}

void RF_app::setup(RFAppMode rf_mode,
        RFAppInterrupt enable_interrupt,
        uint16_t frequency,
        uint8_t *Tx_addr,
        uint8_t packet_size)
{
    switch (rf_mode) {
        case RFAppMode::RX:

            _device->initialize(
                    NRF24L01::OperationMode::RECEIVER, NRF24L01::DataRate::_2MBPS, frequency);
            _device->attach_receive_payload(
                    NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
            if (enable_interrupt == RFAppInterrupt::on_RX) {
                _device->set_interrupt(NRF24L01::InterruptMode::RX_ONLY);
                _device->attach(callback(this, &RF_app::_rf_callback));
            } else {
                _device->set_interrupt(NRF24L01::InterruptMode::NONE);
            }
            _device->start_listening();
            break;

        case RFAppMode::TX:
            _device->initialize(
                    NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, frequency);
            _device->attach_transmitting_payload(
                    NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
            if (enable_interrupt == RFAppInterrupt::on_TX) {
                _device->set_interrupt(NRF24L01::InterruptMode::TX_ONLY);
                _device->attach(callback(this, &RF_app::_rf_callback));
            } else {
                _device->set_interrupt(NRF24L01::InterruptMode::NONE);
            }
            break;
    }
}

void RF_app::setup(RFAppMode rf_mode, uint16_t frequency, uint8_t *Tx_addr, uint8_t packet_size)
{
    switch (rf_mode) {
        case RFAppMode::RX:
            _device->initialize(
                    NRF24L01::OperationMode::RECEIVER, NRF24L01::DataRate::_2MBPS, frequency);
            _device->attach_receive_payload(
                    NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
            _device->set_interrupt(NRF24L01::InterruptMode::RX_ONLY);
            _device->attach(callback(this, &RF_app::_rf_callback));
            _device->start_listening();
            break;

        case RFAppMode::TX:
            _device->initialize(
                    NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, frequency);
            _device->attach_transmitting_payload(
                    NRF24L01::RxAddressPipe::RX_ADDR_P0, Tx_addr, packet_size);
            _device->set_interrupt(NRF24L01::InterruptMode::TX_ONLY);
            _device->attach(callback(this, &RF_app::_rf_callback));
            break;
    }
}

void RF_app::run()
{
        _thread.start(callback(&_event_queue, &EventQueue::dispatch_forever));
}

void RF_app::exit()
{
    _thread.terminate();
}

void RF_app::get_rx_packet()
{	
    _device->clear_interrupt_flags();
    if (((_device->fifo_status_register()) & 0x02) == 0x02) {
        // rx fifo full
        _device->read_packet(_packet, IAToMainBoard_size);

        if (_rx_callback) {
			_rx_callback.call(_packet, IAToMainBoard_size);
		}
    }
}

void RF_app::print_setup()
{
    size_t i;
    uint8_t rx_addr_device[5] = { 0 };
    uint8_t tx_addr_device[5] = { 0 };

    // printf config RF 1
    printf("\r\n\r\n**** RF ****");
    if (_device->mode() == NRF24L01::OperationMode::RECEIVER) {
        printf("\r\nMode : RECEIVER");
    } else {
        printf("\r\nMode : TRANSCEIVER");
    }
    printf("\r\nFrequency : %d Hz", _device->rf_frequency());
    printf("\r\nOutput power : %d dBm", (int)_device->rf_output_power());
    printf("\r\nData rate : %d Kbps", (int)_device->data_rate());
    _device->rx_address(NRF24L01::RxAddressPipe::RX_ADDR_P0, rx_addr_device);
    printf("\r\nRx Address 0x");
    for (i = 0; i < sizeof(rx_addr_device); i++) {
        printf("%.2X", rx_addr_device[i]);
    }
    _device->tx_address(tx_addr_device);
    printf("\r\nTx Address 0x");
    for (i = 0; i < sizeof(tx_addr_device); i++) {
        printf("%.2X", tx_addr_device[i]);
    }
    printf("\r\n");

    printf("Status : 0x%02X\n", (int)_device->status_register());
}

/*__________________________

         Callback
___________________________ */
void RF_app::attach_rx_callback(Callback<void(uint8_t *packet, size_t length)> rx_callback)
{
    _rx_callback = rx_callback;
}

void RF_app::_rf_callback(void)
{
    _event_queue.call(callback(this, &RF_app::get_rx_packet));
}

/*__________________________

 	 Extern C functions
___________________________ */
void print_frame(com_packet_t *packet_to_print)
{
    printf("\r\n");
    switch (packet_to_print->type) {
        case PACKET_MASTER:
            printf("Type : MASTER\r\n");
            break;
        case PACKET_PARAMS:
            printf("Type : PARAMS\r\n");
            break;
        case PACKET_ROBOT:
            printf("Type : ROBOT\r\n");
            break;
    }

    // printf total length
    printf("length : %d bytes\r\n", packet_to_print->total_length);

    switch (packet_to_print->type) {
        case PACKET_MASTER:
            printf("actions : 0x%.2X\r\n", packet_to_print->master.actions);
            printf("x_speed : 0x%.2X\r\n", packet_to_print->master.x_speed);
            printf("y_speed : 0x%.2X\r\n", packet_to_print->master.y_speed);
            printf("t_speed : 0x%.2X\r\n", packet_to_print->master.t_speed);
            printf("kickPower : 0x%.2X\r\n", packet_to_print->master.kickPower);
            break;
        case PACKET_PARAMS:
            printf("kp : %f\r\n", packet_to_print->params.kp);
            printf("ki : %f\r\n", packet_to_print->params.ki);
            printf("kd : %f\r\n", packet_to_print->params.kd);
            break;
        case PACKET_ROBOT:
            printf("id : 0x%.2X\r\n", packet_to_print->robot.id);
            printf("status : 0x%.2X\r\n", packet_to_print->robot.status);
            printf("cap_volt : 0x%.2X\r\n", packet_to_print->robot.cap_volt);
            printf("voltage : 0x%.2X\r\n", packet_to_print->robot.voltage);
            break;
    }
}
