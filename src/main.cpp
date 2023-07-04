#include <mbed.h>
#include <radio_command.pb.h>
#include <radio_feedback.pb.h>
#include <swo.h>

#include "motor/brushless_board.h"
#include "motor/dribbler.h"
#include "rf_app.h"
#include "sensor/ir.h"
#include "ssl-kicker.h"

#define ROBOT_ID 4

#define HALF_PERIOD 500ms
#define ROBOT_RADIUS 0.085

// Radio frequency
#define RF_FREQUENCY_1 2508
#define RF_FREQUENCY_2 2510

using namespace sixtron;

static SWO swo;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &swo;
}

DigitalOut led(LED1);

EventQueue event_queue;

static SPI driver_spi(SPI_MOSI_DRV, SPI_MISO_DRV, SPI_SCK_DRV);
static Mutex spi_mutex;
static Brushless_board motor1(&driver_spi, SPI_CS_DRV1, &spi_mutex);
static Brushless_board motor2(&driver_spi, SPI_CS_DRV2, &spi_mutex);
static Brushless_board motor3(&driver_spi, SPI_CS_DRV3, &spi_mutex);
static Brushless_board motor4(&driver_spi, SPI_CS_DRV4, &spi_mutex);
static Dribbler dribbler(&driver_spi, SPI_CS_DRV5, &spi_mutex);
static SPI radio_spi(SPI_MOSI_RF, SPI_MISO_RF, SPI_SCK_RF);
static NRF24L01 radio1(&radio_spi, SPI_CS_RF1, CE_RF1, IRQ_RF1);
static NRF24L01 radio2(&radio_spi, SPI_CS_RF2, CE_RF2, IRQ_RF2);

static Timeout timeout;

static KICKER kicker(KCK_EN, KCK1, KCK2);

/* Struct definitions */
typedef struct _Motor_speed {
    float speed1;
    float speed2;
    float speed3;
    float speed4;
} Motor_speed;

static uint8_t com_addr1_to_listen[5] = { 0x22, 0x87, 0xe8, 0xf9, 0x01 };
RadioCommand ai_message = RadioCommand_init_zero;

void compute_motor_speed(
        Motor_speed *motor_speed, float normal_speed, float tangential_speed, float angular_speed)
{
    motor_speed->speed1 = -(sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed2 = -(sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed3 = (sqrt(2) / 2.) * normal_speed - (sqrt(2) / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
    motor_speed->speed4 = (sqrt(3) / 2.) * normal_speed + (1. / 2.) * tangential_speed
            + ROBOT_RADIUS * angular_speed;
}

void apply_motor_speed()
{
    if (ai_message.normal_velocity != 0 || ai_message.tangential_velocity != 0
            || ai_message.angular_velocity != 0) {
        // printf("Normal speed: %f\n", ai_message.normal_speed);
        // printf("tangential speed: %f\n", ai_message.tangential_speed);
        // printf("angular speed: %f\n", ai_message.angular_speed);
    }

    Motor_speed motor_speed;
    compute_motor_speed(&motor_speed,
            ai_message.normal_velocity,
            ai_message.tangential_velocity,
            ai_message.angular_velocity);

    motor1.set_state(Commands_RUN);
    motor2.set_state(Commands_RUN);
    motor3.set_state(Commands_RUN);
    motor4.set_state(Commands_RUN);
    motor1.set_speed(motor_speed.speed1);
    motor2.set_speed(motor_speed.speed2);
    motor3.set_speed(motor_speed.speed3);
    motor4.set_speed(motor_speed.speed4);
}

void send_feedback()
{
    RadioFeedback feedback;
    feedback.ir = ir::present();
    feedback.voltage = 25;
    // TODO: Add more feedback

    uint8_t tx_buffer[RadioFeedback_size + 1];

    memset(tx_buffer, 0, sizeof(tx_buffer));

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream = pb_ostream_from_buffer(tx_buffer + 1, RadioFeedback_size);

    bool status = pb_encode(&tx_stream, RadioFeedback_fields, &feedback);

    size_t message_length = tx_stream.bytes_written;

    /* Then just check for any errors.. */
    if (!status) {
        printf("[Feedback] Encoding failed: %s\n", PB_GET_ERROR(&tx_stream));
        return;
    }
    tx_buffer[0] = message_length;

    radio2.send_packet(tx_buffer, RadioFeedback_size + 1);
}

void stop_motors()
{
    motor1.set_state(Commands_STOP);
    motor2.set_state(Commands_STOP);
    motor3.set_state(Commands_STOP);
    motor4.set_state(Commands_STOP);
    motor1.set_speed(0.0);
    motor2.set_speed(0.0);
    motor3.set_speed(0.0);
    motor4.set_speed(0.0);
}

void on_rx_interrupt(uint8_t *data, size_t data_size)
{
    static uint8_t length = 0;
    ai_message = RadioCommand_init_zero;

    length = data[0];
    event_queue.call(printf, "LENGTH: %d\n", length);

    if (length == 0) {
        event_queue.call(apply_motor_speed);
    } else {
        /* Try to decode protobuf response */
        /* Create a stream that reads from the buffer. */
        pb_istream_t rx_stream = pb_istream_from_buffer(&data[1], length);

        /* Now we are ready to decode the message. */
        bool status = pb_decode(&rx_stream, RadioCommand_fields, &ai_message);

        event_queue.call(printf, "[IR]: %d\n", ir::present());

        /* Check for errors... */
        if (!status) {
            event_queue.call(printf, "[IA] Decoding failed: %s\n", PB_GET_ERROR(&rx_stream));
        } else {
            if (ai_message.robot_id != ROBOT_ID) {
                return;
            }

            event_queue.call(apply_motor_speed);
            if (ai_message.kick == Kicker::Kicker_CHIP) {
                kicker.kick1(ai_message.kick_power);
                event_queue.call(printf, "Power %f\n", ai_message.kick_power);
            }
            if (ai_message.kick == Kicker::Kicker_FLAT) {
                kicker.kick2(ai_message.kick_power);
                event_queue.call(printf, "Power %f\n", ai_message.kick_power);
            }
            if (ai_message.dribbler > 0.0) {
                dribbler.set_state(Commands_RUN);
                dribbler.set_speed(400);
                dribbler.send_message();
            } else {
                dribbler.set_state(Commands_STOP);
                dribbler.set_speed(0);
                dribbler.send_message();
            }

            if (ai_message.charge == 1) {
                kicker.enable_charge();
            } else {
                kicker.disable_charge();
            }
        }
    }

    // event_queue.call(send_feedback);

    timeout.detach();
    timeout.attach(stop_motors, 500ms);
}

void print_communication_status()
{
    printf("Motor1 TX errors: %ld\n", motor1.get_tx_error_count());
    printf("Motor1 RX errors: %ld\n", motor1.get_rx_error_count());
    printf("Motor2 TX errors: %ld\n", motor2.get_tx_error_count());
    printf("Motor2 RX errors: %ld\n", motor2.get_rx_error_count());
    printf("Motor3 TX errors: %ld\n", motor3.get_tx_error_count());
    printf("Motor3 RX errors: %ld\n", motor3.get_rx_error_count());
    printf("Motor4 TX errors: %ld\n", motor4.get_tx_error_count());
    printf("Motor4 RX errors: %ld\n", motor4.get_rx_error_count());
}

int main()
{
    driver_spi.frequency(500000);

    // // Remote
    // serial_port.baud(115200);
    // serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    motor1.set_communication_period(10);
    motor2.set_communication_period(10);
    motor3.set_communication_period(10);
    motor4.set_communication_period(10);

    motor1.start_communication();
    motor2.start_communication();
    motor3.start_communication();
    motor4.start_communication();

    // event_queue.call_every(1s, print_communication_status);
    event_queue.call_every(16ms, ir::compute);

    // Radio
    RF_app rf_app1(&radio1,
            RF_app::RFAppMode::RX,
            RF_FREQUENCY_1,
            com_addr1_to_listen,
            RadioCommand_size + 1);
    rf_app1.print_setup();
    rf_app1.attach_rx_callback(&on_rx_interrupt);
    rf_app1.run();

    radio2.initialize(
            NRF24L01::OperationMode::TRANSCEIVER, NRF24L01::DataRate::_2MBPS, RF_FREQUENCY_2);
    radio2.attach_transmitting_payload(
            NRF24L01::RxAddressPipe::RX_ADDR_P0, com_addr1_to_listen, RadioFeedback_size + 1);
    radio2.set_payload_size(NRF24L01::RxAddressPipe::RX_ADDR_P0, RadioFeedback_size + 1);
    radio2.set_interrupt(NRF24L01::InterruptMode::NONE);

    event_queue.dispatch_forever();

    while (true) { }
}
