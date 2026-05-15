
#include "mtf02.h"
#include <math.h>

#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG (180.0f / M_PI)

/*
Users can use microlink_decode as their serial port data processing function
The minimum effective distance value is 10 (mm), and 0 indicates that the distance value is not available
optical flow velocity value unit：cm/s@1m
Calculation formula: speed(cm/s) = optical flow velocity * height(m)
*/

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);

/**
  * @brief  Декодирует сообщения протокола MicoLink от датчика потока MTF-01
  * @param  data: Входной байт для парсинга
  * @param  distance: Указатель для сохранения измерения расстояния (мм)
  * @param  distance_strength: Указатель для сохранения силы сигнала измерения расстояния
  * @param  distance_precision: Указатель для сохранения уровня точности измерения расстояния
  * @param  distance_status: Указатель для сохранения статуса измерения расстояния
  * @param  flow_velocity_x: Указатель для сохранения скорости потока по оси X (см/с)
  * @param  flow_velocity_y: Указатель для сохранения скорости потока по оси Y (см/с)
  * @param  flow_quality: Указатель для сохранения индикатора качества измерения потока
  * @param  flow_status: Указатель для сохранения статуса измерения потока
  * @retval None
  * @note   Эта функция реализует декодирование протокола MicoLink в соответствии с:
  *         https://micoair.com/docs/decoding-micolink-messages-from-mtf-01/
  */
void micolink_decode(uint8_t data,
        uint16_t *distance, uint16_t *distance_strength, uint8_t *distance_precision, uint8_t *distance_status,
        int16_t *flow_velocity_x, int16_t *flow_velocity_y, uint8_t *flow_quality, uint8_t *flow_status)
{
    // Статическая структура сообщения для сохранения состояния между вызовами функции
    static MICOLINK_MSG_t msg;
    // Парсинг входящего байта - возврат если сообщение еще не полностью получено
    if(micolink_parse_char(&msg, data) == false)
        return;
    // Обработка полного сообщения на основе идентификатора сообщения
    switch(msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;

            // Копирование данных полезной нагрузки в структурированный формат
            memcpy(&payload, msg.payload, msg.len);
            // Извлечение данных измерения расстояния
            *distance = payload.distance;
            *distance_strength = payload.strength;
            *distance_precision = payload.precision;
            *distance_status = payload.dis_status;
            // Извлечение данных оптического потока
            *flow_velocity_x = payload.flow_vel_x;
            *flow_velocity_y = payload.flow_vel_y;
            *flow_quality = payload.flow_quality;
            *flow_status = payload.flow_status;
            break;
        }
        default:
            // Игнорирование неподдерживаемых типов сообщений
            break;
    }
}

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
{
    switch(msg->status)
    {
    case 0:
        if(data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1:     // device id
        msg->dev_id = data;
        msg->status++;
        break;

    case 2:     // system id
        msg->sys_id = data;
        msg->status++;
        break;

    case 3:     // message id
        msg->msg_id = data;
        msg->status++;
        break;

    case 4:     //
        msg->seq = data;
        msg->status++;
        break;

    case 5:     // payload length
        msg->len = data;
        if(msg->len == 0)
            msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6:     // payload receive
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7:     // check sum
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg))
        {
            return true;
        }

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}
