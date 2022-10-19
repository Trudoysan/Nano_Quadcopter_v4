

#ifndef COMPONENTS_DRIVERS_GENERAL_ESPNOW_INCLUDE_ESPNOW_H_
#define COMPONENTS_DRIVERS_GENERAL_ESPNOW_INCLUDE_ESPNOW_H_


 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include <stdlib.h>
 #include "esp_now.h"

xQueueHandle espnow_queueRx;
xQueueHandle espnow_queueTx;

void wifi_init(void);
esp_err_t espnow_init(void);

#define ESPNOW_CHANNEL  0

#define ESPNOW_WIFI_MODE WIFI_MODE_APSTA //WIFI_MODE_AP
#define ESPNOW_WIFI_IF  ESP_IF_WIFI_AP

#define ESPNOW_QUEUE_SIZE           6
/*
 #define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)
 */
typedef struct struct_message {
	int roll, pitch, yaw, throttle;
	int flyDron;
	int param;
} myData_message;

typedef struct struct_return {
	int a, b, c, d, e, f, g, h;
	float fa, fb, fc, fd, fe, ff, fg, fh;
} returnData_message;

/*
 typedef enum {
 ESPNOW_SEND_CB,
 ESPNOW_RECV_CB,
 } espnow_event_id_t;

 typedef struct {
 uint8_t mac_addr[ESP_NOW_ETH_ALEN];
 esp_now_send_status_t status;
 } espnow_event_send_cb_t;

 typedef struct {
 uint8_t mac_addr[ESP_NOW_ETH_ALEN];
 uint8_t *data;
 int data_len;
 }espnow_event_recv_cb_t;

 typedef union {
 espnow_event_send_cb_t send_cb;
 espnow_event_recv_cb_t recv_cb;
 } espnow_event_info_t;


 typedef struct {
 espnow_event_id_t id;
 espnow_event_info_t info;
 } espnow_event_t;

 enum {
 ESPNOW_DATA_BROADCAST,
 ESPNOW_DATA_UNICAST,
 ESPNOW_DATA_MAX,
 };


 typedef struct {
 uint8_t type;                         //Broadcast or unicast ESPNOW data.
 uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
 uint16_t seq_num;                     //Sequence number of ESPNOW data.
 uint16_t crc;                         //CRC16 value of ESPNOW data.
 uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
 uint8_t payload[0];                   //Real payload of ESPNOW data.
 } __attribute__((packed)) espnow_data_t;


 typedef struct {
 bool unicast;                         //Send unicast ESPNOW data.
 bool broadcast;                       //Send broadcast ESPNOW data.
 uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
 uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
 uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
 uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
 int len;                              //Length of ESPNOW data to be sent, unit: byte.
 uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
 uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
 } espnow_send_param_t;

 */

#endif /* COMPONENTS_DRIVERS_GENERAL_ESPNOW_INCLUDE_ESPNOW_H_ */
