
#include <string.h>
/*
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_event.h"
#include "esp_netif.h"
*/
#include "esp_wifi.h"
#include "espnow.h"
#include "esp_now.h"
#include "esp_log.h"
#include "config.h"
/*
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "static_mem.h"
#include "stabilizer.h"
*/
/*
 static xQueueHandle s_example_espnow_queue;
*/
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0x7C, 0x9E, 0xBD, 0xE3, 0xB9, 0x8D };
/*
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

 static void example_espnow_deinit(example_espnow_send_param_t *send_param);

*/

static myData_message myData;
static returnData_message returnData;

// WiFi should start before using ESPNOW
void wifi_init(void) {
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
	ESP_ERROR_CHECK(esp_wifi_start());
/*
#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif*/
}

// ESPNOW sending or receiving callback function is called in WiFi task.
// * Users should not do lengthy operations from this task. Instead, post
// * necessary data to a queue and handle it from a lower priority task.

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
/*		  char macStr[18];
	 snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	 ESP_LOGI("","Last Packet Sent to: %s, status %s",macStr, status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
	 */

	if (status != ESP_NOW_SEND_SUCCESS) {
		char macStr[18];
		snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
				mac_addr[5]);
		ESP_LOGI("espnow_send_cb", "No connection with: %s", macStr);
//		emergStop = true;
	}
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {

	if (mac_addr == NULL || incomingData == NULL || len <= 0) {
		ESP_LOGE("espnow_recv_cb", "Receive cb arg error");
		return;
	}

	memcpy(&myData, incomingData, sizeof(myData_message));

	//ESP_LOGI("espnow_recv_cb", "%i",myData.pitch);

	if (xQueueOverwrite(espnow_queueRx, &myData) != pdTRUE) {
		ESP_LOGW("espnow_recv_cb", "Send receive queue fail");
	}
}

static void espnow_task(void *pvParameter) {

	while (1) {
		if ((xQueueReceive(espnow_queueTx, &returnData, portMAX_DELAY) == pdTRUE)) {
			ESP_ERROR_CHECK(esp_now_send(s_broadcast_mac, (uint8_t* ) &returnData, sizeof(returnData)));
		}
	}
}

esp_err_t espnow_init(void) {
	//espnow_send_param_t *send_param;

	espnow_queueRx = xQueueCreate(1, sizeof(myData_message));
	espnow_queueTx = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(returnData_message));

	// Initialize ESPNOW and register sending and receiving callback function.
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

	// Set primary master key.
	//SP_ERROR_CHECK(esp_now_set_pmk((uint8_t* )CONFIG_ESPNOW_PMK));

	// Add broadcast peer information to peer list.
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (peer == NULL) {
		ESP_LOGE("espnow_init", "Malloc peer information fail");
//		esp_now_deinit();
		return ESP_FAIL;
	}
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK(esp_now_add_peer(peer));
	free(peer);

	xTaskCreate(espnow_task, ESPNOW_TASK_NAME, ESPNOW_TASK_STACKSIZE, NULL,
	ESPNOW_TASK_PRI, NULL);
	ESP_LOGI("espnow_init", "Task created");

	return ESP_OK;
}

