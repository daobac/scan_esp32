#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"


#define MAX_DISCOVERED_DEVICES 50//tim t_bi
esp_bd_addr_t discovered_devices[MAX_DISCOVERED_DEVICES];
int discovered_devices_num = 0;//t_bi phat hien

//scan t_bi
static esp_ble_scan_params_t ble_scan_params = {
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval          = 0x50,// time 50ms
		.scan_window            = 0x30//se quet ltuc 25ms
	};

//kiem tra 
bool alreadyDiscovered(esp_bd_addr_t address) {

	bool found = false;
	
	for(int i = 0; i < discovered_devices_num; i++) {
		
		for(int j = 0; j < ESP_BD_ADDR_LEN; j++)
			found = (discovered_devices[i][j] == address[j]);
		
		if(found) break;
	}
	
	return found;
}

//cho t_bi vo danh sach h_thi
void addDevice(esp_bd_addr_t address) {
	
	discovered_devices_num++;
	if(discovered_devices_num > MAX_DISCOVERED_DEVICES) return;

	for(int i = 0; i < ESP_BD_ADDR_LEN; i++)
		discovered_devices[discovered_devices_num - 1][i] = address[i];
}


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
		
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 	
			
			if(param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan sau 15 seconds\n\n");
				esp_ble_gap_start_scanning(15);
			}
			else printf("-Loi........... %d\n\n", param->scan_param_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			
			if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				printf("Scan......\n\n");
			}
			else printf("loi..........%d\n\n", param->scan_start_cmpl.status);
			break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			
			if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
				
				if(!alreadyDiscovered(param->scan_rst.bda)) {
					
					
					printf("thiet bi: Adress=");
					for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
						printf("%02X", param->scan_rst.bda[i]);
						if(i != ESP_BD_ADDR_LEN -1) printf(":");
					}
					//ten t_bi
					uint8_t *adv_name = NULL;
					uint8_t adv_name_len = 0;
					adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
					if(adv_name) {
						printf("\nname =");
						for(int i = 0; i < adv_name_len; i++) printf("%c", adv_name[i]);
					}
					printf("\n\n");
					addDevice(param->scan_rst.bda);
				}
				
			}
			else if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
				printf("Complete\n\n");
			break;
		
		default:
		
			printf("Event %d chua xu ly\n\n", event);
			break;
	}
}


void app_main() {
	esp_log_level_set("*", ESP_LOG_ERROR);
	ESP_ERROR_CHECK(nvs_flash_init());
	
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
	
	// tao thu vien
	esp_bluedroid_init();
    esp_bluedroid_enable();
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	esp_ble_gap_set_scan_params(&ble_scan_params);
}