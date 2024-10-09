#include <stdio.h>
// #include <sys/_timeval.h>
#include "string.h"
#include "wit_c_sdk.h"
#include "driver/i2c.h"

#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"

#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define DEFAULT_SCAN_LIST_SIZE 10
#define DEFAULT_SSID "Transient"
#define DEFAULT_PASS "Oiseau2001"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// udp服务器的地址：这里的 255.255.255.255是在局域网发送，不指定某个设备
#define UDP_SERVER_IP "255.255.255.255"
#define UDP_SERVER_PORT 8265

#define ADC_ATTEN_3100mV        ADC_ATTEN_DB_11
// #define ADC_ATTEN_1750mV        ADC_ATTEN_DB_6
#define ADC1_CHAN1_NTC          ADC_CHANNEL_0
#define ADC1_CHAN1_FSR          ADC_CHANNEL_1
#define ADC1_CHAN1_VCC          ADC_CHANNEL_8

#define MAX_SIZE_BUF 512

// DAQ board v2.1
#define GPIO_FSRMUX_EN GPIO_NUM_21
#define GPIO_FSRMUX_S0 GPIO_NUM_10
#define GPIO_FSRMUX_S1 GPIO_NUM_11
#define GPIO_FSRMUX_S2 GPIO_NUM_12
#define GPIO_FSRPIN_SEL ((1ULL << GPIO_FSRMUX_EN)|(1ULL << GPIO_FSRMUX_S0)|(1ULL << GPIO_FSRMUX_S1)|(1ULL << GPIO_FSRMUX_S2))

#define GPIO_NTCMUX_EN GPIO_NUM_4
#define GPIO_NTCMUX_S0 GPIO_NUM_5
#define GPIO_NTCMUX_S1 GPIO_NUM_6
#define GPIO_NTCPIN_SEL ((1ULL << GPIO_NTCMUX_EN)|(1ULL << GPIO_NTCMUX_S0)|(1ULL << GPIO_NTCMUX_S1))

// JY901s 
// #define BUF_SIZE 1024
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define TEMP_UPDATE		0x10   //set the determine signal
#define READ_UPDATE		0x80

typedef unsigned char byte;

static const char *TAG = "proj test";

static EventGroupHandle_t s_wifi_event_group;
static QueueHandle_t xqueue_estimate;
static TaskHandle_t adc_task_handle = NULL;
static TaskHandle_t tcp_send_handle = NULL;
static TimerHandle_t xTimeHandle;
struct sensorMessage{
    /* data */
    size_t data_len;
    uint8_t crc;
    uint8_t uc_data[256];
} estMessage, *pxMsg; //发送与测量的数据

struct frameHeader{
    size_t header_len;
    uint8_t header[2];
} xheader;

static int s_retry_num = 0;
static int sock;
static int udp_sock;

static byte ip_esp32s3[4];
static struct sockaddr_in udp_remote_addr;
// static char *ip_server;

// one shot adc
uint32_t packetCounter;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan1_handle = NULL;
static adc_cali_handle_t adc1_cali_chan8_handle = NULL;

static gpio_config_t io_cfg = {};

// jy901s:
static volatile char s_cDataUpdate = 0;
static int i2c_master_port = 0;
static void AutoScanSensor(void);
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

static int flag = 0;
static bool hasConnected = false;
static bool isTcpConnected = false;

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 8,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 7,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static int32_t WitIICRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    int ret;
    int i;

    if(uiLen == 0)
    	return 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ucReg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr | 0x01, ACK_CHECK_EN);
    for(i=0; i<uiLen; i++)
    {
    	if(i == uiLen-1)	// last pack
    		i2c_master_read_byte(cmd, p_ucVal+i, NACK_VAL);
    	else
    		i2c_master_read_byte(cmd, p_ucVal+i, ACK_VAL);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int32_t WitIICWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    int ret;
    int i;

    if(uiLen == 0)
    	return 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ucReg, ACK_CHECK_EN);
    for(i=0; i<uiLen; i++)
    {
    	if(i == uiLen-1)	// last pack
    		i2c_master_read_byte(cmd, p_ucVal+i, NACK_VAL);
    	else
    		i2c_master_read_byte(cmd, p_ucVal+i, ACK_VAL);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void doubleToByte(byte *data, double value){
    memcpy(data, &value, sizeof(double));
}

static void doubleArrayToByte(byte *data, double *src, int src_len){
    int b_idx = 0;
    for(int i=0;i<src_len;i++){
        b_idx = i*8;
        doubleToByte(&data[b_idx], src[i]);
    }
}

static void intToByte(byte *b_value, int value){
    // 注意数组本身是地址，如果采用中间值则传入的数组实质上没有变化
    // 2byte就够了
    b_value[0] = (byte) ((value>>8) & 0xFF);
    b_value[1] = (byte) (value & 0xFF);
}

static void intTo4Byte(byte *b_value, int value){
    // 传递counter uint32_t
    b_value[0] = (byte) ((value>>24) & 0xFF);
    b_value[1] = (byte) ((value>>16) & 0xFF);
    b_value[2] = (byte) ((value>>8) & 0xFF);
    b_value[3] = (byte) (value & 0xFF);
}

static void adc_vol_to_bytes(byte *b_array, size_t byte_len, int *vol, size_t vol_len){
    if (byte_len < 2 * vol_len){
        ESP_LOGE(TAG, "BYTE length too short!");
    }
    int b_idx = 0;
    for(int i=0;i<vol_len;i++){
        b_idx = i * 2;
        intToByte(&b_array[b_idx], vol[i]);
    }
}

static bool adc_cali_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle){
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret =  adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret ==ESP_OK){
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated){
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else{
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_cali_deinit(adc_cali_handle_t handle){
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}

static void adc_init(){
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_3100mV,
    };

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1_FSR, &config);
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1_NTC, &config);
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1_VCC, &config);

    if (adc_cali_init(ADC_UNIT_1, ADC1_CHAN1_FSR, ADC_ATTEN_3100mV, &adc1_cali_chan1_handle)){
        ESP_LOGI(TAG, "FSR ADC calibrated!");
    }
    if (adc_cali_init(ADC_UNIT_1, ADC1_CHAN1_NTC, ADC_ATTEN_3100mV, &adc1_cali_chan0_handle)){
        ESP_LOGI(TAG, "NTC ADC calibrated!");
    }
    if (adc_cali_init(ADC_UNIT_1, ADC1_CHAN1_VCC, ADC_ATTEN_3100mV, &adc1_cali_chan8_handle)){
        ESP_LOGI(TAG, "VCC ADC calibrated!");
    }

    
}

static void witJy901sInit(){
    i2c_master_init();
    WitInit(WIT_PROTOCOL_I2C, 0x50);
    WitRegisterCallBack(SensorDataUpdate);
    WitI2cFuncRegister(WitIICWrite, WitIICRead);
    WitDelayMsRegister(Delayms);
    AutoScanSensor();
}

static void fsr_sel(int fsr_num){
    // pin1:y5 pin2:y7 pin3:y6 pin4:y4 pin5:y2 || pin7:y1 pin8:y0 pin9:y3
    // 3和4线焊反了
    switch (fsr_num)
    {
    case 0:
        gpio_set_level(GPIO_FSRMUX_S0, 1);
        gpio_set_level(GPIO_FSRMUX_S1, 0);
        gpio_set_level(GPIO_FSRMUX_S2, 1);
        break;
    case 1:
        gpio_set_level(GPIO_FSRMUX_S0, 1);
        gpio_set_level(GPIO_FSRMUX_S1, 1);
        gpio_set_level(GPIO_FSRMUX_S2, 1);
        break;
    case 2:
        gpio_set_level(GPIO_FSRMUX_S0, 0);
        gpio_set_level(GPIO_FSRMUX_S1, 1);
        gpio_set_level(GPIO_FSRMUX_S2, 1);
        break;
    case 3:
        gpio_set_level(GPIO_FSRMUX_S0, 0);
        gpio_set_level(GPIO_FSRMUX_S1, 0);
        gpio_set_level(GPIO_FSRMUX_S2, 1);
        break;
    case 4:
        gpio_set_level(GPIO_FSRMUX_S0, 0);
        gpio_set_level(GPIO_FSRMUX_S1, 1);
        gpio_set_level(GPIO_FSRMUX_S2, 0);
        break;
    case 5:
        gpio_set_level(GPIO_FSRMUX_S0, 1);
        gpio_set_level(GPIO_FSRMUX_S1, 0);
        gpio_set_level(GPIO_FSRMUX_S2, 0);
        break;
    case 6:
        gpio_set_level(GPIO_FSRMUX_S0, 0);
        gpio_set_level(GPIO_FSRMUX_S1, 0);
        gpio_set_level(GPIO_FSRMUX_S2, 0);
        break;
    case 7:
        gpio_set_level(GPIO_FSRMUX_S0, 1);
        gpio_set_level(GPIO_FSRMUX_S1, 1);
        gpio_set_level(GPIO_FSRMUX_S2, 0);
        break;
    default:
    ESP_LOGE(TAG, "Wrong fsr channel");
        break;
    }
}

static void ntc_sel(int ntc_num){
    // pin11 1I0; pin12 1I1; pin13 1I2; pin14 1I3;
    switch (ntc_num)
    {
    case 0:
        gpio_set_level(GPIO_NTCMUX_S0, 0);
        gpio_set_level(GPIO_NTCMUX_S1, 0);
        break;
    case 1:
        gpio_set_level(GPIO_NTCMUX_S0, 1);
        gpio_set_level(GPIO_NTCMUX_S1, 0);
        break;
    case 2:
        gpio_set_level(GPIO_NTCMUX_S0, 0);
        gpio_set_level(GPIO_NTCMUX_S1, 1);
        break;
    case 3:
        gpio_set_level(GPIO_NTCMUX_S0, 1);
        gpio_set_level(GPIO_NTCMUX_S1, 1);
        break;
    default:
    ESP_LOGE(TAG, "Wrong ntc channel");
        break;
    }
}

static void timeCallBackTask(){
    xTaskNotifyGive(adc_task_handle);
}

static void adc_task(void *pvParameters){
    // read adc & i2c
    esp_err_t ret = 0;
    uint32_t ret_num;
    int adc_num = 13; // 8fsr 4ntc 1vcc
    int adc_byte = adc_num * 2;
    // 8 fsr + 4 ntc + 1 vcc
    int adc_raw[13] = {0};
    int voltage[13] = {0};

    xheader.header_len = 2;
    // 打入header 2byte
    xheader.header[0] = 0x5A;
    xheader.header[1] = 0x55;
    ESP_LOGI(TAG, "it's %u and %u", xheader.header[0], xheader.header[1]);
    struct sensorMessage *pucMessage =  &estMessage;

    double dimu_AGE[9];
    int imu_byte = 72; //9*8byte

    int ip_byte = 4;

    ret_num = 6 + adc_byte + imu_byte + ip_byte; // 108 byte
    byte result[ret_num];
    memset(result, 0xcc, ret_num);

    packetCounter = 0;
    
    byte packetCounterByte[4] = {0};
    uint64_t start, end;
    xTimerStart(xTimeHandle, 0);
    start = esp_timer_get_time();
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // 包头
        memcpy(result, xheader.header, 2);
        intTo4Byte(packetCounterByte, packetCounter);
        memcpy(&result[2], packetCounterByte, 4); 
        // 包尾
        memcpy(&result[6 + adc_byte + imu_byte], ip_esp32s3, ip_byte);

        ret = adc_oneshot_read(adc1_handle, ADC1_CHAN1_VCC, &adc_raw[0]);
        for(int i=0;i<8;i++){
            fsr_sel(i);
            ret = ret + adc_oneshot_read(adc1_handle, ADC1_CHAN1_FSR, &adc_raw[i+1]);
            // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1_FSR, adc_raw[i]);
        }

        for(int i=0;i<4;i++){
            ntc_sel(i);
            ret = ret + adc_oneshot_read(adc1_handle, ADC1_CHAN1_NTC, &adc_raw[i+9]);
            // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1_FSR, adc_raw[0]);
        }
        

        WitReadReg(AX, 13);
        if(s_cDataUpdate){
            for(int i=0;i<3;i++){
                dimu_AGE[i] = sReg[AX+i] / 32768.0 * 16.0;
                dimu_AGE[i+3] = sReg[GX+i] / 32768.0 * 16.0;
                dimu_AGE[i+6] = sReg[Roll+i] / 32768.0 * 16.0;
            }
            if(s_cDataUpdate & ACC_UPDATE){
                // ESP_LOGI(TAG, "acc:%f %f %f\r\n", dimu_AGE[0], dimu_AGE[1], dimu_AGE[2]);
                s_cDataUpdate &= ~ACC_UPDATE;
            }
            if(s_cDataUpdate & GYRO_UPDATE){
                // ESP_LOGI(TAG, "gyro:%f %f %f\r\n", dimu_AGE[3], dimu_AGE[4], dimu_AGE[5]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE){
                // ESP_LOGI(TAG, "angle:%f %f %f\r\n", dimu_AGE[6], dimu_AGE[7], dimu_AGE[8]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
            doubleArrayToByte(&result[6+adc_byte], dimu_AGE, 9);
        } else{
            for(int i=0;i<9;i++){
                dimu_AGE[i] = 0.0;
            }
            doubleArrayToByte(&result[6+adc_byte], dimu_AGE, 9);
        }
        if (ret == ESP_OK){
            adc_cali_raw_to_voltage(adc1_cali_chan8_handle, adc_raw[0], &voltage[0]);
            for(int i=0;i<8;i++){
                adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[i+1], &voltage[i+1]);
                // ESP_LOGI(TAG, "ADC%d Channel[%d] Vol[%d] Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1_FSR, i, voltage[i]);
            }
            for(int i=0;i<4;i++){
                adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[i+9], &voltage[i+9]);
                // ESP_LOGI(TAG, "ADC%d Channel[%d] Vol[%d] Data: %d", ADC_UNIT_1 + 1, ADC1_CHAN1_NTC, i, voltage[i]);
            }
            
            adc_vol_to_bytes(&result[6], adc_byte, voltage, 13);

            memcpy(pucMessage->uc_data, result, ret_num);
            pucMessage->data_len = ret_num;
            xQueueSend(xqueue_estimate, (void*) &pucMessage, (TickType_t) 0);
        }
        else{
            ESP_LOGE(TAG, "ADC READ ERROR");
        }
        
        packetCounter++;
        if(packetCounter%1000==0){
            end = esp_timer_get_time();
            ESP_LOGI(TAG, "counter: %llu", end-start);
            start = end;
        }
    }

    adc_oneshot_del_unit(adc1_handle);
    adc_cali_deinit(adc1_cali_chan0_handle);
    adc_cali_deinit(adc1_cali_chan1_handle);
    adc_cali_deinit(adc1_cali_chan8_handle);
};

static void gpio_init(){
    io_cfg.intr_type = GPIO_INTR_DISABLE;
    io_cfg.mode = GPIO_MODE_OUTPUT;
    io_cfg.pin_bit_mask = GPIO_FSRPIN_SEL|GPIO_NTCPIN_SEL;
    io_cfg.pull_down_en = 0;
    io_cfg.pull_up_en = 0;
    gpio_config(&io_cfg);
    
    gpio_set_level(GPIO_FSRMUX_EN, 0);
    gpio_set_level(GPIO_NTCMUX_EN, 0);
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI("wifi event: ", "flag: %d; event_id: %ld", flag, event_id);
    flag++;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (hasConnected) return;
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        
        const char *ip_str = inet_ntoa(event->ip_info.ip);
        inet_pton(AF_INET, ip_str, ip_esp32s3);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifiInit(){
    s_wifi_event_group = xEventGroupCreate();
    flag = 0;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    // esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    // assert(sta_netif); //调试用的 可以去掉

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); //该参数需要编译后生成sdkconfig.h才能使用
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // connecting
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 DEFAULT_SSID, DEFAULT_PASS);
        hasConnected = true;
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 DEFAULT_SSID, DEFAULT_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return false;
}

static bool tcpInit(u32_t ip_int){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0){
        ESP_LOGE(TAG, "create socket failed");
        return 0;
    }
    ESP_LOGI(TAG, "create socket successfully");
    
    struct sockaddr_in destaddr = {};
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons(8080);
    destaddr.sin_addr.s_addr = ip_int; //修改此addr影响另一个addr inet函数也有问题
    socklen_t len = sizeof(struct sockaddr);
    if (connect(sock, (struct sockaddr *)&destaddr, len) < 0){
        ESP_LOGE(TAG, "connect to server failed!");
        close(sock);
        return 0;
    }
    ESP_LOGI(TAG, "connect to server successfully!");
    isTcpConnected = true;
    
    char buff[512] = "hello, I am tcp_client!";
    send(sock, buff, strlen(buff), 0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    return 1;
}

// 持续发送 任务
static void tcp_client_send_task(void *pvParameters){
    esp_err_t ret;
    struct sensorMessage *pTcpMessage;
    while (1){
        if(xqueue_estimate != 0){
            ret = xQueueReceive(xqueue_estimate, &pTcpMessage, (TickType_t) 0);
            if (ret == pdPASS){
                int err = send(sock, &(pTcpMessage->uc_data), pTcpMessage->data_len, 0);
                if (err < 0){
                    ESP_LOGE(TAG, "TCP Sending ERROR: errno %d", errno);
                    break;
                }
            }
        }
    }
}

static void tcp_client_recv_task(void *pvParameters){
    // esp_err_t ret;
    int len = 0;
    uint8_t recv_buff[512];
    
    const char *start_record = "CMD: start record";
    const char *stop_record = "CMD: stop record";
    const char *pause_record = "CMD: pause record";
    const char *resume_record = "CMD: resume record";

    while (len!=-1){
        len = recv(sock, recv_buff, sizeof(recv_buff), 0);
        if(len > 0){
            // recv_msg = recv_buff;
            char recv_msg[len];
            memcpy(recv_msg, recv_buff, len);
            // int test = strncmp(recv_msg, start_record, sizeof(start_record)-1 == 0)
            if (strncmp(recv_msg, start_record, len) == 0){
                ESP_LOGI(TAG, "Start data collection and send task");
                if(!tcp_send_handle){
                    xTaskCreate(tcp_client_send_task, "tcp_client_send", 4096, NULL, 1, &tcp_send_handle);
                }
                if(!adc_task_handle){
                    xTaskCreate(adc_task, "adc_read", 4096, NULL, 1, &adc_task_handle);
                }
            }
            else if (strncmp(recv_msg, stop_record, len) == 0){
                ESP_LOGI(TAG, "Tasks stops. Socket will be disconnected.");
                if(tcp_send_handle){
                    vTaskDelete(tcp_send_handle);
                    tcp_send_handle = NULL;
                }

                // 先处理tasknotify
                if(adc_task_handle){
                    xTimerStop(xTimeHandle, 0);
                    vTaskDelete(adc_task_handle);
                    adc_task_handle = NULL;
                }
            }
            else if (strncmp(recv_msg, pause_record, len) == 0){
                ESP_LOGI(TAG, "Tasks paused");
                if(tcp_send_handle){
                    vTaskSuspend(tcp_send_handle);
                }
                if(adc_task_handle){
                    xTimerStop(xTimeHandle, 0);
                    vTaskSuspend(adc_task_handle);
                }
            }
            else if (strncmp(recv_msg, resume_record, len) == 0){
                ESP_LOGI(TAG, "Tasks resumed");
                packetCounter = 0;
                xQueueReset(xqueue_estimate);
                if(tcp_send_handle){
                    vTaskResume(tcp_send_handle);
                }
                if(adc_task_handle){
                    xTimerStart(xTimeHandle, 0);
                    vTaskResume(adc_task_handle);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void wifiClose(){
    esp_wifi_stop();
    esp_wifi_deinit();
}

esp_err_t create_udp_client(){
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(udp_sock<0){
        ESP_LOGE(TAG, "create socket failed");
        return ESP_FAIL;
    }

    udp_remote_addr.sin_family = AF_INET;
    udp_remote_addr.sin_port = htons(UDP_SERVER_PORT);
    udp_remote_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    ESP_LOGI(TAG, "udp addr: %lu!", udp_remote_addr.sin_addr.s_addr);

    struct timeval tv = {5, 0};
    int ret = setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, (char*) &tv, sizeof(tv));
    if(ret){
        ESP_LOGE(TAG, "Set socket opt failed.");
        closesocket(udp_sock);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool isIpValid(const char *ip_server) {
    char *ipAddr;
    ipAddr = strdup(ip_server);
    // char buff[512];
    // ipAddr = strcpy(buff, ip_server);
    char *token;
    int num;
    int i = 0;
    
    ESP_LOGI(TAG, "ip valid before: %s!", ip_server);
    while ((token = strtok_r(ipAddr, ".", &ipAddr))) {
        i ++;
        num = atoi(token);
        if (num < 0 || num > 255 || i > 4){
            free(ipAddr);
            return false;
        }
    }
    ESP_LOGI(TAG, "ip valid: %s!", ip_server);
    free(ipAddr);
    return i == 4;
}

char* send_and_recv(){
    char *ip_server;
    socklen_t socklen = sizeof(udp_remote_addr);
    char sendBuff[512] = "Here is esp32s3.";
    const char *recvIp = "IP:";
    
    while(true){
        int len = sendto(udp_sock, sendBuff, strlen(sendBuff), 0, (struct sockaddr *) &udp_remote_addr, sizeof(udp_remote_addr));
        if(len>0){
            ESP_LOGI(TAG, "succeed transfer data to %s:%u\n", inet_ntoa(udp_remote_addr.sin_addr), ntohs(udp_remote_addr.sin_port));
        } else{
            ESP_LOGE(TAG, "udp send message failed");
            close(udp_sock);
        }
        // 接收
        char recvBuff[512];
        memset(recvBuff, 0x00, sizeof(recvBuff));
        // recvfrom函数的传入参数与sendto不同，需要socklen的引用
        int recv_len = recvfrom(udp_sock, recvBuff, sizeof(recvBuff), 0, (struct sockaddr *) &udp_remote_addr, &socklen);
        if(recv_len > 0 && strncmp(recvBuff, recvIp, strlen(recvIp)) == 0){
            ESP_LOGI(TAG, "udp recv: %s\n", recvBuff);
            const char *tmp = recvBuff + strlen(recvIp);
            ip_server = strcpy(recvBuff, tmp);
            ESP_LOGI(TAG, "ip tmp server: %s\n", tmp);
            
            // ip_server不对，
            if(!isIpValid(ip_server)){
                ESP_LOGE(TAG, "wrong ip!");
                close(udp_sock);
                return "";
            } else{
                close(udp_sock);
                return ip_server;
            }
        } else if (recv_len<=0){
            ESP_LOGE(TAG, "recvfrom failed");
        }
    }
    return "";
}

int my_ip4addr_aton(const char *cp, ip4_addr_t *addr){
    u32_t val;
    u8_t base;
    char c;
    u32_t parts[4];
    u32_t *pp = parts;

    c = *cp;
    for (;;) {
        /*
        * Collect number up to ``.''.
        * Values are specified as for C:
        * 0x=hex, 0=octal, 1-9=decimal.
        */
        if (!lwip_isdigit(c)) {
            return 0;
        }
        val = 0;
        base = 10;
        if (c == '0') {
            c = *++cp;
            if (c == 'x' || c == 'X') {
                base = 16;
                c = *++cp;
            } else {
                base = 8;
            }
        }
        for (;;) {
            if (lwip_isdigit(c)) {
                if((base == 8) && ((u32_t)(c - '0') >= 8))
                    break;
                val = (val * base) + (u32_t)(c - '0');
                c = *++cp;
            } else if (base == 16 && lwip_isxdigit(c)) {
                val = (val << 4) | (u32_t)(c + 10 - (lwip_islower(c) ? 'a' : 'A'));
                c = *++cp;
            } else {
                break;
            }
        }
        if (c == '.') {
            /*
            * Internet format:
            *  a.b.c.d
            *  a.b.c   (with c treated as 16 bits)
            *  a.b (with b treated as 24 bits)
            */
            if (pp >= parts + 3) {
                ESP_LOGE(TAG, "pp");
                return 0;
            }
            *pp++ = val;
            c = *++cp;
        } else {
            break;
        }
    }
    /*
    * Check for trailing characters.
    */
    if (c != '\0' && !lwip_isspace(c)) {
        return 0;
    }
    /*
    * Concoct the address according to
    * the number of parts specified.
    */
    switch (pp - parts + 1) {

        case 0:
            return 0;       /* initial nondigit */

        case 1:             /* a -- 32 bits */
            break;

        case 2:             /* a.b -- 8.24 bits */
            if (val > 0xffffffUL) {
                return 0;
            }
            if (parts[0] > 0xff) {
                return 0;
            }
            val |= parts[0] << 24;
            break;

        case 3:             /* a.b.c -- 8.8.16 bits */
            if (val > 0xffff) {
                return 0;
            }
            if ((parts[0] > 0xff) || (parts[1] > 0xff)) {
                return 0;
            }
            val |= (parts[0] << 24) | (parts[1] << 16);
            break;

        case 4:             /* a.b.c.d -- 8.8.8.8 bits */
            if (val > 0xff) {
                return 0;
            }
            if ((parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff)) {
                return 0;
            }
            val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
            break;
        default:
            LWIP_ASSERT("unhandled", 0);
            break;
    }
    if (addr) {
        ip4_addr_set_u32(addr, lwip_htonl(val));
    }
    return 1;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret ==  ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xqueue_estimate = xQueueCreate(10, sizeof(struct sensorMessage*));
    if (xqueue_estimate==0){
        ESP_LOGE(TAG, "create xQueueEst failed!");
    }

    wifiInit();
    gpio_init();
    adc_init();
    witJy901sInit();

    create_udp_client();
    const char* ip_server = send_and_recv();
    ip4_addr_t val;
    if(!my_ip4addr_aton(ip_server, &val)){
        ESP_LOGE(TAG, "invalid ip");
    }

    tcpInit(val.addr);
    
    xTimeHandle = xTimerCreate("adc_freq", pdMS_TO_TICKS(1000/100), pdTRUE, (void *)1, timeCallBackTask);
    xTaskCreate(tcp_client_recv_task, "tcp_client_recv", 4096, NULL, 1, NULL);

    // 命令行控制jy901s的方法已被删去，可以去sdk中找到
    // vTaskStartScheduler();
    // wifiClose();  // 不可执行close，否则导致无法连接
}

static void Delayms(uint16_t usMs){
	vTaskDelay(usMs / portTICK_PERIOD_MS);
}

static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum){
	int i;
    for(i = 0; i < uiRegNum; i++){
        switch(uiReg){
            // case AX:
            // case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            // case GX:
            // case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            // case HX:
            // case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            // case Roll:
            // case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
			case TEMP://indicates that the corresponding data has been updated
				s_cDataUpdate |= TEMP_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void){
	int i, iRetry;

	for(i = 0; i < 0x7F; i++){
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do {
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(10);
			if(s_cDataUpdate != 0)
			{
				ESP_LOGI(TAG, "find 0x%02X addr sensor\r\n", i);
				return ;
			}
			iRetry--;
		} while (iRetry);
	}
	ESP_LOGW(TAG, "can not find sensor\r\n");
	ESP_LOGW(TAG, "please check your connection\r\n");
}


