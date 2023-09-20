#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"

/*------------------------->DEFINIÇÃO DAS PORTAS DE SAÍDA<------------------------------*/
/*#define GPIO_OUTPUT_IO_0    CONFIG_GPIO_OUTPUT_0    GPIO_OUTPUT_IO_0 = 0000000000000000000001000000000000000000
#define GPIO_OUTPUT_IO_1    CONFIG_GPIO_OUTPUT_1    GPIO_OUTPUT_IO_1 = 0000000000000000000010000000000000000000    
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
*/
#define GPIO_OUTPUT_IO_0 2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0)) //Vetor dos pinos de saída

/*----------------------->DEFINIÇÃO DAS PORTAS DE ENTRADA<------------------------------*/
/*#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0     GPIO_INPUT_IO_0 = 0000000000000000000000000000000000010000
#define GPIO_INPUT_IO_1     CONFIG_GPIO_INPUT_1     GPIO_INPUT_IO_1 = 0000000000000000000000000000000000100000
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
*/
#define GPIO_INPUT_IO_0 21
#define GPIO_INPUT_IO_1 22
#define GPIO_INPUT_IO_2 23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))  //Vetor dos pinos de entrada


/*-------------------------->CRIAÇÃO DA FLAG DE INTERRUPÇÃO<---------------------------*/
#define ESP_INTR_FLAG_DEFAULT 0

/*DECLARANDO AS TAGS*/
static const char* TAG_INFO_SYS_01 = "DADOS_SISTEMA";
static const char* TAG_BOT_LED_02 = "BOT_LED";
static const char* TAG_TIMER = "TIMER";

/*---------------------------------->DECLARANDO A FILA<-------------------------------*/
static QueueHandle_t gpio_evt_queue = NULL; // do IO
static QueueHandle_t queue_timer = NULL; 

typedef struct {    // do timer
    uint64_t event_count;
    uint64_t alarm_value;
} queue_element_TIMER;

gptimer_handle_t gptimer = NULL; // do TIMER

/*--------------------------------->INTERRUPÇÃO DO GPIO<-------------------------------------*/
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    //Enviando endereço do pino que gerou a interrupção para a fila
}

/*-------------------------->STRUCT DE UMA TASK A SER CRIADA<-----------------------*/
bool estado = false;
static void gpio_task_button(void* arg) //Tarefa associada aos botões
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            /*ESTRUTURA CONDICIONAL PARA ACIONAMENTO DO LED*/
            if (gpio_get_level(GPIO_INPUT_IO_0)){
                estado = true;
                gpio_set_level(GPIO_OUTPUT_IO_0,estado);
            }
            if (gpio_get_level(GPIO_INPUT_IO_1)){
                estado = false;
                gpio_set_level(GPIO_OUTPUT_IO_0,estado);
            }
            if (gpio_get_level(GPIO_INPUT_IO_2)){
                gpio_set_level(GPIO_OUTPUT_IO_0,!estado);
            }
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            ESP_LOGI(TAG_BOT_LED_02,"GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num)); //Printando os dados do pino e o nível lógico associado
            vTaskDelay(10);
        }
    }
}
/*------------------>TASK DO TIMER<------------------------*/
int hora = 0, minuto = 0, segundo = 0;

static void timer_task(void* arg) //Tarefa associada ao timer
{
    queue_element_TIMER element; //handle do timer
    for(;;) {
        if(xQueueReceive(queue_timer, &element, portMAX_DELAY)) {       
            if(segundo == 60){
                minuto ++;
                segundo = 0;
            }
            if(minuto == 60){
                hora ++;
                minuto = 0;
            }
            if(hora == 24){
                hora = 0;
                minuto = 0;
                segundo = 0;
            }
            segundo ++;
            ESP_LOGI(TAG_TIMER,"%d:%d:%d\n", hora, minuto, segundo);
        }
    }
}


/*---------------------->CRIAÇÃO DA CALLBACK DO TIMER - ESTRUTURA<---------------------------*/
static bool IRAM_ATTR callback_timer_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
   
    // Retrieve count value and send to queue
    queue_element_TIMER element = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(queue_timer, &element, &high_task_awoken);
    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 1000000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

void app_main(void)
{
    /*----------------------->*PRINT DOS DADOS DO CHIP - PT1<----------------------*/

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG_INFO_SYS_01, "This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
           CONFIG_IDF_TARGET,
            chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;

    ESP_LOGI(TAG_INFO_SYS_01,"silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGI(TAG_INFO_SYS_01,"Get flash size failed");
        return;
    }

    /*----------------------->*CONFIGURAÇÃO DO GPIO - PT2<-------------------------*/

    /*CONFIGURANDO AS SAÍDAS*/ 
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    /*CARREGAR CONFIGURAÇÕES NOS PINOS DE SAÍDA*/
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    /*------>INTERRUPÇÃO NA BORDA DE DESCIDA PARA TODOS OS PINOS DE SAÍDA<--------*/
    /*CONFIGURANDO AS ENTRADAS*/ 
    //interrupt of fallen edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE; //Interrupção na boda de descida
    //bit mask of the pins, use GPIO 0 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    /*CARREGAR CONFIGURAÇÕES NOS PINOS DE ENTRADA*/
    gpio_config(&io_conf);

    /*------------------------------->CRIAÇÃO DA FILA<----------------------------*/
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));


    /*---------------------->INSTALANDO A INTERRUPÇÃO<--------------------------*/
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    /*---------->ATRIBUIÇÃO DA INTERRUPÇÃO AOS PINOS DE ENTRADA<----------------*/
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);

    /*----------------------->CONFIGURAÇÃO DO TIMER - PT3<-------------------------*/    
        /*CRIAÇÃO DA FILA DO TIMER*/
    queue_timer = xQueueCreate(10, sizeof(queue_element_TIMER));
    if (!queue_timer) {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }

    /*INICIALIZAÇÃO D0 TIMER*/
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,//Contagem UP
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    /*REGISTRO (INICIALIZACAO) DO CALLBACK*/
    gptimer_event_callbacks_t cbs = {
        .on_alarm = callback_timer_1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    /*HABILITAÇÃO DO TIMER*/
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    /*CONFIGURAÇÃO DO ALARME (PERÍODO DE CONTAGEM) - QUE VAI GERAR A INTERRUPCAO*/
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 10000000, // period = 1s
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    /*LOOP DA MAIN (PODE SER ELIMINADO)*/

    /*----------->CRIAÇÃO DE UMA TASK ATRIBUÍDA À STRUCT CRIADA<-----------------*/
    //start gpio task
    xTaskCreate(gpio_task_button, "gpio_task_button", 2048, NULL, 10, NULL);

    //start timer task
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 9, NULL);
}
