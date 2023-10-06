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
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/semphr.h"


/****************************************************************************************/
/*---------------------------------->VARIÁVEIS GLOBAIS<---------------------------------*/
/****************************************************************************************/

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

/*---------->-VARIÁVEIS GLOBAIS COM OS ATRIBUTOS DE CONFIGURAÇÃO DO PWM<---------------*/
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (16) // Define the output GPIO 16 (VERDE) LED (17 - AZUL)
#define PWM_OUTPUT_IO           (32) // Define the output PWM (PODE SER 32 OU 33)
#define LEDC_CHANNEL            LEDC_CHANNEL_0 //Configuração do canal do LED
#define PWM_CHANNEL             LEDC_CHANNEL_1 //Configuração do canal do PWM
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


/****************************************************************************************/
/*-------------------------->CRIAÇÃO DA FLAG DE INTERRUPÇÃO<---------------------------*/
/****************************************************************************************/
#define ESP_INTR_FLAG_DEFAULT 0

/****************************************************************************************/
/*-------------------------------->DECLARANDO AS TAGS<----------------------------------*/
/****************************************************************************************/
static const char* TAG_INFO_SYS_01 = "DADOS_SISTEMA";
static const char* TAG_BOT_LED_02 = "BOT_LED";
static const char* TAG_TIMER = "TIMER";
static const char* TAG_PWM = "PWM";

/****************************************************************************************/
/*---------------------------------->DECLARANDO A FILA<---------------------------------*/
/****************************************************************************************/
static QueueHandle_t gpio_evt_queue = NULL; // do IO
static QueueHandle_t queue_timer = NULL; //do Timer
static QueueHandle_t queue_pwm   = NULL; //do PWM

//Struct para passagem de parâmetros do TIMER para a fila
typedef struct { 
    uint64_t event_count;
    uint64_t alarm_count;
} queue_element_TIMER;

//Handle do TIMER
gptimer_handle_t gptimer = NULL; //Desclaração do handle 

//Handle PWM
typedef struct { 
    bool mode_auto;
    int16_t pwm_queue;
} PWM_elements_t;


//Handle do semaforo
static SemaphoreHandle_t semaphore_pwm = NULL;// Semaforo

/****************************************************************************************/
/*-------------------------->ESTRUTURAS E ROTINAS DAS TASKS<----------------------------*/
/****************************************************************************************/

/*------------------------------->TASK DO GPIO<----------------------------------------*/
//Declaração de variáveis
//bool estado = false;

//Task propriamente dita
static void gpio_task_button(void* arg) //Tarefa associada aos botões
{
    char modo;

    uint32_t io_num;

    PWM_elements_t elementos_PWM; //Elemento de passagem para a fila do PWM

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            switch (io_num)
            {
                case 21: //21
                    modo = 'A';
                    //estado = true;
                    
                    gpio_set_level(GPIO_OUTPUT_IO_0,1);

                    elementos_PWM.mode_auto = true;
                    elementos_PWM.pwm_queue = 0;
                    
                    ESP_LOGI(TAG_PWM,"Modo: %c,Duty cycle: %d\n", modo,elementos_PWM.pwm_queue); //Printando os dados do o duty cycle atual
                    break;
                
                case 22: //22
                    modo = 'M';
                    //estado = false;
                    gpio_set_level(GPIO_OUTPUT_IO_0,1);

                    elementos_PWM.mode_auto = false;
                    elementos_PWM.pwm_queue = 0;
                    ESP_LOGI(TAG_PWM,"Modo: %c,Duty cycle: %d\n", modo,elementos_PWM.pwm_queue);
                    break;
                
                case 23: //23
                    gpio_set_level(GPIO_OUTPUT_IO_0,1);

                    if(!elementos_PWM.mode_auto){
                        elementos_PWM.pwm_queue = elementos_PWM.pwm_queue + 250;

                        if(elementos_PWM.pwm_queue >= 4095*2){
                           elementos_PWM.pwm_queue=0; 
                        }

                        ESP_LOGI(TAG_PWM,"Modo: %c,\tDuty cycle: %d\n", 'M',elementos_PWM.pwm_queue);
                    }
                    
                    //ESP_LOGI(TAG_PWM,"Modo: %c, Duty cycle: %d\n", modo, elementos_PWM.pwm_queue);
                    break;
            }
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            //ESP_LOGI(TAG_BOT_LED_02,"GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num)); //Printando os dados do pino e o nível lógico associado
            //Mandar dados para a fila do PWM
            xQueueSendFromISR(queue_pwm, &elementos_PWM, NULL);           
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }
}
/*------------------------------->TASK DO TIMER<----------------------------------------*/
//Declaração de variáveis
int hora = 0, minuto = 0, segundo = 0, cemms = 0;

queue_element_TIMER element; //Parâmetro da fila, declarado com tipo igual à struct criada

//Task propriamente dita
static void timer_task(void* arg) //Tarefa associada ao timer
{
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 100));
    
    uint64_t count_timer;

    ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &count_timer));

    for(;;) {
        if(xQueueReceive(queue_timer, &element, portMAX_DELAY)) {       
            if(segundo == 59){
                minuto ++;
                segundo = 0;
            }
            if(minuto == 59){
                hora ++;
                minuto = 0;
            }
            if(hora == 24){
                hora = 0;
                minuto = 0;
                segundo = 0;
            }
            cemms ++;

            if(cemms == 10){
                segundo ++; //10x 
                cemms = 0;
                ESP_LOGI(TAG_TIMER,"%d:%d:%d; Timer = %llu; Alarm = %llu\n", hora, minuto, segundo, element.event_count, element.alarm_count);
            }
            xSemaphoreGive(semaphore_pwm);//Semaforo para PWM
        }
    }
}


/*------------------------------->TASK DO PWM<----------------------------------------*/
//Função de configuração do PWM
static void example_PWMledc_init(void)
{
    // Preparação da configuração do TIMER DO PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); //Carregando a configuração do timer do PWM

    // Preparação da configuração do canal PWM do LED
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));//Carregando a configuração do canal do LED

    // Preparação da configuração do canal PWM da saída
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));//Carregando a configuração do PWM

}

PWM_elements_t elementos_PWM_r; //Elemento de recebimento da fila do PWM

int i = 0;

//Task propriamente dita
static void PWM_task(void* arg) //Tarefa associada ao PWM
{

    // Set the LEDC peripheral configuration
    example_PWMledc_init();

    while(1){
        if(xSemaphoreTake(semaphore_pwm, portMAX_DELAY)){
           if(xQueueReceive(queue_pwm, &elementos_PWM_r, 10/portTICK_PERIOD_MS)){ 
                i = 0;
            }
            switch (elementos_PWM_r.mode_auto){
                case true:
                    
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, PWM_CHANNEL,i));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, PWM_CHANNEL));

                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,i));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

                    if(i >= 4095*2){
                           i = 0; 
                    }
                    i = i + 100;
                    ESP_LOGI(TAG_PWM,"Modo: %c,\tDuty cycle: %d\n", 'A',i);
                    break;

                case false:
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, PWM_CHANNEL,elementos_PWM_r.pwm_queue));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, PWM_CHANNEL));

                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,elementos_PWM_r.pwm_queue));
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

                    break;
            }     
        }
    }
}

/****************************************************************************************/
/*------------------------------------>INTERRUPÇÕES<------------------------------------*/
/****************************************************************************************/

/*--------------------------------->INTERRUPÇÃO DO GPIO<--------------------------------*/
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    //Enviando endereço do pino que gerou a interrupção para a fila
}

/*-------------------->CRIAÇÃO DA CALLBACK DO TIMER - ESTRUTURA<-------------------------*/
static bool IRAM_ATTR callback_timer_1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
   
    // Retrieve count value and send to queue
    queue_element_TIMER element = {
        .event_count = edata->count_value,
        .alarm_count = edata->alarm_value,
    };
    xQueueSendFromISR(queue_timer, &element, &high_task_awoken);
    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = edata->alarm_value + 1000000, // alarm in next 1s
        //.flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

/****************************************************************************************/
/*---------------------------------------->MAIN<----------------------------------------*/
/****************************************************************************************/

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
        .resolution_hz = 10000000, // 10 MHz, 1 tick = 0.1us
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
        .alarm_count = 1000000, // 6 zeros // period = 100 mili fixo
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    /*----------------------->CONFIGURAÇÃO DO PWM - PT4<-------------------------*/

    semaphore_pwm = xSemaphoreCreateBinary(); // criação do semaphore binario
    //xSemaphoreGive(semaphore_pwm); //Função na TASK Timer para sincronizar com a task 
    //xSemaphoreTake(semaphore_pwm, portMAX_DELAY);

    queue_pwm = xQueueCreate(10, sizeof(PWM_elements_t));
    if (!queue_pwm) {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }

    /*LOOP DA MAIN (PODE SER ELIMINADO)*/

    /*----------->CRIAÇÃO DE UMA TASK ATRIBUÍDA À STRUCT CRIADA<-----------------*/
    //start gpio task
    xTaskCreate(gpio_task_button, "gpio_task_button", 2048, NULL, 10, NULL);

    //start timer task
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 9, NULL);

    //start PWM task
    xTaskCreate(PWM_task, "PWM_task", 2048, NULL, 8, NULL);
}
