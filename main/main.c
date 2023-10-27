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
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_system.h"
#include "driver/uart.h"


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
#define PWM_OUTPUT_IO           (33) // Define the output PWM (PODE SER 32 OU 33)
#define LEDC_CHANNEL            LEDC_CHANNEL_0 //Configuração do canal do LED
#define PWM_CHANNEL             LEDC_CHANNEL_1 //Configuração do canal do PWM
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

/*---------->-VARIÁVEIS GLOBAIS COM OS ATRIBUTOS DE CONFIGURAÇÃO DO ADC<---------------*/
//Definição de qual ADC utilizar e do canal correspondente
#if CONFIG_IDF_TARGET_ESP32
    #define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_3
#endif

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11
static int adc_raw[2][10];
static int adc_voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

/*---------->-VARIÁVEIS GLOBAIS COM OS ATRIBUTOS DE CONFIGURAÇÃO DO UART<---------------*/
#define TXD_PIN     (GPIO_NUM_5)
#define RXD_PIN     (GPIO_NUM_4)

/****************************************************************************************/
/*-------------------------->CRIAÇÃO DA FLAG DE INTERRUPÇÃO<---------------------------*/
/****************************************************************************************/
#define ESP_INTR_FLAG_DEFAULT 0

/****************************************************************************************/
/*-------------------------------->DECLARANDO AS TAGS<----------------------------------*/
/****************************************************************************************/
static const char* TAG_INFO_SYS_01 = "DADOS_SISTEMA";
//static const char* TAG_BOT_LED_02 = "BOT_LED";
static const char* TAG_TIMER = "TIMER";
static const char* TAG_PWM = "PWM";
static const char* TAG_ADC = "ADC";
static const int RX_BUF_SIZE = 1024;

/****************************************************************************************/
/*---------------------------------->DECLARANDO A FILA<---------------------------------*/
/****************************************************************************************/
static QueueHandle_t gpio_evt_queue = NULL; // do IO
static QueueHandle_t queue_timer = NULL; //do Timer
static QueueHandle_t queue_pwm   = NULL; //do PWM
static QueueHandle_t queue_adc   = NULL; //do ADC


/****************************************************************************************/
/*------------------------------->STRUCTS DE PARÂMETRO<---------------------------------*/
/****************************************************************************************/

//Struct para passagem de parâmetros do TIMER para a fila
typedef struct { 
    uint64_t event_count;
    uint64_t alarm_count;
} queue_element_TIMER;

//Struct para passagem de parâmetros do PWM para a fila
typedef struct { 
    bool mode_auto;
    int16_t pwm_queue;
} PWM_elements_t;

//Struct para passagem de parâmetros do ADC para a fila
typedef struct { 
    int valor_raw;
    int valor_volt;
} ADC_elements_t;

/****************************************************************************************/
/*--------------------------------------->HANDLES<--------------------------------------*/
/****************************************************************************************/

//Handle do TIMER
gptimer_handle_t gptimer = NULL; //Desclaração do handle do timer

//Handle do ADC
adc_oneshot_unit_handle_t adc1_handle;

//Handle dos semaforos
static SemaphoreHandle_t semaphore_pwm = NULL;// Semaforo do PWM
static SemaphoreHandle_t semaphore_adc = NULL;// Semaforo do ADC

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
int horareal = 0, minutoreal = 0, segundoreal = 0;

queue_element_TIMER element; //Parâmetro da fila, declarado com tipo igual à struct criada

//Elemento para recebimento de dados do ADC
ADC_elements_t elementos_ADC_r;

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
                ESP_LOGI(TAG_TIMER,"%d:%d:%d; Timer = %llu; Alarm = %llu\n", hora+horareal, minuto+minutoreal, segundo+segundoreal, element.event_count, element.alarm_count);
                    if(xQueueReceive(queue_adc, &elementos_ADC_r, (10/portTICK_PERIOD_MS)))
                    {
                        ESP_LOGI(TAG_ADC,"Valor em bits: %d | Valor em Volts: %d\n",elementos_ADC_r.valor_raw,elementos_ADC_r.valor_volt);
                    }
            }
            xSemaphoreGive(semaphore_pwm);//Semaforo para PWM
            xSemaphoreGive(semaphore_adc);//Semaforo para ADC
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

/*-------------------------------->TASK DO ADC<-----------------------------------------*/
//Task propriamente dita
static void ADC_task(void* arg) //Tarefa associada ao ADC
{
    //Elemento para passagem de dados
    ADC_elements_t elementos_ADC;

    //Inicialização do ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));//"set" da inicialização

    //Configuração do ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));//Carregando a configuração

    //Calibração inicial do ADC
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);    
    
    while (1)
    {
        if(xSemaphoreTake(semaphore_adc, portMAX_DELAY)){
            //Leitura do ADC
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
            elementos_ADC.valor_raw = adc_raw[0][0];
            //printf("RAW: %i\n", adc_raw[0][0]);

            //Calibração -> Valor na unidade desejada
            if (do_calibration1) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &adc_voltage[0][0]));
                elementos_ADC.valor_volt = adc_voltage[0][0];
                //printf("ADC: %i\n", adc_voltage[0][0]);
            }
        }
            //Encaminhando dados para a fila do ADC
            xQueueSendToBack(queue_adc, &elementos_ADC, NULL);  
    }
}

/*-------------------------------->TASK DO UART<-----------------------------------------*/
void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

///////////////////////////////////////////////////// TASK TX
int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        //sendData(TX_TASK_TAG, "Hello world\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

///////////////////////////////////////////////////// TASK RX
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "[OK] - Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            
            horareal = ((data[0]-0x30)*10)+(data[1]-0x30);
            minutoreal = ((data[2]-0x30)*10)+(data[3]-0x30);
            segundoreal = ((data[4]-0x30)*10)+(data[5]-0x30);

        }
    }
    free(data);
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

    
    /*-------------------------------->ADC - PT5<---------------------------------*/
    
    //Criação da fila do ADC
    queue_adc = xQueueCreate(10, sizeof(ADC_elements_t));

    semaphore_adc = xSemaphoreCreateBinary(); // criação do semaphore binario

    /*LOOP DA MAIN (PODE SER ELIMINADO)*/

    /*----------->CRIAÇÃO DE UMA TASK ATRIBUÍDA À STRUCT CRIADA<-----------------*/
    //start gpio task
    xTaskCreate(gpio_task_button, "gpio_task_button", 2048, NULL, 10, NULL);

    //start timer task
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 9, NULL);

    //start PWM task
    xTaskCreate(PWM_task, "PWM_task", 2048, NULL, 8, NULL);

    //start ADC task
    xTaskCreate(ADC_task, "ADC_task", 2048, NULL, 7, NULL);


    /*-------------------------------->UART - PT6<---------------------------------*/
    init();
    //start RX UART task
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //start TX UART task
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
/*//////////////////////////////////////////////////////////////////////////////////////*/
/*------------------------->FUNÇÃO DE CALIBRAÇÃO DO ADC<--------------------------------*/
/*//////////////////////////////////////////////////////////////////////////////////////*/

static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_ADC, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG_ADC, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG_ADC, "Invalid arg or no memory");
    }

    return calibrated;
}
