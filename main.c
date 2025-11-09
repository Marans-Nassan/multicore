#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/bmp280.h"
#include "lib/aht20.h"
#include "hardware/watchdog.h"


#define i2c_port_a i2c0
#define i2c_sda_a 0
#define i2c_scl_a 1
#define press_nivel_mar 101325.0
#define bot_a 5
#define i2c_port_b i2c1
#define i2c_sda_b 14
#define i2c_scl_b 15
#define i2c_endereco 0x3c
#define buzzer_a 21

ssd1306_t ssd;
AHT20_Data data;
int32_t raw_temp_bmp;
int32_t raw_pressure;
struct bmp280_calib_param params;
double altitude = 0;
int temperature = 0;
int nivel_atual = 0;
float humidity = 0;
float media = 0;
char str_tmp1[5]; 
char str_alt[5];   
char str_tmp2[5];  
char str_umi[5]; 
typedef struct checks{
    bool error1;
    bool error2;
    bool error3;
} checks; 
checks c = {false, false, false};

uint8_t slice = 0;
typedef struct {
    float dc;           // wrap value do PWM
    float div;          // divisor de clock
    bool alarm_state;   // estado da sirene
    bool alarm_react;   // permite reativar alarme
    alarm_id_t alarm_pwm;
} pwm_struct;
pwm_struct pw = {7812.5, 32.0, false, true, 0};

mutex_t data_mutex;

void core1(void); // Função executada no segundo núcleo (core1) do RP2040. Responsável por atualizar o display OLED continuamente.
void init_botoes(void); // Inicializa os botões como entrada digital com pull-up interno.
void init_i2c0(void); // Inicializa a interface I2C0 (usada pelos sensores BMP280 e AHT20).
void init_i2c1(void); // Inicializa a interface I2C1 (usada pelo display OLED).
void init_oled(void); // Inicializa o display OLED SSD1306.
void init_bmp280(void); // Inicializa o sensor BMP280, incluindo leitura dos parâmetros de calibração.
void init_aht20(void); // Inicializa o sensor AHT20, incluindo reset e configuração inicial.
void pwm_setup(void); // Configura o PWM do buzzer (frequência e wrap).
void pwm_on(uint8_t duty_cycle); // Ativa o PWM do buzzer com duty cycle especificado.
void pwm_off(void); // Desliga o PWM do buzzer e garante que não haja vazamento de sinal.
void gpio_irq_handler(uint gpio, uint32_t events); // Handler de interrupção para o botão. É usado para desligar o alarme quando bot_a é pressionado.
double calcular_altitude(double pressure); // Calcula a altitude estimada em metros a partir da pressão em Pa.
int64_t variacao_temp(alarm_id_t, void *user_data); // Callback de alarme usado pelo PWM quando a temperatura está fora da faixa. Ativa o buzzer e retorna 1 para manter o alarme ativo.

int main(){
    stdio_init_all();
    if (watchdog_caused_reboot()) { 
        printf("Reinciado pelo Watchdog!\n");
    }
    watchdog_enable(4000, true); //Sistema watchdog contra falhas. Caso o programa trave por mais de 4 segundos sem o watchdog_update resetar, o programa é forçado a reiniciar.
    mutex_init(&data_mutex);
    multicore_launch_core1(core1);
    init_botoes();
    init_i2c0();
    init_bmp280();
    init_aht20();
    pwm_setup();
    
    gpio_set_irq_enabled_with_callback(bot_a, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   
    while (true) {
        // Leitura do BMP280
        bmp280_read_raw(i2c_port_a, &raw_temp_bmp, &raw_pressure);
        temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Checagem BMP280: valores plausíveis
        mutex_enter_blocking(&data_mutex);
        if (temperature < -4000 || temperature > 8500 || pressure < 30000 || pressure > 110000) {
            printf("Erro na leitura do BMP280!\n");
            c.error2 = false;
            sprintf(str_tmp1, "---");
            sprintf(str_alt, "---");
        } else {
            c.error2 = true;
            altitude = calcular_altitude(pressure);
            printf("Pressao = %.3f kPa\n", pressure / 1000.0);
            printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
            printf("Altitude estimada: %.2f m\n", altitude);
            sprintf(str_tmp1, "%.1fC", temperature / 100.0);
            sprintf(str_alt, "%.0fm", altitude);
        }

        // Leitura do AHT20
        if (aht20_read(i2c_port_a, &data)) {
            c.error3 = true;
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
            sprintf(str_tmp2, "%.1fC", data.temperature);
            sprintf(str_umi, "%.1f%%", data.humidity);
        } else {
            printf("Erro na leitura do AHT10!\n\n\n");
            c.error3 = false;
        }
        if(!c.error2 && !c.error3) {c.error1 = true;}
        else {c.error1 = false;}

        mutex_exit(&data_mutex);
 
        if ((temperature < 1000 || temperature > 4000) && !pw.alarm_state && pw.alarm_react) {
            pw.alarm_pwm = add_alarm_in_ms(3000, variacao_temp, NULL, false);
            pw.alarm_state = true;
            pw.alarm_react = false;
        }
        if (temperature >= 1000 && temperature <= 4000) {
            if (pw.alarm_state) {
                pwm_off();
                pw.alarm_state = false;
                cancel_alarm(pw.alarm_pwm);
            }
            pw.alarm_react = true;
        }
        media = ((temperature/100.0) + data.temperature)/2 ;

        sleep_ms(1);
        watchdog_update();
    }
}

void core1(void) {
    sleep_ms(5000);
    bool cor = true;
    init_i2c1();
    init_oled();
    
    while(true) {
        ssd1306_fill(&ssd, !cor);                          
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);            
        ssd1306_draw_string(&ssd, " Meteorologia", 8, 6);  
        ssd1306_draw_string(&ssd, "BMP280  AHT10", 10, 28); 
        ssd1306_line(&ssd, 63, 25, 63, 60, cor);           
 
        mutex_enter_blocking(&data_mutex);        
        if(c.error2) {
            ssd1306_draw_string(&ssd, str_tmp1, 14, 41);             
            ssd1306_draw_string(&ssd, str_alt, 14, 52);             
        } else {
            ssd1306_draw_string(&ssd, "---", 14, 41);
            ssd1306_draw_string(&ssd, "---", 14, 52);
        }
        
        if(c.error3) {
            ssd1306_draw_string(&ssd, str_tmp2, 73, 41);             
            ssd1306_draw_string(&ssd, str_umi, 73, 52);
        } else {
            ssd1306_draw_string(&ssd, "---", 73, 41);
            ssd1306_draw_string(&ssd, "---", 73, 52);
        }
        mutex_exit(&data_mutex);
        ssd1306_send_data(&ssd);
        sleep_ms(1000);
    }
}

void init_botoes(void) {
    for (uint8_t botoes = 5; botoes < 7; botoes++){
        gpio_init(botoes);
        gpio_set_dir(botoes, GPIO_IN);
        gpio_pull_up(botoes);
    }
}

void init_i2c0(void) {
    i2c_init(i2c_port_a, 400*1000);
    for(uint8_t init_i2c0 = 0 ; init_i2c0 < 2; init_i2c0 ++){
        gpio_set_function(init_i2c0, GPIO_FUNC_I2C);
        gpio_pull_up(init_i2c0);
    }
}

void init_i2c1(void) {
    i2c_init(i2c_port_b, 400*1000);
    for(uint8_t init_i2c1 = 14 ; init_i2c1 < 16; init_i2c1 ++){
        gpio_set_function(init_i2c1, GPIO_FUNC_I2C);
        gpio_pull_up(init_i2c1);
    }
}

void init_oled(void) {
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, i2c_endereco, i2c_port_b);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void init_bmp280(void) {
    bmp280_init(i2c_port_a);
    bmp280_get_calib_params(i2c_port_a, &params);
}

void init_aht20(void) {
    aht20_reset(i2c_port_a);
    aht20_init(i2c_port_a);
}

void pwm_setup(void) {
    gpio_set_function(buzzer_a, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(buzzer_a);
    pwm_set_clkdiv(slice, pw.div);
    pwm_set_wrap(slice, pw.dc);
    pwm_set_enabled(slice, false);
}

void pwm_on(uint8_t duty_cycle) {
    gpio_set_function(buzzer_a, GPIO_FUNC_PWM);
    pwm_set_gpio_level(buzzer_a, (uint16_t)((pw.dc * duty_cycle) / 100));
    pwm_set_enabled(slice, true);
}

void pwm_off(void) {
    pwm_set_enabled(slice, false);
    gpio_set_function(buzzer_a, GPIO_FUNC_SIO);
    gpio_put(buzzer_a, 0);
}

void gpio_irq_handler(uint gpio, uint32_t events) {
    uint64_t current_time = to_ms_since_boot(get_absolute_time());
    static uint64_t last_time_a = 0, last_time_b = 0;
    if(gpio == bot_a && (current_time - last_time_a > 300)) {
        pwm_off();
        pw.alarm_state = false;
        pw.alarm_react = false;
        cancel_alarm(pw.alarm_pwm);
        last_time_a = current_time;
    }
}

double calcular_altitude(double pressure) {
    return 44330.0 * (1.0 - pow(pressure / press_nivel_mar, 0.1903));
}

int64_t variacao_temp(alarm_id_t, void *user_data) {
    pwm_on(50);
    return 1;
}