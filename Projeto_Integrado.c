/*
  Projeto: "Sistema de Monitoramento e manejo de plantas domésticas"

  Descrição: Este projeto é um sistema embarcado projetado para monitorar e controlar automaticamente as condições ambientais de plantas domésticas. Ele mede temperatura, umidade do solo e luminosidade, além de permitir a rega automática e manual. Também emite alertas visuais e sonoros caso os parâmetros estejam fora do ideal. Por fim, possui interfaces interativas e de fácil visualização de informações para o usuário.


  Desenvolvedor: Theógenes Gabriel Aráujo de Andrade
  Data: 21/04/2025
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "math.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pio.h"

#include "ws2812b.pio.h"

//Configura o i2c
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define btnA 5  // Botão A
#define btnB 6  //Botão B
#define VERDE 11  //Led Verde
#define RED 13    //Led Vermelho
#define BLUE 12 // Representa a mine Bomba de água
#define buzzer 21  //Buzzer
#define LED_PIN 7   //Pino da matriz de Led
const uint microfone = 28;     // GPIO28 para ADC2 


#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define sw 22              // Controla o botão do joystick

// Paramentros de planta saudável
#define LUZ_MIN 50
#define LUZ_MAX 90
#define TEMP_MIN 20
#define TEMP_MAX 35
#define UMIDADE_MIN 10
#define UMIDADE_MAX 50
#define LED_COUNT 25

//#define INTERVALO_24H 6000
#define INTERVALO_24H 86400000 // 24 horas em milissegundos (1000 * 60 * 60 * 24)

// flags de controle
volatile uint32_t tmp_ant = 0;
volatile uint32_t tmp_ant2 = 0;
volatile uint8_t mov = 0;
volatile uint8_t ap = 0;
volatile uint8_t cont = 3;
volatile uint8_t problemas = 0;
volatile uint8_t cont_molhadas = 0;
volatile uint8_t molhadas = 0;
volatile uint8_t flag_clear = 0;
volatile uint8_t flag_rega = 0;
volatile uint8_t cont2 = 5;

// Variáveis Booleanas de controle de Estado
volatile bool x = false;
volatile bool v = false;
volatile bool z = false;
volatile bool botao_apertado = false; // Variável global para indicar que o botão foi pressionado
volatile bool flag = false;

// Variáveis de Controle de Tempo
volatile uint32_t tempo_anterior = 0;
volatile uint32_t tempo_anterior2 = 0;
volatile uint32_t tempo_anterior3 = 0;
volatile uint32_t ultimo_tempo_apertado = 0; // Tempo do último aperto (para debounce)
volatile uint32_t tempo_decorrido = 0;

const uint amostras_por_segundo = 8000; // Frequência de amostragem (8 kHz)


void draw_tree(ssd1306_t *ssd);                                                                 // Desenha a árvore
void efect_tree(ssd1306_t *ssd, int x, int y);                                                 // Movimento da árvore
void tela_inicial(ssd1306_t *ssd, uint8_t ap, uint16_t adc_value_x, uint16_t adc_value_y, uint16_t luminosidade, bool k);
void init_disp();                                                                              // Inicializa os periféricos
void init_ADC();                                                                               // Inicializa os disp. ADC
void regar(ssd1306_t *ssd, bool val);                                                          // função para rega

const char* avaliarSaude(int luz, int temp, int umidade);                                      // Avalia qual estado de saude
void lumi_temp(ssd1306_t *ssd, uint16_t adc_value_x, uint16_t adc_value_y, bool v);            // Dispara quando a umidade é baixa
void play_sound(uint frequency, uint duration_ms);                                             // Função Sonora de Alerta
void rega_automatica(int adc_value_x, int adc_value_y, int molhada);                           // Habilita a rega automática
void teste(ssd1306_t *ssd, uint16_t adc_value_x, uint16_t adc_value_y);                                                                                  // Teste de ADC (Joystick)
void smile_face(ssd1306_t *ssd, PIO pio, uint sm);                                                                                            // Função Smile
void clear_leds();                                                                              // Limpa(apaga) os Leds da Matriz
void print_leds(PIO pio, uint sm);                                                              // Desenha os Leds na Matriz
void set_led(uint8_t indice, uint8_t r, uint8_t g, uint8_t b);                                  // Seta os Leds que serão ativados

void button_a_isr(uint gpio, uint32_t events){
  uint32_t agora = to_us_since_boot(get_absolute_time());
  if(flag){ //verifica se a rega automática não está acionada
    if (agora - tmp_ant > 200) {  // Debounce de 200 us
      if(!gpio_get(btnA)){
        ap = 0;
        x = 1;
        cont = 4;
        flag_clear = 0;
      }
        tmp_ant = agora;
    }
  }
}

// Função de callback do timer
bool timer_callback(repeating_timer_t *rt) {
  tempo_decorrido += 1000; // Incrementa 1 segundo (1000 ms)
  if (tempo_decorrido >= INTERVALO_24H) {
      cont_molhadas = 0;   // zera a quantidade de molhadas, para inicializar um novo
      tempo_decorrido = 0; // Reinicia a contagem após 24 horas
      flag_rega = 0;       //flag para o controle da rega 
  }
  return true; // Mantém o timer ativo
}



int main() {
    // Inicializações
    PIO pio = pio0; 
    bool ok;
    ok = set_sys_clock_khz(128000, false);

    stdio_init_all();
    init_disp();
    init_ADC();

    //configurações da PIO
    uint offset = pio_add_program(pio, &ws2812b_program);
    uint sm = pio_claim_unused_sm(pio, true);
    ws2812b_program_init(pio, sm, offset, LED_PIN);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line

    ssd1306_t ssd; // Inicializa a estrutura do display
    ssd1306_init(&ssd, 128, 64, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    uint16_t adc_value_x;
    uint16_t adc_value_y;
    
    repeating_timer_t timer;

    gpio_set_irq_enabled_with_callback(btnA, GPIO_IRQ_EDGE_FALL, true, &button_a_isr);
    
    adc_gpio_init(microfone);  // Configura GPIO28 como entrada ADC para o microfone

     // Define o intervalo entre amostras (em microsegundos)
     uint64_t intervalo_us = 1000000 / amostras_por_segundo;

    while (true) {

        adc_select_input(1); // Seleciona o ADC para eixo X(Temperatura). O pino 26 como entrada analógica
        adc_value_x = adc_read();
        adc_select_input(0); // Seleciona o ADC para eixo Y(Umidade). O pino 27 como entrada analógica
        adc_value_y = adc_read();    
        adc_select_input(2);          // Seleciona o canal 2 (GPIO28)
        uint16_t mic_value = adc_read(); // Lê o ADC
        uint16_t intensity = mic_value;
        uint8_t pct_um = (100 - ((adc_value_y * 100) / 4095));
        
        //Verifica se o botão B não foi precisonado
        if(!gpio_get(btnB)){
          x = 0;
          // Limpa a tela
          ssd1306_fill(&ssd, false);
          ssd1306_send_data(&ssd);
          // Vai passando as telas
          ap++;
          
          //Caso a tela seja a sétima volta para a inicial.
          if(ap == 7){
            ap = 1;
          }
          
        }

        //Verifica se o botão SW não foi pressionado
        if(!gpio_get(sw)){

          cont_molhadas++;
          // Permite que a rega automática seja feita somente 1 vez por dia
          if(cont_molhadas == 1){ //se a planta for regada apenas 1 vez
            if(!flag){
              add_repeating_timer_ms(1000, timer_callback, NULL, &timer); // ativa o temporizador o timer para 24h é 1000
              // Led verde indica a rega automática está ativada
              gpio_put(VERDE, 1);
              sleep_ms(1000);
              gpio_put(VERDE, 0);
              flag_rega = 1;
              ap = 6;
            }
            
          }
          //verifica se a rega automática está em OFF
          if(flag == true){ //pressionada no off
            gpio_put(RED, 1);
            sleep_ms(1000);
            gpio_put(RED, 0);
          }
        }
        if(x == 1){
          flag_clear++;
          if(flag_clear == 1){
            ssd1306_fill(&ssd, false);
            ssd1306_send_data(&ssd);
          }
          smile_face(&ssd, pio, sm);
          printf("planta molhada: %d\n", molhadas);
        }
        if(flag_rega == 1){
          rega_automatica(adc_value_x, adc_value_y, cont_molhadas);
        }
        
        //Exibe a tela inicial
        tela_inicial(&ssd, ap, adc_value_x, adc_value_y, intensity,  v);
        
        sleep_ms(195); // sleep para leitura do botão, debounce
    }
}
//Função que controla o teste do joystick
void teste(ssd1306_t *ssd, uint16_t adc_value_x, uint16_t adc_value_y){
    char buffer[10];  // Buffer para conversão de número em string
    ssd1306_draw_string(ssd, "teste adc", 12, 32);
    ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
    
    uint32_t agora1 = to_us_since_boot(get_absolute_time());
    
    if(cont2 > 0){
      if (agora1 - tmp_ant2 > 1000000) {
        cont2--;
        tmp_ant2 = agora1;
      }
      sprintf(buffer, "%d...", cont2); // Converte para string
      ssd1306_draw_string(ssd, buffer, 88, 32);
      ssd1306_send_data(ssd);
    }else{
      ssd1306_fill(ssd, false);
      ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
      uint8_t square_x = (adc_value_x * 120) / 4095;
      uint8_t square_y = (56 - ((adc_value_y * 56) / 4095));
      ssd1306_rect(ssd, square_y, square_x, 8, 8, true, true); // Desenha um retângulo 
      ssd1306_send_data(ssd);
      ssd1306_fill(ssd, false);

    }
}




void tela_inicial(ssd1306_t *ssd, uint8_t ap, uint16_t adc_value_x, uint16_t adc_value_y, uint16_t luminosidade, bool k) {
    char buffer[10];  // Buffer para conversão de número em string

    //mapeamento dos sensores: adc_x, adc_y e microfone
    uint8_t pct_temp = 40;
    uint8_t temp = 32;
    uint8_t pct_um = 60;
    uint8_t lumi = 50;

    if(ap != 0){
      pct_temp = (adc_value_x * 100) / 4095;
      temp = (pct_temp * 64) / 100;
      pct_um = (100 - ((adc_value_y * 100) / 4095) - 1);
      lumi = luminosidade/255;
    }

    sleep_ms(150);
    
    // Como o microfone é impreciso para efeito de demonstração usa-se um valor mínimo de luminosidade para garatnir o funcionamento
    if(lumi < 50){
       lumi += 50; 
    }
    problemas = 0;
    
    //verifica se a umidade não está baixa nem a luminosidade muito alta
    if((pct_um < 30 && x != 1) || lumi > 80){  
      uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
      lumi_temp(ssd, adc_value_x, adc_value_y, k);
      if(tempo_atual - tempo_anterior3 > 200){
        z = !z;
        tempo_anterior3 = tempo_atual;
      }
      play_sound(1000, 500);
      gpio_put(RED, z);
      gpio_put(buzzer, z);
    }else{
      gpio_put(RED, 0);
      gpio_put(buzzer, 0);
      ssd1306_fill(ssd, false);
      if (ap == 2)
      {
        ssd1306_draw_string(ssd, "DADOS COLETADOS", 7, 6);
        ssd1306_vline(ssd, 57, 32, 64, true);
        ssd1306_rect(ssd, 0, 0, 128, 18, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 32, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
        ssd1306_draw_string(ssd, "<temperatura>", 14, 21);
        ssd1306_draw_string(ssd, "ideal:", 7, 34);
        ssd1306_draw_string(ssd, "20-35c", 63, 34);
        ssd1306_hline(ssd, 0, 128, 47, true);
        ssd1306_draw_string(ssd, "atual:", 7, 52);

        sprintf(buffer, "%d", temp); // Converte para string
        ssd1306_draw_string(ssd, buffer, 63, 52);
        ssd1306_draw_string(ssd, "c", 80, 52);

        ssd1306_send_data(ssd);
      }
      else if (ap == 3)
      {
        ssd1306_draw_string(ssd, "DADOS COLETADOS", 7, 6);
        ssd1306_vline(ssd, 57, 32, 64, true);
        ssd1306_rect(ssd, 0, 0, 128, 18, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 32, true, false);
        ssd1306_draw_string(ssd, "<luminosidade>", 10, 21);
        ssd1306_draw_string(ssd, "ideal:", 7, 34);
        ssd1306_hline(ssd, 0, 128, 47, true);
        ssd1306_draw_string(ssd, "50-70%", 63, 34);
        ssd1306_draw_string(ssd, "atual:", 7, 52);
        ssd1306_draw_string(ssd, "%", 80, 52);

        sprintf(buffer, "%d", lumi); // Converte para string
        ssd1306_draw_string(ssd, buffer, 63, 52);
   
        
        //ssd1306_draw_string(ssd, "%", 80, 52);
        ssd1306_send_data(ssd);
      }
      else if (ap == 4)
      {
        ssd1306_draw_string(ssd, "DADOS COLETADOS", 7, 6);
        ssd1306_rect(ssd, 0, 0, 128, 18, true, false);
        ssd1306_vline(ssd, 57, 32, 64, true);
        ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 32, true, false);
        ssd1306_draw_string(ssd, "<umidade>", 26, 21);
        ssd1306_draw_string(ssd, "ideal:", 6, 34);
        ssd1306_draw_string(ssd, "atual:", 7, 52);
        ssd1306_draw_string(ssd, "20-30%", 63, 34);
        ssd1306_hline(ssd, 0, 128, 47, true);

        sprintf(buffer, "%d", pct_um); // Converte para string
        ssd1306_draw_string(ssd, buffer, 63, 52);
        ssd1306_draw_string(ssd, "%", 80, 52);
        ssd1306_send_data(ssd);
      }else if(ap == 5){
        ssd1306_draw_string(ssd, "PAINEL DE SAUDE", 3, 6);
        ssd1306_rect(ssd, 0, 0, 128, 18, true, false);
        ssd1306_rect(ssd, 0, 0, 128, 64, true, false);
        const char* status_saude = avaliarSaude(lumi, temp, pct_um);
        // Exibe o status da saúde da planta
        ssd1306_draw_string(ssd, "Status:", 10, 25);
        ssd1306_draw_string(ssd, status_saude, 10, 34);
        ssd1306_send_data(ssd);
      }else if(ap == 1){
        ssd1306_draw_string(ssd, "REGA AUTOMATICA", 3, 6);
        ssd1306_draw_string(ssd, "ON", 32, 34);
        ssd1306_draw_string(ssd, "OFF", 78, 34);
        if(adc_value_x > 3080){
          flag = true;
        }else if(adc_value_x < 1000){
          flag = false;
        }else{
          if(flag){
            ssd1306_rect(ssd, 25, 74, 30, 22, true, false);
          }else{
            ssd1306_rect(ssd, 25, 24, 30, 22, true, false);
          }
        }
        
        ssd1306_send_data(ssd);
      }else if(ap == 6){
        ssd1306_fill(ssd, false);
        ssd1306_send_data(ssd);
      }else if(ap == 0){
        teste(ssd,adc_value_x, adc_value_y);
      }
    }
}

void rega_automatica(int adc_value_x, int adc_value_y, int molhada){
  uint8_t pct_temp = (adc_value_x * 100) / 4095;
  uint8_t temp = (pct_temp * 64) / 100;
  uint8_t umi = (100 - ((adc_value_y * 100) / 4095) - 1);
  uint8_t lumi = 17;   // parametro pré definido como 17 para simular um horário favorável ex: 8h da manhã
  if(molhada == 1 && flag == false && flag_rega == 1){
    if(umi){  // verifica se a umidade não está no mínimo
      if(lumi >= 15 && lumi <= 20){ //presumindo que a planta esteja em um local pouco insolarado as 8h
        if(temp < 25){  //evita molhar quando estiver quente
          gpio_put(BLUE, 1);
          sleep_ms(4000);
          gpio_put(BLUE, 0);
          flag_rega = 0;
        }
      }
    }
  }
}

// Desenho da árvore
void efect_tree(ssd1306_t *ssd, int x, int y){
  // Desenho da Borda
  ssd1306_rect(ssd, 0, 0, 128, 64, true, false);

 // Tronco da árvore (metade do tamanho original)
 ssd1306_rect(ssd, 37 + x , 62 + y, 4, 8, true, false); // Tronco centralizado

 // Vaso da planta (um pequeno retângulo horizontal abaixo do tronco)
 ssd1306_rect(ssd, 45 + x, 57 + y, 14, 6, true, true); // Vaso
 ssd1306_rect(ssd, 51 + x, 58 + y, 12, 1, true, true); // Vaso: Base arredondada

 // Copa da árvore (metade do tamanho original)
 ssd1306_rect(ssd, 31 + x, 58 + y, 12, 6, true, true); // Base da copa
 ssd1306_rect(ssd, 27 + x, 60 + y, 8, 4, true, true);  // Parte do meio
 ssd1306_rect(ssd, 23 + x, 62 + y, 4, 4, true, true);  // Topo da copa
}

// Exibição para pressinar o botão "A" quando a luminosidade estiver baixa
void regar(ssd1306_t *ssd, bool val){
  uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
  if(tempo_atual - tempo_anterior2 > 3000){
    v = !v;
    tempo_anterior2 = tempo_atual;
    
  }else if(v){
    ssd1306_fill(ssd, false);
    ssd1306_draw_string(ssd, "regue a planta!", 6, 30);
    ssd1306_draw_string(ssd, "<pressione A>", 16, 48);
    ssd1306_send_data(ssd);

  }else{
    draw_tree(ssd);
  }
  
}

//Desenha a árvore
void draw_tree(ssd1306_t *ssd) {
  // Limpa a tela antes de desenhar
  ssd1306_fill(ssd, false);
  
  // Tempo atual em milissegundos
  uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
  // Atualiza a posição da árvore a cada 200ms
  if (tempo_atual - tempo_anterior > 100) {
      mov++;  // Alterna a movimentação
      tempo_anterior = tempo_atual;
  }

  // Define a posição baseada no valor de mov
  uint8_t x = (mov % 2 == 0) ? 4 : 0;
  uint8_t y = (mov % 2 == 0) ? 2 : 0;
  
  // Desenha a árvore na posição ajustada
  efect_tree(ssd, x, y);

  // Exibe o texto fixo na tela
  ssd1306_draw_string(ssd, "ESTOU COM SEDE", 7, 6);

  // Atualiza o display com os novos desenhos
  ssd1306_send_data(ssd);
}

// Inicializa os dipositivos
void init_disp(){
  // inicialização 
  gpio_init(btnA);
  gpio_init(btnB);
  gpio_init(RED);
  gpio_init(VERDE);
  gpio_init(BLUE);
  gpio_init(buzzer);
  gpio_init(sw);

  // Setando a direção
  gpio_set_dir(btnA, GPIO_IN);
  gpio_set_dir(btnB, GPIO_IN);
  gpio_set_dir(sw, GPIO_IN);
  gpio_set_dir(RED, GPIO_OUT);
  gpio_set_dir(VERDE, GPIO_OUT);
  gpio_set_dir(BLUE, GPIO_OUT);
  gpio_set_dir(buzzer, GPIO_OUT);

  gpio_pull_up(btnA);
  gpio_pull_up(btnB);
  gpio_pull_up(sw);

  gpio_put(buzzer, 0);
}

// Função para avaliar a saúde da planta
const char* avaliarSaude(int luz, int temp, int umidade) {
  
  
  if (luz < LUZ_MIN) {
      problemas++;
  } else if (luz > LUZ_MAX) {
      problemas++;
  }
  
  if (temp < TEMP_MIN) {
      problemas++;
  } else if (temp > TEMP_MAX) {
      problemas++;
  }
  
  if (umidade < UMIDADE_MIN) {
      problemas++;
  } else if (umidade > UMIDADE_MAX) {
      problemas++;
  }
  // conta a quantidade de problemas, isto é, parametros fora da margem idela e retorna um status de saúde
  if (problemas == 0) {
      return "saudavel";
  } else if (problemas == 1) {
      return "leve estresse";
  } else if (problemas == 2) {
      return "medio estresse";
  } else {
      return "grave estresse";
  }
}

void lumi_temp(ssd1306_t *ssd, uint16_t adc_value_x, uint16_t adc_value_y, bool v){
  
  uint8_t pct_temp = (adc_value_x * 100) / 4095;
  uint8_t pct_um = (100 - ((adc_value_y * 100) / 4095));

  if(pct_um < 10 ){
    regar(ssd, v);
  }
}

//inicializa dispositivos ADC
void init_ADC(){
  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);  
}

// Emite um som de alerma em situações críticas, exemplo: Luminosidade alta ou a quando a planta estiver com sede.
void play_sound(uint frequency, uint duration_ms) {
  uint delay = 1000000 / (2 * frequency); // Calcula o atraso (microsegundos) para a frequência
  uint cycles = frequency * duration_ms / 1000; // Número de ciclos para a duração

  for (uint i = 0; i < cycles; i++) {
      gpio_put(buzzer, 1); // Liga o buzzer
      sleep_us(delay);         // Espera metade do período
      gpio_put(buzzer, 0); // Desliga o buzzer
      sleep_us(delay);         // Espera a outra metade
  }
}

// Estrutura com os dados de cor e luminozidade para um led
typedef struct{
  uint8_t R;
  uint8_t G;
  uint8_t B;
} led;

// Array que armazena os dados dos 25 leds
led matriz_led[LED_COUNT]; 

// Função que junta 3 bytes com informações de cores e mais 1 byte vazio para temporização
uint32_t valor_rgb(uint8_t B, uint8_t R, uint8_t G){
return (G << 24) | (R << 16) | (B << 8);
}

// Função que configura um led específico
void set_led(uint8_t indice, uint8_t r, uint8_t g, uint8_t b){
  if(indice < 25){
  matriz_led[indice].R = r;
  matriz_led[indice].G = g;
  matriz_led[indice].B = b;
  }
}

// Função de limpar o array de leds
void clear_leds(){
  for(uint8_t i = 0; i < LED_COUNT; i++){
      matriz_led[i].R = 0;
      matriz_led[i].B = 0;
      matriz_led[i].G = 0;
  }
}

// Função que envia os dados do array para os leds via PIO
void print_leds(PIO pio, uint sm){
  uint32_t valor;
  for(uint8_t i = 0; i < LED_COUNT; i++){
      valor = valor_rgb(matriz_led[i].B, matriz_led[i].R,matriz_led[i].G);
      pio_sm_put_blocking(pio, sm, valor);
  }
}

void smile_face(ssd1306_t *ssd, PIO pio, uint sm) {
  molhadas++;
  sleep_ms(10);
  // Acende os olhos
  set_led(18, 0, 100, 0); 
  set_led(16, 0, 100, 0); 
  print_leds(pio, sm);
  sleep_ms(500); 
  // Acende o sorriso progressivamente
  int smile[5] = {9, 1, 2, 3, 5};
  for(int i = 0; i < 5; i++) {
      set_led(smile[i], 0, 100, 0);
      print_leds(pio, sm);
      sleep_ms(100); 
  }
  sleep_ms(1000); 
  // Pisca o olho direito
  set_led(18, 0, 0, 0); 
  print_leds(pio, sm);
  sleep_ms(700); 
  set_led(18, 0, 100, 0); 
  print_leds(pio, sm);

  sleep_ms(500);
  clear_leds();
  print_leds(pio, sm);

  ap = 1;
  x = 0;
}