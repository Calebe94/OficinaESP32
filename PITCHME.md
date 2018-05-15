# ESP32
## e Protocolo I2C

<!-- ### Oficina do IOT Maker Hub -->

---

### Informações do Instrutor:

<br>
Edimar Calebe Castanho
<br>
Aluno de Engenharia da Computação do 4º ano
- @fa[github gp-contact](Calebe94)
- @fa[reddit gp-contact](Calebe94)
- @fa[fa-telegram gp-contact](calebe94)

---

### Oque é o ESP32?

<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRoMf0HGjAsmxbRkZ37B_Uqslcc58JVv2cI_zdW_hcHCL7JLShf" style="width: 150px;"/>
* System on a Chip (SoC) - Microcontrolador;
* Baixo Custo;
* "Baixo Consumo";
* com Wi-Fi embutido;
* Bluetooth 4.2 e BLE embutido;
* Dual-core;

---

### Quem o Projetou?
<img src="https://www.exploreembedded.com/wiki/images/thumb/f/f0/0_ESP32_chip.jpg/447px-0_ESP32_chip.jpg" style="width: 150px;" />


* Espressif Systems (Shanghai - China);
* Empresa apenas projeta os CI's(Não os fabrica);
* Mesma empresa do ESP8266;
* Desenvolve soluções para IOT;

---

### Características de Hardware:

* Microprocessador dual-core Tensilica Xtensa LX6(32 bit);
* Frequência de Clock de 80Mhz até 240MHz;
* Co-processador de baixíssimo consumo
  * Permite fazer conversões ADC em **deep sleep**
* Comunicações sem fio:
  * Wi-Fi: 802.11b/g/n/e/i (2.4GHz)
  * Bluetooth: v4.2 e BLE

---

### Características de Hardware:
#### Memória Interna:

  * ROM: 448 KiB (Usada para Boot e funções do core);
  * SRAM: 520 KiB (Para dados e instruções);
  * RTC fast SRAM: 8 KiB (para armazenamento e CPU durante o boot do RTC);
  * RTC slow SRAM: 8 KiB (Utilizado pelo co-processador durante o deep-sleep);
  * Flash: entre 0 e 4 MiB (Suporta até 16MiB);

---

### Características de Hardware:
#### Periféricos IO:

* 36 GPIOs
  - 10 pinos com Touch capacitivo;
  - Conversores 18 pinos ADCs e 2 pinos DACs;
  - 2 I2C e 2 I2S;
  - 3 UART;
  - CAN 2.0;
  - 3 SPI;
  - 16 PWM;

---

### Características de Hardware:
#### Segurança:

* Padrão de segurança IEEE 802.11, incluindo WFA, WPA/WPA2;
* Secure Boot;
* Flash Encryption;
* Criptografia em hardware: AES, SHA-2, RSA, ECC e RNG;

---

### Diagrama de Blocos de Funções

<img src="http://esp32.net/images/_resources/ESP32_Function_Block_Diagram.svg" style="width: 500px;"/>

---
### Comparação com outros uC's:

|   | ESP32 | ESP8266 | Arduino UNO |
|:--:|:--:|:----:|
| Cores | 2 | 1 | 1 | 
| Arquitetura(bits) | 32  | 32  | 8 |
| Clock(MHz) | 160 | 80 | 16 |
| Wifi | Sim | Sim | Não |
| Bluetooth | Sim | Não | Não |
| RAM (KB) | 512 | 160 | 2 |

---

### Comparação com outros uC's:

|   | ESP32 | ESP8266 | Arduino UNO |
|:--:|:--:|:----:|
| Flash | 16Mb | 16 Mb | 32KB |
| GPIO | 36 | 17 | 14 |
| Interfaces | SPI/I2C/</br>UART/I2S/CAN | SPI/I2C/</br>UART/I2S | SPI/I2C/UART |
| ADC | 18 | 1 | 6 |
| DAC | 2 | 0 | 0 |

---

### Comparação de preço:

| ESP32 | ESP8266 | Arduino UNO |
|-------|---------|-------------|
|![esp32](https://i.imgur.com/4YzMzHU.png) | ![esp8266](https://i.imgur.com/Ar8TaMH.png) | ![arduino uno](https://i.imgur.com/MkJchEC.png) |

---

### Como programar?:

| Arduino Framework | ESP-IDF (C/C++) | MicroPython |
|-------------------|-----------------|-------------|
| ![esp32 and arduino](https://mjrobot.files.wordpress.com/2017/09/esp32-portada.png) | ![esp-idf](https://avatars1.githubusercontent.com/u/9460735?s=200&v=4) | ![micropython+esp32](https://i.ytimg.com/vi/QPNmQZrG8ZU/maxresdefault.jpg) |

---?code=project/main/main.c&title=C Main File

@[1,3-6](Present code found within any repo source file.)
@[8-18](Without ever leaving your slideshow.)
@[19-28](Using GitPitch code-presenting with (optional) annotations.)

---

@title[Hello World com Arduino]

<p><span class="slide-title">Pisca LED</span></p>

```c
void setup() {
  pinMode (2, OUTPUT);
}
void loop() {
  digitalWrite (2, HIGH);
  delay(500);
  digitalWrite (2, LOW);
  delay(500);
}
```

@[1,3](função setup() seta o GPIO 5 como saída digital)
@[4](Cria loop infinito)
@[5,6](Seta GPIO 2 para estado lógico ALTO e espera 500ms)
@[7,8](Seta GPIO 2 para estado lógico BAIXO e espera 500ms)

---

@title[Hello World com ESP-IDF]

<p><span class="slide-title">Pisca LED</span></p>

```c
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
void app_main(){
  gpio_pad_select_gpio(GPIO_NUM_2);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  while(1){
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(500/portTICK_RATE_MS);
    gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(500/portTICK_RATE_MS);        
  }
}
```

@[1,2](Importa as bibliotecas de FreeRTOS necessárias para a utilização do vTaskDelay() - delay
@[3](Importa a bilblioteca de acesso à GPIO)
@[5,6](Exporta a GPIO 2 e seta como saída)
@[7](Cria o loop infinito)
@[8,9](Seta GPIO 2 para estado lógico ALTO e espera 500ms)
@[10,11](Seta GPIO 2 para estado lógico BAIXO e espera 500ms)

---

@title[Hello World com MicroPython]

<p><span class="slide-title">Pisca LED</span></p>

```python
import utime
import machine
pin2 = machine.Pin(2, machine.Pin.OUT)
while True:
        pin2.value(1)
        utime.sleep_ms(500)
        pin2.value(0)
        utime.sleep_ms(500)
```

@[1](Importa módulo de tempo p/ o "delay")
@[2](Import módulo de acesso ao hardware)
@[3](Instancia a classe de Pin setando o GPIO 2 para saída)
@[4](Cria loop infinito)
@[5,6](Seta GPIO 2 para estado lógico ALTO e espera 500ms)
@[7,8](Seta GPIO 2 para estado lógico BAIXO e espera 500ms)

---

### Perguntas?

<br>


@fa[github gp-contact](https://github.com/Calebe94/)

@fa[reddit gp-contact](https://reddit.com/u/Calebe94)

@fa[telegram gp-contact](http://t.me/calebe94)

