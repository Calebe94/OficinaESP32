# ESP32
## e Protocolo I2C

<!-- ### Oficina do IOT Maker Hub -->

---

### Informações do Instrutor:

<br>
** Edimar Calebe Castanho **
<br> 
** Aluno de Engenharia da Computação do 4º ano **
###### Redes Sociais

| [Calebe94](https://github.com/Calebe94) | [/u/Calebe94](http://reddit.com/user/Calebe94) | [calebe94](http://t.me/calebe94) |
|:-----:|:----:|:----:|
| [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/github-512.png" style="width: 150px;"/>](https://github.com/Calebe94) | [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/reddit-512.png" style="width: 150px;"/>](http://reddit.com/user/Calebe94) | [<img src="https://cdn2.iconfinder.com/data/icons/telegram/154/logotype-telegram-round-blue-logo-512.png" style="width: 150px;"/>](http://t.me/calebe94) |

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

@[1,2,3](função setup() seta o GPIO 5 como saída digital)
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

@[1,2](Importa as bibliotecas de FreeRTOS necessárias para a utilização do vTaskDelay - delay)
@[3](Importa a bilblioteca de acesso à GPIO)
@[4](Igual a função main do C ANSII)
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

## I2C
### Descrição
* Inter-Integrated Circuit(Circuito Inter-Integrado);
* Comunicação Serial Síncrona;
* Multi-Mestre;
* Multi-Escravo;
* Desenvolvido pela Philips em 1982(Atual NXP Semiconductors)

---
## I2C
### Descrição
* Muito utilizada para conectar perifericos:
    * de baixa velocidade;
    * em curtas distâncias;

aos microcontroladores e processadores;
---
## I2C

* Desde 2006 nenhuma taxa é cobrada para implementar o protocolo I2C.
* Porém, taxas são cobradas para obter endereços I2C para dispositivos escravos.
* Vários concorrentes, como Texas Instruments, STMicroelectronics e Motorola possuem produtos que implementam o protocolo desde a década de 90.

---

## Especificação:

* I2C utiliza apenas 2 linhas bidirecionais de dreno aberto;
    * SDA (Serial Data) - Dados Seriais;
    * SCL (Serial Clock) - Clock Serial (Gerado pelo Mestre);
* As duas linhas precisam utilizar resistores de pull-up;
* Tensões tipicamente utilizadas são 5V ou 3,3V.

---

## Especificação:

* Possui endereçamento de 7 bits ( e 10 bits em casos raros )
* velocidade do barramento:
  * Low-speed mode: 10 kbit/s;
  * Standard mode: 100 kbit/s;
  * Fast Mode: 400 kbit/s;
  * Fast Mode Plus: 1 Mbit/s;
  * High Speed Mode: 3.4 Mbit/s.
---

## Condição de START e STOP:

![start_stop](http://i2c.info/wp-content/images/i2c.info/start-stop.gif)

* Todas as transações:
  * Iniciam com um **START**(S);
  * E terminam com um **STOP**(P);
* Sempre **geradas** pelo **MESTRE**
---

## Condição de START e STOP:

![i2c-start_stop](http://i2c.info/wp-content/images/i2c.info/start-stop.gif)

* **START**: **ALTO** p/ **BAIXO** em *SDA* enquanto *SCL* está em **ALTO**;
* **STOP**: **BAIXO** p/ **ALTO** em *SDA* enquanto *SCL* está em **ALTO**.
---

## Condição de START e STOP:

* O barramento é considerado **ocupado** depois de uma condição de START;
* O barramento é considerado **livre** novamente após a condição de STOP;
* O barramento permanece **ocupado** se um **START repetido** (Sr) for gerado ao invés de um **STOP**;
  - **START** e **Repeated START** (Sr) são funcionalmente identicas;
---

## Mensagem

![i2c-message](https://opencores.org/usercontent,img,1352582784)
* Cada BYTE colocado no **SDA** deve ter 8 bits;
* Cada byte deve ser seguido de um **bit de Reconhecimento**(*Acknowledge bit);
* O bit mais significativo (**MSB**)são transferidos primeiro;
---

## ACK e NACK
* Terminologia:
  * ACK - Acknowledge: Bit de Reconhecimento;
  * NACK - Not Acknowledge: Bit de Não Reconhecimento;
---

## ACK e NACK

* ACK:
  * Ocorre após cada byte.
  * Permite ao **RECEIVER** informar o **TRASMITTER** que o byte foi recebido com sucesso.
  * O mestre gera todos os pulsos de clock.

---

## ACK e NACK

* O Bit de Reconhecimento é definido da seguinte forma:
  * O **TRANSMISSOR** libera a linha **SDA** durante o clock de reconhecimento(nono pulso de clock);
  * Então o **RECEPTOR** pode colocar a linha **SDA** para estado **BAIXO** e gerar o bit de **RECONHECIMENTO**;

---

## ACK e NACK
* O Bit de Reconhecimento é definido da seguinte forma:
  * Ou o **RECEPTOR** pode manter a linha **SDA** em estado **ALTO** e gerar um bit de **NÃO RECONHECIMENTO**;
    * O **MESTRE** então pode gerar uma condição de **STOP** para abortar a transferência;
    * Ou pode gerar uma condição de ** Repeated START**-(START REPETIDO), para inicializar uma nova transferência.
---

## ACK e NACK

* Existem 5 condições que podem gerar um Bit de **Não Reconhecimento**:
  1. Não existe nenhum receptor no barramento que reconhece o endereço transmitido;
  2. O Receptor está imcapacitado de receber ou trasmitir pois está executando alguma função interna;

---
## ACK e NACK:

* Existem 5 condições que podem gerar um Bit de **Não Reconhecimento**:
  3. Durante a transferência, o receptor recebeu dados ou comandos que ele não reconhece;
  4. Durante a transferência, o receptor não pode receber mais dados;
  5. Um **MESTRE-RECEPTOR** deve sinalizar o final da tranferência para um **ESCRAVO-TRANSMISSOR**.
---
## Endereçamento e o bit R/W

![i2c-complete_data_transfer](http://i2c.info/wp-content/images/i2c.info/data-transfer.gif)

* A transferência de dados segue o formato da figura acima;
---

## Endereçamento e o bit R/W

* Após a condição de **START**(S) gerada pelo **MESTRE**, um endereço de escravo é enviado;
* Este endereço possui 7 bits, seguido por um 8º bit, que é a direção dos dados (Leitura ou Escria);
  * WRITE: 0
  * READ:  1
---

## Endereçamento e o bit R/W
* A transferência de dados é sempre terminada pela condição de **STOP**(P) gerada pelo **MESTRE**
* Entretanto, se o **MESTRE** ainda querer se comunicar com o barramento, ele pode gerar uma condição **Repeated START**(Sr) sem ter que gerar um **STOP**;
---

## Transferências Possíveis

#### **MESTRE-Transmissor** trasmitindo para um **ESCRAVO-Receptor**:

![master-trasmitter](http://i2c.info/wp-content/images/i2c.info/7-bit-address-writing.gif)
---

## Transferências Possíveis

#### **MESTRE-Receptor** recebendo de um **ESCRAVO-Trasnmissor**:

![master-receiver](http://i2c.info/wp-content/images/i2c.info/7-bit-address-reading.gif)
---

## Transferências Possíveis

#### Formato Combinado:

![combined-format](http://i2c.info/wp-content/images/i2c.info/7-bit-address-writing-reading.gif)
---

### Perguntas?

<br>

| [Calebe94](https://github.com/Calebe94) | [/u/Calebe94](http://reddit.com/user/Calebe94) | [calebe94](http://t.me/calebe94) |
|:-----:|:----:|:----:|
| [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/github-512.png" style="width: 150px;"/>](https://github.com/Calebe94) | [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/reddit-512.png" style="width: 150px;"/>](http://reddit.com/user/Calebe94) | [<img src="https://cdn2.iconfinder.com/data/icons/telegram/154/logotype-telegram-round-blue-logo-512.png" style="width: 150px;"/>](http://t.me/calebe94) |

