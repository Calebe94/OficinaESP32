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

  * ROM: 448 kB (Usada para Boot e funções do core);
  * SRAM: 520 kB (Para dados e instruções);
  * RTC fast SRAM: 8 kB (para armazenamento e CPU durante o boot do RTC);
  * RTC slow SRAM: 8 kB (Utilizado pelo co-processador durante o deep-sleep);
  * Flash: entre 0 e 4 MB (Suporta até 16MB);

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
|[![esp32](https://i.imgur.com/4YzMzHU.png)](https://lista.mercadolivre.com.br/esp32#D[A:esp32]) | [![esp8266](https://i.imgur.com/Ar8TaMH.png)](https://lista.mercadolivre.com.br/esp8266-nodemcu#D[A:esp8266-nodemcu]) | [![arduino uno](https://i.imgur.com/MkJchEC.png)](https://lista.mercadolivre.com.br/arduino-uno-r3#D[A:arduino-uno-r3]) |

---

### Como programar?:

| Arduino Framework | ESP-IDF (C/C++) | MicroPython |
|-------------------|-----------------|-------------|
| [![esp32 and arduino](https://mjrobot.files.wordpress.com/2017/09/esp32-portada.png)](https://www.hackster.io/nikil511/esp32-arduino-ide-hello-world-df2565) | [![esp-idf](https://avatars1.githubusercontent.com/u/9460735?s=200&v=4)](https://esp-idf.readthedocs.io/en/latest/) | [![micropython+esp32](https://i.ytimg.com/vi/QPNmQZrG8ZU/maxresdefault.jpg)](https://micropython.org/download/#esp32) |

---

### Hello World com Arduino

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

@[1,2,3](função setup() seta o GPIO 2 como saída digital)
@[4](Cria loop infinito)
@[5,6](Seta GPIO 2 para estado lógico ALTO e espera 500ms)
@[7,8](Seta GPIO 2 para estado lógico BAIXO e espera 500ms)

---

### Hello World com ESP-IDF

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
@[4](Igual a função main do C Padrão)
@[5,6](Exporta a GPIO 2 e seta como saída)
@[7](Cria o loop infinito)
@[8,9](Seta GPIO 2 para estado lógico ALTO e espera 500ms)
@[10,11](Seta GPIO 2 para estado lógico BAIXO e espera 500ms)

---

### Hello World com MicroPython

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

## Projetos 

### PocketSprite

[![](https://cdn.shopify.com/s/files/1/2306/4595/t/11/assets/PocketSprite-Sega-Emulator-1.png?16204621230715661419)](https://hackaday.com/2018/02/12/hands-on-with-the-smallest-game-boy-ever-made/)
---

## Projetos 

### PocketSprite

[![](https://cdn.shopify.com/s/files/1/2306/4595/files/PocketSprite-Palm-Sized-1.png?14979960333489014589)](https://hackaday.com/2018/02/12/hands-on-with-the-smallest-game-boy-ever-made/)
---

## Projetos
### Vídeo Composto no ESP32

[![](https://hackadaycom.files.wordpress.com/2018/02/bitluni3-e1519262186711.png?w=700&zoom=0)](https://hackaday.com/2018/02/22/software-defined-television-on-an-esp32/)
---

# I2C
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
    * em curtas distâncias(Máx. 1m, além disso podemos ter problemas de impedância);

aos microcontroladores e processadores;
---

## I2C

![i2c_bus](https://howtomechatronics.com/wp-content/uploads/2015/10/I2C-Communication-How-It-Works.png?x57244)
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
  * 1. Não existe nenhum receptor no barramento que reconhece o endereço transmitido;
  * 2. O Receptor está imcapacitado de receber ou trasmitir pois está executando alguma função interna;

---
## ACK e NACK:

* Existem 5 condições que podem gerar um Bit de **Não Reconhecimento**:
  * 3. Durante a transferência, o receptor recebeu dados ou comandos que ele não reconhece;
  * 4. Durante a transferência, o receptor não pode receber mais dados;
  * 5. Um **MESTRE-RECEPTOR** deve sinalizar o final da tranferência para um **ESCRAVO-TRANSMISSOR**.
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
## Projeto de Exemplo

<!-- ![projeto_esp+i2c](https://i.imgur.com/GrYw4Oo.jpg?3) -->
<img src="https://i.imgur.com/orpS9dQ.jpg" alt="projeto_esp+i2c" style="width:600px;height:500px;">
---

## Projeto de Exemplo

* Utiliza o barramento **I2C0** do ESP32;
  * configura o **CI DS1307** (que é um relógio de tempo real com calendário)
  * configura a **CI AT24C32** (EEPROM que  fornece 4096 palavras de 8 bits cada)
    * os CI's estão montados em um módulo chamado de *TinyRTC* que pode ser visto na figura abaixo.
---

## Projeto de Exemplo

![TinyRTC](http://mekhos.com.br/mekhos/wp-content/uploads/2017/01/Tiny-RTC.jpg)
---

## Projeto de Exemplo

* Funcionamento 
  * Dois botões
    * um botão armazena os valores de tempo do RTC na EEPROM(**SAVE**);
    * o outro botão imprimir no terminal serial os valores adquiridos na EEPROM(**GET**).
---

## Projeto de Exemplo

<!-- ![projeto_esp+i2c](https://i.imgur.com/GrYw4Oo.jpg?3) -->
<img src="https://i.imgur.com/gWNeKXw.jpg" alt="projeto_esp+i2c" style="width:600px;height:500px;">
---

## Projeto de Exemplo - Datasheets

* Verificar os datasheets para obter os endereços;

| [AT24C32](http://ww1.microchip.com/downloads/en/devicedoc/doc0336.pdf) | [DS1307](https://datasheets.maximintegrated.com/en/ds/DS1307.pdf) |
|:-------:|:------:|
| ![AT24C32 DATA ADDRESS](https://i.imgur.com/mSuC6x2.png) | ![DS1307 DATA ADDRESS](https://i.imgur.com/XECUG1n.png ) |
| 0x50 | 0x68 |
---

## Projeto de Exemplo - Caso Específico

* Como o AT24C32 tem 4096 palavras, 1 byte(8 bits) não é capaz de endereçar os offsets;
  * pois o valor máximo que cabe em 8 bits é 255;
  * para endereçar 4096 palavras é necessário 12 bits;
  Então ...
---

## Projeto de Exemplo - AT24C32

![AT24C32 - DATA WRITE](https://i.imgur.com/hE3Znb8.png)
---

## Projeto de Exemplo - DS1307

* O DS1307 auto-incrementa o endereço do registrador durante a leitura e escrita
![DS1307 - OFFSETS](https://i.imgur.com/aRBkLBD.png)
---

## Projeto de Exemplo - Código
### EEPROM - AT24C32
---?code=project/main/at24c32.c&lang=c&title=AT24C32

@[1](Inclui variáveis de 8 bits-uint8_t- e 16 bits-uint16_t- ...)
@[2](Inclui as funções para Habilitar a I2C)
@[3](Inclui defines do kernel FreeRTOS)
@[4](Inclui apenas os protótipos das funções)
@[6](Endereço da EEPROM)
@[7](Número máximo de offsets da EEPROM)

@[9](Função para gravar dados na EEPROM)
@[11](Gera a condição de START)
@[12](Envia o Endereço da EEPROM, informa que é escrita e informa que espera o bit ACK)
@[13](Envia os bits mais significativos do offset)
@[14](Envia os bits menos significativos do offset)
@[15](Envia o valor para ser gravado na EEPROM)
@[16](Gera a condição de STOP)
@[17](Informa ao periférico I2C0 para iniciar a transação)

@[22](Função para ler dados da EEPROM)
@[24-27](Informa o Offset para a EEPROM)
@[29](Gera a condição de START Repetido)
@[31](Informa que quer ler)
@[32](Lê o valor do offset e grava na variável data)
@[33-34](Gera a condição de STOP e inicia a transação)

---?code=project/main/ds1307.c&lang=c&title=DS1307

@[1-5](Mesma coisa da EEPROM)
@[7](Endereço I2C do DS1307)
@[9](Função para obter o tempo do DS1307, retorna o tempo em time_t - C Padrão -)
@[11](Gera condição de Start)
@[12](Envia o Endereço do Relógio e informa que a operação é de escrita)
@[13](Informa o registrador primeiro registrador)
@[14](Gera condição de Start Repetido)
@[15](Dessa vez é leitura)
@[18](Lê os 7 registradores- Segundos, Minutos, Horas, ... e Força o NACK)
@[19](Gera a Condição de STOP)
@[22](Transforma o vetor de 7 posições lido do Relógio em uma variável time_t)

---?code=project/main/main.c&lang=c&title=MAIN

@[1-7](Inclui todas as Bibliotecas necessárias)
@[9-12](Defines das GPIOs utilizadas)

@[14-25](Habilita o periférico I2C0 do ESP32)

@[66](app_main = main)
@[67-71](Habilita as GPIO 32 e 33 como entrada e habilido o I2C0)
@[73](loop)
@[75-79](Verifica se o botão SAVE foi apertado e chama a função save_to_eeprom)

@[27](Função para gravar a Data e Hora na EEPROM)
@[29-32](Transforma o time_t -32 bits- para 4 variáveis de 4 bits)
@[36-45](Procura um espaço vazio na EEPROM para armazenar o tempo e hora)

@[80-83](Verifica se o botão GET foi apertado e chama a função get_from_eeprom)
@[49](Função para pegar a data e hora da EEPROM)
@[53-63](Lê todos os offsets, imprimo a data e hora na terminal serial)
---

## Funcionamento

[![esp32_i2c](https://i.imgur.com/cU9kiAt.mp4)](https://imgur.com/IKNSJcq)
---

### Perguntas?

<br>

| [Calebe94](https://github.com/Calebe94) | [/u/Calebe94](http://reddit.com/user/Calebe94) | [calebe94](http://t.me/calebe94) |
|:-----:|:----:|:----:|
| [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/github-512.png" style="width: 150px;"/>](https://github.com/Calebe94) | [<img src="https://cdn4.iconfinder.com/data/icons/iconsimple-logotypes/512/reddit-512.png" style="width: 150px;"/>](http://reddit.com/user/Calebe94) | [<img src="https://cdn2.iconfinder.com/data/icons/telegram/154/logotype-telegram-round-blue-logo-512.png" style="width: 150px;"/>](http://t.me/calebe94) |
---

### Referências
#### ESP32
* [ESP32 - FEATURES & SPECIFICATIONS](http://esp32.net)
* [Getting Started with MicroPython on ESP32 – Hello World, GPIO, and WiFi](https://www.cnx-software.com/2017/10/16/esp32-micropython-tutorials/)
* [Spakfun - ESP32 Thing Hookup Guide](https://learn.sparkfun.com/tutorials/esp32-thing-hookup-guide)
* [Overview of ESP32 features. What do they practically mean?](https://www.exploreembedded.com/wiki/Overview_of_ESP32_features._What_do_they_practically_mean%3F)
* [Hello World with ESP32 Explained](https://exploreembedded.com/wiki/Hello_World_with_ESP32_Explained)
* [ESP32 Arduino IDE "Hello World"](https://www.hackster.io/nikil511/esp32-arduino-ide-hello-world-df256)
---

### Referências
#### I2C:
* [Conheça o Barramento I2C](http://www.newtoncbraga.com.br/index.php/microcontroladores/143-tecnologia/12085-conheca-o-barramento-i2c-mic098)
* [I2C – Protocolo de Comunicação](http://www.arduinobr.com/arduino/i2c-protocolo-de-comunicacao/)
* [I2C Info – I2C Bus, Interface and Protocol](http://i2c.info)
* [The I2C-bus and how to use it (including specifications)](https://www.i2c-bus.org/fileadmin/ftp/i2c_bus_specification_1995.pdf)
* [I2C-bus specification and user manual](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
* [Sparfun - I2C](https://learn.sparkfun.com/tutorials/i2c)

---
### Referências

#### Projetos

* [SOFTWARE DEFINED TELEVISION ON AN ESP32](https://hackaday.com/2018/02/22/software-defined-television-on-an-esp32/)
* [HANDS ON WITH THE SMALLEST GAME BOY EVER MADE](https://hackaday.com/2018/02/12/hands-on-with-the-smallest-game-boy-ever-made/)
* [PORTING NES TO THE ESP32](https://hackaday.com/2016/10/10/porting-nes-to-the-esp32/)
