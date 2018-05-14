# OficinaESP32

Este repositório hospeda a apresentação que será apresentada na oficina **ESP32 e Protocolo I2C**, no **IOT Maker Hub** da Universidade Positivo no dia *23/05/2018*.

Esta oficina tem o objetivo mostrar as principais características e recursos do microcontrolador **ESP32** e também explicar como é o funcionamento e as características do protocolo de comunicação **I2C**, através de um simples exemplo utilizando o NodeMCU ESP32 e o módulo TinyRTC, que utiliza os CI's de Real Time Clock [DS1307](https://datasheets.maximintegrated.com/en/ds/DS1307.pdf) e a EEPROM [AT24C32](http://ww1.microchip.com/downloads/en/devicedoc/doc0336.pdf).

# Acesso ao conteúdo da apresentação:

A hospedagem é feita pelo [GitHub](https://github.com/), mas a apresentação é realizada através da plataforma [GitPitch](https://gitpitch.com/). A apresentação no formato **Markdown** pode ser lida clicando [aqui](https://github.com/Calebe94/OficinaESP32/blob/master/PITCHME.md), ou acessando o arquivo [PITCHME.md] na pasta raiz desse repositório.
Para ver o arquivo em modo de apresentação basta acessar: https://gitpitch.com/Calebe94/OficinaESP32#/

# Acesso ao código de exemplo:

Este exemplo utiliza o barramento **I2C0** do ESP32 para configurar o **CI DS1307** que é um relógio de tempo real com calendário e o **CI AT24C32** que  fornece 4096 palavras de 8 bits cada, os CI's estão montados em um módulo chamado de *TinyRTC* que pode ser visto na figura abaixo.

![TinyRTC](http://mekhos.com.br/mekhos/wp-content/uploads/2017/01/Tiny-RTC.jpg) 

O funcionamento do projeto é dado da seguinte forma: Haverá dois botões, um botão que quando pressionado irá armazenar os valores do RTC na EEPROM. E o outro botão quando pressionado irá imprimir no terminal os valores que serão adquiridos na EEPROM.

O código desse projeto foi desenvolvido utilizando a framework [esp-idf](https://esp-idf.readthedocs.io/en/v2.0/index.html), que é a principal framework para desenvolvimento com o chip ESP32. Todas as instruções para a instalação do **esp-idf** estão neste [site](https://esp-idf.readthedocs.io/en/v2.0/index.html).  O código deste projeto pode ser acessado na pasta **project/** no diretório raiz deste repositório.

# Referências:

* [I2C (Inter-Integrated Circuit) Bus Technical Overview and Frequently Asked Questions](http://www.esacademy.com/en/library/technical-articles-and-documents/miscellaneous/i2c-bus.html)
* [NXP Semiconductors - I2C-bus specification and user manual](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
* [Philips Semiconductors - I2C MANUAL](https://www.nxp.com/docs/en/application-note/AN10216.pdf)
* [Wikipedia - I2C](https://en.wikipedia.org/wiki/I²C)

