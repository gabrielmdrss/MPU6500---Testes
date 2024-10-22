# MPU6500 com STM32F407VET6

Este projeto utiliza o sensor de movimento **MPU6500** conectado a uma placa **STM32F407VET6** para medir aceleração e giroscópio.

## Visão Geral

O **MPU6500** é um sensor que combina acelerômetro e giroscópio de 6 eixos, permitindo a medição de movimento em três direções (X, Y, Z). Este projeto demonstra como ler os dados do sensor utilizando a interface I2C, processá-los e enviá-los via UART.

## Componentes Utilizados

- **STM32F407VET6**: Microcontrolador de 32 bits da família STM32 com arquitetura ARM Cortex-M4.
- **MPU6500**: Sensor de 6 eixos com acelerômetro e giroscópio.
- **Interface SPI**: Utilizada para comunicação entre o STM32 e o MPU6500.
- **USART**: Utilizada para enviar os dados lidos do sensor para um terminal de monitoramento (via USB).
