# Monitoramento de gases inflamáveis com SigFox

Este repositório contém o projeto (hardware e software) do monitoramento de gases inflamáveis com SigFox.

## Componentes

Este projeto utiliza os seguinte componentes e placas:

* Placa Edukit SigFox (link para aquisição: https://www.curtocircuito.com.br/kit-iot-sigfox-com-edukit-redfox.html), que utiliza um HT32SX para comunicação SigFox
* Placa Arduino Nano
* Sensor MQ-2
* Sensor DHT11
* Resistor de 100k / 0.25W
* Protoboard de 400 pontos
* Jumpers diversos

## Funcionamento simplificado

O funcionamento simplificado do projeto é:

1. O Arduino Nano faz, periodicamente (uma vez por segundo), a leitura dos sensores DHT11 e MQ-2.
2. Uma vez a cada meia hora o Arduino Nano envia, através do HT32SX, as leituras para a rede SigFox. A comunicação entre Arduino Nano e HT32SX é feita via UART, com comandos AT.
3. Uma vez recebido por um ou mais gateways, os dados enviados vão para o backend SigFox e redirecionados conforme callbacks configurados pelo usuário.