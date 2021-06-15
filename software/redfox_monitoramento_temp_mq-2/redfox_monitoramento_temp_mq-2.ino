/* Programa: monitoramento de temperatura ambiente
 *           e gases inflamáveis/fumaça via SigFox
 * Autor: Pedro Bertoleti
 */
#include <SoftwareSerial.h>
#include "DHT.h"

/* Definições - sensores */
#define DHTPIN                        5
#define DHTTYPE                       DHT11
#define GPIO_MQ2                      6
#define GAS_INFLAMAVEL_NAO_DETECTADO  0x00
#define GAS_INFLAMAVEL_DETECTADO      0x01
//#define ESCREVE_DEBUG_SENSORES

/* Definição - breathing light */
#define TEMPO_BREATHING_LIGHT     500 //ms

/* Definições - comandos AT para o HT32SX */
#define CMD_AT_HT32SX_RESET            "AT+RESET;"
#define CMD_AT_HT32SX_RCZ2             "AT+CFGRCZ=2;"
#define CMD_AT_HT32SX_MANDA_BYTES      "AT+SEND=0:" //sem downlink
#define CMD_AT_HT32SX_ENTRA_DEEP_SLEEP "AT+DEEPSLEEP;"
#define CMD_AT_HT32SX_SAI_DEEP_SLEEP   "AT+WKP;"

/* Definições - tempo entre envios SigFox */
#define TEMPO_ENTRE_ENVIOS_SIGFOX         1800000 /* 1800000ms = 30 minutos */

/* Definições - GPIOs usados na comunicação com HT32SX*/
#define RESET  4   /* Reset no HT32SX */
#define TX     2   /* Serial TX (Nano -> HT32SX) */
#define RX     3   /* Serial RX (Nano <- HT32SX) */

/* Definição - GPIO do LED */
#define LED    13 

/* Definições - baudrates das comunicações seriais */
#define BAUDRATE_SERIAL_DEBUG     115200
#define BAUDRATE_SERIAL_HT32SX    9600

/* Objeto para controle da software serial
   (para comunicação Nano <-> HT32SX */
SoftwareSerial serial_HT32SX(RX, TX);

/* Objeto para leitura do DHT11 */
DHT dht(DHTPIN, DHTTYPE);

/* Variáveis globais */
unsigned long timestamp_medicao_temp_gas;
unsigned long timestamp_envio_sigfox;
unsigned long timestamp_breathing_light;
int temperatura = 0;
char sensor_gas = GAS_INFLAMAVEL_NAO_DETECTADO;
bool led_aceso = false;
char comando_at_envio_sigfox[20] = {0};

/* Protótipos */
void hardware_reset_HT32SX(void);
void envia_comando_AT_HT32SX(char * pt_comando);
unsigned long diferenca_tempo(unsigned long tref);
void aguarda_e_mostra_recepcao_HT32SX(void);

/* Função: reseta (via hardware) o HT32SX 
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void hardware_reset_HT32SX(void) 
{
    digitalWrite(RESET, HIGH);
    delay(1000);
    digitalWrite(RESET, LOW);
    delay(100);    
}

/* Função: envia comando AT para o HT32SX
 * Parâmetros: ponteiro para string de comando
 * Retorno: nenhum
 */
void envia_comando_AT_HT32SX(char * pt_comando)
{
    char cmd_buffer[50] = {0};
    memcpy(cmd_buffer, pt_comando, strlen(pt_comando));
    serial_HT32SX.write(cmd_buffer, strlen(cmd_buffer));
    serial_HT32SX.flush();
}

/* Função: calcula a diferença entre instante atual e
 *         uma referência de tempo
 * Parâmetros: referência de tempo
 * Retorno: diferença de tempo calculada
 */
unsigned long diferenca_tempo(unsigned long tref)
{
    return (millis() - tref);
}

/* Função: aguarda recepcao de dados do HT32SX 
 *         pela UART
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void aguarda_e_mostra_recepcao_HT32SX(void)
{
    unsigned long timestamp_recepcao_serial;
    char c;
    
    timestamp_recepcao_serial = millis();
        
    while(diferenca_tempo(timestamp_recepcao_serial) <= 3000)
    {
        if (serial_HT32SX.available()) 
        {
            c = serial_HT32SX.read(); 
            Serial.print(c);
        }
    }  
}

void setup() 
{
    /* Inicializa as comunicações seriais */
    Serial.begin(BAUDRATE_SERIAL_DEBUG);
    serial_HT32SX.begin(BAUDRATE_SERIAL_HT32SX);
    Serial.println("SigFox - monitor de temperatura e gases inflamaveis");
    Serial.println("Aguarde 8 segundos...");

    /* Inicializa GPIOs */
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    pinMode(GPIO_MQ2, INPUT);

    /* Inicializa DHT11 */
    dht.begin();

    /* Reseta HT32SX via hardware, via software e configura zona para RCZ2*/
    hardware_reset_HT32SX();
    delay(8000);
    envia_comando_AT_HT32SX(CMD_AT_HT32SX_ENTRA_DEEP_SLEEP);
    
    /* Inicializa temporização da medição de: 
     * - Sensores (temperatura e gases inflamaveis)
     * - Envio SigFox 
     * - Breathing light
    */
    timestamp_medicao_temp_gas = millis();
    timestamp_envio_sigfox = millis();
    timestamp_breathing_light = millis();
}

void loop() 
{
    char c;        
    unsigned char tensao_byte;
    
    /* Escreve mensagens recebidas da UART com o HT32SX */
    if (serial_HT32SX.available()) 
    {
        c = serial_HT32SX.read(); 
        Serial.print(c);
    }

    /* Pisca breathing light */
    if (diferenca_tempo(timestamp_breathing_light) >= TEMPO_BREATHING_LIGHT)
    {
        if (led_aceso == false)
        {
            digitalWrite(LED, HIGH);
            led_aceso = true;
        }
        else
        {
            digitalWrite(LED, LOW);
            led_aceso = false;
        }
        
        timestamp_breathing_light = millis();
    }

    /* Mede a temperatura e le sensor de gas a cada segundo */
    if (diferenca_tempo(timestamp_medicao_temp_gas) >= 1000)
    {
        /* Le a temperatura atual */
        do
        {
            temperatura = (int)dht.readTemperature();
        }while( (isnan(temperatura)) || (temperatura <= 0) );

        /* Le sensor de gás MQ-2 */
        if (digitalRead(GPIO_MQ2) == HIGH)
            sensor_gas = GAS_INFLAMAVEL_DETECTADO;
        else
            sensor_gas = GAS_INFLAMAVEL_NAO_DETECTADO;    

        #ifdef ESCREVE_DEBUG_SENSORES
        Serial.print("Temperatura ambiente: ");
        Serial.print(temperatura);
        Serial.println("C");

        if (sensor_gas == GAS_INFLAMAVEL_DETECTADO)
            Serial.println("Gas inflamavel: detectado!");
        else
            Serial.println("Gas inflamavel: nada detectado");    
            
        Serial.println("------");
        #endif

        timestamp_medicao_temp_gas = millis();
    }

    /* Verifica se é o momento de enviar medições via SigFox */
    if (diferenca_tempo(timestamp_envio_sigfox) >= TEMPO_ENTRE_ENVIOS_SIGFOX)
    {
        /* Acorda HT32SX, faz a configuração da zona de comunicação 
           e envia mensagem SigFox */
        envia_comando_AT_HT32SX(CMD_AT_HT32SX_SAI_DEEP_SLEEP);
        aguarda_e_mostra_recepcao_HT32SX();
        hardware_reset_HT32SX();
        aguarda_e_mostra_recepcao_HT32SX();
        envia_comando_AT_HT32SX(CMD_AT_HT32SX_RCZ2);    
        aguarda_e_mostra_recepcao_HT32SX();

        /* Formata e envia comando AT */
        memset(comando_at_envio_sigfox, 0x00, sizeof(comando_at_envio_sigfox));
        sprintf(comando_at_envio_sigfox, "%s%02x%02x;", CMD_AT_HT32SX_MANDA_BYTES, 
                                                        (unsigned char)temperatura,
                                                        (unsigned char)sensor_gas);

        #ifdef ESCREVE_DEBUG_SENSORES
        Serial.print("Comando AT: ");
        Serial.println(comando_at_envio_sigfox);
        #endif
        
        envia_comando_AT_HT32SX(comando_at_envio_sigfox);
        aguarda_e_mostra_recepcao_HT32SX();
        
        /* Entra em modo sleep novamente */
        envia_comando_AT_HT32SX(CMD_AT_HT32SX_ENTRA_DEEP_SLEEP);
        aguarda_e_mostra_recepcao_HT32SX();
        
        timestamp_envio_sigfox = millis();
    }
}
