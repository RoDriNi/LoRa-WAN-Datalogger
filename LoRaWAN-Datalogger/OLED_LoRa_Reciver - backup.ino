/*
  This is a simple example show the LoRa recived data in OLED.

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16

  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn

  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/
#include <SPI.h> //responsável pela comunicação serial
#include <LoRa.h> //responsável pela comunicação com o WIFI Lora
#include <Wire.h>  //responsável pela comunicação i2c
#include "SdFat.h"
#include "SSD1306.h" //responsável pela comunicação com o display
#include "images.h" //contém o logo para usarmos ao iniciar o display
#include <DS3231.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <locale.h>
#include <SPIMemory.h>

// Definição dos pinos
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    868E6  //Frequencia do radio - podemos utilizar ainda : 433E6, 868E6, 915E6
#define PABOOST true

// SD SPI pins can be chosen freely, but cannot overlap with other ports!
#define SD_CS_PRINCIPAL 23
#define SD_CS_SECUNDARIO 22
#define SD_SCK 5//17
#define SD_MOSI 27//12
#define SD_MISO 19//13
#define LOG_PATH "/lora_recv.log"

//parametros: address,SDA,SCL
SSD1306 display(0x3c, 4, 15); //construtor do objeto que controlaremos o display

String rssi = "RSSI --";
String packSize = "--";
String packet;


SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;

#define SD_CONFIG_1 SdSpiConfig(SD_CS_PRINCIPAL, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
#define SD_CONFIG_2 SdSpiConfig(SD_CS_SECUNDARIO, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
 //sck, miso, mosi, ss.
//int8_t spi_pins[4] = {SD_SCK, SD_MISO, SD_MOSI, SD_CS_SECUNDARIO};
SPIFlash flash(SD_CS_SECUNDARIO, &SPI);

SdFat sd;
File file;



DS3231 myClock;
bool century = false;
bool h12Flag;
bool pmFlag;
//RTClib myRTC;


///////////////////////////////////////


typedef struct canal_analogico{
 char   nome[50];//identificador
 int    pino;
 double valor_instantaneo;
 double inclinacao;//a  => y = ax+b
 double acrescimo; //b  => y = ax+b
 int    casas_decimais; //quantidade de casas decimais
 double amplitude;//por exemplo MCA
};

typedef struct canal_digital_pulso_acumulador{
 char   nome[50];//identificador
 int    pino;
 double valor_acumulado;
 double inclinacao;//a  => y = ax+b
 double acrescimo; //b  => y = ax+b
 int    intervalo_reset;// em minutos
 int    casas_decimais; //quantidade de casas decimais
};

typedef struct canal_digital{
 char   nome[50];//identificador
 int    pino;
 double valor_instantaneo;
};

typedef struct estacao{
 char nome[100];
 int  intervalo_transmissao;//em minutos
 int  codigo_mit;
 char codigo_ana[8];
 char matriz_transmissao[50][5];//Cód_MIT Mes Dia Hora Bateria Painel Nível Chuva
                                  //exemplo 1: Mes Dia Hora canal3_1 canal4_1 canal2_1 canal1_1 -->Todos os dados da própira estação
                                  //exemplo 2: Mes Dia Hora canal3_1 canal4_1 canal2_2 canal1_1 -->Dados de Bateria, Painel e Chuva da propria estação, mas Nível é da estação filha da linha 2.

 int  codigo_mit_filhas[5];       //auxilia na fila de requisição da estações filhas


 struct canal_digital_pulso_acumulador canal_01; //chuva;            //Chuva Horaria Acumulada [mm]
 struct canal_analogico                canal_02; //nivel;            //Nível de Água [cm]
 struct canal_analogico                canal_03; //bateria;          //Tensão Bateria Equalizada [V]
 struct canal_analogico                canal_04; //painel;           //Corrente Painel [A]
 struct canal_analogico                canal_05; //consumo;          //Corrente Alimentação [A]
 struct canal_analogico                canal_06; //temperatura;      //temperatura do clock (não ultiliza nenhum pin)
 struct canal_digital                  canal_07; //abertura de porta //faz um registro a cada abertura

};

struct estacao estacao;//estacao; //Parametros gerais da Estação


int alterar_estacao(struct estacao * estacao);
//int alterar_canal_analogico(canal_analogico * canal);
//int alterar_canal_digital_pulso_acumulador(canal_digital_pulso_acumulador * canal);
//int alterar_canal_digital(canal_digital * canal);


int atualizar_dados();
int atualizar_horario();

int configurar_horario();

int gravar_dados();
int gravar_configuracoes();


int conectar_sd_princial();
int conectar_sd_secundario();

int inicializar_lora();
int inicializa_display();
int inicializar_pinagem();
int inicializar_estacao();
int carregar_arquivo_estacao();
//int inicializar_canais(canal_digital_pulso_acumulador * canal1, canal_analogico * canal2, canal_analogico * canal3 canal_analogico * canal4, canal_digital * canal5);

//////////////////////////////////////
void logo();

int   segundos  = 0;
int   horas     = 0;
int   minutos   = 0;
int   dia       = 0;
int   mes       = 0;
int   ano       = 0;

char  data_hora[21] = "00/00/0000 00:00:00\0";

char  arquivo[3000];

int x=0;
int minutos_acumulados = 0;


void setup() {
  pinMode(SD_CS_PRINCIPAL, OUTPUT);
  pinMode(SD_CS_SECUNDARIO, OUTPUT);
  pinMode(SS, OUTPUT);

  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(2000);
  digitalWrite(21, HIGH);

  setlocale(LC_ALL, "Portuguese");
  Serial.begin(115200);//inicialização serial
  while(!Serial);
  delay(1000);

  inicializar_pinagem();

  inicializa_display();

  logo();//exibe logo
  delay(2000);

  Wire.begin();//conexão com o relógio e LoRa inicializadas
  //configurar_horario();

  //

  atualizar_horario();

  digitalWrite(SS, HIGH);
  digitalWrite(SD_CS_PRINCIPAL, LOW);
  digitalWrite(SD_CS_SECUNDARIO, HIGH);

  if (conectar_sd_princial()){
    carregar_arquivo_estacao();
    inicializar_estacao();



    atualizar_horario();
    if ( myClock.getMinute() >= estacao.intervalo_transmissao ){
      int aux = (int) myClock.getMinute()/estacao.intervalo_transmissao;
      minutos_acumulados = myClock.getMinute() - estacao.intervalo_transmissao*aux;
      if( minutos_acumulados == 0 ){
        minutos_acumulados = estacao.intervalo_transmissao;
      }
    }else{
      minutos_acumulados = myClock.getMinute();
    }

  }



  digitalWrite(SS, HIGH);
  digitalWrite(SD_CS_PRINCIPAL, HIGH);
  digitalWrite(SD_CS_SECUNDARIO, LOW);

  SPI.begin(SCK,MISO,MOSI,SD_CS_SECUNDARIO);

  flash.begin();
  uint32_t strAddr;
  //randomSeed(analogRead(RANDPIN));
  strAddr = 1;//random(0, flash.getCapacity());
  Serial.print(F("Capacidade: "));
  Serial.println(flash.getCapacity());
  String inputString = "This is a test String";
  flash.writeStr(strAddr, inputString);
  Serial.print(F("Written string: "));
  Serial.println(inputString);
  Serial.print(F("To address: "));
  Serial.println(strAddr);
  String outputString = "";
  if (flash.readStr(strAddr, outputString)) {
    Serial.print(F("Read string: "));
    Serial.println(outputString);
    Serial.print(F("From address: "));
    Serial.println(strAddr);
  }
  while (!flash.eraseSector(strAddr));
  Serial.println("deu certo spimemory");
//  display.clear();
//  display.drawString(0, 0, estacao.nome);
//  display.drawString(0, 20,data_hora);
//  display.display();
  //delay(2000);


  digitalWrite(SS, LOW);
  digitalWrite(SD_CS_PRINCIPAL, HIGH);
  digitalWrite(SD_CS_SECUNDARIO, HIGH);

  inicializar_lora();
  delay(2000);
  //inicializar_estacao();
  //LoRa.onReceive(cbk);
  LoRa.receive(); //habili ta o Lora para receber dados
  //int teste=10;


}







//int inicializar_canais(canal_digital_pulso_acumulador * canal1, canal_analogico * canal2, canal_analogico * canal3 canal_analogico * canal4, canal_digital * canal5){//inicializa os canais e estação com os parametros definidos no cartão principal
  // return 0;
//}

int alterar_estacao(struct estacao *estacao){
   return 0;
}

// int alterar_canal_analogico(canal_analogico * canal){
//    return 0;
// }
//
// int alterar_canal_digital_pulso_acumulador(canal_digital_pulso_acumulador * canal){
//    return 0;
// }
//
// int alterar_canal_digital(canal_digital * canal){
//    return 0;
// }

int atualizar_horario(){
  char   aux_dia[10];
  char   aux_mes[10];
  char   aux_ano[10];
  char   aux_horas[10] ;
  char   aux_minutos[10];
  char   aux_segundos[10];

  dia      = myClock.getDate();
  mes      = myClock.getMonth(century);
  ano      = myClock.getYear();
  horas    = myClock.getHour(h12Flag, pmFlag);
  minutos  = myClock.getMinute();
  segundos = myClock.getSecond();

  itoa(dia,aux_dia,10);
  itoa(mes,aux_mes,10);
  itoa(ano,aux_ano,10);
  itoa(horas,aux_horas,10);
  itoa(minutos,aux_minutos,10);
  itoa(segundos,aux_segundos,10);

  sprintf(data_hora,"%s/%s/%s %s:%s:%s",aux_dia,aux_mes,aux_ano,aux_horas,aux_minutos,aux_segundos);

}


int configurar_horario(){
  myClock.setClockMode(false);  // set to 24h
  //setClockMode(true); // set to 12h
////
  myClock.setYear(21);
  myClock.setMonth(7);
  myClock.setDate(2);
  myClock.setDoW(6);
  myClock.setHour(18);
  myClock.setMinute(37);
  myClock.setSecond(0);
  return 1;
}

int atualizar_dados(){
  return 0;
}

int gravar_dados(){//grava apenas no secundario
  char  aux[100];
  char  aux_mes[10];
  char  aux_ano[10];
  itoa(mes,aux_mes,10);
  itoa(ano,aux_ano,10);

  file.close();
  sprintf(aux,"/%s/%s-%s/%s-%s-%s.csv",estacao.nome,aux_ano,aux_mes,aux_ano,aux_mes,estacao.nome);
  if (!file.open(aux, FILE_WRITE)) {
    file.close();
    sprintf(aux,"/%s/%s-%s/",estacao.nome,aux_ano,aux_mes);
    //Serial.println(aux);
    if (!sd.mkdir(aux)) {
      Serial.println("00 05");//erro tentando gravar dados;
      int x =0;
      while (!sd.mkdir(aux) && x<10){
        x++;
      }
    }
    delay(1000);
    file.close();
    sprintf(aux,"/%s/%s-%s/%s-%s-%s.csv",estacao.nome,aux_ano,aux_mes,aux_ano,aux_mes,estacao.nome);
    if(file.open(aux, FILE_WRITE)){
      sprintf(aux,"Nome Estacao:;%s;Codigo MIT:;%d;Codigo ANA:;%s",estacao.nome,estacao.codigo_mit,estacao.codigo_ana);
      file.println(aux);
      file.println("");
      file.println("DATA_HORA;NIVEL;BATERIA;CHUVA");
    }else{
      Serial.println("00 09");
    }
  }
  //Serial.println("BOA!");


  sprintf(aux,"%s;%d;123;0,4",data_hora,analogRead(32));
  file.println(aux);
  Serial.println(aux);
//  file.rewind();
//  char a[30];
//  int x = 0;
//
//  while (file.available()) {
//    a[x] = file.read();
//    //Serial.write(a);
//    x++;
//    if(x>30){
//      x=0;
//    }
//  }
//  x=0;
//  int flag=0;
//  Serial.print("->");
//  while(x<=30){
//
//    if(a[x]=='\n'){
//      flag=1;
//      x=x+1;
//    }
//    if(flag){
//      Serial.print(a[x]);
//    }
//
//    x++;
//  }
//  Serial.println("");

  file.close();
  return 1;

  delay(2000);

}

int gravar_configuracoes(){//grava apenas no principal
  if (!file.open("/SoftSPI.txt", FILE_WRITE)) {
    Serial.println("SD Card: writing file failed.");
    return 0;
  } else {
    file.println("CONFIGURAÇÕES SD1");
    file.rewind();
    char aux;
    while (file.available()) {
      aux = file.read();
      Serial.write(aux);
      Serial.write("-");
    }
    file.close();
    return 1;
  }
}

int inicializar_lora(){
  SPI.begin(SCK,MISO,MOSI,SS); //inicia a comunicação serial com o Lora
  LoRa.setPins(SS,RST,DI00); //configura os pinos que serão utlizados pela biblioteca (deve ser chamado antes do LoRa.begin)
  //inicializa o Lora com a frequencia específica.
  if (!LoRa.begin(BAND)) {
    display.clear();
    display.drawString(0, 0, "Inialização LoRa falhou!");
    Serial.println("00 03");//erro 2, A LoRa não foi inicializada corretamente!
    display.display();
    while (1);
  }
  return 0;
}

int conectar_sd_princial(){
  //digitalWrite(SD_CS_PRINCIPAL, LOW);
  //digitalWrite(SD_CS_SECUNDARIO, HIGH);
  if (!sd.begin(SD_CONFIG_1)){
    //sd.initErrorHalt();
    display.clear();
    display.drawString(0, 0, "Conexão SD-1 falhou!");
    display.drawString(0, 10, "Erro 00 01");
    display.drawString(0, 20, "Contate o fabricante.");
    Serial.println("00 01");//erro 1, SD card principal falhou!
    display.display();
    delay(3000);
    int x=0;
    while (!sd.begin(SD_CONFIG_1) && x<10){
      x++;
        Serial.println(x);
    }
    return 0;
  }
  display.clear();
  display.drawString(0, 0, "Conexão SD-1 OK!!!");
  display.display();
  delay(1000);
  return 1;
}

int conectar_sd_secundario(){
  if (!sd.begin(SD_CONFIG_2)){
    display.clear();
    display.drawString(0, 0, "Conexão SD-2 falhou!");
    display.drawString(0, 10, "Erro 00 02");
    display.drawString(0, 20, "Tente inserir o cartão");
    display.drawString(0, 30, "novamente.");
    Serial.println("00 02");//erro 2, SD card secundario falhou!
    display.display();
    int x =0;
    while (!sd.begin(SD_CONFIG_2) && x<10){
      x++;
    }
  }
  display.clear();
  display.drawString(0, 0, "Conexão SD-2 OK!!!");
  display.display();
  delay(1000);
  return 1;
}

int inicializa_display(){
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  return 0;
}

int inicializar_pinagem(){
  //configura os pinos como saída e entradas
  pinMode(16,OUTPUT); //RST do oled
  //pinMode(SD_CS_PRINCIPAL, OUTPUT);
  //pinMode(SD_CS_SECUNDARIO, OUTPUT);

  digitalWrite(16, LOW);    // reseta o OLED
  delay(50);
  digitalWrite(16, HIGH); // enquanto o OLED estiver ligado, GPIO16 deve estar HIGH

  pinMode(32,INPUT_PULLDOWN); //Painel
  pinMode(39,INPUT); //Sonda

  return 0;
}

int inicializar_estacao(){

  char *configuracoes;

  configuracoes = strtok(arquivo, "\n");

  while( configuracoes != NULL ) {

    configuracoes = strtok(NULL, ":");

    if( !strcmp(configuracoes,"############ARQUIVO_DE_CONFIGURAÇÃO#######################;" ) ){
      char aux[10];
      itoa(estacao.codigo_mit,aux,10);
      display.clear();
      display.drawString(0, 0, "Dados carregados com");
      display.drawString(0, 10, "sucesso!");
      display.drawString(0, 20, "Estação:");
      display.drawString(0, 30, estacao.nome);
      display.drawString(0, 40, "Código:");
      display.drawString(0, 50, aux);
      Serial.println("01 01");//Sucesso! Cartão Carregado com Informações da Estação!
      display.display();
      delay(3000);
      return 1;
    }

    char parametro[100];
    configuracoes[strlen(configuracoes)]='\0';
    strncpy(parametro, configuracoes, strlen(configuracoes));
    parametro[strlen(configuracoes)]='\0';

    configuracoes = strtok(NULL, "\n");

    char parametro_valor[100];
    configuracoes[strlen(configuracoes)]='\0';
    strncpy(parametro_valor, configuracoes, strlen(configuracoes)-2);
    parametro_valor[strlen(configuracoes)-2]='\0';

    /*////////Configurações ESTAÇÃO////////////////*/
    if(!strcmp(parametro,"DADOS_ESTAÇÃO_NOME")){
        strncpy(estacao.nome, parametro_valor, strlen(parametro_valor));
        estacao.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <<%s>>\n", estacao.nome );
    }else if(!strcmp(parametro,"DADOS_ESTAÇÃO_INTERVALO_TRANSMISSÃO")){
        estacao.intervalo_transmissao = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.intervalo_transmissao );
    }else if(!strcmp(parametro,"DADOS_ESTAÇÃO_CÓDIGO_MIT")){
        estacao.codigo_mit = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.codigo_mit );
    }else if(!strcmp(parametro,"DADOS_ESTAÇÃO_CÓDIGO_ANA")){
        strncpy(estacao.codigo_ana, parametro_valor, strlen(parametro_valor));
        estacao.codigo_ana[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.codigo_ana );
    }else
    /*////////Configurações CANAL_01////////////////*/
    if(!strcmp(parametro,"CANAL_01_NOME")){
        strncpy(estacao.canal_01.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_01.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.canal_01.nome );
    }else if(!strcmp(parametro,"CANAL_01_PINO")){
        estacao.canal_01.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_01.pino );
    }else if(!strcmp(parametro,"CANAL_01_INCLINAÇÃO")){
        estacao.canal_01.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_01.inclinacao );
    }else if(!strcmp(parametro,"CANAL_01_ACRÉSCIMO")){
        estacao.canal_01.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_01.acrescimo );
    }else if(!strcmp(parametro,"CANAL_01_INTERVALO_RESET")){
        estacao.canal_01.intervalo_reset = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_01.intervalo_reset );
    }else if(!strcmp(parametro,"CANAL_01_CASAS_DECIMAIS")){
        estacao.canal_01.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_01.casas_decimais );
    }else
    /*////////Configurações CANAL_02////////////////*/
    if(!strcmp(parametro,"CANAL_02_NOME")){
        strncpy(estacao.canal_02.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_02.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.canal_02.nome );
    }else if(!strcmp(parametro,"CANAL_02_PINO")){
        estacao.canal_02.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_02.pino );
    }else if(!strcmp(parametro,"CANAL_02_INCLINAÇÃO")){
        estacao.canal_02.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_02.inclinacao );
    }else if(!strcmp(parametro,"CANAL_02_ACRÉSCIMO")){
        estacao.canal_02.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_02.acrescimo );
    }else if(!strcmp(parametro,"CANAL_02_CASAS_DECIMAIS")){
        estacao.canal_02.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_02.casas_decimais );
    }else if(!strcmp(parametro,"CANAL_02_AMPLITUDE")){
        estacao.canal_02.amplitude = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_02.amplitude );
    }else
    /*////////Configurações CANAL_03////////////////*/
    if(!strcmp(parametro,"CANAL_03_NOME")){
        strncpy(estacao.canal_03.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_03.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.canal_03.nome );
    }else if(!strcmp(parametro,"CANAL_03_PINO")){
        estacao.canal_03.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_03.pino );
    }else if(!strcmp(parametro,"CANAL_03_INCLINAÇÃO")){
        estacao.canal_03.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_03.inclinacao );
    }else if(!strcmp(parametro,"CANAL_03_ACRÉSCIMO")){
        estacao.canal_03.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_03.acrescimo );
    }else if(!strcmp(parametro,"CANAL_03_CASAS_DECIMAIS")){
        estacao.canal_03.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_03.casas_decimais );
    }else if(!strcmp(parametro,"CANAL_03_AMPLITUDE")){
        estacao.canal_03.amplitude = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_03.amplitude );
    }else
    /*////////Configurações CANAL_04////////////////*/
    if(!strcmp(parametro,"CANAL_04_NOME")){
        strncpy(estacao.canal_04.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_04.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.canal_04.nome );
    }else if(!strcmp(parametro,"CANAL_04_PINO")){
        estacao.canal_04.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_04.pino );
    }else if(!strcmp(parametro,"CANAL_04_INCLINAÇÃO")){
        estacao.canal_04.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_04.inclinacao );
    }else if(!strcmp(parametro,"CANAL_04_ACRÉSCIMO")){
        estacao.canal_04.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_04.acrescimo );
    }else if(!strcmp(parametro,"CANAL_04_CASAS_DECIMAIS")){
        estacao.canal_04.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_04.casas_decimais );
    }else if(!strcmp(parametro,"CANAL_04_AMPLITUDE")){
        estacao.canal_04.amplitude = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_04.amplitude );
    }else
    /*////////Configurações CANAL_05////////////////*/
    if(!strcmp(parametro,"CANAL_05_NOME")){
        strncpy(estacao.canal_05.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_05.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <%s>\n", estacao.canal_05.nome );
    }else if(!strcmp(parametro,"CANAL_05_PINO")){
        estacao.canal_05.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_05.pino );
    }else if(!strcmp(parametro,"CANAL_05_INCLINAÇÃO")){
        estacao.canal_05.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_05.inclinacao );
    }else if(!strcmp(parametro,"CANAL_05_ACRÉSCIMO")){
        estacao.canal_05.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_05.acrescimo );
    }else if(!strcmp(parametro,"CANAL_05_CASAS_DECIMAIS")){
        estacao.canal_05.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_05.casas_decimais );
    }else if(!strcmp(parametro,"CANAL_05_AMPLITUDE")){
        estacao.canal_05.amplitude = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_05.amplitude );
    }else
    /*////////Configurações CANAL_06////////////////*/
    if(!strcmp(parametro,"CANAL_06_NOME")){
        strncpy(estacao.canal_06.nome, parametro_valor, strlen(parametro_valor));
        estacao.canal_06.nome[strlen(parametro_valor)]='\0';
        //Serial.printf( "   <<%s>>\n", estacao.canal_06.nome );
    }else if(!strcmp(parametro,"CANAL_06_PINO")){
        estacao.canal_06.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_06.pino );
    }else if(!strcmp(parametro,"CANAL_06_INCLINAÇÃO")){
        estacao.canal_06.inclinacao = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_06.inclinacao );
    }else if(!strcmp(parametro,"CANAL_06_ACRÉSCIMO")){
        estacao.canal_06.acrescimo = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_06.acrescimo );
    }else if(!strcmp(parametro,"CANAL_06_CASAS_DECIMAIS")){
        estacao.canal_06.casas_decimais = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_06.casas_decimais );
    }else if(!strcmp(parametro,"CANAL_06_AMPLITUDE")){
        estacao.canal_06.amplitude = atof( parametro_valor );
        //Serial.printf( "   <%f>\n", estacao.canal_06.amplitude );
    }else
    /*////////Configurações CANAL_07////////////////*/
    if(!strcmp(parametro,"CANAL_07_NOME")){
      strncpy(estacao.canal_07.nome, parametro_valor, strlen(parametro_valor));
      estacao.canal_07.nome[strlen(parametro_valor)]='\0';
      //Serial.printf( "   <%s>\n", estacao.canal_07.nome );
    }else if(!strcmp(parametro,"CANAL_07_PINO")){
        estacao.canal_07.pino = atoi( parametro_valor );
        //Serial.printf( "   <%d>\n", estacao.canal_07.pino );
    }else
    /*////////MATRIZ TRANSMISSÃO////////////////*/
    if(!strcmp(parametro,"MATRIZ_TRANSMISSÃO")){
        //strncpy(estacao.ma, parametro_valor, strlen(parametro_valor));
        //estacao.nome[strlen(parametro_valor)]='\0';
        ////Serial.printf( "   <%s>\n", estacao.nome );
    }

  }
  return 1;
}

int carregar_arquivo_estacao(){
  if(!file.open("/config.ini", FILE_WRITE)) {
    display.clear();
    display.drawString(0, 0, "Erro ao abrir SD-1!");
    display.drawString(0, 10, "Erro 00 04");
    display.drawString(0, 20, "Informações da PCD não");
    display.drawString(0, 30, "encontradas! As configurações");
    display.drawString(0, 40, "padrões serão estabelecidas.");
    Serial.println("00 04");//erro 4, leitura SD card principal falhou!
    display.display();
    delay(5000);
    return 0;
  }else{
    int   count=0;
    int   tamanho_arquivo = file.size();
    //char  arquivo[tamanho_arquivo];
    file.rewind();//rebobina o arquivo (volta para o inicio)
    while (file.available()) {
      char progresso[7];//porcentagem de carregamento do arquivo "loading"
      double a = 100*100*(count+1)/tamanho_arquivo;
      if(tamanho_arquivo>0){
        sprintf(progresso,"%.2f%%",a/100);
      }else{
        sprintf(progresso, "Não há dados!",count);
      }
      display.clear();
      display.drawString(0, 0, "Leitura dos dados da");
      display.drawString(0, 10,"estação em progresso...");
      display.drawString(0, 30, progresso);
      display.display();
      arquivo[count]=file.read();
      //Serial.printf("%c",arquivo[count]);
      count++;//acrescenta +1 a cada caractere que lê do arquivo config.ini
    }
    delay(200);
    file.close();
    //Serial.printf("%s", arquivo);
    //Serial.printf("%d", strlen(arquivo));
    return 1;
  }
}

double a = 0;
int i = 0;
double vetor[100]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
//analogSetCycles(32,255);
  int comando_serial = 0;
  char aux[6]="";





void loop() {
  //double b = analogRead(32);
  //double c = ((100*(b-678.811f)/3394.054f*1000));
  //if(((a/c-1)<=0,95)||((a/c-1)>=-0.95)){
    //a = (0.98f*a+(100*(b-678.811f)/3394.054f*1000))*0.02f;
    //printf("aqui");
  //}
  //a = 0.95f*a + (100*((analogRead(32)-678.811f)/3394.054f*1000))*0.05f;
  //a = (100*((analogRead(32)-678.811f)/3394.054f*1000));
  //a = ((100*analogRead(32))/4095*1000);
  //a = 0.95f*a+0.05f*((100*analogRead(32))/4095*1000);


  if (Serial.available()) {
    comando_serial = Serial.read()-48;
    if( comando_serial!=-38){
      char cmd[4];
      itoa(comando_serial,cmd,10);
      strcat(aux,cmd);
    }else{
      aux[6]='\0';
      if(!strcmp(aux,"110103")){
        Serial.print("Comando: ");
        Serial.println(aux);
        Serial.println("Comando Reconhecido!");
      }else{
        Serial.print("Comando: ");
        Serial.println(aux);
        Serial.println("Comando Não Reconhecido!");
      }
      comando_serial = 0;
      sprintf(aux,"",comando_serial);
    }
  }



  a = ((100*analogRead(39))/4095*1000);
  //a = 0.95f*a+0.05f*((100*analogRead(32))/4095*1000);
  vetor[i]=a/100;
  i++;
  if(i==101){
    i=0;
  }
  double acumulado = 0;
  for(int x = 0;x<=100;x++){
    acumulado = acumulado+vetor[x];
  }

  //Serial.println(acumulado/100);
  //Serial.print(" ");

  if( segundos!=myClock.getSecond() ){//contagem dos segundos

    //segundos = myClock.getSecond();
    //if(segundos==59){
    //segundos=0;
    //}
    atualizar_horario();
    display.clear();
    display.drawString(x*10, 0, estacao.nome);
    display.drawString(0, 10,data_hora);
    char aux1[10];
    char aux2[10];
    //itoa(analogRead(38)/4095*2,76,aux1,10);
    //double a = 100*((analogRead(32)-678.811f)/3394.054f*1000);
    //printf("%.0f",a/100);


    sprintf(aux1,"%.0f cm",acumulado/100-156);
    //Serial.println(acumulado/100);
    //Serial.print(" ");
    display.drawString(0, 20,aux1);
    itoa(analogRead(32),aux2,10);
    display.drawString(0, 30,aux2);

    display.display();
    if( strlen(estacao.nome) > 30 ){
      x--;
      if( x*10 <= 64-strlen(estacao.nome)*4 ){
        x=0;
      }
    }

    if( segundos == 0 ){
      if ( myClock.getMinute() >= estacao.intervalo_transmissao || myClock.getMinute() == 0 ){
        int aux = (int) myClock.getMinute()/estacao.intervalo_transmissao;
        minutos_acumulados = myClock.getMinute() - estacao.intervalo_transmissao*aux;
        if( minutos_acumulados == 0 ){
          minutos_acumulados = estacao.intervalo_transmissao;
        }
      }else{
        minutos_acumulados = myClock.getMinute();
      }
    }

    if( minutos_acumulados == estacao.intervalo_transmissao ){
      //minutos  = myClock.getMinute();//coleta um minuto antes
      minutos_acumulados=0;
      //DateTime now = myClock.now();
      Serial.print(myClock.getDate(), DEC);
      Serial.print('/');
      Serial.print(myClock.getMonth(century), DEC);
      Serial.print('/');
      Serial.print(myClock.getYear(), DEC);
      Serial.print(' ');
      Serial.print(myClock.getHour(h12Flag, pmFlag), DEC);
      Serial.print(':');
      Serial.print(myClock.getMinute(), DEC);
      Serial.print(':');
      Serial.print(myClock.getSecond(), DEC);
      Serial.println();
      if ( conectar_sd_princial() ) {
        gravar_dados();
        digitalWrite(SD_CS_PRINCIPAL, HIGH);
        digitalWrite(SD_CS_SECUNDARIO, HIGH);
      }else{
        delay(1000);
        minutos_acumulados=estacao.intervalo_transmissao;
      }

      //display.clear();
      //display.drawString(0, 0, estacao.nome);
      //display.drawString(0, 10,data_hora);
      //display.drawString(0, 30,data_hora);





//            //parsePacket: checa se um pacote foi recebido
//      //retorno: tamanho do pacote em bytes. Se retornar 0 (ZERO) nenhum pacote foi recebido
//      int packetSize = LoRa.parsePacket();
//      //caso tenha recebido pacote chama a função para configurar os dados que serão mostrados em tela
//      if (packetSize) {
//        cbk(packetSize);
//            File test = SD.open(LOG_PATH, FILE_APPEND);
//        if (!test) {
//            Serial.println("SD Card: writing file failed.");
//        } else {
//            Serial.printf("SD Card: appending data to %s.\n", LOG_PATH);
//            //test.write(lora_buf, lora_len);
//            test.printf("DADO;%c\n",packet[0]);
//            //Serial.printf("DADO;%s\n",LoRa.read());
//           test.close();
//        }
//        delay(2000);
//      }


    }

  }


    //Serial.print(" since midnight 1/1/1970 = ");
    //Serial.print(now.unixtime());
    //Serial.print("s = ");
    //Serial.print(now.unixtime() / 86400L);
    //Serial.println("d");

}

//essa função apenas imprime o logo na tela do display
void logo()
{
  //apaga o conteúdo do display
  display.clear();
  //imprime o logo presente na biblioteca "images.h"
  display.drawXbm(0,5,logo_width,logo_height,logo_bits);
  display.display();
}

//função responsável por recuperar o conteúdo do pacote recebido
//parametro: tamanho do pacote (bytes)
void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC); //transforma o tamanho do pacote em String para imprimirmos
  for (int i = 0; i < packetSize; i++) {
    packet += (char) LoRa.read(); //recupera o dado recebido e concatena na variável "packet"
    //Serial.printf("DADO;%c\n",packet[i]);
  }

  rssi = "RSSI=  " + String(LoRa.packetRssi(), DEC)+ "dB"; //configura a String de Intensidade de Sinal (RSSI)
  //mostrar dados em tela
  loraData();
}

//função responsável por configurar os dadosque serão exibidos em tela.
//RSSI : primeira linha
//RX packSize : segunda linha
//packet : terceira linha
void loraData(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0 , 18 , "Rx "+ packSize + " bytes");
  display.drawStringMaxWidth(0 , 39 , 128, packet);
  display.drawString(0, 0, rssi);
  display.display();
}
