#include <Arduino.h>
/* 
Pinos USB:      PA11 e PA12 - Evitar.
Pinos 5V:		PB10, PB11, PB12, PB13, PB14, PB15, PA8, PA9, PA10, PA11, PA12, PA15, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB2 (Boot1), PA13 (SWDIO), PA14 (SWCLK)
Pinos PWM: 		PB1, PB0, PA7, PA6, PA3, PA2, PA1, PA0, PA8, PA9, PA11, PB6, PB7, PB8, PB9
Pinos TX/RX:	PA2/PA3, PB10/PB11, PA9/PA8, PB6/PB7
Pinos ADC:      PB1, PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0    (todos 3V3 máx)
*/

// Pinos Sensores utilizados para identificar a linha 
#define S_ESQ2      PB10
#define S_ESQ1      PA5
#define S_CENTRO    PA4
#define S_DIR1      PB14
#define S_DIR2      PB15
#define S_VERDE_ESQ PA15
#define S_VERDE_DIR PB3
// Pinos Motores...
#define M_RODA_DIR_HOR PB8
#define M_RODA_DIR_ANT PB9
#define M_RODA_ESQ_HOR PA0
#define M_RODA_ESQ_ANT PA1
#define M_SERVO1  PB0
#define M_SERVO2  PB1
// Pinos encoders (requerem timers). Não devem mais ser necessários.
// https://github.com/chrisalbertson/quadratureBluePill/blob/master/examples/quadratureTest01/quadratureTest01.ino
// #define DT1     PA8
// #define CLK1    PA9
// #define DT2     PA6
// #define CLK2    PA7
// Pinos ultrassom:
#define TRIG     PB12   //saída
#define ECHO     PB13   //entrada - precisa ser tolerante a 5V!
// Pinos display i2c:
// #define SDA PB7 
// #define SCL PB6
// Pinos Serial HC-05:
// #define TX  PA2
// #define RX  PA3

// Estados da máquina de estados finitos:
#define VERIFICA  0   //Verifica sensores
#define LINHA     1 
#define ESQ90     2
#define DIR90     3   
#define RET180    4   //Faz a volta e retorna
#define CONTORNA  5   //Contorna obstáculo

#define RAMPA       6
//Mais estados a definir posteriormente, para salvamento após a rampa.

#define VEL_NORMAL 32768 //Velocidade padrão do PWM (até 65535);
#define TEMPO_90   200   //tempo para girar aprox. 90 graus em ms. Definir experimentalmente;
#define TEMPO_180  350   //tempo para girar aprox. 180 graus em ms Definir experimentalmente;

int estado = VERIFICA;                //estado atual da máquina de estados finitos
//int dist_encruzilhada = 0;          //distância calculada pelo encoder.

//Controle PID:
double erro_atual, erro_anterior;
double kp = 25, kd = 25;            //constante proporcional, constante derivativa. Ajustar experimentalmente.

double calculaPID(){
  double p = erro_atual;
  //double i = i + error;   //componente integral não será usada.
  double d = erro_atual-erro_anterior;
  erro_anterior = erro_atual;   //d já calculado, então isso interfere apenas na iteração futura.
  return (kp*p) /*+ (ki*i)*/ + (kd*d);
}

// Funções de acesso aos motores:

void corrige_motores(){
    //usa funções do STM32 em vez de analogWrite para melhorar a resolução do PWM (16 vs 8 bits):
    pwmWrite(M_RODA_DIR_HOR, VEL_NORMAL + calculaPID()); 
    pwmWrite(M_RODA_ESQ_HOR, VEL_NORMAL - calculaPID());
}

void curva_verde(int tipo){
    if (tipo == RET180 ){
        for(int i=0 ; i < TEMPO_180 ; i++){
            pwmWrite(M_RODA_DIR_HOR, VEL_NORMAL); 
            pwmWrite(M_RODA_ESQ_ANT, VEL_NORMAL);
            delay(1);
        }
    } else if (tipo == ESQ90 ){
        for(int i=0 ; i < TEMPO_90 ; i++){
            pwmWrite(M_RODA_DIR_ANT, VEL_NORMAL); 
            pwmWrite(M_RODA_ESQ_HOR, VEL_NORMAL);
            delay(1);
        }
    } else if (tipo == DIR90 ){
        for(int i=0 ; i < TEMPO_90 ; i++){
            pwmWrite(M_RODA_DIR_HOR, VEL_NORMAL); 
            pwmWrite(M_RODA_ESQ_ANT, VEL_NORMAL);
            delay(1);
        }
    }
}

// Funções de acesso aos sensores 

int verifica_acc(){
    return 0; //retornam 0 se normal, 1 se entrou na rampa:
}

int verifica_obst(){    //A implementar. Deve ser relativamente simples, divirta-se.
    return 0; //retornam 0 se normal, 1 se enncontrou caixa
}

void verifica_verdes(){
//    if ( dist_encruzilhada > 100 ) {    //Não deve ser mais necessário.
        int verde_esq = digitalRead(S_VERDE_ESQ);
        int verde_dir = digitalRead(S_VERDE_DIR);
        int linha_esq = digitalRead(S_ESQ2);
        int linha_dir = digitalRead(S_DIR2);

        if      ( verde_esq && verde_dir && linha_esq && linha_dir )    estado = RET180;
        else if ( verde_esq && verde_dir && linha_esq && !linha_dir )   estado = RET180;
        else if ( verde_esq && verde_dir && !linha_esq && linha_dir )   estado = RET180;
        else if ( verde_esq && linha_esq && !verde_dir )                estado = ESQ90;
        else if ( verde_dir && linha_dir && !verde_esq )                estado = DIR90;
        else                                                            estado = LINHA;
//    } else  estado = LINHA;
}

void linha(){
    // Primeiro garantir que ignorará encruzilhada
    while ( digitalRead(S_ESQ2) && digitalRead(S_DIR2) ){
        //Continua em linha reta até terminar encruzilhada. 
        //Releia sensores a cada iteração do loop.
        //dist_encruzilhada = 0; //Não deve ser mais necessário.
        erro_atual = 0; //avance em linha reta.
        corrige_motores();
    }

    int sd2 = digitalRead(S_DIR2);
    int sd1 = digitalRead(S_DIR1);
    int sc =  digitalRead(S_CENTRO);
    int se1 = digitalRead(S_ESQ1);
    int se2 = digitalRead(S_ESQ2);

    //Cobrir gap: continua em linha reta se não há sensor lido;
    /* Proposta de lógica para corrigir desalinhamento */
    // Usar PID? Sugestão para controle via PID, menos compacta que a versão apresentada aqui: 
    // https://www.instructables.com/id/Line-Follower-Robot-PID-Control-Android-Setup/
    if      ( se2 && !se1 && !sc && !sd1 && !sd2) erro_atual = -4; //10000
    else if ( se2 &&  se1 && !sc && !sd1 && !sd2) erro_atual = -3; //11000
    else if (!se2 &&  se1 && !sc && !sd1 && !sd2) erro_atual = -2; //01000
    else if (!se2 &&  se1 &&  sc && !sd1 && !sd2) erro_atual = -1; //01100
    else if (!se2 && !se1 &&  sc && !sd1 && !sd2) erro_atual =  0; //00100
    else if (!se2 && !se1 &&  sc &&  sd1 && !sd2) erro_atual =  1; //00110
    else if (!se2 && !se1 && !sc &&  sd1 && !sd2) erro_atual =  2; //00010
    else if (!se2 && !se1 && !sc &&  sd1 &&  sd2) erro_atual =  3; //00011
    else if (!se2 && !se1 && !sc && !sd1 &&  sd2) erro_atual =  4; //00001
    else erro_atual =  0; //sem linha ou leitura anormal, continue para frente
    //if (!se2 && !se1 && !sc && !sd1 && !sd2) erro_atual = 0;   //sem linha, continue para frente. Movido acima
    corrige_motores();
}

// Geral

void setup() {
    pinMode(S_DIR1,     INPUT);
    pinMode(S_DIR2,     INPUT);
    pinMode(S_ESQ1,     INPUT);
    pinMode(S_ESQ2,     INPUT);
    pinMode(S_CENTRO,   INPUT);

    pinMode(M_RODA_DIR_HOR,  OUTPUT);
    pinMode(M_RODA_ESQ_HOR,  OUTPUT);
    pinMode(M_RODA_DIR_ANT,  OUTPUT);
    pinMode(M_RODA_ESQ_ANT,  OUTPUT);
    pinMode(M_SERVO1,    OUTPUT);
    pinMode(M_SERVO2,    OUTPUT);
    
    pinMode(TRIG, INPUT);
    pinMode(ECHO, INPUT);

    pinMode(PC13, OUTPUT);  //LED integrado
    Serial.begin(115200);
}

void loop() {
    switch (estado){
        case VERIFICA:
            if      (verifica_acc())  estado = RAMPA;
            else if (verifica_obst()) estado = CONTORNA;
            else    verifica_verdes();
            break;
        case LINHA:
            linha();
            estado = VERIFICA; break;
        case ESQ90:
            curva_verde(ESQ90);
            estado = VERIFICA; break;
        case DIR90:
            curva_verde(DIR90);
            estado = VERIFICA; break;
        case RET180:
            curva_verde(RET180);
            estado = VERIFICA; break;
        case RAMPA:
            /* Daqui pra frente, fodeu */
            break;
    }
}