#include <Arduino.h>
/* 
Pinos USB:      PA11 e PA12 - Evitar.
Pinos 5V:		PB10, PB11, PB12, PB13, PB14, PB15, PA8, PA9, PA10, PA11, PA12, PA15, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB2 (Boot1), PA13 (SWDIO), PA14 (SWCLK)
Pinos PWM: 		PB1, PB0, PA7, PA6, PA3, PA2, PA1, PA0, PA8, PA9, PA11, PB6, PB7, PB8, PB9
Pinos TX/RX:	PA2/PA3, PB10/PB11, PA9/PA8, PB6/PB7
Pinos ADC:      xPB1, xPB0, xPA7, xPA6, PA5, PA4, PA3, PA2, PA1, PA0    (todos 3V3 máx)
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

// Pinos ultrassom:
#define TRIG     PB12   //saída
#define ECHO     PB13   //entrada - precisa ser tolerante a 5V!

char sprintf_buffer[255];

//Controle PID:
int sd2, sd1, sc, se1, se2;
float dist_obst;
float erro_atual, erro_anterior;
float kp = 25, kd = 25;            //constante proporcional, constante derivativa. Ajustar experimentalmente.

float calculaPID(){
  float p = erro_atual;
  //float i = i + error;   //componente integral não será usada.
  float d = erro_atual-erro_anterior;
  erro_anterior = erro_atual;   //d já calculado, então isso interfere apenas na iteração futura.
  return (kp*p) /*+ (ki*i)*/ + (kd*d);
}

// Funções de acesso aos motores:

int verifica_obst(){    
    //http://stm32duino.com/viewtopic.php?t=106
    digitalWrite(TRIG,HIGH);
    delay(1);
    digitalWrite(TRIG,LOW);

    dist_obst  = pulseIn(ECHO,HIGH) / 58.138f;
}

void linha(){
    sd2 = digitalRead(S_DIR2);
    sd1 = digitalRead(S_DIR1);
    sc =  digitalRead(S_CENTRO);
    se1 = digitalRead(S_ESQ1);
    se2 = digitalRead(S_ESQ2);

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
    
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    pinMode(PC13, OUTPUT);  //LED integrado
    Serial.begin(115200);
    Serial.print(F("Setup OK\n\n"));
}

void loop() {
    static int lastmillis;
    verifica_obst();
    linha();
    if (millis() - lastmillis > 250) {
        sprintf(sprintf_buffer,"Leitura: %d %d %d %d %d\tErro Atual/Anterior: %f/%f\n Obstáculo a: %f cm\n\n",se2, se1, sc, sd1, sd2, erro_atual, erro_anterior, dist_obst);
        Serial.print(sprintf_buffer);
        lastmillis = millis();
    }
}