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
#define M_SERVO1  PB0      // <===========================================    A definir ! ========================================================>
#define M_SERVO2  PB1      // Suprimir caso se opte por atuador RC Servo.
// Pinos encoders (requerem timers): https://github.com/chrisalbertson/quadratureBluePill/blob/master/examples/quadratureTest01/quadratureTest01.ino
#define DT1     PA8
#define CLK1    PA9
#define DT2     PA6
#define CLK2    PA7
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

int estado = 0;             //estado atual da máquina de estados finitos
int dist_encruzilhada = 0;  //distância calculada pelo encoder.

int verifica_acc(){
    return 0; //retornam 0 se normal, 1 se entrou na rampa:
}

int verifica_obst(){
    return 0; //retornam 0 se normal, 1 se enncontrou caixa
}

void verifica_verdes(){
    if ( dist_encruzilhada > 100 ) {    //verifica verdes apenas se não acabou de sair de encruzilhada.

        int verde_esq = digitalRead(S_VERDE_ESQ);
        int verde_dir = digitalRead(S_VERDE_DIR);

        if      ( verde_esq == 1 && verde_dir == 1 ) estado = RET180;
        else if ( verde_esq == 1 && verde_dir == 0 ) estado = ESQ90;
        else if ( verde_esq == 0 && verde_dir == 1 ) estado = DIR90;
        else    estado = LINHA;
    } else  estado = LINHA;
}

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
            if          (verifica_acc())  estado = RAMPA;
            else if     (verifica_obst()) estado = CONTORNA;
            else        verifica_verdes();
            break;
        case LINHA:
            /* incluir aqui lógica para corrigir desalinhamento*/

            while ( digitalRead(S_ESQ2) && digitalRead(S_DIR2) ){
                //Reseta trajeto lido pelo encoder e continua em linha reta até terminar encruzilhada
                dist_encruzilhada = 0;
                analogWrite(M_RODA_DIR_HOR, 128);
                analogWrite(M_RODA_DIR_HOR, 128);
            };
            estado = VERIFICA; break;
        case ESQ90:
            
            estado = VERIFICA; break;
        case DIR90:
            
            estado = VERIFICA; break;
        case RET180:
            
            estado = VERIFICA; break;
        case RAMPA:
            
            break;
    }
}