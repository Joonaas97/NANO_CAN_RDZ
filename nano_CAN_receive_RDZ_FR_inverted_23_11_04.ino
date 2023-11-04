// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>
#include <defaults.h>
#include <math.h>
#include <global.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10
double FL=0;  //Raddrehzahl FL
double FL2=0;  //Raddrehzahl FL2 für Timer2


void setup()
{
  Serial.begin(1200);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  



DDRD = DDRD | B00111100;  // sets Arduino pins D2 to D5 as outputs
DDRC = DDRC |B00001111;  // sets pins A0 to A3 as outputs
delay(1000);


cli();//stop all interrupts
  // turn on CTC mode
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaler 8 https://nerd-corner.com/de/arduino-timer-interrupts-arduino-register-programmieren/
  TCCR1B |= (1 << CS11);
  //TCCR1B |= (1 << CS10); 
  
  //initialize counter value to 0;
  TCNT1  = 0;
  
  // set timer count for 50Hz increments
  OCR1A = 24999;  // = (16*10^6) / (10*64) - 1  25000=100ms/10Hz bei CS10 und CS11=1
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
  //END TIMER SETUP

setupTimer2();

}



ISR(TIMER1_COMPA_vect) {
        if(OCR1A<65000){
         
        //PORTC= B00001111 ; 
        PORTD= B00111100 ;
 
        delayMicroseconds(52);
         //PORTC =B00000000; 
         PORTD =B00000000;       
        delayMicroseconds(26);

        //PORTD= B00111100 ;
        PORTC= B00001111 ;         
 

        delayMicroseconds(26);

         //PORTD =B00000000;
         PORTC =B00000000;
         delayMicroseconds(26);

        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(26);
        
        //PORTD =B00000000;
        PORTC =B00000000;
                delayMicroseconds(26);
        
        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(26);
        
        //PORTD =B00000000;
        PORTC =B00000000;
                 delayMicroseconds(52);
        
        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(51);
        
        //PORTD =B00000000;
        PORTC =B00000000;
                 delayMicroseconds(51);
        
        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(51);
        
        //PORTD =B00000000;
        PORTC =B00000000;
                 delayMicroseconds(51);
        
        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(26);
        
        //PORTD =B00000000;
        PORTC =B00000000;
                 delayMicroseconds(26);
        
        //PORTD= B00111100;
        PORTC= B00001111 ;
                delayMicroseconds(26);
        
        //PORTD =B00000000;
        PORTC =B00000000;

}


        //Serial.println(FL,DEC);
         
        //OCR1A =2000000*FL;// = (16*10^6) / (50*64) - 1  25000=100ms
        //OCR1A =65000; Maximum!!
        //Serial.println(FL);
        //delay(FL);
        //Serial.println(micros());  
}

//*************Timer 2****************//

volatile int divider=0;


void setupTimer2() {
  noInterrupts();
  // Clear registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // 668.4491978609626 Hz (16000000/((186+1)*128))
  OCR2A = 186;
  // CTC
  TCCR2A |= (1 << WGM21);
  // Prescaler 128
  TCCR2B |= (1 << CS22) | (1 << CS20);
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}




ISR(TIMER2_COMPA_vect) {
  if(divider==0){
        //PORTD= B00111100 ; 
        PORTC= B00001111 ;        
        delayMicroseconds(51);

         PORTC =B00000000;
         delayMicroseconds(26);

        //PORTD= B00111100;
        PORTC= B00001111 ;
        delayMicroseconds(26);

        PORTC =B00000000;
        
        delayMicroseconds(51);

        //PORTD= B00111100;
        PORTC= B00001111 ;
        delayMicroseconds(51);
  
          PORTC =B00000000;       
         delayMicroseconds(26);
  
        //PORTD= B00111100;       //1
        PORTC= B00001111 ;
        delayMicroseconds(26);
        
        PORTC =B00000000;       
         delayMicroseconds(26);
  
        //PORTD= B00111100;      // 2
        PORTC= B00001111 ;
        delayMicroseconds(26);
        PORTC =B00000000;
         delayMicroseconds(26);
      
        //PORTD= B00111100;       //3
        PORTC= B00001111 ;
        delayMicroseconds(26);
        PORTC =B00000000;
         delayMicroseconds(26);
  
        //PORTD= B00111100;       //4
        PORTC= B00001111 ;
        delayMicroseconds(26);
        PORTC =B00000000;
         delayMicroseconds(26);
  
        //PORTD= B00111100;       //5
        PORTC= B00001111 ;
        delayMicroseconds(26);
        PORTC= B00000000 ;
        delayMicroseconds(51);
        PORTC= B00001111 ;
        delayMicroseconds(26);
        PORTC =B00000000;

          

  }
  divider++;
  divider%=100;
}





void loop()
{
    if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
      {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
        if((rxId) == 0xCE) 
          {
            //FL =(rxBuf[0])|(rxBuf[1]<<8); 0/1=FL ; 2/3=FR ;4/5=RL ;6/7=RR
            FL =(rxBuf[2])|(rxBuf[3]<<8);
            //Serial.print(FL);
            //FL=2567000/FL;
            FL=0.95*2567000/FL;
            FL2=FL;
            if(FL>65000)
            {FL=65000;
              }
            OCR1A=FL;
            }
          
      }
    
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
