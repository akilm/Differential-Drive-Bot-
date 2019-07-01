#include <avr/io.h> 
#include <avr/interrupt.h>
#define FOSC 16000000                                         // Clock Speed
#define BAUD 9600                
#define MYUBRR FOSC/16/BAUD -1
#define pi 3.14
float wheel1=0,wheel2=0;
uint8_t rpm_1=0,rpm_2=0,R=0.03,H=10;
uint8_t spd=0;
char s;
float kp,ki,kd;    // values need to be tuned appropriately
float cur_speed=0,error=0,prev_error=0,PID;
float X=0,Y=0;
float Vl,Vr,del_l,del_r;
float R_turn,Wd,heading ;
float set_speed;
int PID_flag=0;
float pid_p=0,pid_d=0,pid_i=0;
void timer1_speedometer_init()
{
    TCCR1B |= (1 << WGM12)|(1 << CS12)  ;                     // set up timer with prescaler = 256 and CTC mode
    TCNT1 = 0 ;                                               // initialize counter
    OCR1A = 31249 ;                                           // initialize compare value
    TIMSK1 |= (1 << OCIE1A);                                  // enable interrupt which calls the ISR every 0.5 seconds                                                    
}
void timer2_x_y_init()
{
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22)|(1 << CS21);
    TCNT2 = 0;
    OCR2A = 15624  ;
    TIMSK2 |= (1<<OCIE2A);
}
unsigned char usart_receive (void)
{
    while(!(UCSR0A & (1<<7)));                                // wait while data is being received
    return UDR0;                                              // return 8-bit data
}
void usart_transmit (uint8_t data)
{
    while (!( UCSR0A & (1<<5)));                              // wait while register is free
    UDR0 = data;                                              // load data in the register
}
float bound_Angle(float theta)
{  
  
  if(theta>360.0)
    theta=theta-360;
   else
   if(theta<0)
    theta=theta+360;
  
}
  
ISR (USART_RXC_vect)
{
    s = UDR0;
}
/* Speedometer : calculates the speed of the bot with the help of RPM's of both the wheels 
 */
ISR (INT0_vect)  
{ wheel1 += .05;
}
ISR(INT1_vect)
{ wheel2 += .05;
}
ISR(TIMER1_COMPA_vect)
{ 
  rpm_1=(wheel1/0.5)*60;
  rpm_2=(wheel2/0.5)*60;
  wheel1=0;
  wheel2=0;
  cur_speed= ((R*(rpm_1+rpm_2)*2*pi)/(2*60))*100;             // speed in cm per s         
  usart_transmit(spd);                                       //speed transmitted serially to PC using HC05(bluetooth module)  
  usart_transmit(X);
  usart_transmit(Y);
  
}
ISR(TIMER2_COMPA_vect)
{ 
  
  Vl = (rpm_1/60)*2*pi*R;
  del_l= Vl*0.25;
  Vr = (rpm_2/60)*2*pi*R;
  del_r= Vr*0.25;
  R_turn= H * (del_l + del_r) / (2 * (del_r - del_l));
  Wd = (del_r - del_l) / H;
  X = X + R_turn * sin(Wd + heading) -  R_turn * sin(heading);
  Y = Y - R_turn * cos(Wd + heading) +  R_turn * cos(heading);
  heading = bound_Angle(heading + Wd);
  
}

int main(void)
{   
  
    UBRR0H = (MYUBRR >> 8);
    UBRR0L = MYUBRR;
         // Enable receiver and transmitter
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);                       
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);   // Set frame: 8data, 1 stp
    DDRD |= (1 << 6)|(1<< 5);
    
    TCCR0A |= (1 << COM0A1)|(1 << COM0B1);
    TCCR0A |= (1 << WGM00); 
    TCCR0B |= (1 << CS01);
    OCR0A = OCR0B = 0;                                                    //PD5(pin 5):IN1 and PD6(pin 6):IN3 used for pwm 
    DDRB|=(1<<3)|(1<<4);                                                  //PD3(pin 3):IN2 and PD4(pin 4):IN4 used for turning and reversing
    PORTB&=~(1<<3)|(1<<4);
    
    timer1_speedometer_init();
    timer2_x_y_init();
    
    DDRD&=~(1<<2)|(1<<3);
    
    EICRA |= (1 << ISC11)|(1 << ISC10)|(1 << ISC01)|(1 << ISC00);
    EIMSK |= (1 << INT0)|(1 << INT1); 
    sei(); 
    
    set_speed=uint8_t(usart_receive());
    UCSR0B |= (1 << RXCIE0);                   //enable receive interrupt
    while(1)
    {   
      
      if(s=='w')                //move forward
      { OCR0A = 255;
        OCR0B = 255;
        s='0';
        PID_flag=1;
      }
      else
      if(s=='a')                //turn right
      { OCR0A=255;
        OCR0B=0;
        s='0';
        PORTB|=(1<<4);
        _delay_ms(700);
        PORTB&=~(1<<4);
        OCR0A=0;
        PID_flag=0;
      }
      else
      if(s=='d')                //turn left
      { 
        OCR0B=255;
        OCR0A=0;
        s='0';
        PORTB|=(1<<3);
        _delay_ms(700);
        PORTB&=~(1<<3);
        OCR0B=0;
        PID_flag=0;
      }
      else
      if(s=='s')                //move backward
      { 
        OCR0B=0;
        OCR0A=0;
        s='0'; 
        PID_flag=0;                
      }
      else
      if(s=='x')                //stop 
      { OCR0B=0;
        OCR0A=0;
        s='0';
        PID_flag=0;
       }
     if(PID_flag==1)
     {  
        error=set_speed-cur_speed;
        pid_p = kp*error;
        pid_i = pid_i+(ki*error);
        pid_d = kd*(error - prev_error);
        prev_error=error;
        PID = pid_p + pid_i + pid_d;
        OCR0A += PID;
        OCR0B += PID;
     }
   }
    return 0;
}
