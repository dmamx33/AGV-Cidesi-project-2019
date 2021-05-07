#include <18F4550.h>          //Microcontrolador a utilizar
//#include <math.h>
#device adc=10                 //Lecturas análogas de 8 bits
#device HIGH_INTS=TRUE //Activamos niveles de prioridad

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale

//Para operacion 16 Mhz con cristal de 16MHz
#FUSES HSPLL                        //High speed Osc (> 4mhz)
#FUSES PLL5                    //Divide By 5(20MHz oscillator input)
#FUSES CPUDIV1                //Post preescalador del frecuencia de PL
#use delay(clock=48000000)
//#use delay(clock=16MHz ,crystal=16MHz)

#FUSES NOPROTECT                //Code not protected from readin
#FUSES BROWNOUT                 //Reset when brownout detected
#FUSES BORV21                   //Brownout reset at 2.0V
#FUSES PUT                  //Power Up Timer Activado
#FUSES NOCPD                    //No EE protection
#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NODEBUG                  //No Debug mode FOR ICD
#FUSES NOLVP                  // No Low Voltage Programming on B3(PIC16) or B5(PIC18)
#FUSES NOWRT                    //Program memory not write protected
#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES IESO                     //Internal External SWITCH Over mode enabled
#FUSES FCMEN                    //Fail-safe clock monitor enabled
//#FUSES NOPBADEN                 //PORTB pins are configured as digital I/O on RESET
#FUSES PBADEN                 //PORTB pins are configured as digital I/O on RESET
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOEBTRB                  //Boot block not protected from table reads
#FUSES NOCPB                    //No Boot Block code protection
#FUSES NOMCLR                     //Master Clear Desactivado para usar RE3 como pin Input
#FUSES NOLPT1OSC                //Timer1 configured FOR higher power operation
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
///////////////////////////////////////////////////////////////////////////
#include <math.h>
//#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)
#byte STATUS=0xFD8            //Declaramos el Registro STATUS
#byte ADCON1=0XFC1       
#byte ADCON2=0xFC0
#byte ADCON0=0xFC2

#byte CMCON=0XFB4
#byte INTCON2=0xFF1
#byte INTCON=0xFF2
#byte PIR1=0xF9E
#byte PIE1=0xF9D

#byte UCON=0xF6D
#byte UCFG=0xF6F

#byte TRISA=0XF92             //Declaramos el Registro TRISA
#byte PORTA=0XF80             //Declaramos el Registro PORTA
#byte LATA=0XF89              //Declaramos el Registro LATA

#byte TRISB=0XF93             //Declaramos el Registro TRISB
#byte PORTB=0XF81             //Declaramos el Registro PORTB
#byte LATB=0XF8A              //Declaramos el Registro LATB

#byte TRISC=0XF94             //Declaramos el Registro TRISC
#byte PORTC=0XF82             //Declaramos el Registro PORTC
#byte LATC=0XF8B              //Declaramos el Registro LATC

#byte TRISD=0xF95             //Declaraciones Puerto D
#byte PORTD=0XF83
#byte LATD=0xF8C

#byte TRISE=0xF96             //Declaraciones Puerto E
#byte PORTE=0xF84
#byte LATE=0xF8D

//!#byte CANAL_0 = 0b11000011
//!#byte CANAL_1 = 0b11000111
//!#byte CANAL_2 = 0b11001011
#define BOTON_C5_APRETADO !bit_test(PORTC,5)//BOTON MENU
#define BOTON_C4_APRETADO !bit_test(PORTC,4)//BOTON LOOP MODE
#define MAX_PAGINA 5
#define DERECHA 1
#define IZQUIERDA 0
#define OTRO 2
#define ADC_BUSY !bit_test(ADCON0,1)
#define OPEN_LOOP false
#define CLOSED_LOOP true
#define ON true
#define OFF false
//#define STOP_PWM 102
//#define MAX_PWM 920
#define STOP_PWM 26
#define STOP_PWM1 23
#define MAX_PWM 230
#define MID_POINT 516
#define SENTIDOv1 bit_clear(LATD,3)//SENTIDO MOTOR TRACCION
#define SENTIDOv2 bit_set(LATD,3)
#define SENTIDOd1 bit_set(LATD,4)//SENTIDO MOTOR DIRECCION
#define SENTIDOd2 bit_clear(LATD,4)
#define TOLERANCE 2
//!#define MAX_IZQUIERDA 352
//!#define MAX_DERECHA 261
//!#define CENTRAL   306
#define MAX_IZQUIERDA 388
#define MAX_DERECHA 265
#define CENTRAL   321
//#define CAL_OFFSET -9.0
#define CAL_OFFSET 7.0
//#define MASTER_LOOP_100m //////DEFINICIONES DE TIEMPO DE MUESTREO 
#define MASTER_LOOP_50m
//#define MASTER_LOOP_1m
#ifdef MASTER_LOOP_100m
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_128
#define VAL_PRECARGA 56160
#endif
#ifdef MASTER_LOOP_50m
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_64
#define VAL_PRECARGA 56160
#endif
#ifdef MASTER_LOOP_25m 
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_32
#define VAL_PRECARGA 56160
#endif
#ifdef MASTER_LOOP_10m
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_64
#define VAL_PRECARGA 63660
#endif
#ifdef MASTER_LOOP_5m
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_32
#define VAL_PRECARGA 63660
#endif
#ifdef MASTER_LOOP_1m
#define CONFIG_TIMER T0_INTERNAL|T0_DIV_8 
#define VAL_PRECARGA 64035
#endif
//////
#define MICROMETRO_TICK 3227
//////    CONFIGURACIONES LCD 16X2   ///////
#define LCD_DB4 PIN_E0
#define LCD_DB5 PIN_E1
#define LCD_DB6 PIN_E2
#define LCD_DB7 PIN_D0
#define LCD_RS PIN_C6
#define LCD_E PIN_C7
#include <lcd1.c>
////////////////////////////////////////////

////////////////////////// V A R I A B L E S  ///////////////////////////////
//short test=FALSE;
//char letra = 'a';
int16 dato;
//int8 counter=10;
int8 PAGINA=5;
double dir_abs=0.0;
double dir_rel=0.0;
int16 spx=0;
int16 spy=0;
double MAX_JSPY=1021.0;
double MAX_JSPX=1021.0;
//double MAXXY = 1021.0;
double JoystickY=0.0;
double JoystickX=0.0;
int16 Totalticks=0;
int16 TickPos=0;
int16 TickNeg=0;
//!int16 metros=0;
//!int16 centimetros=0;
//!int16 milimetros=0;
int16 micrometros=0;
//!int16 residuo_cm=0;
//!int16 residuo_mm=0;
int16 residuo_um=0;
int16 TickPartial=0;
//unsigned int32 distancia=0;
int8 lec_encoder=0x00;
int8 lec_encoder_ant=0x01;
int8 DIRECCION=OTRO;
short Control=OPEN_LOOP;
short Control_ant=CLOSED_LOOP;
short SYSTEM=OFF;
short SYSTEM_ant=ON;
short force=false;

////////////////////// I N T E R R U P C I O N E S /////////////////////////
//!#INT_EXT                        ///// E N C O D E R
//!void  EXT_isr(void) 
//!{
//!   //output_toggle(PIN_B1);
//!   if(Totalticks>=9999) Totalticks=0;
//!   else                 Totalticks++;
//!   
//!   if(DIRECCION==DERECHA) TickPos++;
//!   if(DIRECCION==IZQUIERDA) TickNeg++;
//!   ///&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//!   lec_encoder=((PORTD & 0x06 )>>1);   ///TEST
//!      switch(lec_encoder)
//!   {
//!   case 0:{
//!            if(lec_encoder_ant==2)                       DIRECCION=DERECHA;
//!            if(lec_encoder_ant==1)                       DIRECCION=IZQUIERDA;
//!            if(lec_encoder_ant==0 || lec_encoder_ant==3) DIRECCION=OTRO;
//!            break;
//!            }
//!   case 1:{   
//!            if(lec_encoder_ant==0)                       DIRECCION=DERECHA;
//!            if(lec_encoder_ant==3)                       DIRECCION=IZQUIERDA;
//!            if(lec_encoder_ant==1 || lec_encoder_ant==2) DIRECCION=OTRO;
//!            break;
//!            }
//!   case 2:{
//!            if(lec_encoder_ant==3)                       DIRECCION=DERECHA;
//!            if(lec_encoder_ant==0)                       DIRECCION=IZQUIERDA;
//!            if(lec_encoder_ant==1 || lec_encoder_ant==2) DIRECCION=OTRO;
//!            break;
//!            }          
//!   case 3:{
//!            if(lec_encoder_ant==1)                       DIRECCION=DERECHA;
//!            if(lec_encoder_ant==2)                       DIRECCION=IZQUIERDA;
//!            if(lec_encoder_ant==0 || lec_encoder_ant==3) DIRECCION=OTRO; 
//!            break;
//!            }
//!     }
//!    lec_encoder_ant = lec_encoder;
   ///&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   
//!}
/////----------------------------------------------------------------///////
#INT_EXT1   HIGH                ///// B O T O N   G O / S T O P 
void  EXT1_isr(void) 
{
   //output_toggle(PIN_B1);
   SYSTEM_ant=SYSTEM;//nuevo, supongo que aqui debe ser lo correcto,
   SYSTEM = !SYSTEM;
   if(SYSTEM==OFF)
      {
         set_pwm1_duty(STOP_PWM1);                    // Set pwm1 duty cycle to i
         set_pwm2_duty(STOP_PWM); 
      }
     // SYSTEM_ant=SYSTEM;//Originalmente estaba aqui, no se porque
   PAGINA=4;//Habia un 4, no se porque, se joroba todo con 5, ya se porque
   force=true;   
}
/////----------------------------------------------------------------///////
#INT_TIMER0
void  TIMER0_isr(void) 
{
  output_toggle(PIN_D7);////PRUEBA DE TIMER0      
      set_timer0(VAL_PRECARGA);
   ///////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////// 
   /////////////////////////////////////////////////////////////////////////// 
   /////////////////////////////////////////////////////////////////////////// 
   //distancia=distancia+int32(get_timer1());
   Totalticks = Totalticks + get_timer1();
   TickPartial = get_timer1();
   set_timer1(0);
   
        micrometros = TickPartial * MICROMETRO_TICK ;  //micrometros
        residuo_um = micrometros % 1000;
         

   
    ///////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////// 
   /////////////////////////////////////////////////////////////////////////// 
   /////////////////////////////////////////////////////////////////////////// 
     set_adc_channel(0);////////ADC CANAL CERO GIROMETRO
     //dato=read_adc(ADC_START_AND_READ);
     read_adc(ADC_START_ONLY);
     while(ADC_BUSY);
     dato=read_adc(ADC_READ_ONLY);
     
     set_adc_channel(2); ///// ADC CANAL UNO JOYSTICK X CONTROL DIRECCION
     //spx=read_adc(ADC_START_AND_READ);
     read_adc(ADC_START_ONLY);
     while(ADC_BUSY);
     spx=read_adc(ADC_READ_ONLY);
     
     set_adc_channel(1); ///// ADC CANAL UNO JOYSTICK CONTROL VELOCIDAD LINEAL
     read_adc(ADC_START_ONLY);
     while(ADC_BUSY);
     spy=read_adc(ADC_READ_ONLY);
     
     if(SYSTEM){
      if(CONTROL==OPEN_LOOP)///OPEN LOOP
         {//////////////////////////////////////////////////////////////////
            if(spy >= MID_POINT + TOLERANCE )//////////%%%%%%%%%%%%%%MOVIMIENTO HACIA ADELANTE
            {
               SENTIDOv1; /// BIT 3 PORTD = 1
               set_pwm1_duty( (int8)((spy-451)/2.4852) );
            }
             if(spy <= MID_POINT - TOLERANCE )/////%%%%%%%%%%%%%%MOVIMIENTO HACIA ATRAS
             {
               SENTIDOv2; /// BIT 3 PORTD = 0
               set_pwm1_duty(  (int8)((-0.3953*spy)+230)   );
            }
            if( spy < (MID_POINT + TOLERANCE) && spy > (MID_POINT - TOLERANCE)  )/////DETENIDO-JOYSTICK EN POSICION INTERMEDIA
                     set_pwm1_duty(STOP_PWM1 );
            if((spx>=MID_POINT + TOLERANCE) && (dato < 360) )/////%%%%%%%%%%%%%%MOVIMIENTO HACIA DERECHA MACRO MAX_DERECHA
            {
               SENTIDOd1;/// BIT 4 PORTD = 1
               set_pwm2_duty( (int8)((spx-451)/2.4852) );
            }
             if(spx <= MID_POINT + TOLERANCE && (dato > 250)  )/////%%%%%%%%%%%%%%MOVIMIENTO HACIA IZQUIERDA
             {
               SENTIDOd2;/// BIT 4 PORTD = 0
               set_pwm2_duty( (int8)((-0.3953*spx)+230)  );
            }
            if( spx < (MID_POINT + TOLERANCE) && spx > (MID_POINT - TOLERANCE)  )/////SIN MOVIMIENTO lateral
                     set_pwm2_duty(STOP_PWM1 );
                             
         }//////////////////////////////////////////////////////////////////
      else      ///CLOSED LOOP
         {
             set_pwm1_duty(STOP_PWM  );
             set_pwm2_duty(STOP_PWM  );
         }      
   }///IF SYSTEM
}
/////----------------------------------------------------------------///////
/////----------------------------------------------------------------///////
//!#INT_TIMER1
//!void  TIMER1_isr(void) 
//!{
//!    //output_toggle(PIN_B7);////PRUEBA DE TIMER1  
//!   if(SYSTEM){
//!      if(CONTROL==OPEN_LOOP)///OPEN LOOP
//!         {//////////////////////////////////////////////////////////////////
//!            if(spy >= MID_POINT + TOLERANCE )/////MOVIMIENTO HACIA ADELANTE
//!            {
//!               SENTIDOv1;
//!               set_pwm1_duty( (int8)((spy-451)/2.4852) );
//!            }
//!             if(spy <= MID_POINT - TOLERANCE )/////MOVIMIENTO HACIA ATRAS
//!             {
//!               SENTIDOv2;
//!               set_pwm1_duty(  (int8)((-0.3953*spy)+230)  );
//!            }
//!            if( spy < (MID_POINT + TOLERANCE) && spy > (MID_POINT - TOLERANCE)  )/////SIN MOVIMIENTO
//!                     set_pwm1_duty(STOP_PWM );
//!            if(spx>=MID_POINT + TOLERANCE )/////MOVIMIENTO HACIA DERECHA
//!            {
//!               SENTIDOd1;
//!               set_pwm2_duty(0 );
//!            }
//!             if(0)
//!             {
//!               SENTIDOv2;
//!               set_pwm2_duty( 0 );
//!            }
//!                             
//!         }//////////////////////////////////////////////////////////////////
//!      else      ///CLOSED LOOP
//!         {
//!      
//!         }      
//!   }
//!   //else return;
//!set_timer1(53536); //RECHARGE TIMER VALUE
//!}
//!/////----------------------------------------------------------------///////
//!#INT_AD HIGH
//!void AD_isr()
//!{
//!//test=TRUE;
//!output_toggle(PIN_B3);
//!output_toggle(PIN_B4);
//!output_toggle(PIN_B5);
//!bit_clear(PIR1,6);//Borra bandera interrupcion AD
//!dato=read_adc(ADC_READ_ONLY);
//!//read_adc(ADC_START_ONLY);
//!}
///////////////////////   F U N C I O N E S    ////////////////////////
void configuracion();
//void Sel_Canal_ADC(unsigned int8);
char num_to_char(int8);
//void print_lcd(int16);
//void print_lcd_decimal(signed int16);
void print_lcd_dec(signed int16 ,int8 );
int8 leer_encoder_direccion(void);
///////////////////////////////////////////////////////////////////////////
void main()
{
int8 posx=8;
int8 posx_ant=1;
int8 Joys_posx=13;
int8 Joys_posx_ant=10;
int8 Joys_posy=13;
int8 Joys_posy_ant=10;
//int8 i=0;
//short FLAG=true;
//char aux_ant='0';
// setup_timer_0(RTCC_INTERNAL|RTCC_DIV_8|RTCC_8_BIT);      //21.3 us overflow
//   set_timer0(131);
   configuracion();
//!   enable_interrupts(INT_TIMER0);
//!   enable_interrupts(GLOBAL);
   //lcd_putc("\f  Hola Mundo");
   //lcd_gotoxy(1, 2);
   //lcd_putc("ADC: ");
   //lcd_putc(num_to_char(5));
   //set_adc_channel(0);
   
  set_pwm1_duty(STOP_PWM1);                    // Set pwm1 duty cycle to i
  set_pwm2_duty(STOP_PWM); 
  set_timer1(0);
   SYSTEM=OFF;
   SYSTEM_ant=ON;
   CONTROL=OPEN_LOOP;
   //read_adc(ADC_START_ONLY);
   
  lcd_gotoxy(1,2);
  lcd_putc("\f   AGV CIDESI");
  delay_ms(700);
  lcd_gotoxy(1,2);
  lcd_putc("\f                ");
 
    
   enable_interrupts(GLOBAL); //Habilita interrupciones
   //enable_interrupts(int_AD);  //Habilita interrupción ADC
   //enable_interrupts(int_EXT);
   enable_interrupts(int_EXT1); //BOTON ON/OFF
   //enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_TIMER0);
   SYSTEM=OFF;
   SYSTEM_ant=ON;
   while(TRUE)
   {  
     //output_toggle(PIN_B7);
     //bit_set(PORTB,7);
//!     set_adc_channel(0);////////ADC CANAL CERO GIROMETRO
//!     //dato=read_adc(ADC_START_AND_READ);
//!     read_adc(ADC_START_ONLY);
//!     while(ADC_BUSY);
//!     dato=read_adc(ADC_READ_ONLY);
      //bit_clear(PORTB,7);
     //delay_us(20);
     
//!     set_adc_channel(2); ///// ADC CANAL UNO JOYSTICK X
//!     //spx=read_adc(ADC_START_AND_READ);
//!     read_adc(ADC_START_ONLY);
//!     while(ADC_BUSY);
//!     spx=read_adc(ADC_READ_ONLY);
     JoystickX=spx/MAX_JSPX;
     //delay_us(20);
 
//!     set_adc_channel(1); ///// ADC CANAL UNO JOYSTICK Y
//!     read_adc(ADC_START_ONLY);
//!     while(ADC_BUSY);
//!     spy=read_adc(ADC_READ_ONLY);    
     //spy=read_adc(ADC_START_AND_READ);
     JoystickY=spy/MAX_JSPY;
     //delay_us(20);
 
     lec_encoder=((PORTD & 0x06 )>>1);///$$$$$$$$$$$$$$$$$$$$$$$$$PROBAR EN RUTINA DE INT0
     if(lec_encoder!=lec_encoder_ant)///$$$$$$$$$$$$$$$$$$$$$$$$$PROBAR EN RUTINA DE INT0
         DIRECCION=leer_encoder_direccion();///$$$$$$$$$$$$$$$$$$$$$$$$$PROBAR EN RUTINA DE INT0
     lec_encoder_ant = lec_encoder;///$$$$$$$$$$$$$$$$$$$$$$$$$PROBAR EN RUTINA DE INT0

      

//!     if(lec_encoder==0)
//!     {
//!         if(lec_encoder_ant==2)
//!     }
    // if(lec_encoder!=lec_encoder_ant)
           // lec_encoder_ant = lec_encoder;
//!      if(bit_test(PORTD,1)) {
//!         if(bit_test(PORTD,2)) aux='3';
//!         else                  aux='1';}
//!      else       {
//!         if(bit_test(PORTD,2)) aux='2';
//!         else                  aux='0';}              
 
   if(BOTON_C5_APRETADO || force)//BOTON MENU
   {
   delay_ms(20);
   while(BOTON_C5_APRETADO);
   if (PAGINA>=MAX_PAGINA) PAGINA=1;
   else                  PAGINA++;
   lcd_gotoxy(1,1);
   lcd_putc("\f                ");
   //print_lcd("                ");
   lcd_gotoxy(1,2);
   lcd_putc("\f                ");
   //print_lcd("                ");
   //output_toggle(PIN_B0);
      if(PAGINA==2)
      {
      lcd_gotoxy(1,1);
      lcd_putc("\fSensor AN8:");
      delay_ms(500);
      lcd_gotoxy(1,1);
      lcd_putc("                ");
       lcd_gotoxy(7,1);
       lcd_putc(124);
       lcd_gotoxy(9,1);
       lcd_putc(124);
       posx=8;
       posx_ant=1;
      }
      
      if(PAGINA==1){
            lcd_gotoxy(1,1);
            lcd_putc("\fRegistros ADC:");
            delay_ms(500);
            lcd_gotoxy(1,1);
            lcd_putc("                ");
      }
      
       if(PAGINA==3){
            lcd_gotoxy(1,1);
            lcd_putc("\Encoder:");
            delay_ms(500);
            lcd_gotoxy(1,1);
            lcd_putc("               ");
      }
      
      if(PAGINA==4)
      {
            Joys_posx=13;
            Joys_posx_ant=10;
            Joys_posy=13;
            Joys_posy_ant=10;
            lcd_gotoxy(1,1);
            lcd_putc("\fJoystick:");
            delay_ms(500);
            lcd_gotoxy(1,1);
            lcd_putc("           ");
            lcd_gotoxy(1,1);
            lcd_putc("X=");
            lcd_gotoxy(1,2);
            lcd_putc("Y=");
            lcd_gotoxy(9,1);
            lcd_putc(124);
            lcd_gotoxy(9,2);
            lcd_putc(124);
      }
      
      if(PAGINA==5){
         if(CONTROL==OPEN_LOOP)  lcd_putc("\fLazo Abierto");
         else                    lcd_putc("\fLazo Cerrado");   
         lcd_gotoxy(1,2);
         if(SYSTEM==ON)        lcd_putc("Sistema ON");
         else                 lcd_putc("Sistema OFF");
         force=false;
      }
   }
   
//!   if(SYSTEM==ON) bit_set(PORTB,7);
//!   else           bit_clear(PORTB,7);
   
   
   //LATB= PAGINA;
   switch(PAGINA)
   {
   case 1:      /////PAGINA ADC
   {  
//!     lcd_gotoxy(1,2);
//!  lcd_putc("\fPAGINA 1");
//!  delay_ms(500);   
   //setup_ccp1(CCP_PWM|CCP_PWM_FULL_BRIDGE);
     //delay_ms(1);
//!     set_adc_channel(0);////////ADC CANAL CERO
//!     read_adc(ADC_START_ONLY);
//!     delay_us(20);
     lcd_gotoxy(1,1);
     print_lcd_dec(dato,0);
     //setup_ccp1(CCP_PWM|CCP_PWM_FULL_BRIDGE_REV);
     //delay_ms(1);
//!     set_adc_channel(1);
//!     read_adc(ADC_START_ONLY);
//!     delay_us(20);
     lcd_gotoxy((16-5),1);
     print_lcd_dec(spx,0);
     //setup_ccp1(CCP_PWM|CCP_PWM_FULL_BRIDGE); 
     //delay_ms(1);
//!     set_adc_channel(2);
//!     read_adc(ADC_START_ONLY);
//!     delay_us(20);
     lcd_gotoxy(1,2);
     print_lcd_dec(spy,0);
      //setup_ccp1(CCP_PWM|CCP_PWM_FULL_BRIDGE_REV);
      //delay_ms(1)
      break;
      ;}
   
   case 2:{           /////PAGINA AN8
//!     lcd_gotoxy(1,2);
//!  lcd_putc("\fPAGINA 2");
//!  delay_ms(50);
//!     set_adc_channel(0);
//!     read_adc(ADC_START_ONLY);
     delay_us(20);///cambiar a us
     dir_abs=(0.440097799)*(dato-102) + CAL_OFFSET;
     dir_rel=dir_abs-90.0;
     
     lcd_gotoxy(1,1);
     //print_lcd_decimal((int16)(10*dir_abs));
     print_lcd_dec((int16)(10*dir_abs),1);
     lcd_gotoxy(11,1);
     //print_lcd_decimal((int16)(10*dir_rel));
     print_lcd_dec((int16)(10*dir_rel),1);
     posx=((int8)((dir_rel+45.0)/6))+1;
     if(posx>11) posx=16;
     if(posx<1)   posx=1;
     if(posx!=posx_ant){
     lcd_gotoxy(posx,2);
     lcd_putc(94);
     lcd_gotoxy(posx_ant,2);
     lcd_putc(' '); 
     posx_ant=posx;
     }
     
 //print_lcd(dato);
 //    delay_ms(1000);
//!     while(i<253){
//!    lcd_gotoxy(1,2);
//!    print_lcd(i);
//!    lcd_putc(124);
//!     lcd_putc(94);
//!    lcd_putc(i);
//!     i++;
//!     delay_ms(500);
//!     }
     //print_lcd_decimal(-100);
   break;
   }
   
   case 3:               /////PAGINA ENCODER
   {
   lcd_gotoxy(1,1);
   //lcd_putc("\fEN CONSTRUCCION");
   //delay_ms(50);
   print_lcd_dec(Totalticks,0);
   //print_lcd_dec(get_timer1(),0);
   lcd_gotoxy(8,1);
   print_lcd_dec(TickPos,0);
  // lcd_gotoxy(8,1);//DEBUG
  // print_lcd_dec(lec_encoder,0);//DEBUG
  
   lcd_gotoxy(1,2);
   if(DIRECCION == DERECHA)     lcd_putc("ADEL");
   if(DIRECCION == IZQUIERDA)   lcd_putc("ATRA");
   if(DIRECCION == OTRO)        lcd_putc("UNK");
   
      lcd_gotoxy(8,2);
   print_lcd_dec(TickNeg,0);
   //delay_ms(5);
    //  lcd_gotoxy(10,2);//DEBUG
   //print_lcd_dec(lec_encoder_ant,0);//DEBUG
//!   if(lec_encoder!=lec_encoder_ant)
//!   {
//!         if(i>=16 && flag==true) 
//!            {i=1;
//!            flag=false;}
//!         else     i++;
//!      lcd_gotoxy(i,2);
//!      lcd_putc(lec_encoder+48);
//!      lec_encoder_ant=lec_encoder;
//!   }
   //print_lcd_dec(lec_encoder,0);
   break;
   }
   
   case 4:                /////PAGINA JOYSTICK
   {
   //lcd_gotoxy(1,1);
   //lcd_putc("\fJoystick:");
   // delay_ms(500);
   // lcd_putc("           ");
    lcd_gotoxy(3,1);
    //aux=JoystickX*1000;
    print_lcd_dec((signed int16)((JoystickX*2000)-1010.2),3);
    Joys_posx = ((int8)(spx/146))+10;
//!   lcd_gotoxy(11,1);
//!   print_lcd_dec(Joys_posx,0);
   if(Joys_posx!=Joys_posx_ant)
   {
      lcd_gotoxy(Joys_posx,1);
      lcd_putc(94);
      lcd_gotoxy(Joys_posx_ant,1);
      lcd_putc(" ");
      Joys_posx_ant=Joys_posx;
   }
    lcd_gotoxy(3,2);
    print_lcd_dec((signed int16)((JoystickY*2000)-1010.2),3);
    Joys_posy = ((int8)(spy/146))+10;
       if(Joys_posy!=Joys_posy_ant)
   {
      lcd_gotoxy(Joys_posy_ant,2);
      lcd_putc(" ");   
      lcd_gotoxy(Joys_posy,2);
      lcd_putc(94);
      Joys_posy_ant=Joys_posy;
   }
   
    //print_lcd_dec(9999,2);
    
//!    lcd_gotoxy(8,2);
//!    print_lcd_dec(9999,3);

    break;
   }
   
   case 5:{                    /////PAGINA GENERAL
      if(BOTON_C4_APRETADO && SYSTEM==OFF){
        delay_ms(20);
        while(BOTON_C4_APRETADO);
        Control=!Control;
      } 
      
      if(Control!=Control_ant){
         lcd_gotoxy(1,1);
            if(CONTROL==OPEN_LOOP)
            {
               lcd_putc("\fLazo Abierto");
            }
            else
            {
                lcd_putc("\fLazo Cerrado");
            }
             // lcd_gotoxy(1,2);///////////Agregado por problema de perdida de despliegue
             // lcd_putc("Sistema OFF");////
      Control_ant=Control;
      }
      
      if(SYSTEM!=SYSTEM_ant)
      {
         lcd_gotoxy(1,2);
         if(SYSTEM==ON)
         {
            lcd_putc("Sistema ON");
         }
         else
         {
            lcd_putc("Sistema OFF");
         }
     }
      
      break;
   }
  
   
   }////CIERRA SWITCH(PAGINA)
   

//     output_toggle(PIN_B7);
//!   for(i=100;i>1;i--){
//!   lcd_gotoxy(6,2);
//!   print_lcd(i);
//!   delay_ms(100);
//!   }
   }

}
///////////////////////////////////

///////////////////////////////////

///////////////////////////////////

///////////////////////////////////
int8 leer_encoder_direccion(void)
{
int8 val=2;
      
   switch(lec_encoder)
   {
   case 0:{
            if(lec_encoder_ant==2)                       val=DERECHA;
            if(lec_encoder_ant==1)                       val=IZQUIERDA;
            if(lec_encoder_ant==0 || lec_encoder_ant==3) val=OTRO;
            break;
            }
   case 1:{   
            if(lec_encoder_ant==0)                       val=DERECHA;
            if(lec_encoder_ant==3)                       val=IZQUIERDA;
            if(lec_encoder_ant==1 || lec_encoder_ant==2) val=OTRO;
            break;
            }
   case 2:{
            if(lec_encoder_ant==3)                       val=DERECHA;
            if(lec_encoder_ant==0)                       val=IZQUIERDA;
            if(lec_encoder_ant==1 || lec_encoder_ant==2) val=OTRO;
            break;
            }          
   case 3:{
            if(lec_encoder_ant==1)                       val=DERECHA;
            if(lec_encoder_ant==2)                       val=IZQUIERDA;
            if(lec_encoder_ant==0 || lec_encoder_ant==3) val=OTRO; 
            break;
            }
//!   default:{
//!            val=OTRO; 
//!            break;
//!            }
   
  // lec_encoder_ant = lec_encoder;
     }
   return val;
}
///////////////////////////////////
void print_lcd_dec(signed int16 numero,int8 pos_decimal)
{
int8 millares=0;
int8 centenas=0;
int8 decenas=0;
int8 unidades=0;

if(numero < 0)
{
numero = abs(numero);
lcd_putc('-');
}

millares = (int8)(numero /1000);
lcd_putc(num_to_char(millares));

if(pos_decimal==3)
lcd_putc('.');

centenas=(int8)((numero %1000)/100);   
lcd_putc(num_to_char(centenas));

if(pos_decimal==2)
lcd_putc('.');

decenas= (int8)((numero%100)/10);
lcd_putc(num_to_char(decenas));

if(pos_decimal==1)
lcd_putc('.');

unidades = (int8)((numero%10));
lcd_putc(num_to_char(unidades));

}
///////////////////////////////////
//!void print_lcd_decimal(signed int16 numero)
//!{
//!int8 millares=0;
//!int8 centenas=0;
//!int8 decenas=0;
//!int8 unidades=0;
//!
//!if(numero < 0)
//!{
//!numero = abs(numero);
//!lcd_putc('-');
//!}
//!
//!millares = (int8)(numero /1000);
//!lcd_putc(num_to_char(millares));
//!
//!centenas=(int8)((numero %1000)/100);   
//!lcd_putc(num_to_char(centenas));
//!
//!decenas= (int8)((numero%100)/10);
//!lcd_putc(num_to_char(decenas));
//!
//!lcd_putc('.');
//!
//!unidades = (int8)((numero%10));
//!lcd_putc(num_to_char(unidades));
//!
//!}
//!///////////////////////////////////
//!void print_lcd(int16 numero)
//!{
//!int8 millares=0;
//!int8 centenas=0;
//!int8 decenas=0;
//!int8 unidades=0;
//!
//!millares = (int8)(numero /1000);
//!lcd_putc(num_to_char(millares));
//!
//!centenas=(int8)((numero %1000)/100);   
//!lcd_putc(num_to_char(centenas));
//!
//!decenas= (int8)((numero%100)/10);
//!lcd_putc(num_to_char(decenas));
//!
//!unidades = (int8)((numero%10));
//!lcd_putc(num_to_char(unidades));
//!
//!}
///////////////////////////////////
char num_to_char(int8 numero)
{
const char val=48;

return (val+numero);
}
///////////////////////////////////
//!void Sel_Canal_ADC(unsigned int8 chann)
//!{
//!unsigned int8 temp=0;
//!
//!ADCON0 &= 0b11000011;
//!temp= chann << 2;
//!ADCON0 += temp;
//!}
//////////////CONFIGUACION////////////////
void configuracion()
{
   setup_wdt (WDT_OFF);      //Desactivamos el perro guardián
   setup_comparator (NC_NC_NC_NC);   //Desactivamos el comparador
   //setup_spi (FALSE);      //Desactivamos el bus SPI
   Setup_spi(SPI_SS_DISABLED);
   setup_low_volt_detect (FALSE) ;
   setup_oscillator (False);       //Desactiva oscilador interno RC
   // Lectura análoga    //
   setup_adc_ports(AN0_TO_AN2|VSS_VDD);
   setup_adc(ADC_CLOCK_DIV_16|ADC_TAD_MUL_20);
   //setup_adc_ports (NO_ANALOGS|VSS_VDD); //Activamos las entradas analogas
   //setup_adc (ADC_OFF);      //
   //setup_vref (FALSE);      //Desactivamos el vref
   /////////// PWM Configurations///////////
  //setup_ccp1(CCP_PWM|CCP_PWM_FULL_BRIDGE);                   // Configure CCP1 as a PWM
  setup_ccp1(CCP_PWM);  
  setup_ccp2(CCP_PWM);
/////////////////////////////////////////// 
/////////// INT0 Configurations///////////
//ext_int_edge(H_TO_L);
//ext_int_edge(0,H_TO_L);
ext_int_edge(1,L_TO_H);
///////////////////////////////////////////
   // TMR0       //
   //setup_timer_0 (RTCC_INTERNAL|RTCC_DIV_256);   //TMR0 será utilizado para contar el tiempo
   //setup_timer_0 (RTCC_INTERNAL|RTCC_DIV_256);
   setup_timer_0 (CONFIG_TIMER);
   set_timer0(VAL_PRECARGA);
   //set_timer0(18660);
   //setup_timer_0 (RTCC_DIV_256);

   //setup_timer_0 (RTCC_ENABLED);
   // TMR1       //
   setup_timer_1 (T1_EXTERNAL) ;///CONTEO DE PULSOS DE ENCODER
//!   setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);
//!   set_timer1(53536);

   // TMR2       //PREESCALER 16 PARA FREQ PWM
   setup_timer_2 (T2_DIV_BY_16, 255, 1);//2.92KHz
   //setup_timer_2 (T2_DIV_BY_16, 149, 1);//5KHz
   // TMR3       //
   setup_timer_3 (T3_DISABLED|T3_DIV_BY_1) ;

   lcd_init();
   // PUERTOS      //
   //  "1" lógico = Entrada y "0" lógico = Salida).
//!   LATA = 0x00;         //INICIALIZACION PUERTO A (7 BITS)
//!   PORTA = 0X00;
//!   TRISA =0xFF;        //A0 A1 A2 A3 A4 A5 COMO ENTRADAS 
   CMCON=0x07;
   TRISB = 0X00;        //Todo el puerto B como ENTRADAS
                        //RB se activa con fusible NOPBADEN
   PORTB = 0X00;        //INICIALIZACION PUERTO A (8 BITS)
   LATB = 0X00;         //Limpiamos el puerto B
   bit_set(TRISB,0);    //Pin B0 entrada
   bit_set(TRISB,1);    //Pin B0 entrada
   //ADCON1=0x0F;
   //ADCON0=0X00;

   INTCON2 &=  ~ (0x80); //Reset /RBPU de INTCON2 para habilitar pullup en PORTB                   
   //Port_B_Pullups(TRUE);       // Configuración para el PIC 18F4550.
   LATC=0x00;
   PORTC=0x00;
   TRISC = 0x00;
  
   
   bit_clear(UCON,3);
   bit_set(UCFG,3);
   bit_set(TRISC,5);//ENTRADA BOTON MENU
   bit_set(TRISC,4);//ENTRADA BOTON MODO
   bit_set(TRISC,0);//ENTRADA T1CKI CONTEO DE PULSOS ENCODER
   //LATC=0xFF;

   LATD = 0X00;         //INICIALIZACION PUERTO D (8 BITS)
   PORTD = 0X00;        //Limpiamos el puerto D    
   TRISD = 0X00;        //D0-3 como salidas, D4-7 Entradas
   bit_set(TRISD,1);
   bit_set(TRISD,2);
  // LATE  = 0x00;    //Habilitacion de PUllUP Puerto D
   
   LATE = 0X00;         //INICIALIZACION PUERTO D (4 BITS)
   PORTE = 0X00;        //Limpiamos el puerto D  
   TRISE = 0X00;        //Puerto E como entradas 
//!                        //RE3 digital se activa con fusible NOMCLR

////////  A D C  ---   C O N F I G /////////


LATA = 0x00;         //
PORTA = 0X00;        //
TRISA =0xFF;        //A0 A1 A2 A3 A4 A5 COMO ENTRADAS 

//!bit_clear(ADCON2,5);//Acquisition Time = 4Tad
//!bit_set(ADCON2,4);
//!bit_clear(ADCON2,3);
//!
//!bit_set(ADCON2,2);//Convertion clock = Fosck/64
//!bit_set(ADCON2,1);
//!bit_clear(ADCON2,0);

//bit_set(ADCON0,0);//Encender modulo ADC

//bit_clear(PIR1,6);//Borra bandera interrupcion AD

//bit_set(PIE1,6);//Habilita interrupciones perifericos
////////////////////////////////////////////  
   
}
//////*******************************************************************




