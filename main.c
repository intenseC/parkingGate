
/*
ATtiny13               433mHz    Parking     Lot   Gate  Opener
*
by Larry_! 'intenceC'
*
ver. 002a      
+   this device exploits so called 'sync bug' in rolling code   +

*/
//*****************************************************************************
 		
		/*   timing values   */
		#define	__PRESET1	
        #define	__TIMINGS0
	
		
		#define F_CPU 9600000UL
        #ifdef __TIMINGS1
		#define GAP   21200   // delay between packets  // 22800
		#define DELAY   308  // segment length   // 320 
		#define PAD     DELAY * 2  //  640   620   //   
		#define HEADER_OFFS   420  // 
		#define TMR_OFFS   100  // 
 		#define HEADER           1750  // 1850 - TMR_OFFS
 		
		#elif defined __TIMINGS0
		#define DELAY                  405  // segment length   // 0.43 ms 
		#define PAD                      DELAY * 2  //  
 		#define HEADER               DELAY * 6          // 2.55 ms  2450
    #define GAP        HEADER * 10           //24300   // delay between packets  // 25.5 ms
		#endif
		
		#define MAXBIT           52  //
//*****************************************************************************
 
    #ifdef __PRESET1		
	#define PULSETRAIN   12
	#define REFRAIN   500
	#elif defined __PRESET2
	#define PULSETRAIN   6
	#define REFRAIN   250
	#elif defined __PRESET3
	#define PULSETRAIN   4
	#define REFRAIN   125
	#endif	
	
	#define LONGPRESSGAP   1222

//*****************************************************************************
          /*   macros   */  
        #define maxval(val, max, min)  \
        if( val > max ) { val = min; } \
        if( val < min )  { val = max; }\
				while (0)
        #define sbi(var, mask) ((var) |= (1 << mask))  // Set   bit
        #define cbi(var, mask) ((var) &= ~(1 << mask)) // Clear bit

        #define PUSHZERO()      cbi(PORT_PWM, PIN_PWM);  _delay_us(PAD); \
        sbi(PORT_PWM, PIN_PWM); _delay_us(DELAY)
        											          
		    #define PUSHONE()     cbi(PORT_PWM, PIN_PWM); _delay_us(DELAY); \
        sbi(PORT_PWM, PIN_PWM); _delay_us(PAD)
											                
//*****************************************************************************
//place includes  below
//*****************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
//
//*****************************************************************************
// put variables  below
//*****************************************************************************
    uint16_t longPessTmr = 0;
    uint8_t key = 0;
    uint8_t longKey = 0; 
	  uint8_t buff[7];
					          
			  struct gateKey
                          {
                          uint8_t hdr[5];
                          uint8_t rollingKeys[4][2];
						   }  *ptrKey;

                /*   the actual keycodes. You should replace these with yours  */
            				 struct gateKey keyHome =
                                  {                             //    home
							  { 0x00, 0xA5, 0xC6, 0xC2, 0x80 }, 
							 {	 { 0B00010100, 0B01010010 },
	  							 { 0B00001000, 0B01110100 },
	  							 { 0B00000101, 0B01101111 },
	 							 { 0B01010101, 0B00011111 }   }
							        };

            				  struct gateKey keyWork  =
                                         {                          //    work
							    { 0x00, 0xAE, 0x62, 0x5F, 0x80 },
								 {   { 0xcc, 0xd0 },
	  							 	 { 0x30, 0xe1 },
	  							 	 { 0x22, 0xb9 },
	  							 	 { 0x06, 0x95 }    }
							              };

//*****************************************************************************
// define pins below
//*****************************************************************************
#define PORT_AUX_LED                      PORTB
#define AUX_LED	                          PINB                         // aux pin / reserved
#define PIN_LED	                          PB4

#define PORT_PWM                   PORTB
#define PWM	                       PINB                        // pwm out to radio 
#define PIN_PWM	                   PB0

#define PORT_TRIG        	       PORTB
#define TRIG	                	 PINB                        // trigger key
#define PIN_TRIG	    	         PB1

//*****************************************************************************
//*****************************************************************************

void init_io(void)
{
 PORTB = 0b00101110;   													     /* activate pull-ups on 5 - 3 - 2 - 1-*/
 DDRB   = 0b00010001;       													/*  pins  0 & 4   output,  rest are  input */  
}

void timer_init (void)
     {   
	  cli();
    TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);            //enable timer0 with 1024 prescaler - 9375Hz
		TCNT0 = 0;
	  sei();
     }
//*****************************************************************************
//*****************************************************************************

/*  pulse width modulator */

void transmitPacket(uint8_t *packet, uint8_t size)
{
	  uint8_t offs = 1;  // one byte discarded, nibble of zeroes inserted instead
	  int n = 0; 
	  sbi(PORT_PWM, PIN_PWM);  cbi(PORT_AUX_LED, PIN_LED);
	  _delay_us(HEADER);
   
    for( int i = 0; i < 4; i++ ) { PUSHZERO(); n++; }       /* INSERT FOUR ZEROES */
	     // byte parser
    for( int i = offs; i < size; i++ )  // optimization needed / chose either size or MAXBIT, no need both
        { 	                                                                            
	  for( int y = 7; y >= 0 && n < MAXBIT; y-- )  {  
		   if((packet[i] >> y) & 1) {  PUSHONE(); } else {  PUSHZERO(); }             n++;  }
		}
      cbi(PORT_PWM, PIN_PWM);  sbi(PORT_AUX_LED, PIN_LED);
	_delay_us(GAP);
}


//*****************************************************************************
//*****************************************************************************

       void transmit(uint8_t lap)
      {
            uint16_t x;
            ptrKey = lap ? &keyWork : &keyHome;
				  uint8_t frame = sizeof(ptrKey->rollingKeys) / sizeof(ptrKey->rollingKeys[0]) - 1; 
					   for(x = 0; x <= frame; x++ ) {
             memcpy(buff, ptrKey->hdr, sizeof(ptrKey->hdr)); 
	         memcpy(buff + sizeof(ptrKey->hdr), ptrKey->rollingKeys[x], sizeof(ptrKey->rollingKeys[x]));
	         for(int y = PULSETRAIN; y > 0; y--)   transmitPacket(buff, 7); 
	         wdt_reset(); 
	         _delay_ms(REFRAIN); 
		   } 
       
		}

//*****************************************************************************
//*****************************************************************************
int main(void)      
{
   init_io();
   timer_init();
   sei();
  
   wdt_enable(WDTO_2S);
   cbi(PORT_PWM, PIN_PWM); sbi(PORT_AUX_LED, PIN_LED);
//*****************************************************************************

     for(;;)  // eternal loop start
	     { 
        if( TCNT0 >= 255 ) // 37 Hz
	                { 
		     // a simple variable timed keypress handler
               if(key == 0)   
				         {
			    longPessTmr = LONGPRESSGAP;
			  	if(bit_is_clear(TRIG, PIN_TRIG)) key = 1;
			              }	 else  {        
				if( --longPessTmr == 0)   { 
			    if(bit_is_clear(TRIG, PIN_TRIG))   {   transmit(1);  key = 0;  }  // long keypress
				      }	else 	if(bit_is_set(TRIG, PIN_TRIG) &&  key != 0) 
				    {
				 transmit(0);   key = 0;  // short keypress
			      }
		 	    }  
		wdt_reset();
                    }
               // tick out
        	  }  // for() out

    }          // main out
//*****************************************************************************
//*****************************************************************************
