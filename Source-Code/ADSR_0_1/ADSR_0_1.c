/*
* ADSR_0_1.c
*
* Created: 6/28/2016 1:45:45 PM
*  Author: TimR
*/

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "avr/io.h"
#include "avr/interrupt.h"

// Prototypes
void process_adsr(void);
void	init_ports(void);
void	init_devices(void);
void	pgm_init(void);
void	timer_init(void);
void SendToShiftyTypes(void);


// Hardware Defines
#define LEDS_ATTACK		0x01
#define LEDS_DECAY		0x04
#define LEDS_SUSTAIN	0x02
#define LEDS_RELEASE	0x08
#define LEDS_IDLE_MASK	0xF0

#define SW_AR		PIND&0x80		// 0=AR, 1=ADSR
#define SW_LIN		PIND&0x40		// 0=lin, 1=expo

#define	EOC_HI		PORTB|=0x02;
#define	EOC_LO		PORTB&=~0x02;

// Shifty Types
#define	SS_LO		PORTB&=~0x04;
#define	SS_HI		PORTB|=0x04;

// LED macros
#define LED_ATTACK	{PORTD|=~LEDS_IDLE_MASK;PORTD&=~LEDS_ATTACK;}
#define LED_DECAY	{PORTD|=~LEDS_IDLE_MASK;PORTD&=~LEDS_DECAY;}
#define LED_SUSTAIN	{PORTD|=~LEDS_IDLE_MASK;PORTD&=~LEDS_SUSTAIN;}
#define LED_RELEASE	{PORTD|=~LEDS_IDLE_MASK;PORTD&=~LEDS_RELEASE;}
#define LED_IDLE	{PORTD|=~LEDS_IDLE_MASK;}

//#define	EXPOEPSILON		0x01FFFFFF
#define	EXPOEPSILON		0x0000FFFF
#define ADCCHANNELMAX	6
#define	EOC_TIMEOUT		9			// about ?mS

// Globals

enum states {
	s_idle,
	s_att,
	s_dec,
	s_sus,
	s_rel
};

struct adsr {
	uint32_t	attack;
	uint32_t	decay;
	uint32_t	sustain;
	uint32_t	release;
	uint32_t	accum;
	uint32_t	curDac;
	uint8_t		state;
};
volatile struct adsr thisADSR;

volatile	uint16_t	rawAdcPots[4];	// raw pot values from ADC
volatile	uint16_t	rawAdcSum[4];	// sum of raw pot values and adsr/rel values
uint16_t	rel_cv,adsr_cv;				// raw ADC values from rel and adsr jacks
uint8_t		flgShape;					//sets lin or expo shape, 
uint8_t		flgAR,flgARold;				//sets AR mode -- from FP switch
volatile uint8_t	flgGate;			// set when a rising Gate is detected
uint8_t		thisAdcChannel;				// current ADC mux setting
uint8_t		flgProcess;					// shows its time to process the ADSR
volatile uint8_t		flgEOC,tmrEOC;

int main(void)
{
	cli();
	init_ports();
	init_devices();
	timer_init();
	pgm_init();
	sei();
	
	ADCSRA |= (1<<ADIE);					// enable ADC interrupts
	ADCSRA |= (1<<ADSC);				//start next conversion

	while(1)
	{
		// wait for gate flag
		if(flgGate)
		{
			flgGate=0;			// clear flag
			LED_ATTACK
			thisADSR.state = s_att;
		}
		// wait for processing flag
		// decouples from the irq so the processing does not happen
		// inside an irq.
		if(flgProcess)
		{
			flgProcess=0;			// clear flag
			// set env based on switch
			if(SW_AR)
				flgAR=0;		// adsr
			else
				flgAR=1;		//ar
			if(flgARold!=flgAR)
				thisADSR.state = s_att;
			flgARold=flgAR;
			
			// set shape based on switch
			if(SW_LIN)
				flgShape=0;		// expo
			else
				flgShape=1;		// lin			
			process_adsr();
			SendToShiftyTypes();
		}
	}
}


void	init_ports(void)
{
	DDRB = 0x2E;
	DDRC = 0x00;		// all inputs of the ADC variety
	PORTD = 0xC0;		// pull ups on PD6 & 7
	DDRD = 0x0F;		//PD0-3 outputs, rest inputs
}


void process_adsr(void)
{
	static uint32_t		NextExpoAmt;
	//	static uint8_t		flgRetrigOld;
	//	uint16_t			index;
	cli();
	
	switch (thisADSR.state) {
		case s_idle:
		break;
		
		case s_att:
		if(flgShape) {
			//linear processing
			if((int64_t)(0x7fffffff-(thisADSR.accum)) < (thisADSR.attack)) {
				thisADSR.accum=0x7fffffff;
				if(flgAR) {
					thisADSR.state=s_rel;
					LED_RELEASE
				} else {
					thisADSR.state=s_dec;
					LED_DECAY
				}
			}
			else
			thisADSR.accum+=((thisADSR.attack)/2);
			thisADSR.curDac=thisADSR.accum*2;
		}
		else {
			//expo processing
			NextExpoAmt=(0x9fffffff-thisADSR.accum);
			NextExpoAmt=NextExpoAmt/((rawAdcSum[0]*2)+8);
			NextExpoAmt=NextExpoAmt*3;
			NextExpoAmt=NextExpoAmt/4;
			
			if(NextExpoAmt>0x7ffffff0)
			NextExpoAmt=0x7ffffff0;
			if(NextExpoAmt<0)
			NextExpoAmt=0x7ffffff0;
			if((0x7fffffff-(thisADSR.accum)) <= NextExpoAmt+EXPOEPSILON) {
				thisADSR.accum=0x7fffffff;
				if(flgAR) {
					thisADSR.state=s_rel;
					LED_RELEASE
					} else {
					thisADSR.state=s_dec;
					LED_DECAY
				}
			}
			else
			thisADSR.accum+=NextExpoAmt;
			thisADSR.curDac=thisADSR.accum*2;
		}
		
		break;
		
		case s_dec:
		if(flgShape) {
			//linear processing
			if(((int64_t)thisADSR.accum - (int64_t)(thisADSR.decay)) < thisADSR.sustain/2) {
				thisADSR.accum=thisADSR.sustain/2;
				thisADSR.state=s_sus;
				LED_SUSTAIN
			}
			else
			thisADSR.accum-=(thisADSR.decay/2);
			
			thisADSR.curDac=thisADSR.accum*2;
			//break;
			
			} else {
			//expo processing
			NextExpoAmt=(thisADSR.accum-(thisADSR.sustain/2));
			NextExpoAmt=NextExpoAmt/((rawAdcSum[1]*3)+12);
			NextExpoAmt=((NextExpoAmt*12)/10);
			NextExpoAmt=NextExpoAmt;

			if((int64_t)thisADSR.accum-((thisADSR.sustain/2)+EXPOEPSILON)<NextExpoAmt) {
				thisADSR.accum-=NextExpoAmt*2;
				thisADSR.state=s_sus;
				LED_SUSTAIN
			}
			else
			thisADSR.accum-=NextExpoAmt*2;
			thisADSR.curDac=thisADSR.accum*2;
		}

		break;
		
		case s_sus:
		thisADSR.accum=thisADSR.sustain/2;
		thisADSR.curDac=thisADSR.accum*2;
		//if(PORTA.IN & PIN6_bm)
		if((PIND & 0x10)==0x10)	{					// is Gate went away i.e. high (inverted)
			thisADSR.state=s_rel;						// jump to release state
			LED_RELEASE
		}
		break;

		case s_rel:
		if(flgShape) {
			//linear processing
			if(thisADSR.accum <= (thisADSR.release)) {
				thisADSR.accum=0;
				thisADSR.state=s_idle;
				LED_IDLE
				
				// use for cycle mode...
				EOC_HI;
				tmrEOC=0;
				flgEOC=1;
				
			}
			else
			thisADSR.accum-=((thisADSR.release)/2);
			
			//process exp curve crap
			thisADSR.curDac=thisADSR.accum*2;
			} else {
			//process exp curve
			NextExpoAmt=thisADSR.accum;
			NextExpoAmt=NextExpoAmt/((rawAdcSum[3]*3)+12);
			NextExpoAmt=((NextExpoAmt*3)/2);
			NextExpoAmt=NextExpoAmt;

			//if(thisADSR.accum <= EXPOEPSILON) {
			if((int32_t)thisADSR.accum<(NextExpoAmt+EXPOEPSILON)) {
				thisADSR.accum=0;
				thisADSR.state=s_idle;
				LED_IDLE
				// use for cycle mode...
				EOC_HI;
				tmrEOC=0;
				flgEOC=1;

			}
			
			else
			thisADSR.accum-=((NextExpoAmt/8)*18);
			thisADSR.curDac=thisADSR.accum*2;
			break;
		}
	}
	sei();
}


void init_devices(void)
{
	//
	// sets up ADC system
	//
	ADMUX = (1<<REFS0);		//AVCC as reference
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

	//
	// enable IRQ on Gate jack input
	PCMSK2 = (1<<PCINT20);		
	PCICR = (1<<PCIE2);						//enable interrupts
	
	// ***SPI***
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);	// SPI enable	
}


void pgm_init(void)
{
	// setup mux channel
	thisAdcChannel=0;
	ADMUX |= thisAdcChannel;
	LED_IDLE
	SS_HI;
}


void adcProcess(void)
{
	rawAdcSum[0]=rawAdcPots[0]+adsr_cv;
	rawAdcSum[1]=rawAdcPots[1]+adsr_cv;
	rawAdcSum[2]=rawAdcPots[2];
	rawAdcSum[3]=rawAdcPots[3]+adsr_cv+rel_cv;
	
	thisADSR.attack=0x20000000/((rawAdcSum[0]+0x02)/0x02);
	thisADSR.decay=0x20000000/((rawAdcSum[1]+0x02)/0x02);
	thisADSR.sustain=(uint32_t)rawAdcSum[2]<<22;
	thisADSR.release=0x20000000/((rawAdcSum[3]+0x02)/0x02);
}


void SendToShiftyTypes(void)
{
	uint8_t		dummy;
	uint16_t	sendVal;

	cli();
	SS_LO;
	
	sendVal = thisADSR.curDac>>20;
	sendVal = sendVal | 0x3000;			//gain=1,active mode
	
	SPDR=sendVal>>8;					// send MSB
	do {} while (!(SPSR&(1<<SPIF)));
	dummy=SPDR; 				//dummy read to clear flag

	SPDR=sendVal&0x00FF;				// send LSB
	do {} while (!(SPSR&(1<<SPIF)));
	dummy=SPDR; 				//dummy read to clear flag
	
	SS_HI;
	sei();
}

void timer_init(void)
{
	// for 20.0000MHz clock
	// timer 0 is the 1mS (0.5mS?) tick
	TCCR0A  = (1<<WGM01);					// ctc mode
	TCCR0B  = (1<<CS02); 					// divide by 256
	OCR0A   = 0x26;							// or 38 decimal -- 0.5mS
	TIMSK0	= 1<<OCIE0A;					// output compare A match irq
}


//	ADC interrupt
ISR(ADC_vect)
{
	//process results
	switch (thisAdcChannel) {
		case 0:
		rel_cv = ADC;
		break;
		case 1:
		adsr_cv = ADC;
		break;
		case 2:
		rawAdcPots[1] = ADC;
		break;
		case 3:
		rawAdcPots[0] = ADC;
		break;
		case 4:
		rawAdcPots[3] = ADC;
		break;
		case 5:
		rawAdcPots[2] = ADC;
		break;
		
	}
	// do the math
	adcProcess();
	//set up for next channel
	thisAdcChannel+=1;
	if(thisAdcChannel >= ADCCHANNELMAX)
	thisAdcChannel=0;
	ADMUX &= 0xF0;
	ADMUX |= thisAdcChannel;
	ADCSRA |= (1<<ADSC);				//start next conversion
}


ISR (TIMER0_COMPA_vect)
{
	// end of cycle timer
	// terminates pulse began at end of release
	// width ~10mS
	if(flgEOC){
		tmrEOC++;
		if(tmrEOC>=EOC_TIMEOUT) {
			EOC_LO;
			flgEOC=0;
		}
	}

	flgProcess=1;
}


ISR(PCINT2_vect)
{
	static uint8_t	pinDmem=0x02;
	uint8_t	pinDstate;
	
	// processes Gate interrupt
	// uses pinDmem to remember old state to check for change
	// and to detect falling edge (Gate is inverted in hardware)
	
	pinDstate=PIND;					//get the state of PIND once	
	if((pinDstate&0x10)!=(pinDmem&0x10)) {		//did it change?
		if((pinDstate&0x10)==0x00)	{			//is it our edge?
			flgGate=1;
		}
	}
	
	pinDmem=pinDstate;					//remember this port sample
}
