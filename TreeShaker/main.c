// INCLUDES ///////////////////////////////////////////////////////////////////
#include <avr/io.h>
#define F_CPU 8000000UL // this needs to be defined for the delay functions
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>
#include <stdbool.h>

// MACROS /////////////////////////////////////////////////////////////////////
#define BAUD 38400UL
#define MYUBRR (F_CPU/16/BAUD-1)
#define MAX_INPUT_SIZE 40

#define TRIGGER_HIGH PORTB |= (1<<PINB1)
#define TRIGGER_LOW PORTB &= ~(1<<PINB1)
#define MOTOR_ON PORTD |= (1<<PINB4)
#define MOTOR_OFF PORTD &= ~(1<<PINB4)

// TYPES //////////////////////////////////////////////////////////////////////
typedef enum
{
	ARG1,
	ARG2,
	ARG3,
	ARG4,
	ARG5,
	ARG6,
	ARG7,
	ARG8,
	ARG9,
	N_ARGS,
	CMD,
	ARG_BUFF_SIZE
} input_args_t;

// GLOBAL VARIABLES ///////////////////////////////////////////////////////////

// STATIC GOLBAL VARIABLES ////////////////////////////////////////////////////
static char* arg_buff[ARG_BUFF_SIZE];
static uint16_t threshold_cm;
static uint16_t shake_time_ms;
static volatile bool armed;

// STATIC FUNCTION PROTOTYPES /////////////////////////////////////////////////
static int usart_send(char usart_data, FILE *stream);
static int usart_receive(FILE *stream);
static void init_usart(void);
static void	init_gpio( void );
static uint8_t parse_user_input( char* user_input, char* arg_buff[] );
static void cli_process( char* user_input );
static uint16_t get_distance(void);
static void init_tc0( void );
static void	init_tc1( void );
static void set_duty( uint8_t duty_cycle );
static uint8_t get_duty( void );
static void	shake( void );
static void delay_ms( uint16_t delay );

/* Set up for printf: */
FILE usart0_stream = FDEV_SETUP_STREAM(usart_send, usart_receive,
                                                               _FDEV_SETUP_RW);

// MAIN ///////////////////////////////////////////////////////////////////////
int main( void )
{
	stdin = stdout = &usart0_stream;
	
	/* Initialize peripherals: */
	init_usart();
	init_gpio();
	init_tc0();
	init_tc1();
	/* Enable interrupts: */
	sei();
	
	printf( "\x1B[2J" ); // clear the screen
	printf( "\x1B[0m" );
	printf( "\x1B[1;1H" );
	printf("\ntreeshaker>> ");
		
	MOTOR_OFF;
	
	/* Defaults: */
	shake_time_ms = 1000;
	threshold_cm = 500;
	set_duty( 75 );	
	armed = false;
		
    while( 1 ) 
	{
		if(armed)
		{
			if( threshold_cm > get_distance() )
			{
				shake();
			}
		}	
	}
}

// STATIC FUNCTIONS ///////////////////////////////////////////////////////////
static void init_usart(void)
{
	/* Set the baud rate: */
	UBRR0L = MYUBRR;
	/* Enable the receiver, transmitter, and the receive complete interrupt: */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
}

static void	init_gpio( void )
{
	//1 = output, 0 = input
		
	//Set    ECHO        TRIGGER   
	DDRB = (0<<PINB0) | (1<<PINB1);	
	
	//Set      RXD         TXD       MOTOR CTRL      PWM
	DDRD = (0<<PIND0) | (1<<PIND1) | (1<<PIND4) | (1<<PIND5);
}

static void init_tc0( void )
{
	TCCR0A = (0<<WGM02) | (1<<WGM01) | (1<<WGM00) | (1<<COM0B1) | (0<<COM0B0); // mode 3 - fast PWM
	TCCR0B = (1<<CS12) | (0<<CS11) | (0<<CS10); // set frequency to CLK/1024
	OCR0B = 0; // start with a 0% duty cycle
}

static void init_tc1( void )
{
	TCCR1B |= ((0<<CS12) | (1<<CS11) | (0<<CS10)); // set frequency to F_CPU/8
}

static int usart_send(char usart_data, FILE *stream)
{
	if(usart_data == '\n') usart_send('\r', 0);
	// Wait while previous byte is completed
	while(!(UCSR0A & (1<<UDRE0)));
	// Transmit data
	UDR0 = usart_data;
	return 0;
}

static int usart_receive(FILE *stream)
{
	char usart_data;
	// Wait for byte to be received
	while(!(UCSR0A & (1<<RXC0)));
	usart_data = UDR0;
	return usart_data;
}

static uint8_t parse_user_input( char* user_input, char* arg_buff[] )
{
	uint8_t arg_ct = 0;
	uint8_t idx    = ARG1;
	char* token;
	
	token = strtok( user_input, " " ); // get the command
	arg_buff[CMD] = token;

	while( arg_ct < N_ARGS )
	{
		if( NULL == ( token = strtok( NULL, " " ) ) ) // no arguments found
		{
			token = strtok( arg_buff[CMD], "\r" ); // remove the carriage return from the command
			arg_buff[CMD] = token;
			break;
		}
		else
		{
			arg_buff[idx++] = token;
		}
		arg_ct++;
	}
	return arg_ct;
}

static void cli_process( char* user_input )
{
	uint8_t arg_ct;
	int val;
	
	arg_ct = parse_user_input( user_input, arg_buff );
	
	if( !strcmp( arg_buff[CMD], "dist" ) )
    {
		printf( "distance = %u\n", get_distance() );
    }	
	
	else if( !strcmp( arg_buff[CMD], "motor_on" ) )
	{
		MOTOR_ON;
		printf("motor is on\n");
	}
	
	else if( !strcmp( arg_buff[CMD], "motor_off" ) )
	{
		MOTOR_OFF;
		printf("motor is off\n");
	}	
	
    else if( !strcmp( arg_buff[CMD], "set_duty" ) )
    {
	    if( arg_ct < 1 )
	    {
		    printf( "ERROR: Not enough arguments specified\n" );
		    return;
	    }
	    val = atoi( arg_buff[ARG1] );
		val = (val > 0xFF) ? 0xFF : val;
		set_duty(val);
		printf("duty cycle set to %u/255\n", val);
    }	
	
	else if( !strcmp( arg_buff[CMD], "stats" ) )
	{
		printf("duty cycle      = %u/255\n", get_duty() );
		printf("shake time [ms] = %u\n", shake_time_ms );
		printf("threshold [cm]  = %u\n", threshold_cm );
		if( armed )
		{
			printf("armed            = true\n");
		}
		else
		{
			printf("armed           = false\n");
		}
	}	
	
	else if( !strcmp( arg_buff[CMD], "set_threshold" ) )
	{
	    if( arg_ct < 1 )
	    {
			threshold_cm = get_distance() - 15;
	    }	
		else
		{	
			val = atoi( arg_buff[ARG1] );
			val = (val > 500) ? 500 : val;
			threshold_cm = val;
		}
		printf("threshold set to %u [cm]\n",threshold_cm );		
	}
	
	else if( !strcmp( arg_buff[CMD], "set_shake_time" ) )
	{
		if( arg_ct < 1 )
		{
			printf( "ERROR: Not enough arguments specified\n" );
			return;
		}
		val = atoi( arg_buff[ARG1] );
		shake_time_ms = val;
		printf("shake time set to %u [ms]\n",shake_time_ms );
	}	
	
	else if( !strcmp( arg_buff[CMD], "shake" ) )
	{
		shake();
	}	
	
	else if( !strcmp( arg_buff[CMD], "arm" ) )
	{
		if( arg_ct < 1 )
		{
			printf( "ERROR: Not enough arguments specified\n" );
			return;
		}	
		val = atoi( arg_buff[ARG1] );
		if( 0 == val )
		{
			printf("TreeShaker is not armed\n");
			armed = false;
		}	
		else
		{
			printf("TreeShaker is armed\n");
			armed = true;			
		}	
	}
	
	else if( !strcmp( arg_buff[CMD], "help" ) )
	{
		printf("dist\n");
		printf("motor_on\n");
		printf("motor_off\n");
		printf("set_duty\n");
		printf("stats\n");
		printf("set_threshold\n");
		printf("set_shake_time\n");
		printf("shake\n");
		printf("arm\n");		
		printf("help\n");
	}
};

uint16_t get_distance(void)
{
	uint16_t distance;
	
	uint16_t count;

	TRIGGER_HIGH;
	_delay_us(10);
	TRIGGER_LOW;
	
	while(!(PINB & (1<<PINB0))); // wait for echo to go high
	//TIMMER_ON;
	TIFR1 |= (1<<TOV1); // clear overflow flag
	TCNT1 = 0;
	while(PINB & (1<<PINB0)); // wait for echo to go low
	count = TCNT1;

	//Calculate distance:
	if( ( TIFR1 & (1<<TOV1) ) || (count > 38000 ) ) // 38ms if no obstacle
	{
		//Set distance to maximum if overflow occurs:
		distance = 500; // cm 
		
		//Clear overflow flag:
	}
	else distance = count / 58;
	_delay_ms(100);

	return distance; 
}

static void set_duty( uint8_t duty_cycle )
{
	OCR0B = duty_cycle;
}

static uint8_t get_duty( void )
{
	return OCR0B;
}

static void shake( void )
{
	MOTOR_ON;
	delay_ms( shake_time_ms );
	MOTOR_OFF;
}

static void delay_ms( uint16_t delay )
{
	uint16_t i;
	
	for( i = 0; i < delay; i++ )
	{
		_delay_ms(1);
	}
}

// ISR ////////////////////////////////////////////////////////////////////////
ISR( USART_RX_vect )
{
	char user_input[MAX_INPUT_SIZE];
	char c;
	uint8_t i = 0;
	
	while( ( 0x0d != c ) && ( MAX_INPUT_SIZE > i ) )
	{
		c = fgetc( &usart0_stream );
		user_input[i++] = c;
	}
	cli_process( user_input );
	printf("treeshaker>> ");
}



