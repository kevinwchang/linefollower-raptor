/* linefolllower - an application for the Pololu Baby Orangutan B
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 2013-07-02 16:26:20
 *  Author: kevin
 */

#define DIR_L IO_B0
#define DIR_R IO_D7
#define PWM_L IO_D6
#define PWM_R IO_D5

#define BUTTON IO_D3

#define LED_L IO_B2
#define LED_R IO_B1

#include <pololu/orangutandigital.h>
#include <pololu/orangutantime.h>
#include <pololu/qtr.h>
#include <stdlib.h>

void init_motors()
{
	// configure for non-inverted fast PWM output on motor control pins:
	//  clear OCxx on compare match, set on timer overflow
	//  Timer0 counts up from 0 to 255 and then overflows directly to 0
	TCCR0A = 0b00000011;
	
	// use the system clock/8 (=2.5 MHz) as the timer clock,
	// which will produce a PWM frequency of 10 kHz
	TCCR0B = 0b00000010;

	// initialize all PWMs to 0% duty cycle (braking)
	OCR0A = OCR0B = 0;
	
	set_digital_output(PWM_L, LOW);
	set_digital_output(PWM_R, LOW);
}

void set_left_motor(int speed)
{
  if (speed == 0)
  TCCR0A &= ~(1 << COM0B1);
  else
  {
    set_digital_output(DIR_R, (speed > 0));
    OCR0B = abs(speed);
    TCCR0A |= (1 << COM0B1);
  }
}

void set_right_motor(int speed)
{
	if (speed == 0)
		TCCR0A &= ~(1 << COM0A1);
	else
	{
		set_digital_output(DIR_L, (speed > 0));
		OCR0A = abs(speed);
		TCCR0A |= (1 << COM0A1);
	}
}

void set_motors(int left_speed, int right_speed)
{
	set_left_motor(left_speed);
	set_right_motor(right_speed);
}

void init_leds()
{
	// Timer 1 configuration
	// prescaler: clockI/O / 1
	// outputs enabled
	// fast PWM, 8-bit
	//
	// PWM frequency calculation
	// 20MHz / 8 (prescaler) / 256 (8-bit) = ~10kHz
	TCCR1A = 0b00000001;
	TCCR1B = 0b00001010;
	
	// initialize all PWMs to 0% duty cycle
	OCR1A = OCR1B = 0;
	  
	set_digital_output(LED_L, LOW);
	set_digital_output(LED_R, LOW);
}

void set_left_led(unsigned char brightness)
{
	if (brightness == 0)
		TCCR1A &= ~(1 << COM1B1);
	else
	{
		OCR1B = brightness;
		TCCR1A |= (1 << COM1B1);
	}
}

void set_right_led(unsigned char brightness)
{
	if (brightness == 0)
		TCCR1A &= ~(1 << COM1A1);
	else
	{
		OCR1A = brightness;
		TCCR1A |= (1 << COM1A1);
	}
}

void set_leds(unsigned char left_brightness, unsigned char right_brightness)
{
	set_left_led(left_brightness);
	set_right_led(right_brightness);
}

unsigned char button_is_pressed()
{
	return !is_digital_input_high(BUTTON);
}

void wait_for_button_press()
{
	do 
	{
		while(!button_is_pressed());
		delay_ms(10);
	} while (!button_is_pressed());
}

void wait_for_button_release()
{
	do
	{
		while(button_is_pressed());
		delay_ms(10);
	} while (button_is_pressed());
}


void wait_for_button()
{
	wait_for_button_press();
	wait_for_button_release();
}

unsigned char get_single_debounced_button_release()
{
	static unsigned char state = 0;
	static unsigned long prev_time_millis = 0;
	
	unsigned long time_millis = millis();
	
	switch (state)
	{
		case 0:
		if (button_is_pressed())					// if button is pressed
		{
			prev_time_millis = time_millis;
			state = 1;								// proceed to next state
		}
		break;
		
		case 1:
		if (time_millis - prev_time_millis >= 15)	// if 15 ms or longer has elapsed
		{
			if (button_is_pressed())                // and button is still pressed
			state = 2;								// proceed to next state
			else
			state = 0;								// go back to previous (initial) state
		}
		break;
		
		case 2:
		if (!button_is_pressed())                   // if button is now released
		{
			prev_time_millis = time_millis;
			state = 3;								// proceed to next state
		}
		break;
		
		case 3:
		if (time_millis - prev_time_millis >= 15)	// if 15 ms or longer has elapsed
		{
			if (!button_is_pressed())				// and button is still released
			{
				state = 0;                          // next state becomes initial state
				return 1;							// report button release
			}
			else
			state = 2;								// go back to previous state
		}
		break;
	}
	
	return 0;
}

#define NUM_AVG 64
#define MAX(a, b) ( ((a) > (b)) ? (a) : (b) )
#define MIN(a, b) ( ((a) < (b)) ? (a) : (b) )

void update_turn_signals(int proportional)
{
	static int old_proportionals[NUM_AVG];
		
	int avg_proportional = 0;
			
	for (int i = 0; i < (NUM_AVG - 1); i++)
	{
		old_proportionals[i] = old_proportionals[i + 1];
		avg_proportional += old_proportionals[i];
	}
	old_proportionals[NUM_AVG - 1] = proportional/4;
	avg_proportional = (avg_proportional + old_proportionals[NUM_AVG - 1]) / NUM_AVG;
			
	if (avg_proportional < -10)
	{
		int brightness = 400 - ((millis() << 1) & 0x1FF);
		set_left_led(MAX(MIN(brightness, 255), 0));
	}
	else if (avg_proportional > 15)
	{
		set_left_led(0);
	}
	
	if (avg_proportional > 75) 
	{
		int brightness = 400 - ((millis() << 1) & 0x1FF);
		set_right_led(MAX(MIN(brightness, 255), 0));
	}
	else  if (avg_proportional < 50)
	{
		set_right_led(0);
	}
}

int main()
{
	unsigned char qtr_pins[] = {IO_D4, IO_B3, IO_D2};
	unsigned int sensors[3];
	unsigned int last_proportional = 0;
	long integral = 0;
  
  int max = 0;
  int const MaxEver = 180;
  unsigned int startMs;

	init_motors();
	init_leds();
	qtr_rc_init(qtr_pins, 3, 1000, 255);
	
	set_digital_input(BUTTON, PULL_UP_ENABLED);
	
	set_digital_output(DIR_L, HIGH);
	set_digital_output(DIR_R, HIGH);

	while (1)
	{
		int brightness = MIN(abs(255 - ((millis() >> 2) & 0x1FF)), 255);
		set_leds(brightness, 255 - brightness);
		if (get_single_debounced_button_release()) break;
	}
	set_leds(0, 0);
	delay_ms(1000);
	
	for(int counter = 0; counter < 80; counter++)
	{
		if (counter < 20 || counter >= 60)
		{
			set_leds(0, 255);
			set_motors(50,-50);
		}
		else
		{
			set_leds(255, 0);
			set_motors(-50,50);
		}

		// This function records a set of sensor readings and keeps
		// track of the minimum and maximum values encountered.  The
		// IR_EMITTERS_ON argument means that the IR LEDs will be
		// turned on during the reading, which is usually what you
		// want.
		qtr_calibrate(QTR_EMITTERS_ON);

		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1600 ms.
		delay_ms(20);
	}
	set_motors(0,0);
	
	
	while (1)
	{
		while (1)
		{
			int brightness = (unsigned char)(millis() >> 2);
			set_leds(brightness, brightness);
			if (get_single_debounced_button_release()) break;
		}
		set_leds(0, 0);
		delay_ms(1000);
    startMs = millis();
	
		while (1)
		{
      if (max < MaxEver)
      {
        max = (unsigned int)(millis() - startMs) / 2;
        if (max > MaxEver)
          max = MaxEver;
      }
              
			// Get the position of the line.  Note that we *must* provide
			// the "sensors" argument to read_line() here, even though we
			// are not interested in the individual sensor readings.
			unsigned int position = qtr_read_line(sensors, QTR_EMITTERS_ON);

			// The "proportional" term should be 0 when we are on the line.
			int proportional = ((int)position) - 1000;
		
			// Compute the derivative (change) and integral (sum) of the
			// position.
			int derivative = proportional - last_proportional;
			integral += proportional;

			// Remember the last position.
			last_proportional = proportional;

			// Compute the difference between the two motor power settings,
			// m1 - m2.  If this is a positive number the robot will turn
			// to the right.  If it is a negative number, the robot will
			// turn to the left, and the magnitude of the number determines
			// the sharpness of the turn.
			//int power_difference = proportional/4 + derivative*18;
			int power_difference = proportional*4/6 + derivative*30;

			// Compute the actual motor settings.  We never set either motor
			// to a negative value.
			const int diff_max = max;
			if(power_difference > diff_max)
			power_difference = diff_max;
			if(power_difference < -diff_max)
			power_difference = -diff_max;

			if(power_difference < 0)
			set_motors(max + power_difference, max);
			else
			set_motors(max, max - power_difference);
		
			//update_turn_signals(proportional);
		
			if (button_is_pressed()) break;
		}
	
		set_motors(0, 0);
		wait_for_button_release();
	}
}
