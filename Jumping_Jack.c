//importing header files
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include <avr/interrupt.h>
#include <string.h>

//input pins from the keypad for columns
#define KEYPAD_C1 0   //PORTB       
#define KEYPAD_C2 1   //PORTB 
#define KEYPAD_C3 3   //PORTC
#define KEYPAD_C4 4   //PORTC

//output pins from the keypad for rows
#define KEYPAD_R1 2   //PORTB
#define KEYPAD_R2 3   //PORTB
#define KEYPAD_R3 4   //PORTB
#define KEYPAD_R4 5   //PORTB


#define BUZZER            1         //output port for playing buzzer sounds                         - PORTB
#define PUSH_BUTTON       2         //intterupt input for jump                                      - PORTD(PWM)
#define LCD_BRIGHTNESS    3         //output port for auto change brightness of the lcd             - PORTD(PWM)
#define LDR               5         //input port of ldr for getting values for adjust the brightness- PORTC

#define BAUD_RATE 103


/* 
-----------------------------------------------EEPROM-----------------------------------------------
*/
void EEPROMwrite(unsigned int address, char data){  //EEPROM write function
    while(EECR &(1<<EEPE));             //Wait for the completion of previous writes

    EEAR=address;                       //setup address
    EEDR=data;                          //setup data register

    EECR |=(1<<EEMPE);                  //writing logical 1 to EEMPE(EEPROM master write enable)

    EECR |=(1<<EEPE);                   //start the EEPROM write(EEPROM write enable)
}

char EEPROMread(unsigned int address){  //EEPROM read function
    while(EECR &(1<<EEPE));             //Wait for thecompletion of previous writes

    EEAR=address;                       //setup address

    EECR |=(1<<EERE);                   //start the EEPROM read(EEPROM read enable)

    return EEDR;                        //return the data in the address
}


/* 
-----------------------------------------------Serial communication-----------------------------------------------
*/
void usart_init(unsigned short X){
	UBRR0= X;	                            //Bauder rate value assigned
  	
  	UCSR0B |= (1<<TXEN0);	                      //Turn on transmitter
  	UCSR0B |= (1<<RXEN0);                       //Turn on receiver
  
  	UCSR0C &= ~((1<<UPM00)|(1<<UPM01)) ;        //no parity
    UCSR0C &= ~((1<<UMSEL00)|(1<<UMSEL01));     //asynchronus operation
    UCSR0C &= ~(1<<USBS0);                      //one stop bit

    UCSR0C &= ~(1<<UCSZ02);                     //8-bit character
    UCSR0C |= ((1<<UCSZ01)|(1<<UCSZ00));
}

void usart_send(char C){                       //serial monitor send function
  while((UCSR0A &(1 << UDRE0))==0);	           //monitor the flag to check if UDR0 register is ready to accept new data
  UDR0=C;                       	             //when reading is done give next bite	
}

void transmit(char data[]){                    //function to transmit the given data character by character
    for(int i=0;i<strlen(data);i++){
		usart_send(data[i]);
    }
}

/* 
--------------------------------------------------Custom delay function--------------------------------------------------
*/
void custom_delay_ms(int delay){
    for(int i=0;i<delay/2;i++){ 
        TCNT0 = 131;	              //initial value decalared as 131 for 2 ms delay

        TCCR0A = 0x00;	            
        TCCR0B = 0x04;              //precaler 1:256 enabled

        while((TIFR0 & 0x01) == 0);	//counting until integer overflow occurs

        TCCR0A = 0x00;	            //resetting the prescaler
        TCCR0B = 0x00;

        TIFR0 = 0x01;	              //Making TIFR0=1 to clear the timer overflow bit(T0V0) for the next round
    }
}


/* 
--------------------------------------------------KeyPad--------------------------------------------------
*/
char keypad[4][4]={         //keypad structure with rows and columns combination
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

int inputRow[4]={KEYPAD_R1,KEYPAD_R2,KEYPAD_R3,KEYPAD_R4};                  //Row output pins
int inputCol[4]={KEYPAD_C1,KEYPAD_C2,KEYPAD_C3,KEYPAD_C4};                  //Column input pins

char getkey(){                                                              //using polling to get the key
    DDRB &= ~(1<<KEYPAD_C2);                                                //turning the buzzer output into INPUT for keypad use
    while (1){                                                              
       for(int i=0;i<4;i++){
           PORTB &= ~((1<<KEYPAD_R1) |(1<<KEYPAD_R2) | (1<<KEYPAD_R3) | (1<<KEYPAD_R4) );  //clearing the Row output pins

           PORTB |= 1<<inputRow[i];                                         //Making 1 Row output pin HIGH at a time

           for(int j=0;j<4;j++){                                            //Checking every column for a HIGH line 
               if(j<2){
                   if(PINB & (1<<inputCol[j])){
                       custom_delay_ms(200);
                       return (keypad[i][j]);                              //If a HIGH combination is found return the character corresponding to relevant row and column
                   }
               }else{
                   if(PINC & (1<<inputCol[j])){
                       custom_delay_ms(200);
                       return (keypad[i][j]);
                   }
               }
           } 
       }
    }
}


/* 
-----------------------------------------------Custom characters-----------------------------------------------
*/
static const PROGMEM unsigned char customCharacters[] ={   
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  //Empty obstacle block          - lcdputc[0]
  0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,  //Full obstacle block           - lcdputc[1]
  0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,  //Half obstacle block           - lcdputc[2]
  0x0E,0x0E,0x15,0x1F,0x0E,0x04,0x0E,0x1B,  //Character ground position     - lcdputc[3]
  0x0E,0x0E,0x15,0x1F,0x0E,0x15,0x1F,0x00,  //Character Jump position       - lcdputc[4]
  0x11,0x11,0x0A,0x00,0x11,0x1B,0x11,0x04,  //Character caught in obstacle  - lcdputc[5]
  0x00,0x04,0x0A,0x15,0x0E,0x1B,0x11,0x00,  //UP arrow                      - lcdputc[6]
  0x00,0x11,0x1B,0x0E,0x15,0x0A,0x04,0x00   //DOWN arrow                    - lcdputc[7]
};


/* 
-----------------------------------------------Obstacles generation-----------------------------------------------
*/
//0- Empty space, 1-Obstacle,  2- Halfobstacle

int obstacles_e[]={                     //Obstacles for Easy mode - 48 
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,1,0,0,0,1,1,1,0,1,0,0,0,
    1,1,1,0,0,0,1,1,0,0,0,0,1,0,0,1,
    0,0,1,1,0,0,1,1,0,0,1,0,0,1,0,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

int obstacles_m[]={                     //Obstacles for Medium mode - 112
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,1,1,1,1,0,0,0,1,1,0,0,0,
    1,1,0,0,1,1,0,0,0,1,1,0,0,0,1,1,
    0,0,1,0,0,1,1,1,0,0,0,1,1,0,0,0,
    1,1,1,0,0,0,1,1,1,0,0,1,1,0,0,0,
    1,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

int obstacles_h[]={                     //Obstacles for Hard mode - 160
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,1,1,0,0,0,1,1,0,0,0,2,0,
    1,1,1,0,0,0,1,1,0,0,2,0,2,1,0,2,
    1,1,2,0,0,0,1,1,0,0,0,1,1,0,0,0,
    1,1,2,0,0,0,1,1,1,0,0,1,1,0,0,2,
    1,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0,
    1,0,0,0,2,1,1,2,0,0,1,1,1,0,0,0,
    2,1,1,2,0,0,1,0,1,0,1,1,0,0,0,0,
    1,1,0,2,2,0,2,1,2,0,0,1,0,0,1,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};

/* 
-----------------------------------------------Buzzer control-----------------------------------------------
*/
void buzzer_sound(int vol, int freq){
  DDRB |= (1<<BUZZER);                  //turning the input for keypad pin into OUTPUT for Buzzer use  
  TCCR1A |= 1<<COM1A1;                  // non inverting mode
  TCCR1A |= (1<<WGM11)|(1<<WGM10) ;     //fast PWM mode
  TCCR1B = freq;                        //selecting 64 prescaler

  OCR1A = vol;                          //Turning on the buzzer
  custom_delay_ms(1000);                //sound the buzzer for 1000ms
  OCR1A = 0;                            //turn off the buzzer
}


/* 
-----------------------------------------------LCD brightness control-----------------------------------------------
*/
void LCD_brightness(){
  if((ADCSRA & (1<<ADIF))==0x10){	                    //Check for the flag
    if(ADCH>127){                                     //high Brightness (full scale)
      OCR2B = 255;
    }
    else if(ADCH>63){                                 //mid Brightness (half scale)
      OCR2B = 127 ;
    }else{                                            //low Brightness (Quarter scale)
      OCR2B = 63 ;                                    
    }   
    ADCSRA |= 1<<ADSC;                              	//Turning back on the convertion process. 
  }
}


/* 
-----------------------------------------------Game Play-----------------------------------------------
*/

//Global Variables for the game 
int JUMP=0;                     //Jump variable to select the jump position [JUMP==1 - Jump up, JUMP==0 - stay on ground]
int volume;                     //volume variable


void gamePlay(int length, char obstacles[], int score, char name[], int EEPROMcount, int inc, int speed){
  char buffer[7];                       //Store score in string format
  int over=0;                           //Game OVER variable [over==1 - Game over, over==0 - Game no over] 
  int value;                            //Variable to refer from EEPROM stored score
  JUMP=0;                               //Stay on ground at the begining of the game

  lcd_clrscr();                         //Clear screen
  lcd_gotoxy(2,0);                      //Cursor position selection
  lcd_puts("Starting Game");            //Print "Starting Game" on screen. 
  buzzer_sound(volume,4);               //play buzzer Sound

  custom_delay_ms(2000);                //display start game for 2000ms
  lcd_clrscr();                         //clear screen

  while (over==0) {/* loop until game over */
        
       /*
        * load two userdefined characters from program memory
        * into LCD controller CG RAM location 0 and 5
        */
        lcd_command(_BV(LCD_CGRAM));    /* set CG RAM start address 0 */
        for(int i=0; i<48; i++){
           lcd_data(pgm_read_byte_near(&customCharacters[i]));
        }
 
      
       
        for(int k= length ;k>=0;k--){                       //Printing the assigned charactes for the 16 positions in LCD 

            LCD_brightness();                               //Check LDR and select the brightness before every print
            lcd_gotoxy(0,1);                                //Cursor position selection

                for(int j=k;j<k+16;j++){                    //Printing the respective set of obtsacles from the obtsacle sequence array
                    lcd_putc(obstacles[j]);
                }

                if(JUMP==0 && obstacles[k+12]==0){        //Checks for authorized ground position
                    lcd_gotoxy(12,0);                     //Cursor position selection 
                    lcd_putc(0);                          //print a empty obstacle
                    lcd_gotoxy(12,1);                     //Cursor position selection
                    lcd_putc(3);                          //Character in grund position 
                }else if(JUMP==0 && (obstacles[k+12]==1 ||obstacles[k+12]==2) ){         //Checks if hero is hitting any obstavles while at ground 
                    lcd_gotoxy(12,0);                     //Cursor position selection
                    lcd_putc(0);                          //print empty cell
                    lcd_gotoxy(12,1);                     //Cursor position selection
                    lcd_putc(5);                          //print obstacle caught hero character 

                    custom_delay_ms(3000);                //display  it for 3000ms 

                    lcd_clrscr();                         //clear screen 
                    lcd_gotoxy(3,0);                      //Cursor position selection
                    lcd_puts("GAME OVER!");               //print game over on the first line of the screen
                    buzzer_sound(volume,2);               //play buzzer sound for given volume

                    custom_delay_ms(2000);                //display it for 2000ms

                    lcd_clrscr();                         //clear screen
                    lcd_gotoxy(1,0);                      //Cursor position selection
                    lcd_puts("Your Score is:");           //printing "Your Score is "
                    lcd_gotoxy(7,1);                      //Cursor position selection 
                    lcd_puts(buffer);                     //print the obtained score during the gameplay

                    custom_delay_ms(2000);                //display screen for 2000ms 
                    
                    over=1;                               //game over =1 , Game over and continue to storing process
                    break;


                }else if(JUMP==1 ){                       //if interrupt changes JUMP into 1 
                    lcd_gotoxy(12,0);                     //Cursor position selection
                    lcd_putc(4);                          //printing jump character on the uppper line 
                    lcd_gotoxy(12,1);                     //Cursor position selection
                    lcd_putc(obstacles[k+12]);            //printing obstacle relevant to lower cell 
                    score+=inc;                           //adding score for each jump with respect to given level easy, meadium , or hard  
                }

                itoa( score , buffer, 10);                //converting score into a string
                lcd_gotoxy(0,0);                          //Cursor position selection
                lcd_puts(buffer);                         //display the score string at the left upper corner 

                for(int k=0;buffer[k]!='\0';k++){         //send score to the serial monitor at every jump
                  usart_send(buffer[k]);
                }
                usart_send('\n');                         //line break after every score sent

                
                custom_delay_ms(speed);                   //This delay will decide the moving speed of the obstacles                  
       }
        
        
        custom_delay_ms(20);     
    }

  custom_delay_ms(2000);                                  //When game over stay for 2000ms

  //Checking the leaderboard
  int m= EEPROMcount+68;                                  //Go to the EEPROM address of the 10th data set's 6 bit(which is the 10th power position of the score)
  value=(EEPROMread(m)-'0')*10  + (EEPROMread(m+1)-'0') ; //Get the value stored in EEPROM as integer
  while(score>value && m>=EEPROMcount){                   //while the scored value in current > value in EEPROM position and the EEPROM address is higher than the lower margin adress of 10 set of data
    m=m-7;                                                //Go up a 1 data set - 7 addresses UP in leaderboard
    value=(EEPROMread(m)-'0')*10  + (EEPROMread(m+1)-'0');//Get the value
  }

  if(m<EEPROMcount+68){                     //if the current score is greator than the score of the 10th data set
    for(int l=EEPROMcount+69 ;l>=m+2;l--){  //shift the current data sets down by 1 set 
      value=EEPROMread(l-7);
      EEPROMwrite(l,value);
    }

    if(score<10){                           //add the score into the relavant EEPROM position by considering it's range
      EEPROMwrite(m+7, '0');
      EEPROMwrite(m+7+1, buffer[0]);
    }else{
      EEPROMwrite(m+7, buffer[0]);
      EEPROMwrite(m+7+1, buffer[1]);
    }
    for(int l=0;l<5;l++){                   //add the name into the relavant EEPROM positions 
      EEPROMwrite(m+2+l, name[l]);
    }
  }
  if(m+7<EEPROMcount+6){                      //check is the current score came to the 1st place in leaderboard
    lcd_clrscr();
    lcd_gotoxy(5,0);                        //Print the highest score message in LCD 
    lcd_puts("Yayyy!");
    lcd_gotoxy(1,1);
    lcd_puts("New High Score");
    buzzer_sound(volume,4);                 //Sound buzzer sound with 2 different frequencies after printing
    buzzer_sound(volume,5);

    custom_delay_ms(2000);                  //display for 2000ms 
  }

  score=0;                                  //at the end make the score 0
}

ISR(INT0_vect){	//Interrupt service routine for INT0 vector
  JUMP^=1;      //toggle jump value for each intterupt 
} 


/* 
-----------------------------------------------Main-----------------------------------------------
*/
int main(void){
  //Port defining for Keypad
  DDRB |= ((1<<KEYPAD_R1) |(1<<KEYPAD_R2) | (1<<KEYPAD_R3) | (1<<KEYPAD_R4));             //defining 2,3,4,5 pins of PORTB as output for the keypad
  DDRB &= ~(1<<KEYPAD_C1 | 1<<KEYPAD_C2);                     //defining 4,5 pins of PORTB as inputs
  DDRC &= ~(1<<KEYPAD_C3 | 1<<KEYPAD_C4);                     //defining 3,4 pins of PORTC as inputs

  DDRD |= (1<<LCD_BRIGHTNESS);                 //defining pin 3 of PORTD as output for the LCD Brightness
  DDRC &=  ~(1<<LDR);                         //defining 5th pin of PORTC as input from LDR

  //configurations for ADC 
  ADCSRA |= (1<<ADEN);                            //Enabling the ADC
  ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));   //128 as ADC Prescale
  ADMUX |= (1<<0 | 1<<2);                         //selecting PIN5 in analog PORTC
  ADMUX &= ~ (1<<1 | 1<<3);
  ADMUX |= (1<<REFS0);	                          //Selecting the Reference voltage- AVcc with external capacitor at AREF pin 
  ADMUX &= ~(1<<REFS1);
  ADCSRA |= (1<<ADSC);                            //Starting the ADC conversion
  ADMUX |= (1<<ADLAR);                            //Right justified

  //configuration for LCD PWM
  TCCR2A |= 1<<COM2B1;                // non inverting mode
  TCCR2A |= (1<<WGM21)|(1<<WGM20) ;   //fast PWM mode
  TCCR2B = 0x03;                      //64 prescaler 
  OCR2B = 255;                        //Keep the LCD on maximum brightness from begining

  //Pushbutton and related external interrupts configuration
  DDRD &= ~(1<<PUSH_BUTTON);        //set the interrupt pin as an input
  EICRA |= (1<<ISC01);	            //set for falling and rising edge detection
	EICRA |= (1<<ISC00);	            //set for falling and rising edge detection
  sei();	                          //enable global interrupts
  EIMSK |= (1<<INT0);	              //Enable external interrupts for INT0 pin

  char name[5];                     //variable to store the name for 5 character length
  char input;                       //char variable to store the pressed key
  char input2;                      //char variable to store the secondary value of a pressed key
  char obstacles[160];              //variable array for obstacles
  char value;

  int score=0;                      //current score , which will be update for each jump score changes with the level
  int pointX;                       //cursor point X
  int speed;                        //variable for speed of ostacles in each level   
  int ObsLength;                    //variable for obstacle length for each level  
  int EEPROMcount;                  //This will decide the lower margin of the EEPROM store address for leaderboards in different mode
  int increment;                    //To store the relavant score increment value for different modes
  

  //system defaults for easy level  as default 
  ObsLength=64;                     //obstacle length for easy level
  speed= 350;                       //speed for the easy level
  EEPROMcount=0;                    //Start from 0 address in EEPROM
  increment=1;                      //score increment is 1
  volume=127;                       //default volume 
  for(int i=0;i<80;i++){            //add the obstacle array related to EASY for the obstacle array
    obstacles[i]=obstacles_e[i];
  }

  
  usart_init(BAUD_RATE);            //serial communiction initialization with 9600 baud rate

  lcd_init(LCD_DISP_ON);            //Turning LCD ON

  lcd_clrscr();                     //clear the screen

  lcd_command(_BV(LCD_CGRAM));    /* set CG RAM start address 0 */
  for(int i=0; i<48; i++){
    lcd_data(pgm_read_byte_near(&customCharacters[i]));
  }

  
  lcd_gotoxy(3,0);                  //setting the cursor point
  lcd_puts("Welcome To");           //Print the string given

  lcd_gotoxy(0,1);                  //setting the cursor point
  lcd_putc(3);

  lcd_gotoxy(2,1);                  //setting the cursor point
  lcd_puts("JUMPING JACK");         //Print Jumping jack on the lower line

  lcd_gotoxy(15,1);                  //setting the cursor point
  lcd_putc(4); 

  buzzer_sound(volume,3);           //play buzzer sound for given volume 

  custom_delay_ms(4000);            //display it for 4000ms 

  lcd_clrscr();                     //clear the screen 

  lcd_gotoxy(0,0);                  //setiing the cursor point
  lcd_puts("Enter Username:");      //Print the string given

  /*
  ---------------------------------------------------------------Getting the name from keypad---------------------------------------------------------------
  */

  for(int k=0;k<5;k++){             //empty the final array before adding new word
    name[k]='\0';
  }

  pointX=0;                         //0th X position for cursor

  transmit("Username: ");
  while(input!='#'){                //until done key='#' given
                
    input = getkey();               //Wait for key to be pressed and get the key
                
    lcd_gotoxy(pointX,1);
    lcd_putc(input);                //print the entered letter on LCD

    if(input!='#'){                 //if it's the done key don't add it to final array
      name[pointX]=input;
      usart_send(input);
    }
                
    pointX++;                       //move in x-axis by 1 point 
    custom_delay_ms(500);           //delay for 500ms 
  }
  usart_send('\n');                 //print a line break on serial monitor

  custom_delay_ms(1000);            //delay for 1000ms 

  /*
  ---------------------------------------------------------------Main menu---------------------------------------------------------------
  */
  while(1){
    lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
    for(int i=0; i<64; i++){
      lcd_data(pgm_read_byte_near(&customCharacters[i]));
    }

    lcd_clrscr();                   //clear lcd screen 

    lcd_gotoxy(0,0);                //setiing the cursor point
    lcd_puts("1-Start Game");       //start game option

    lcd_gotoxy(0,1);                //setiing the cursor point
    lcd_puts("2-Difficulty");       //Change difficulty option

    lcd_gotoxy(14,1);                //Lower line right aligned
    lcd_putc(7);                     //scroll down option
    lcd_putc('D');                    

    input = getkey();               //Wait for key to be pressed and get the key
    custom_delay_ms(500);           //display it for 500ms 

    /*
    ---------------------------------------------------------------Game play---------------------------------------------------------------
    */
    if(input=='1'){                 
      gamePlay(ObsLength,obstacles,score,name, EEPROMcount,increment, speed);     //Initialize the game
    }
    
    /*
    ---------------------------------------------------------------Difficulty change---------------------------------------------------------------
    */
    else if(input=='2'){
      while(input!='B'){
        lcd_clrscr();

        lcd_gotoxy(0,0);                //setiing the cursor point
        lcd_puts("1-Easy");             //print the 1-Easy option on the upper line 
        lcd_gotoxy(8,0);                //setiing the cursor point
        lcd_puts("2-Medium");           //print the 2-Medium option on the upper line 

        lcd_gotoxy(0,1);                //setiing the cursor point
        lcd_puts("3-Hard");             //print the 3- hard  option on the lower line 
        lcd_gotoxy(8,1);                //setiing the cursor point
        lcd_puts("B-Back");             //print the B-Back option on the lower line 

        input=getkey();                 
        custom_delay_ms(500);           //delay for 500ms  

        if(input=='1'){                 //making system settings for EASY mode as same as the default mode 
          ObsLength=64;
          speed= 350;                   
          EEPROMcount=0;
          increment=1;
          for(int i=0;i<80;i++){
            obstacles[i]=obstacles_e[i];
          }

          lcd_clrscr();                   //clear the screen 
          lcd_gotoxy(3,0);                //setiing the cursor point
          lcd_puts("Game Mode:");         //print game mode on the upper line 
          lcd_gotoxy(6,1);                //setiing the cursor point
          lcd_puts("EASY");               //print easy on lower line 
          custom_delay_ms(500);           // display it for 500ms 
          input='B';                      //return back to the menu 

        }else if(input=='2'){             //making system settings for medium level
          ObsLength=96;
          speed= 250;
          EEPROMcount=70;
          increment=2;
          for(int i=0;i<112;i++){
            obstacles[i]=obstacles_m[i];
          }

          lcd_clrscr();
          lcd_gotoxy(3,0);                //setiing the cursor point
          lcd_puts("Game Mode:");         //print game mode on upper line 
          lcd_gotoxy(5,1);                //setiing the cursor point
          lcd_puts("MEDIUM");             //print Medium on lower line 
          custom_delay_ms(500);           //display it for 500ms 
          input='B';                      //return back to the menu 


        }else if(input=='3'){             //making system settings for hard level
          ObsLength=144;
          speed= 200;
          EEPROMcount=140;
          increment=3;
          for(int i=0;i<160;i++){
            obstacles[i]=obstacles_h[i];
          }

          lcd_clrscr();
          lcd_gotoxy(3,0);                //setiing the cursor point
          lcd_puts("Game Mode:");         //print game mode on upper line
          lcd_gotoxy(6,1);                //setiing the cursor point
          lcd_puts("HARD");               //print hard on lower line
          custom_delay_ms(500);           //display it for 500ms 
          input='B';


        }else if(input=='B'){             //do nothin and return to main menu 

        }else{                            // chech for non given inputs and if fails and print Invalid input on upper line 
          lcd_clrscr();                   //clear screen
          lcd_gotoxy(0,0);                //selecting cursor point 
          lcd_puts("Inavlid Input!");     //print invalid input   
          custom_delay_ms(1000);          //display it for 1000ms 
        }
      }


    }
    /*
    ---------------------------------------------------------------Level 2 menu---------------------------------------------------------------
    */
    else if(input=='D'){
      while(input!='C'){
      
        lcd_clrscr();

        lcd_gotoxy(0,0);                  //setiing the cursor point
        lcd_puts("3-Volume");             //print volume  option

        lcd_gotoxy(14,0);                //Upper line right aligned
        lcd_putc(6);                     //scroll up option
        lcd_putc('C');

        lcd_gotoxy(0,1);                 //setiing the cursor point
        lcd_puts("4-Username");          //print username option 

        lcd_gotoxy(14,1);                //Lower line right aligned
        lcd_putc(7);                     //scroll down option
        lcd_putc('D');

        input = getkey();
        
        /*
        ---------------------------------------------------------------Volume Change---------------------------------------------------------------
        */
        if(input=='3'){                      //volume change option 
          while(input!='B'){
            lcd_clrscr();                   //clear screeen 

            lcd_gotoxy(0,0);                //setiing the cursor point
            lcd_puts("1-Low");              //print option for low volume
            lcd_gotoxy(8,0);                //setiing the cursor point
            lcd_puts("2-Mid");              //print option for mid  volume 

            lcd_gotoxy(0,1);                //setiing the cursor point
            lcd_puts("3-High");             //print option for high volume  
            lcd_gotoxy(8,1);                //setiing the cursor point
            lcd_puts("B-Back");             //print option for back 

            input=getkey();
            custom_delay_ms(1000);

            if(input=='1'){                   //if selected option was low
              volume=152;                     //adjust volume 
              lcd_clrscr();
              lcd_gotoxy(5,0);                //setiing the cursor point
              lcd_puts("Volume");             //print volume on upper line 
              lcd_gotoxy(6,1);                //setiing the cursor point
              lcd_puts("LOW");                //print low on lower line 
              custom_delay_ms(1000);          //display for 1000ms 
              input='B';

            }else if(input=='2'){             //if selected option was mid 
              volume=100;                     //adjust volume 
              lcd_clrscr();                   //clear screen 
              lcd_gotoxy(5,0);                //setiing the cursor point
              lcd_puts("Volume");             //print volume on upper line 
              lcd_gotoxy(6,1);                //setiing the cursor point
              lcd_puts("MID");                //print mid on lower line 
              custom_delay_ms(1000);          //display it for 1000ms 
              input='B';

            }else if(input=='3'){             //if selected option was high 
              volume=50;                      //adjust volume
              lcd_clrscr();                   //clear the screen 
              lcd_gotoxy(5,0);                //setiing the cursor point
              lcd_puts("Volume");             //print volume on upper line 
              lcd_gotoxy(6,1);                //setiing the cursor point
              lcd_puts("HIGH");               //print high on lower line 
              custom_delay_ms(1000);          //display it for 1000ms
              input='B';

            }else if(input=='B'){             //if option was back do nothing and go back in menu

            }else{                            //if is an invalid input 
              lcd_clrscr();                   //clear screen 
              lcd_gotoxy(0,0);                //selecting the cursor point 
              lcd_puts("Inavlid Input!");     //print invalid input on second line 
              custom_delay_ms(1000);          //display it for 1000ms 
            }
          }
        }

        /*
        ---------------------------------------------------------------Username Change---------------------------------------------------------------
        */
        else if(input=='4'){                  //input option was username 
          lcd_clrscr();                       //clear screen
          lcd_gotoxy(0,0);                    //setiing the cursor point
          lcd_puts("Enter Username:");        //Print the string given

          /*
          Getting the name from keypad
          */

          for(int k=0;k<5;k++){                //empty the final array before adding new word
            name[k]='\0';
          }

          pointX=0;
          transmit("Username: ");       //transmit to serial monitor
          while(input!='#'){            //until done key='#' 
                        
            input = getkey();
                        
            lcd_gotoxy(pointX,1);           
            lcd_putc(input);            //print the entered letter on LCD

            if(input!='#'){             //if it's the done key don't add it to final array
              name[pointX]=input;
              usart_send(input);
            }
                        
            pointX++;                   //move in x-axis by 1 point 
            custom_delay_ms(500);       //delay for 500ms 
          }
          usart_send('\n');             //print a line break on serial monitor

          custom_delay_ms(1000);        //delay for 1000ms 

          input='C';                    //go back in menu 

        }

        /*
        ---------------------------------------------------------------Level 3 menu---------------------------------------------------------------
        */
        else if(input=='D'){
          input2=0;
          while(input2!='C'){
            input=0;
            lcd_clrscr();

            lcd_gotoxy(0,0);            //setiing the cursor point
            lcd_puts("5-Scores");       //Scores option

            lcd_gotoxy(14,0);           //Lower line left aligned
            lcd_putc(6);                //up arrow 
            lcd_putc('C');

            lcd_gotoxy(0,1);            //setiing the cursor point
            lcd_puts("6-Exit");         //Exit option

            input2 = getkey();

            /*
            ---------------------------------------------------------------Leaderboard---------------------------------------------------------------
            */
            if(input2=='5'){                    //scores 
              while(input!='B'){
                lcd_clrscr();                   //clear screen

                lcd_gotoxy(0,0);                //setiing the cursor point
                lcd_puts("1-Easy");             //option of scores for easy level
                lcd_gotoxy(8,0);                //setiing the cursor point
                lcd_puts("2-Medium");           //option of scores for meadium level

                lcd_gotoxy(0,1);                //setiing the cursor point
                lcd_puts("3-Hard");             //option of scores for hard level
                lcd_gotoxy(8,1);                //setiing the cursor point
                lcd_puts("B-Back");             //option for back 

                input=getkey();
                custom_delay_ms(500);                 //delay for 500ms 

                int G=0;                              //custom variable to keep a count on the name 
                if(input=='1'){                               
                  transmit("Leaderboard: EASY\n");    //transmit easy level scoreboard 
                  for(int a=0;a<70;a++){
                    usart_send(EEPROMread(a));
                    G++;                              //increase name count after each character prints
                    if(G==5){                         //if the name is printed, print a '-' in serial monitor
                      usart_send('-');
                    }
                    if((a+1)%7==0 && a>0){            //if all the 7 data are printed, print a line break and name counter to 0
                      usart_send('\n');
                      G=0;
                    }
                  }
                  lcd_clrscr();                           //clear screen 
                  lcd_gotoxy(2,0);                        //setiing the cursor point
                  lcd_puts("leaderboard");                //print leaderboard on upper line 
                  lcd_gotoxy(6,1);                        //setiing the cursor point
                  lcd_puts("EASY");                       //print easy on lower line 
                  custom_delay_ms(1000);                  //display it for 1000ms 
                  input='B';


                }else if(input=='2'){                     //option for meadium scoreboard 
                  transmit("Leaderboard: MEDIUM\n");      //transmit meadium level scoreboard 
                  for(int a=70;a<140;a++){
                    usart_send(EEPROMread(a));
                    G++;
                    if(G==5){
                      usart_send('-');
                    }
                    if((a+1)%7==0 && a>0){
                      usart_send('\n');
                      G=0;
                    }
                  }
                  lcd_clrscr();
                  lcd_gotoxy(2,0);                         //setiing the cursor point
                  lcd_puts("leaderboard");                 //print Leaderboard on upper line 
                  lcd_gotoxy(5,1);                         //setiing the cursor point
                  lcd_puts("MEDIUM");                      //print meadium on lower line 
                  custom_delay_ms(1000);                   //display it for 1000ms 
                  input='B';


                }else if(input=='3'){                       //option for hard level scoreboard 
                  transmit("Leaderboard: HARD\n");          //transmit hard level scoreboard 
                  for(int a=140;a<210;a++){
                    usart_send(EEPROMread(a));
                    G++;
                    if(G==5){
                      usart_send('-');
                    }
                    if((a+1)%7==0 && a>0){
                      usart_send('\n');
                      G=0;
                    }
                  }
                  lcd_clrscr();
                  lcd_gotoxy(2,0);                          //setiing the cursor point
                  lcd_puts("leaderboard");                  //print Leaderboard on upper line 
                  lcd_gotoxy(6,1);                          //setiing the cursor point
                  lcd_puts("HARD");                         //print hard on lower line
                  custom_delay_ms(1000);                    //display it for 1000ms 
                  input='B';

                  
                }else if(input=='B'){                       //select option B back menu 
                  
                }else{                                      //for unrequired inputs
                  lcd_clrscr();                             //clear screen
                  lcd_gotoxy(0,0);                          //setting cursor point 
                  lcd_puts("Inavlid Input!");               //print invalid input 
                  custom_delay_ms(1000);                    //display for 100ms 
                }
              }
              

            /*
            ---------------------------------------------------------------Exit---------------------------------------------------------------
            */
            }else if(input2=='6'){                          //game closing option     
              lcd_clrscr();                                 //clear screen 
              lcd_gotoxy(2,0);                              //setiing the cursor point
              lcd_puts("Game Closing");                     //print game closing lower screen 

              custom_delay_ms(500);                         //display for 500ms 
              lcd_clrscr();                                 //clear screen

              return 0;

            }else if(input2=='C'){

            }else{
              lcd_clrscr();                                 //clear screen 
              lcd_gotoxy(0,0);                              //setting the cursor point
              lcd_puts("Inavlid Input!");                   //print Invalid input on lower line      
              custom_delay_ms(1000);                        //display it for 1000ms
            }
          }
          
        }else if(input=='C'){

        }else{
          lcd_clrscr();                                     //clear screen 
          lcd_gotoxy(0,0);                                  //setting the cursor point 
          lcd_puts("Inavlid Input!");                       //print Invalid input on lower line 
          custom_delay_ms(1000);                            //display it for 1000ms 
        }
      }

    }else{
      lcd_clrscr();                                         //clear screen 
      lcd_gotoxy(0,0);                                      //setting the cursor point 
      lcd_puts("Inavlid Input!");                           //print Invalid Input 
     custom_delay_ms(1000);                                 //display it for 1000ms 
    }
  }
  return 0;
}
