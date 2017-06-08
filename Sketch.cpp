/*
Tranzistor
(1) - Baza - D9
(2) - Colector - 5V
(3) - Emitor - GND
*/

/*
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 */

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ###################### Libraries ######################
#include <PID_v1.h>
#include<EEPROM.h>
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "MainMenu.h"


// ############# Structure - save to EEPROM ##############
// EEPROM =====================================
// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 32


int flag = 1;

// ============================================


struct Parameters
{
	char version[4];
	float tmp_racire;             
	float tmp_incalzire;               
	float tmp_mentinere;               
	float Kp;            
	float Ki;                
	float Kd; 
	float tmp_setata;                
} param = {
			CONFIG_VERSION,
			50, 60, 70, // timpi PID
			20, 2, 0,	// parametrii PID
			44};		// temp setata



// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ################ Variables declaration ################
//
#define PIN_INPUT 2 //citire temperatura analog
#define PIN_OUTPUT 9 //comanda pwm bec 

float newParameterValue = 0; // the updated parameter value after user configuration

// float RealTime = 0;
float snapTime = 0;
int analogPin = 0;		// analog pin to read the buttons
int ButtonVal;			// analog value of the buttons			
int startCol, endCol;	// LCD start/end column
int animationContor = 0;// Duration of the animation
bool animationFlag = 0; // start animation Flag (Name of creators) 	

enum buttonState {
				 previous,	// 0 - previous
				 down,		// 1 - down
 				 up,		// 2 - up
 				 next		// 3 - next
 				 };
buttonState selectedButton;

enum menuPage	{
				menuKp,				// 0
				menuKi,				// 1
				menuKd,				// 2
				menuTempSetata,		// 3
				menuTempIncrement,	// 4
				menuTempRacire,		// 5
				menuTempMentinere,	// 6
				menuStartProces,	// 7
				menuFactoryReset	// 8
				};
menuPage selectedMenuPage;
int nextDisplayedMenu;	// menu that will be displayed next
int nextDisplayedValue;	// the value after increment/decrement
int contorCurrentMenu = 0;	// contor for the current page(selected)
// #######################################################





// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ##################### PROTOTYPES FUNCTIONS ######################

void startPID(void); // PID PROT

void loadConfig(void); // EEPROM
void saveConfig(void); // EEPROM


void salvareEEPROM(void);
void meniuTemperaturaIncalzire(void);
void meniuTemperaturaMentinere(void);
void meniuTemperaturaRacire(void);

void meniuTemperaturaRacire(float incDecValue);
void meniuTemperaturaMentinere(float incDecValue);
void meniuTemperaturaIncalzire(float incDecValue);
void meniuSetareTemperatura(float incDecValue);

void meniuSetareTemperatura(void);
void clearLCD_Function(void );
void timeDisplayFunction(void );
void meniuParametruKp(void );
void meniuParametruKi(void );
void meniuParametruKd(void );

void updateMeniuKp(float Kd);
void updateMeniuKi(float Kd);
void updateMeniuKd(float Kd);

void meniuValoareTemperatura(float incDecValue);
void meniuValoareTemperatura();
void displayNextPage(int nextDisplayedMenu);
void clearAll_LCD_Function(void);
void meniuParametruKpID(float incDecValue);
void meniuParametruKiID(float incDecValue);
void meniuParametruKdID(float incDecValue);
void IncrementDecrementParametersValue(int nextDisplayedMenu, float incDecValue);
void keyRelease(void);
void keyPress(void);
void meniuFactoryReset(void);

// #######################################################





// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ##################### LCD INSTANCE ####################
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// #######################################################



// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ##################### PID INSTANCE & PARAM ####################
byte numbers[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double realTemp;
float cel;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double timpScurs;


// tROOM - variabila pt prima citire, reprezinta temp camerei
// daca tROOM >= 28 => va fi setata pe 24 Celsius
float tROOM;
// int tmp_setata = 40; // date de user
int tempSetata = 43; // date de user

double rawCelsius; // citire direct de la AnalogPin

PID myPID(&realTemp, &Output, &Setpoint, param.Kp, param.Ki, param.Kd, DIRECT);

// #######################################################






// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ###################### SETUP ######################
void setup() {

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  
  meniuParametruKp();

    rawCelsius = analogRead(PIN_INPUT);
    cel = ( rawCelsius / 1024.0) * 5000;
    Input = cel / 10;

    tROOM = Input;
    if (tROOM >= 28)
    {
	    tROOM = 24;
    }
    
    myPID.SetMode(AUTOMATIC);
}




// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// ######################## LOOP #########################
void loop() {
	
  // initialize LCD position + clear
  startCol = 0;
  endCol = 15;
  lcd.setCursor(startCol,0);  

  // citesc analogic butoanele
  ButtonVal = analogRead(analogPin);
  //lcd.setCursor(0,0);
  //lcd.print(ButtonVal);
  delay(100);
  //delay(1500);
  //lcd.clear();

   /************************************************************************/
   /* BUTTONS SECTION                                                      */
   /************************************************************************/
 if(ButtonVal >= 150 && ButtonVal <= 250)
 {
	 // ***************  LEFT  *************** //
	selectedButton = previous;
	nextDisplayedMenu = nextDisplayedMenu - 1;

	if(nextDisplayedMenu < 0)
	{
		// reset the menu if it exceed the maximum page number
		nextDisplayedMenu = 8;
	}
	
	 lcd.setCursor(15,1);
	 lcd.print(nextDisplayedMenu);
	 delay(250);
	// Next page to be displayed
	displayNextPage(nextDisplayedMenu);
 }




 if(ButtonVal >= 480 && ButtonVal <= 560 && (nextDisplayedMenu!=3))
 {
	 // ***************  DOWN  *************** //
	 selectedButton = down;
	 //nextDisplayedValue = currentValue + 0.1;
	 if(selectedMenuPage != menuFactoryReset)
	 {
	 IncrementDecrementParametersValue(nextDisplayedMenu, -0.1);
	 delay(250);
	 }
	 
	 
 }
 
 if(selectedMenuPage==menuFactoryReset && (ButtonVal >= 380 && ButtonVal <= 460 ) )
 {
 // FACTORY RESET
 	 loadConfig();
 	 clearAll_LCD_Function();
 	 lcd.setCursor(0,0);
 	 lcd.print("Factory Reset");
 	 lcd.setCursor(0,1);
 	 lcd.print("Done!");
 }



 if(selectedMenuPage==menuFactoryReset && (ButtonVal >= 480 && ButtonVal <= 560 ))
 {
	// SALVARE EEPROM
	 saveConfig();
	 clearAll_LCD_Function();
	 lcd.setCursor(0,0);
	 lcd.print("Saved in EEPROM");
 }


 if(ButtonVal >= 380 && ButtonVal <= 460 )
 {
	 // ***************  UP  *************** //
	if(selectedMenuPage == menuTempSetata)
	{
	
	selectedButton = up;
	startPID();
		 delay(2000);
	}
	 if(selectedMenuPage != menuFactoryReset)
	 {
	 selectedButton = up;
	 //nextDisplayedValue = currentValue + 0.1;
	 IncrementDecrementParametersValue(nextDisplayedMenu, 0.1);
	 delay(250);
     }
 }
 
 
 if(ButtonVal >= 10 && ButtonVal <= 50)
 {
	 // ***************  RIGHT  *************** //
	 selectedButton = next;
	 nextDisplayedMenu = nextDisplayedMenu + 1;

	 if(nextDisplayedMenu > 8)
	 {
	 // reset the menu if it exceed the maximum page number
	 nextDisplayedMenu = 0;
	 }
	 lcd.setCursor(15,1);
	 lcd.print(nextDisplayedMenu);
	 delay(250);
	 // Next page to be displayed
	 displayNextPage(nextDisplayedMenu);

	 }


  // LCD Animation start only if the flag is set
  if(animationFlag)
  {

  while(animationContor <= 2)
  {  

  for(startCol; startCol < endCol; startCol++)
  {
    lcd.setCursor(startCol, 0);
    lcd.print("Paul & Danutzz");
    delay(1000);
    clearLCD_Function();
    timeDisplayFunction();
	animationContor++;
  }
  }
  }
  }




  
  


// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
// #################### FUNCTIONS BODY ###################


void IncrementDecrementParametersValue(int nextDisplayedMenu, float incDecValue)
{
switch(nextDisplayedMenu)
{
	case 0:
	clearAll_LCD_Function();
	meniuParametruKpID(incDecValue);
	break;

	case 1:
	clearAll_LCD_Function();
	meniuParametruKiID(incDecValue);
	break;

	case 2:
	clearAll_LCD_Function();
	meniuParametruKdID(incDecValue);
	break;

	case 3:
	clearAll_LCD_Function();
	//lcd.print("Meniu 4");
	meniuValoareTemperatura(incDecValue);
	break;

	case 4:
	clearAll_LCD_Function();
	//lcd.print("Meniu 5");
	//meniuSetareTemperatura(incDecValue);
	
	meniuSetareTemperatura(incDecValue);
	break;

	case 5:
	clearAll_LCD_Function();
	//lcd.print("Meniu 6");
	
	meniuTemperaturaRacire(incDecValue);
	break;

	case 6:
	clearAll_LCD_Function();
	//lcd.print("Meniu 7");
	
	meniuTemperaturaIncalzire(incDecValue);
	break;

	case 7:
	clearAll_LCD_Function();
	//lcd.print("Meniu 8");
	
	meniuTemperaturaMentinere(incDecValue);
	break;

	case 8:
	clearAll_LCD_Function();
	meniuFactoryReset();
	break;
}
}






void displayNextPage(int nextDisplayedMenu)
{
lcd.setCursor(14,1);
lcd.print(nextDisplayedMenu);
	switch(nextDisplayedMenu)
	{
		case 0:
		clearAll_LCD_Function();
		meniuParametruKp();
		break;

		case 1:
		clearAll_LCD_Function();
		meniuParametruKi();
		break;

		case 2:
		clearAll_LCD_Function();
		meniuParametruKd();
		break;

		case 3:
		clearAll_LCD_Function();
		//lcd.print("Meniu 4");
		meniuValoareTemperatura();
		break;

		case 4:
		clearAll_LCD_Function();
		//lcd.print("Meniu 5");
		meniuSetareTemperatura();
		break;

		case 5:
		clearAll_LCD_Function();
		//lcd.print("Meniu 6");
		
		meniuTemperaturaRacire();
		break;

		case 6:
		clearAll_LCD_Function();
		//lcd.print("Meniu 7");
		
		meniuTemperaturaIncalzire();
		break;

		case 7:
		clearAll_LCD_Function();
		//lcd.print("Meniu 8");
		
		meniuTemperaturaMentinere();
		break;

		case 8:
		clearAll_LCD_Function();
		meniuFactoryReset();
		break;
	}
}





void meniuParametruKp(void)
{
	selectedMenuPage = menuKp;
	
	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Kp:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.Kp);
}

void meniuParametruKi(void)
{
	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Ki:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.Ki);
}

void meniuParametruKd(void)
{
	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Kd:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.Kd);
}





void meniuValoareTemperatura(void)
{
	lcd.setCursor(0,0);
	lcd.print("Start PID:");

	lcd.setCursor(0,1);
	lcd.print("Press Red Btn ");
}

void meniuParametruKpID(float incDecValue)
{
	selectedMenuPage = menuKp;
	newParameterValue = param.Kp + incDecValue;
	param.Kp = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Kp:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void meniuParametruKiID(float incDecValue)
{
	selectedMenuPage = menuKp;
	newParameterValue = param.Ki + incDecValue;
	param.Ki = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Ki:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void meniuParametruKdID(float incDecValue)
{
	selectedMenuPage = menuKp;
	newParameterValue = param.Kd + incDecValue;
	param.Kd = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Meniu Param. Kd:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void meniuValoareTemperatura(float incDecValue)
{
    // aici pt PID
	selectedMenuPage = menuKp;
	//newParameterValue = Kd + incDecValue;
	float temp = analogRead(2);
	float mv = ( temp/1024.0)*5000;
	float celsius = mv/10;

	
	startPID();
	
	
	clearAll_LCD_Function();

		lcd.setCursor(0,0);
		lcd.print("Start PID:");

		lcd.setCursor(0,1);
		lcd.print("Press Red Btn ");
}

void meniuTemperaturaRacire(float incDecValue)
{
// racire
	if(incDecValue > 0)
	{
	incDecValue = 0.5;
	}
	else
	{
	incDecValue = -0.5;
	}

	selectedMenuPage = menuTempRacire;
	newParameterValue = param.tmp_racire + incDecValue;
	param.tmp_racire = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Timp racire:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}


void meniuTemperaturaIncalzire(float incDecValue)
{
// incalzire
	if(incDecValue > 0)
	{
		incDecValue = 0.5;
	}
	else
	{
		incDecValue = -0.5;
	}

	selectedMenuPage = menuTempIncrement;
	newParameterValue = param.tmp_incalzire + incDecValue;
	param.tmp_incalzire = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Timp incalzire:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void meniuTemperaturaMentinere(float incDecValue)
{
// mentinere
	if(incDecValue > 0)
	{
		incDecValue = 0.5;
	}
	else
	{
		incDecValue = -0.5;
	}

	selectedMenuPage = menuTempMentinere;
	newParameterValue = param.tmp_mentinere + incDecValue;
	param.tmp_mentinere = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Timp mentinere:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void salvareEEPROM(void)
{
int adr=0;
selectedMenuPage = menuFactoryReset;

clearAll_LCD_Function();
lcd.setCursor(0,0);
lcd.print("Factory Reset:");
lcd.setCursor(0,1);
}

void meniuFactoryReset(void)
{
selectedMenuPage = menuFactoryReset;

clearAll_LCD_Function();
lcd.setCursor(0,0);
lcd.print("Fact.R or EEPROM:");
lcd.setCursor(0,1);
lcd.print("Up or Down");

lcd.setCursor(0,1);
}

void updateMeniuKp(float Kp)
{
lcd.setCursor(0,0);
lcd.print("Meniu Param. Kd:");

lcd.setCursor(0,1);
lcd.print("Valoare: ");
lcd.print(Kp);
}


void updateMeniuKi(float Ki)
{
lcd.setCursor(0,0);
lcd.print("Meniu Param. Kd:");

lcd.setCursor(0,1);
lcd.print("Valoare: ");
lcd.print(Ki);
}


void updateMeniuKd(float Kd)
{
lcd.setCursor(0,0);
lcd.print("Meniu Param. Kd:");

lcd.setCursor(0,1);
lcd.print("Valoare: ");
lcd.print(Kd);
}

void clearLCD_Function(void)
{
	lcd.setCursor(0, 0);
	lcd.print("                ");
}

void clearAll_LCD_Function(void)
{
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0,1);
	lcd.print("                ");
	lcd.setCursor(0, 0);
}


void meniuSetareTemperatura(void)
{
// aici vine codul pentru setarea temperaturii la care va ajunge
lcd.setCursor(0,0);
lcd.print("Meniu temp setata:");

lcd.setCursor(0,1);
lcd.print("Valoare: ");
lcd.print(param.tmp_setata);
}

void meniuSetareTemperatura(float incDecValue)
{
	// racire
	if(incDecValue > 0)
	{
		incDecValue = 0.5;
	}
	else
	{
		incDecValue = -0.5;
	}

	selectedMenuPage = menuTempSetata;
	newParameterValue = param.tmp_setata + incDecValue;
	param.tmp_setata = newParameterValue;

	lcd.setCursor(0,0);
	lcd.print("Meniu temp setata:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(newParameterValue);
}

void meniuTemperaturaRacire(void)
{
	
	lcd.setCursor(0,0);
	lcd.print("Timp racire:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.tmp_racire);
}

void meniuTemperaturaIncalzire(void)
{
	
	lcd.setCursor(0,0);
	lcd.print("Timp incalzire:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.tmp_incalzire);
}

void meniuTemperaturaMentinere(void)
{
	
	lcd.setCursor(0,0);
	lcd.print("Timp mentinere:");

	lcd.setCursor(0,1);
	lcd.print("Valoare: ");
	lcd.print(param.tmp_mentinere);
}

void timeDisplayFunction(void)
{
	lcd.setCursor(0, 1);
	lcd.print(millis() / 1000);
}




// ------------------------------------------------------------------
// PID functions
// ------------------------------------------------------------------
void tempUP()
{
	timpScurs = (millis() / 1000) - snapTime;
	Setpoint = tROOM + ((timpScurs / param.tmp_setata) * (param.tmp_setata - tROOM));
	myPID.SetTunings(param.Kp, param.Ki, param.Kd);
	myPID.Compute();
	if(Output <= 50)
	{
	Output = 50;
	}
	analogWrite(PIN_OUTPUT, Output);
}


void tempMENTINERE()
{
	Setpoint = param.tmp_setata;
	timpScurs = (millis() / 1000) - snapTime;

	myPID.SetTunings(param.Kp, param.Ki, param.Kd);
	myPID.Compute();
	analogWrite(PIN_OUTPUT, Output);
}


void tempDOWN()
{

	timpScurs = (millis() / 1000) - snapTime;
	Setpoint = param.tmp_setata + (param.tmp_setata - ((timpScurs / param.tmp_setata) * (param.tmp_setata - tROOM)));
	myPID.SetTunings(param.Kp, param.Ki, param.Kd);
	myPID.Compute();
	analogWrite(PIN_OUTPUT, Output);
}

void isort(byte *a, int n)
{
	for (int i = 1; i < n; ++i)
	{
		int j = a[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < a[k]); k--)
		{
			a[k + 1] = a[k];
		}
		a[k + 1] = j;
	}
}

float printArray(byte *a, float n)
{
	for (int i = 0; i < n; i++)
	{
		//        Serial.print((float)a[i]);
		//        Serial.print(' ');
	}
	float valFin = 0;
	int contor = 0;
	Serial.println("");
	for ( int i = 3; i < n - 3; i++ )
	{
		contor = contor + 1;
		valFin = valFin + a[i];

		//Serial.print((float)a[i]);
		Serial.print(' ');
	}
	Serial.println();
	//Serial.print("Valoare medie: ");
	//Serial.println(valFin / contor);
	return (valFin / contor);
	delay(250);
}

void startPID(void)
{
// face asta cat timp {timpii adunati crestere, mentinere, coborare}
  
  snapTime = millis() / 1000;
  
  while(((millis() / 1000) - snapTime) <= (param.tmp_incalzire + param.tmp_mentinere + param.tmp_racire))
  {
  // facem 10 citiri pt a ne asigura ca citirea este corecta
  delay(50);
  for (int i = 0; i < 10; i++)
  {
	  rawCelsius = analogRead(PIN_INPUT);
	  cel = ( rawCelsius / 1024.0) * 5000;
	  cel = cel / 10;
	  numbers[i] = cel;
  }

  //printArray(numbers, sizeof(numbers));

  // sortam cele 10 valori citite
  isort(numbers, sizeof(numbers));
  realTemp = printArray(numbers, sizeof(numbers));
  Input = realTemp;

    lcd.setCursor(0,0);
  	lcd.print("out: ");
  	lcd.setCursor(5,0);
  	lcd.print(Output);


	    lcd.setCursor(11,0);
	    lcd.print(timpScurs);

  	// realTemp
  	lcd.setCursor(0,1);
  	lcd.print("Temp: ");
  	lcd.setCursor(6, 1);
  	lcd.print(realTemp);
    // timpScurs
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.print("teMP: ");
  Serial.println(realTemp);
  Serial.print("timp: ");
  Serial.println(timpScurs);

  if (timpScurs < param.tmp_incalzire)
  {
    	lcd.setCursor(12, 1);
    	lcd.print("Ur");
	  Serial.println("Urca");
	  tempUP();
  }
  else if (timpScurs < (param.tmp_incalzire + param.tmp_mentinere))
  {
    	lcd.setCursor(12, 1);
    	lcd.print("Me");
	  Serial.println("Mentine");
	  // mentinere
	  tempMENTINERE();
  }
  else if (timpScurs <= (param.tmp_incalzire + param.tmp_mentinere + param.tmp_racire))
  {
    	lcd.setCursor(12, 1);
    	lcd.print("Co");
	  Serial.println("Coboara");
	  // coborare
	  tempDOWN();
  }

  //analogWrite(PIN_OUTPUT, 0);
  delay(250);
  }
}

void loadConfig() { // load from EEPROM
	// To make sure there are settings, and they are YOURS!
	// If nothing is found it will use the default settings.
	if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
	EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
	EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
	for (unsigned int t = 0; t < sizeof(param); t++)
	*((char*)&param + t) = EEPROM.read(CONFIG_START + t);
}


void saveConfig() { // save to EEPROM
	for (unsigned int t = 0; t < sizeof(param); t++)
	EEPROM.write(CONFIG_START + t, *((char*)&param + t));
}

// load from eeprom when button is pressed


