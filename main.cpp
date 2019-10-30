#include "mbed.h"
#include "MBed_Adafruit_GPS.h"



#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define UTC_DIFF 1

// Example program connecting the TCS34725 Color Sensor to the B-L072Z-LRWAN1 using I2C

Serial pc(USBTX, USBRX, 9600); //9600 baudios - used to print some values

InterruptIn pushbutton(USER_BUTTON);
DigitalOut green1(LED1); //LED of B-L072Z-LRWAN1 board
DigitalOut green2(LED2);

long int numberOfMeasures = 10;
long int time_interval = 2;
int mode = 0;

float valueSM=0.0;
float valueLight = 0.0;
uint16_t CRGB_values[4];
float temp = 0.0, hum = 0.0;
float accel_data[3];
char colour[6] = "     ";
int counter =  0;
extern Adafruit_GPS myGPS;

float soilmoisMin, soilmoisMax;
float lightMin,lightMax;
float tempMin, tempMax;
float humMin, humMax;

float soilmoisSum;
float lightSum;
float tempSum;
float humSum;

float accel_max_values[3];
float accel_min_values[3];

int most_dominant_rgb[3];

extern Thread thread_i2c;
extern Thread thread_serial;
extern Thread thread_analog;
extern Thread thread_alarms;

extern void read_analog(void);
extern void read_i2c(void);
extern void read_serial(void);
extern void trigger_alarm(void);

void printAll(){
    // print sensor readings
  pc.printf("Colour sensor data: Clear (%d); Red (%d); Green (%d); Blue (%d) -- Dominant colour: %s\r\n", CRGB_values[0], CRGB_values[1], CRGB_values[2], CRGB_values[3], colour);
  pc.printf("Accel data: X = %f g; Y = %f g; Z = %f g\n\r", accel_data[0], accel_data[1], accel_data[2]);
  
  pc.printf("Temp/hum sensor data: Temp: %.1f C\tHum: %.1f %%\r\n", temp, hum);
	
  if (myGPS.fix){
    pc.printf("GPS: #Sats: %d; Lat: %5.2f%c; Lon: %5.2f%c; Altitude: %5.2f; ", myGPS.satellites, myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon, myGPS.altitude);
    
		myGPS.hour += UTC_DIFF;
		if(myGPS.hour ==24){
			myGPS.hour = 0;
			myGPS.day += 1;
		}
		
		pc.printf("GPS_time (local_time): %d:%d:%d.%u\r\n",myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
  }else
    pc.printf("GPS_time: %d:%d:%d; GPS not fixed\r\n", myGPS.hour, myGPS.minute, myGPS.seconds);

		pc.printf("Light: %.1f %%\n\r", valueLight);
		pc.printf("Value soil mois: %.1f %%\n\r", valueSM);
    pc.printf("------------------------------------------------------------\r\n");

}

int saveData(){
  counter %= numberOfMeasures;
	if(counter == 0){
    memset(accel_max_values, 0, sizeof(accel_max_values));
    memset(most_dominant_rgb, 0, sizeof(most_dominant_rgb));
    accel_min_values[0] = 1000;
    accel_min_values[1] = 1000;
    accel_min_values[2] = 1000;

    soilmoisMin = 1000;
    soilmoisMax = 0;
    lightMin = 1000;
    lightMax = 0;
    tempMin = 1000;
    tempMax = 0;
    humMin = 1000;
    humMax = 0;
    soilmoisSum = 0;
    lightSum = 0;
    tempSum = 0;
    humSum = 0;
  }
  accel_max_values[0] = MAX(accel_max_values[0], accel_data[0]);
  accel_max_values[1] = MAX(accel_max_values[1], accel_data[1]);
  accel_max_values[2] = MAX(accel_max_values[2], accel_data[2]);

  accel_min_values[0] = MIN(accel_min_values[0], accel_data[0]);
  accel_min_values[1] = MIN(accel_min_values[1], accel_data[1]);
  accel_min_values[2] = MIN(accel_min_values[2], accel_data[2]);
  
  if(!strcmp(colour, "RED  ")) most_dominant_rgb[0] += 1;
  else if(!strcmp(colour, "GREEN")) most_dominant_rgb[1] += 1;
  else if(!strcmp(colour, "BLUE ")) most_dominant_rgb[2] += 1;
  
  soilmoisMax = MAX(soilmoisMax, valueSM);
  soilmoisMin = MIN(soilmoisMin, valueSM);

  lightMax = MAX(lightMax, valueLight);
  lightMin = MIN(lightMin, valueLight);
  
  tempMax = MAX(tempMax, temp);
  tempMin = MIN(tempMin, temp);
  
  humMax = MAX(humMax, hum);
  humMin = MIN(humMin, hum);
  
  soilmoisSum += valueSM;
  lightSum += valueLight;
  tempSum += temp;
  humSum += hum;
    
  counter++;
  return counter;
}
void printMean_Max_Min(){
  pc.printf("************************************************************\r\n");
  pc.printf("                 %d values have been read\r\n", numberOfMeasures);
  pc.printf("\r\n");

  pc.printf("Max values acc: X = %f g; Y = %f g; Z = %f g\r\n", accel_max_values[0], accel_max_values[1], accel_max_values[2]);
  pc.printf("Min values acc: X = %f g; Y = %f g; Z = %f g\r\n", accel_min_values[0], accel_min_values[1], accel_min_values[2]);
  
  if(most_dominant_rgb[0] >= most_dominant_rgb[1] && most_dominant_rgb[0] > most_dominant_rgb[2])
    pc.printf("Most dominant colour in last hour: Red\r\n");
  else if(most_dominant_rgb[1] >= most_dominant_rgb[0] && most_dominant_rgb[1] > most_dominant_rgb[2])
    pc.printf("Most dominant colour in last hour: Green\r\n");
  else if(most_dominant_rgb[2] >= most_dominant_rgb[0] && most_dominant_rgb[2] > most_dominant_rgb[1])
    pc.printf("Most dominant colour in last hour: Blue\r\n");
  
  pc.printf("Soilmois max//min: %.1f %% // %.1f %%\r\n", soilmoisMax, soilmoisMin);
  pc.printf("Temp max//min: %.1f C // %.1f C\r\n", tempMax, tempMin);
  pc.printf("Hum max//min: %.1f %% // %.1f %%\r\n", humMax, humMin);
  pc.printf("Light max//min: %.1f %% // %.1f %%\r\n", lightMax, lightMin);

  pc.printf("Soilmois mean: %.1f %%\r\n", soilmoisSum/numberOfMeasures);
  pc.printf("Temp mean: %.1f C\r\n", tempSum/numberOfMeasures);
  pc.printf("Hum mean: %.1f %%\r\n", humSum/numberOfMeasures);
  pc.printf("Light mean: %.1f %%\r\n", lightSum/numberOfMeasures);
  pc.printf("------------------------------------------------------------\r\n");

}

 int check_num(char *c)
 {
   int i;
   long int num;
   char *read;
   int check=0;
   char *c_error=NULL;
   
   if(c==NULL){
     return -1;
   }
   read=c;
   
   do{
     for(i=0;i<strlen(read);i++){
       //check if is a number in ASCII code
       if (read[i]<48 || read[i]>57){
				 free(read);
		pc.printf("\n\tERROR. Only positives numbers.\n");
		pc.printf("Enter again: ");
		pc.getc();
		scanf("%s",read);
		
		break;
		}else if(i==strlen(read)-1){
			num = strtol(read, &c_error, 10);
			check=1;
		}
	
     }
   }while (check==0);
   
   free(read);
   
   return num;
 }
 void advancedMode(void){
	 pc.printf("\n\n************INFO: ADVANCED MODE INICIALITED************\n");
	 char terminal_1[10];
   pc.printf("\nPlease enter the number of measures:");
   pc.getc();
	 scanf ("%s",terminal_1);
   numberOfMeasures = check_num(terminal_1);
	 free(terminal_1);
	 char terminal_2[10];
	 pc.printf("\n  \t Please enter the time interval:");
   pc.getc();
	 scanf ("%s",terminal_2);
   numberOfMeasures = check_num(terminal_2);
	 free(terminal_2);
 }
 
extern void shutDownLed(void);

void changeMode(void){
	mode += 1;
	mode %= 3;
	/*if(mode == 1)
		thread_alarms.start(trigger_alarm);
	else if(mode == 0)
		if(thread_alarms.get_state()== Thread::Running)
			thread_alarms.join();*/
  //shutDownLed();
}

int main() {

  pushbutton.rise(changeMode);
  thread_analog.start(read_analog);
  thread_i2c.start(read_i2c);
  thread_serial.start(read_serial);


while (true) {
  switch (mode){
    case 0:
			if(thread_alarms.get_state()== Thread::Running)
			thread_alarms.join();
      green1 = 1;
      green2 = 0;
      wait(2);
      printAll();
      break;
    case 1:
			if(thread_alarms.get_state()!= Thread::Running)
			thread_alarms.start(trigger_alarm);
      green1 = 0;
      green2 = 1;
      wait(time_interval);
      printAll();
      if (saveData()==numberOfMeasures)
        printMean_Max_Min();
      break;
		case 2:
			green1 = 1;
			green1 = 1;
			//advancedMode();
			wait(time_interval);
			//printAll();
			
			break;
    default:
      printAll();
      break;
  }
}
}