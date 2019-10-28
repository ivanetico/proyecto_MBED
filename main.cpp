#include "mbed.h"
#include "MMA8451Q.h"
#include "MBed_Adafruit_GPS.h"
#include "TCS34725_colour_sensor.h"

#define MMA8451_I2C_ADDRESS (0x1c<<1)

// Example program connecting the TCS34725 Color Sensor to the B-L072Z-LRWAN1 using I2C

I2C i2c(I2C_SDA, I2C_SCL); //pins for I2C communication (SDA, SCL)
Serial pc(USBTX, USBRX, 9600); //9600 baudios - used to print some values
DigitalOut ledR(PH_0); //RGB led - red light
DigitalOut ledG(PH_1);  //RGB led - green light 
DigitalOut ledB(PB_13);  //RGB led - blue light
AnalogIn soilmois(PA_0); 
AnalogIn light(PA_4);
InterruptIn pushbutton(USER_BUTTON);

DigitalOut ledColour(PB_7); // TCS34725 led
ColourSensor colourSensor(&i2c, &ledColour);

Serial gps(PA_9,PA_10); //serial object for use w/ GPS
Adafruit_GPS myGPS(&gps);

int mode = 0;
float valueSM=0.0;
float valueLight = 0.0;
unsigned short *CRGB_values;
float temp = 0.0, hum = 0.0;
float accel_data[3];

Thread thread_i2c(osPriorityNormal, 1024);
Thread thread_serial(osPriorityNormal, 512);
Thread thread_analog(osPriorityNormal, 1024);


//Variable for ISR
bool readColour =  false;

DigitalOut green(LED1); //LED of B-L072Z-LRWAN1 board
Ticker t;

//Get max value (r,g,b) function


void setLEDColour(unsigned short *CRGB_value) {

	ledR.write(1);
	ledG.write(1);
	ledB.write(1);
	
  if (CRGB_value[1] > CRGB_value[2] && CRGB_value[1] > CRGB_value[3]){
		ledR.write(0);
  }else if(CRGB_value[2] > CRGB_value[1] && CRGB_value[2] > CRGB_value[3]){
		ledG.write(0);
  }else if (CRGB_value[3] > CRGB_value[1] && CRGB_value[3] > CRGB_value[2]){
		ledB.write(0);
  }
	//Switchs the color of the greatest value. First, we switch off all of them
}
void get_accel_values(float* data) {
  MMA8451Q acc(I2C_SDA, I2C_SCL, MMA8451_I2C_ADDRESS);
    float x;
    float y;
    float z;
        x=acc.getAccX();
        y=acc.getAccY();
        z=acc.getAccZ();
        wait(0.2);
	float result[3] = {x,y,z};
  for (int i=0; i< 3; i++) data[i] = result[i];
		
}

void get_TempHum_values(float* temp, float* hum){
	char *tx;
	char rx[2];
	
	int tempHumAddres = 0x80;
	tx[0]=0xE3;
	
	i2c.write(tempHumAddres, tx, 1, true);
	i2c.read(tempHumAddres, rx, 2, false);
	*temp = (((rx[0]<<8) + rx[1])*175.72/65536.0) - 46.85;
	
	tx[0]=0xE5;
	i2c.write(tempHumAddres, tx, 1, true);
	i2c.read(tempHumAddres, rx, 2, false);
	*hum = (((rx[0]<<8) + rx[1])*125.0/65536.0) - 6.0;
}

void read_serial(void){
	
   char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
   Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
   const int refresh_Time = 2000; //refresh time in ms
   
   myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                       //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
   
   myGPS.sendCommand((char*)PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
   myGPS.sendCommand((char*)PMTK_SET_NMEA_UPDATE_1HZ);
   myGPS.sendCommand((char*)PGCMD_ANTENNA);
   
   pc.printf("Connection established at 9600 baud...\n");
   
	 wait(1);
      
   while(true){
       c = myGPS.read();   //queries the GPS
       
       if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
       
       //check if we recieved a new message from GPS, if so, attempt to parse it,
       if ( myGPS.newNMEAreceived() ) {
           if ( !myGPS.parse(myGPS.lastNMEA()) ) {
               continue;   
           }    
       }
       
       //check if enough time has passed to warrant printing GPS info to screen
       //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing

   }
}
	
void read_analog(void) {
		while (true) {
				wait(2);
				valueSM = soilmois*100;
				valueLight = light*100;
			
    }
}


void read_i2c(void){
	while(true){
		//We store the values read in clear, red, green and blue data
		colourSensor.getCRGB(&CRGB_values);					
		//Sets LED higuer colour
		setLEDColour(CRGB_values);
		get_accel_values(accel_data);
		get_TempHum_values(&temp,&hum);

		wait(1.0);
		}
}


void printAll(){
	
	
			// print sensor readings
		pc.printf("Colour sensor data:\r\nClear (%d)\tRed (%d)\tGreen (%d)\tBlue (%d)\r\n", CRGB_values[0], CRGB_values[1], CRGB_values[2], CRGB_values[3]);
		pc.printf("Accel data:\r\nX = (%f)\tY = (%f)\tZ = (%f)\n\r", accel_data[0], accel_data[1], accel_data[2]);
		
		pc.printf("Temp/hum sensor data:\r\n Temp: %.4f\tHum: %.4f\r\n", temp, hum);
		
		pc.printf("Time: %d:%d:%d.%u\n\r", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
		pc.printf("Date: %d/%d/20%d\n\r", myGPS.day, myGPS.month, myGPS.year);
		pc.printf("Fix: %d\n\r", (int) myGPS.fix);
		pc.printf("Quality: %d\n\r", (int) myGPS.fixquality);
		if (myGPS.fix) {
			 pc.printf("Location: %5.2f%c, %5.2f%c\n\r", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
			 pc.printf("Speed: %5.2f knots\n\r", myGPS.speed);
			 pc.printf("Angle: %5.2f\n\r", myGPS.angle);
			 pc.printf("Altitude: %5.2f\n\r", myGPS.altitude);
			 pc.printf("Satellites: %d\n\r", myGPS.satellites);
		}
		pc.printf("Light: %f\n\r", valueLight);
		pc.printf("Value soil mois: %f\n\r", valueSM);
}
int saveData(){
	NULL;
}
void printMean(){
	NULL;
}
void changeMode(void){
	mode += 1;
	mode %= 3;
}
int main() {

	green = 1; // LED of B-L072Z-LRWAN1 board on
	
	pushbutton.rise(changeMode);
	thread_analog.start(read_analog);
	thread_i2c.start(read_i2c);
	thread_serial.start(read_serial);
	

	while (true) {
		switch (mode){
			case 0:
				wait(2);
				printAll();
				break;
			case 1:
				wait(30);
				printAll();
				if (saveData() >= 120)
					printMean();
				break;
			case 2:
				NULL;
				break;
			default:
				printAll();
				break;
		}
  }
}