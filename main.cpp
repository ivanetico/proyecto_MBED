#include "mbed.h"
#include "MMA8451Q.h"
#include "MBed_Adafruit_GPS.h"
#define MMA8451_I2C_ADDRESS (0x1c<<1)

// Example program connecting the TCS34725 Color Sensor to the B-L072Z-LRWAN1 using I2C

I2C i2c(I2C_SDA, I2C_SCL); //pins for I2C communication (SDA, SCL)
Serial pc(USBTX, USBRX, 9600); //9600 baudios - used to print some values
DigitalOut ledColour(PB_7); // TCS34725 led
DigitalOut ledR(PH_0); //RGB led - red light
DigitalOut ledG(PH_1);  //RGB led - green light 
DigitalOut ledB(PB_13);  //RGB led - blue light
AnalogIn soilmois(PA_0); 
AnalogIn light(PA_4);

Serial gps(PA_9,PA_10); //serial object for use w/ GPS
Adafruit_GPS myGPS(&gps);

float valueSM=0.0;
float valueLight = 0.0;
unsigned short *CRGB_value;
float temp = 0.0, hum = 0.0;
float accel_data[3];

Thread thread_i2c(osPriorityNormal, 1024);
Thread thread_serial(osPriorityNormal, 512);
Thread thread_analog(osPriorityNormal, 1024);

// We set the sensor address. For TCS34725 is 0x29 = ‭‭0010 1001‬ (bin) ->> ‭0101 0010‬ (bin) = 0x52
// We shift 1 bit to the left because in I2C protocol slave address is 7-bit. So we discard the 8th bit
int colour_sensor_addr = 0x29 << 1; 
//Variable for ISR
bool readColour =  false;

DigitalOut green(LED1); //LED of B-L072Z-LRWAN1 board
Ticker t;

//Get max value (r,g,b) function



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
		// Read data from color sensor (Clear/Red/Green/Blue)
		char colour_data[8];
		
		char clear_reg[1] = {0xB4}; // {?1011 0100?} -> 0x14 and we set 1st and 3rd bit to 1 for auto-increment
		//Asking for first register value
		i2c.write(colour_sensor_addr,clear_reg, 1, true);
		i2c.read(colour_sensor_addr,colour_data, 8, false);
					
		//We store the values read in clear, red, green and blue data
		CRGB_value = (unsigned short*)(void*)colour_data;					
		//Sets LED higuer colour
		setLEDColour(CRGB_value);
				
		get_accel_values(accel_data);

		wait(1.0);
		}
}

void initialize_colour_sensor(){
	char id_regval[1] = {0x92}; //‭1001 0010‬ (bin)
  char data[1] = {0}; //‭0000 0000‬
	
	/**********************************************************************
	* int write(int address, const char *data, int length, bool repeated) *
	* int read(int address, char *data, int length, bool repeated)        *
	***********************************************************************/
	//We obtain device ID from ID register (0x12)
    i2c.write(colour_sensor_addr,id_regval,1, true);
    i2c.read(colour_sensor_addr,data,1,false); 
   
   //We check that the ID is the TCS34725 one. If it is, we switch off a LED on the board, wait for 2s, and switch on again
	if (data[0]==0x44) { //‭ 0100 0100‬ -> Value for the part number (0x44 for TCS34725)
        green = 0;
        wait (1);
        green = 1;
    } else {
        green = 0;
    }
    
    // Initialize color sensor
    
	// Timing register address 0x01 (0000 0001). We set 1st bit to 1 -> 1000 0001
    char timing_register[2] = {0x81,0x50}; //0x50 ~ 400ms
    i2c.write(colour_sensor_addr,timing_register,2,false); 
    
	// Control register address 0x0F (0000 1111). We set 1st bit to 1 -> 1000 1111
    char control_register[2] = {0x8F,0}; //{0x8F, 0x00}, {1000 1111, 0000 0000} -> 1x gain
    i2c.write(colour_sensor_addr,control_register,2,false);
    
	// Enable register address 0x00 (0000 0000). We set 1st bit to 1 -> 1000 0000
    char enable_register[2] = {0x80,0x03}; //{0x80, 0x03}, {1000 0000, 0000 0011} -> AEN = PON = 1
    i2c.write(colour_sensor_addr,enable_register,2,false);
    

    
    // Turn on the led in the sensor
    ledColour = 1;

}
void printAll(){
	
	
			// print sensor readings
		pc.printf("Colour sensor data:\r\nClear (%d)\tRed (%d)\tGreen (%d)\tBlue (%d)\r\n", CRGB_value[0], CRGB_value[1], CRGB_value[2], CRGB_value[3]);
		pc.printf("Accel data:\r\nX = (%f)\tY = (%f)\tZ = (%f)\n\r", accel_data[0], accel_data[1], accel_data[2]);
		
		get_TempHum_values(&temp,&hum);
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
int main() {

    green = 1; // LED of B-L072Z-LRWAN1 board on
    
		initialize_colour_sensor();
	
		thread_analog.start(read_analog);
		thread_i2c.start(read_i2c);
		thread_serial.start(read_serial);
		
	
    while (true) {
			wait(2);
			printAll();
  }
}