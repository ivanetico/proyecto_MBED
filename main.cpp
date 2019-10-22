#include "mbed.h"
#include "MMA8451Q.h"

#define MMA8451_I2C_ADDRESS (0x1d<<1)

// Example program connecting the TCS34725 Color Sensor to the B-L072Z-LRWAN1 using I2C

I2C i2c(I2C_SDA, I2C_SCL); //pins for I2C communication (SDA, SCL)
Serial pc(USBTX, USBRX, 9600); //9600 baudios - used to print some values
DigitalOut ledColour(PA_10); // TCS34725 led
DigitalOut ledR(PA_14); //RGB led - red light
DigitalOut ledG(PH_1);  //RGB led - green light 
DigitalOut ledB(PA_4);  //RGB led - blue light
Thread thread_i2c;

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
  data = result;
		
}
char getMax(int r, int g, int b) {
  char result;
  int max;
  if (r < g){
    max = g; 
    result = 'g';  
  } else {
    max= r;
    result = 'r';
  }
  if (max < b){
    result = 'b';
  }
  return result;
}

void read_i2c(void){
	while(true){
		// Read data from color sensor (Clear/Red/Green/Blue)
		
		char colour_data[8];
		float accel_data[3];
		
		
		char clear_reg[1] = {0xB4}; // {?1011 0100?} -> 0x14 and we set 1st and 3rd bit to 1 for auto-increment
		//Asking for first register value
		i2c.write(colour_sensor_addr,clear_reg, 1, true);
		i2c.read(colour_sensor_addr,colour_data, 8, false);
					
		//We store the values read in clear, red, green and blue data
		int clear_value = ((int)colour_data[1] << 8) | colour_data[0];
		int red_value = ((int)colour_data[3] << 8) | colour_data[2];
		int green_value = ((int)colour_data[5] << 8) | colour_data[4];
		int blue_value = ((int)colour_data[7] << 8) | colour_data[6];
					
		// print sensor readings
		pc.printf("Clear (%d), Red (%d), Green (%d), Blue (%d)\r\n", clear_value, red_value, green_value, blue_value);
					
		//Obtains which one is the greatest - red, green or blue
		char max = getMax(red_value, green_value, blue_value);
			
		//Switchs the color of the greatest value. First, we switch off all of them
		ledR.write(1);
		ledG.write(1);
		ledB.write(1);
		if (max == 'r'){
			 ledR.write(0);
			 pc.printf("R\r\n");
		} else if(max == 'g'){
			pc.printf("G\r\n");
			ledG.write(0);
		} else{
			pc.printf("B\r\n");
			ledB.write(0);
		}
		
		get_accel_values(accel_data);
		pc.printf("X = (%f), Y = (%f), Z = (%f)\n\r", accel_data[0], accel_data[1], accel_data[2]);

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
void initialize_acc_sensor(){
	
	NULL;
	
	
	
	
	
}

void initialize_temp_hum_sensor(){
	
	
	
	
	
	
}

int main() {

    green = 1; // LED of B-L072Z-LRWAN1 board on
    
		initialize_colour_sensor();
    initialize_acc_sensor();
		initialize_temp_hum_sensor();


		thread_i2c.start(read_i2c);
    while (true) {
			wait(1);
  }
}
