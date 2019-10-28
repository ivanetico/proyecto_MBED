#include "mbed.h"
#include "MBed_Adafruit_GPS.h"

Serial gps(PA_9,PA_10); //serial object for use w/ GPS
Adafruit_GPS myGPS(&gps);
Thread thread_serial(osPriorityNormal, 1024);

void read_serial(void){
	

   myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                       //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
   
   myGPS.sendCommand((char*)PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
   myGPS.sendCommand((char*)PMTK_SET_NMEA_UPDATE_1HZ);
   myGPS.sendCommand((char*)PGCMD_ANTENNA);
      
	 wait(1);
      
   while(true){
       //check if we recieved a new message from GPS, if so, attempt to parse it,
       if ( myGPS.newNMEAreceived() ) {
           if ( !myGPS.parse(myGPS.lastNMEA()) ) {
               continue;   
           }    
       }
       //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing

   }
}