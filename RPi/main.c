/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.c
 * Author: Max
 *
 * Created on May 6, 2020, 5:41 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define i2cAddr 0x5fU



/*
 * 
 */
int main(int argc, char** argv)
{
        int fd = 0;
        uint8_t data = 0;
        int16_t T_Out = 0;
        uint8_t temp_msb = 0;
        uint8_t temp_lsb = 0;
        uint8_t temp_T0_DegC_x8 = 0;
        uint8_t temp_T1_DegC_x8 = 0;
        float temp_T0_DegC = 0.0; // Make floating point for interpolation calculation
        float temp_T1_DegC = 0.0;
        float T0_Out = 0;
        float T1_Out = 0;
        int16_t H_Out = 0;
        uint8_t H_msb = 0;
        uint8_t H_lsb = 0;
        uint8_t HRH = 0;
        
        uint8_t reg32 = 0;
        uint8_t reg33 = 0;
        uint8_t reg35 = 0;
        uint8_t reg3C = 0;
        uint8_t reg3D = 0;
        uint8_t reg3E = 0;
        uint8_t reg3F = 0;
        
        uint8_t reg30 = 0;
        uint8_t reg31 = 0;
        uint8_t reg36 = 0;
        uint8_t reg37 = 0;
        uint8_t reg3A = 0;
        uint8_t reg3B = 0;
        int16_t reg3B3A = 0;
        int16_t reg3736 = 0;

        
        float tempF = 0.0;
        float tempFloat = 0.0;
        float humFloat = 0.0;
        
        uint8_t H0_rH_x2 = 0;
        uint8_t H1_rH_x2 = 0;
        float H0_rH = 0;
        float H1_rH = 0;    
        float H0_T0_Out = 0;
        float H1_T0_Out = 0;
        
        int Read1_loop = 0;
        int Read2_loop = 0;
        
        FILE * pFile;
        
        //char sampleTime[];
        time_t xTime;
        struct tm * timeInfo;

        wiringPiSetup () ;
        fd=wiringPiI2CSetup (0x5f) ;  /*Use i2cdetect command to find your respective device address*/
        if(fd==-1)
        {
                printf("Can't setup the I2C device\n");
                return -1;
        }
        else
        {
          //
          // Read Who Am I
          //
          data = wiringPiI2CReadReg8 (fd , 0x0F);
          if(data==-1)
            {
                printf("Error reading ID register\n");
                return -1;
           }
          //
          // Print data
          //
          printf("Who Am I = %hhX\n", data);
          //
          // Set configuration to run 256 temperature samples and 512 humidity samples
          //
          wiringPiI2CWriteReg8 (fd, 0x10, 0b00111111) ;
          //
          // Then set data rate t0 one shot
          //
          wiringPiI2CWriteReg8 (fd, 0x20, 0b10000000) ;
          //
          // Read temperature coefficients
          //
           reg32 = wiringPiI2CReadReg8 (fd, 0x32);
          //
           reg33 = wiringPiI2CReadReg8 (fd, 0x33);
          //
          // Read MS bits from reg 35
          //
           reg35 = wiringPiI2CReadReg8 (fd, 0x35);
           // Combine LSB and MSB
           // Set as floating point for the interpolation calculation
           // Also retains any fractional remainder from the divide by 8
           temp_T0_DegC = (float)(((reg35 & 0x03) << 8) | reg32)/8.0;
           temp_T1_DegC = (float)(((reg35 & 0x0C) << 6) | reg33)/8.0;
          //
          // Read T0_Out and T1_Out
          //
           reg3C = wiringPiI2CReadReg8 (fd, 0x3C);
           reg3D = wiringPiI2CReadReg8 (fd, 0x3D);
           // Combine LSB and MSB and convert to floating point
           T0_Out = (float)(reg3D << 8 | reg3C);
           //
           reg3E = wiringPiI2CReadReg8 (fd, 0x3E);
           reg3F = wiringPiI2CReadReg8 (fd, 0x3F);
           // Combine LSB and MSB and convert to floating point
           T1_Out = (float)(reg3F << 8 | reg3E);
         
           printf("\n\nT0_DegC = %f \n", temp_T0_DegC);
           printf("T1_DegC = %f \n", temp_T1_DegC);
           
           printf("T0_Out = %f \n", T0_Out);
           printf("T1_Out = %f \n", T1_Out);
           
          //
          // Set up Humidity interpolation
          //
          //
          // Read humidity coefficients
          //
          // Read humidity calibration registers
          //
           reg30 = wiringPiI2CReadReg8 (fd, 0x30);
           // Set as floating point for the interpolation calculation
           // Also retains any fractional remainder from the divide by 2
           H0_rH = (float)(reg30)/2.0;
           //
           reg31 = wiringPiI2CReadReg8 (fd, 0x31);
           // Set as floating point for the interpolation calculation
           // Also retains any fractional remainder from the divide by 2
           H1_rH = (float)(reg31)/2.0;
          //
          // Read H0_T0_Out and H1_T0_Out
          //
           reg36 = wiringPiI2CReadReg8 (fd, 0x36);
           reg37 = wiringPiI2CReadReg8 (fd, 0x37);
           // Combine the LSB and MSB
           // Note: This is a signed integer
           reg3736 = ((reg37 << 8) | reg36);
           // Convert to floating point for the interpolation calculation
           H0_T0_Out = (float)reg3736;
           //
           reg3A = wiringPiI2CReadReg8 (fd, 0x3A);
           reg3B = wiringPiI2CReadReg8 (fd, 0x3B);
           // Combine the LSB and MSB
           // Note: This is a signed integer
           reg3B3A = ((reg3B << 8) | reg3A);
           // Convert to floating point for the interpolation calculation
           H1_T0_Out = (float)reg3B3A;
         
           printf("\n\nH0_rH = %f \n", H0_rH);
           printf("H1_rH = %f \n", H1_rH);
           
           printf("H0_T0_Out = %f \n", H0_T0_Out);
           printf("H1_T0_Out = %f \r\n\r\n", H1_T0_Out);

          //
          //*************************************************
          //
          // Read registers 2B and 29 to clear status bits
          // data is not used
          //
           data = wiringPiI2CReadReg8 (fd, 0x2B);
           data = wiringPiI2CReadReg8 (fd, 0x29);
          //
          // Set one shot enable to start a conversion
          //
          wiringPiI2CWriteReg8 (fd, 0x21, 0b00000001) ;        
          //
          // See if data is ready to read
          // Check status for temperature first
          // Note: Temperature iterations known to complete faster than the
          // the humidity for the settings used
          //
          READ1: data = wiringPiI2CReadReg8 (fd , 0x27);
          // Check for error
          if(data==-1)
            {
                printf("Error reading status\n");
                return -1;
           }
          printf("Status = %hhX\n", data);
          
          if (data & 0x01) // Check if data is ready
           {
           // Read temperature LSB
           temp_lsb = wiringPiI2CReadReg8 (fd, 0x2A);
           T_Out = temp_lsb;
           // Read temperature MSB
           temp_msb = wiringPiI2CReadReg8 (fd, 0x2B);
           T_Out = T_Out | (temp_msb << 8);
           //
           printf("\n\nRaw temperature = %hd \n",T_Out);
           printf("Temp MSB = %hhX \n", temp_msb);
           printf("Temp LSB = %hhX \n\r", temp_lsb);

          //
          // Compute Temperature
          //
          // T_Out = ((temp_T1_DegC - temp_T0_DegC) * (T_Out - T0_Out)/(T1_Out - T0_Out))+temp_T0_DegC;
          //
          // Cast integers to floating point for the interpolation calculation
          //
           tempFloat = ((temp_T1_DegC - temp_T0_DegC) * ((float)T_Out - T0_Out)/(T1_Out - T0_Out)) + temp_T0_DegC;
           tempF = (tempFloat * 9.0/5.0) + 32.0;
          //
            printf("\r\nComputed Temperature = %3.1f °C %3.1f °F \r\n\r\n",tempFloat, tempF);

           
          }
          else 
          {
              Read1_loop++;
              delay(1); // Delay 1 msec before reading status again
              goto READ1; // Wait for humidity conversion to complete
          }
          //
          // Read and compute humidity
          //
          //
          // Read the status
          //          
          READ2: data = wiringPiI2CReadReg8 (fd , 0x27);
          //
          printf("Status = %hhX\n", data);
          //
          if (data & 0x02) // Check if humidity data is ready
          {
           // Read humidity LSB
           H_lsb = wiringPiI2CReadReg8 (fd, 0x28);
           // Read humidity MSB
           H_msb = wiringPiI2CReadReg8 (fd, 0x29);
           // Combine LSB and MSB
           // Note: Result is a signed integer
           H_Out = H_lsb | (H_msb << 8);
           //
           printf("\n\nRaw humidity = %hd \n",H_Out);
           printf("Humidity MSB = %hhX \n", H_msb);
           printf("Humidity LSB = %hhX \n\r", H_lsb);
           //
           // Compute humidity
           //
           //HRH = (H1_rH - H0_rH)*(H_Out - H0_T0_Out)/(H1_T0_Out - H0_T0_Out) + H0_rH;
           //
           // Cast integers to floating point for the interpolation calculation
           //
           humFloat = (((H1_rH - H0_rH)*((float)H_Out - H0_T0_Out))/(H1_T0_Out) - H0_T0_Out) +  H0_rH;
          //
          printf("\r\nComputed Humidity = %3.1f%% \r\n",humFloat);
          }
          else
          {
              Read2_loop++;
              delay(1); // Delay 1 msec before reading status again
              goto READ2; // Wait for data to be ready
          }
        printf("\n\nRead1 Loop Count = %d \n",Read1_loop);        
        printf("Read2 Loop Count = %d \n\n",Read2_loop);          
        }
        //
        // Get the current time for a timestamp in the xml file
        //
        time (&xTime);
        timeInfo = localtime (&xTime);
        
        //
        // Write temperature and humidity to xml file
        //
        pFile = fopen("/var/www/html/data.xml","w");
        if(pFile==NULL){
        printf("Error");
        return 0;
        }
    fprintf (pFile,"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");

    fprintf(pFile,"<hts221>\n");
    fprintf(pFile,"<temperature>%3.1f</temperature>\n",tempFloat);
    fprintf(pFile,"<humidity>%3.1f</humidity>\n",humFloat);
    fprintf(pFile,"<time>Updated: %s</time>\n",asctime(timeInfo));
    fprintf(pFile,"</hts221>\n");
    
    fclose(pFile);
        
        return 0;
}               

