/******************************************************************************* 
This is the code for getting position data from Xsens Mti-g.
It has config mode to configure the sensor to position data.
Then you could record the position in latitude, longitude, altitude onto a 
Datafile. After recieving the data it also does a checksum.

Code by Chidananda Matada Shivananda.
*******************************************************************************/

// Standard #includes.
#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <time.h>

// File descriptors and other stuff used throughout the program.
int fd; /* File descriptor for the port */
int buf_siz=23;
char buffer[23]; 
int statwrite;
FILE *fp; 


// The function for getting the position data from the IMU. 
// Make sure to run the config once before you run this.
int data_imu_pos(){
    char data_pos[19],crc,data[50],data_pos1[19];
    int t=0,i,k=0,j=0,l,i1,j1;
    unsigned int value;
    union {
	int i;float f;
    } hextofloat;
    float latitude,longitude,altitude;
// The standard position message structure.
    data_pos[0]=0xFA;data_pos[1]=0xFF;data_pos[2]=0x32;data_pos[3]=0x0E;

//First compare with the known standard message which the IMU sends.
// if you want longer recording make sure to increase the value in thw while loop.
    while(k<200000){ 
		read(fd,buffer,sizeof(buffer)); 
		for(i = 0; i<23; ++i){ 
			if(buffer[i]!='\n'){
				fprintf(fp,"%X ",buffer[i] & 0xff); 				
				data[j]=buffer[i] & 0xff;
				j++;
// make chunks of 50 bytes of data.
				if(j>49) {
					j=0;
					for (i1=0;i1<50-18;i1++){
						if (data[i1]==data_pos[0] & data[i1+1]==data_pos[1] & data[i1+2]==data_pos[2] & data[i1+3]==data_pos[3]) {
						// If the message structure matches then copy the message contents.
										for(j1=4; j1<19; j1++)
											data_pos[j1]=data[j1+i1] & 0xFF;
						}
					}
				}
			}
		}		
		for(i=0; i<23; ++i)
			buffer[i]='\n'; 
		k++;

// Copy the contents into another array.		
		for(i=0; i<19; ++i)
		data_pos1[i]=data_pos[i];

// Do the Checksum
		crc=data_pos[1]& 0xFF;
		for(i=2; i<19; ++i)
			crc+=data_pos[i];
//		crc=1;

// TO make avoid residual values add zeroes into it.
		for(i=4; i<19; ++i)
		data_pos[i]=0;

// If checksum is found valid then convert the hex values to IEEE float values.
// Get the Lat, Lon, Alt and write them into a Binary Datafile for now.
		if (crc==0) {
			value = (data_pos1[7] & 0xff) | ((data_pos1[6] & 0xff)<<8) | ((data_pos1[5] & 0xff)<<16) | ((data_pos1[4] & 0xff)<<24);
			hextofloat.i = value;
			latitude=hextofloat.f;
			printf("lat= %f ",latitude);
			value = (data_pos1[11] & 0xff) | ((data_pos1[10] & 0xff)<<8) | ((data_pos1[9] & 0xff)<<16) | ((data_pos1[8] & 0xff)<<24);
			hextofloat.i = value;
			longitude=hextofloat.f;
			printf("long= %f ",longitude);
			value = (data_pos1[15] & 0xff) | ((data_pos1[14] & 0xff)<<8) | ((data_pos1[13] & 0xff)<<16) | ((data_pos1[12] & 0xff)<<24);
			hextofloat.i = value;
			altitude=hextofloat.f;
			printf("alti= %f \n",altitude);
			
			fprintf(fp,"\n Latitude= %f Longitude= %f Altitude= %f \n",latitude,longitude,altitude); 
// Remove residual values.
			for(i=0; i<19; ++i)
				data_pos1[i]=0; 	
		}
    }// end of while
    return 0;
}

// This is for reset incase the IMU goes haywire.
int reset_imu(){
    char msgreset[5];
    msgreset[0]=250;msgreset[1]=255;msgreset[2]=64;msgreset[3]=00;msgreset[4]=193;
    statwrite=write(fd,&msgreset,5);
    if (statwrite!=5) {
	perror("Reset unsuccessful");
	statwrite=write(fd,&msgreset,5);
        if (statwrite!=5) perror("Reset unsuccessful");
    }
    return 0;
}

// Function for configure output to position data.
int config_imu_pos(){
    int br=0,t=0,i=0;
    char msgconfig[5],msgconfigack[5],msgout[7],msgoutack[5],msgmeas[5],msgmeasack[5];
    msgconfig[0]=250;msgconfig[1]=255;msgconfig[2]=48;msgconfig[3]=00;msgconfig[4]=209;
    msgconfigack[0]=0xFA;msgconfigack[1]=0xFF;msgconfigack[2]=0x31;msgconfigack[3]=00;msgconfigack[4]=0xD0;
    msgout[0]=250;msgout[1]=01;msgout[2]=208;msgout[3]=02;msgout[4]=00;msgout[5]=16;msgout[6]=29;
//    msgout[0]=0xFA;msgout[1]=01;msgout[2]=0xD0;msgout[3]=02;msgout[4]=00;msgout[5]=04;msgout[6]=0x29;// orien
    msgoutack[0]=250;msgoutack[1]=01;msgoutack[2]=0xD1;msgoutack[3]=00;msgoutack[4]=0x2E;
    msgmeas[0]=250;msgmeas[1]=255;msgmeas[2]=16;msgmeas[3]=00;msgmeas[4]=241;
    msgmeasack[0]=250;msgmeasack[1]=255;msgmeasack[2]=0x11;msgmeasack[3]=00;msgmeasack[4]=0xF0;

// Change the IMU to config mode.
// Also check for acknowledge from the IMU to make sure it is in config mode.
redo:
    br=0;t=0;
    statwrite=write(fd,&msgconfig,5);
    if (statwrite!=5) {
	perror("config unsuccessful");
	statwrite=write(fd,&msgconfig,5);
        if (statwrite!=5) perror("config unsuccessful");
	}
    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgconfigack[0] & buffer[i+1]==msgconfigack[1] & buffer[i+2]==msgconfigack[2] & buffer[i+3]==msgconfigack[3]) {
		printf("Config Successful\n");
		br=1;
	    }
        }
    if (br==1 | t>30) break;
    }
    if (t>30 & br!=1) goto redo;
    sleep(1);


// Change the IMU output mode to Position.
// Also check for acknowledge from the IMU to make sure it is in position output mode.
redo1:
    br=0;t=0;
    statwrite=write(fd,&msgout,7);
    if (statwrite!=7) {
	perror("Position output mode unsuccessful");
	statwrite=write(fd,&msgout,7);
        if (statwrite!=7) perror("Position output mode unsuccessful");
	}
    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgoutack[0] & buffer[i+1]==msgoutack[1] & buffer[i+2]==msgoutack[2] & buffer[i+3]==msgoutack[3]) {
						printf("Position output mode successful\n");
						br=1;
	    }
        }
    if (br==1 | t>30) break;
    }
    if (t>30 & br!=1) goto redo1;
    sleep(1);

//This is to change to Message mode
// No check for Acknowledge done as the data has been pretty concistently coming.
redo2:
    br=0;t=0;
    statwrite=write(fd,&msgmeas,5);
    if (statwrite!=5) {
	perror("Measurement mode unsuccessful");
	statwrite=write(fd,&msgmeas,5);
        if (statwrite!=5) perror("Measurement mode unsuccessful");
	}
/*    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgmeasack[0]) {
		if (buffer[i+1]==msgmeasack[1]) {
			if (buffer[i+2]==msgmeasack[2]) {
				if (buffer[i+3]==msgmeasack[3]) {
						printf("Measurement mode successful\n");
						br=1;
				}
			}
		}
	    }
        }
    if (br==1 | t>15) break;
    }
    if (t>15 & br!=1) goto redo2;mode unsuc
    sleep(1);*/
    return 0;
}


// Function for configure output to Orientation data.

int config_imu_orien(){
    int br=0,t=0,i=0;
    char msgconfig[5],msgconfigack[5],msgout[7],msgoutack[5],msgmeas[5],msgmeasack[5];
    msgconfig[0]=250;msgconfig[1]=255;msgconfig[2]=48;msgconfig[3]=00;msgconfig[4]=209;
    msgconfigack[0]=0xFA;msgconfigack[1]=0xFF;msgconfigack[2]=0x31;msgconfigack[3]=00;msgconfigack[4]=0xD0;
    msgout[0]=250;msgout[1]=01;msgout[2]=208;msgout[3]=02;msgout[4]=00;msgout[5]=16;msgout[6]=29; // pos
//    msgout[0]=0xFA;msgout[1]=01;msgout[2]=0xD0;msgout[3]=02;msgout[4]=00;msgout[5]=04;msgout[6]=0x29;// orien
    msgoutack[0]=250;msgoutack[1]=01;msgoutack[2]=0xD0;msgoutack[3]=00;msgoutack[4]=0x2E;
    msgmeas[0]=250;msgmeas[1]=255;msgmeas[2]=16;msgmeas[3]=00;msgmeas[4]=241;
    msgmeasack[0]=250;msgmeasack[1]=255;msgmeasack[2]=0x11;msgmeasack[3]=00;msgmeasack[4]=0xF0;

// Change the IMU to config mode.
// Also check for acknowledge from the IMU to make sure it is in config mode.
redo01:
    br=0;t=0;
    statwrite=write(fd,&msgconfig,5);
    if (statwrite!=5) {
	perror("config unsuccessful");
	statwrite=write(fd,&msgconfig,5);
        if (statwrite!=5) perror("config unsuccessful");
	}
    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgconfigack[0] & buffer[i+1]==msgconfigack[1] & buffer[i+2]==msgconfigack[2] & buffer[i+3]==msgconfigack[3]) {
		printf("Config Successful\n");
		br=1;
	    }
        }
    if (br==1 | t>30) break;
    }
    if (t>30 & br!=1) goto redo01;
    sleep(2);


// Change the IMU output mode to Position.
// Also check for acknowledge from the IMU to make sure it is in position output mode.
redo11:
    br=0;t=0;
    statwrite=write(fd,&msgout,7);
    if (statwrite!=7) {
	perror("Position output mode unsuccessful");
	statwrite=write(fd,&msgout,7);
        if (statwrite!=7) perror("Position output mode unsuccessful");
	}
    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgoutack[0] & buffer[i+1]==msgoutack[1] & buffer[i+2]==msgoutack[2] & buffer[i+3]==msgoutack[3]) {
						printf("Position output mode successful\n");
						br=1;
	    }
        }
    if (br==1 | t>30) break;
    }
    if (t>30 & br!=1) goto redo11;
    sleep(1);

//This is to change to Message mode
// No check for Acknowledge done as the data has been pretty concistently coming.
redo21:
    br=0;t=0;
    statwrite=write(fd,&msgmeas,5);
    if (statwrite!=5) {
	perror("Measurement mode unsuccessful");
	statwrite=write(fd,&msgmeas,5);
        if (statwrite!=5) perror("Measurement mode unsuccessful");
	}
/*    while(1){
	t++;
	read(fd,buffer,sizeof(buffer)); 
	for (i=0;i<sizeof(buffer)-3;i++) {
            if (buffer[i]==msgmeasack[0]) {
		if (buffer[i+1]==msgmeasack[1]) {
			if (buffer[i+2]==msgmeasack[2]) {
				if (buffer[i+3]==msgmeasack[3]) {
						printf("Measurement mode successful\n");
						br=1;
				}
			}
		}
	    }
        }
    if (br==1 | t>15) break;
    }
    if (t>15 & br!=1) goto redo2;mode unsuc
    sleep(1);*/
    return 0;
}

// For the linux configuration of the serial to USB port
int serialimu_connect(){
    int i,j=0;
    // Open file to write to 
    fp=fopen("DataFile","w"); 
    if(fp==NULL){
    	perror("First Attempt Error: Data File Not Created/Opened"); 
    	fp=fopen("DataFile","w"); 
    	if(fp==NULL){
    		perror("Second Attempt Error: Data File Not Created/Opened"); 
    		return -1; // Unsuccessful - Can't Open File to write to 
    	}
    }
     /*Setup USB serial connection*/
    else{
    	// Attempt to connect to serial port 
    	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    	if (fd == -1){
			perror("First Attempt Error: Unable to open port /dev/ttyUSB0 - ");
			fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
			if (fd == -1){
				perror("Second Attempt Error: Unable to open /dev/ttyUSB0 - ");
				return -2; // Unsuccessful - Can't Connected to Serial 
			}
     	}
     	// Successfully connected to serial port 
     	else{
     		struct termios options;
    		if (tcgetattr(fd, &options) < 0) {
        		perror("First Attempt Error: Couldn't get term attributes");
        		if (tcgetattr(fd, &options) < 0) {
        			perror("First Attempt Error: Couldn't get term attributes");
        			return -3; 
        		}
    		}
    		else{
				redoattribute: 
    			// BAUD RATE SET 
    			cfsetispeed(&options, B115200);
    			cfsetospeed(&options, B115200);
  
    			/* 8N2*/
    			//options.c_cflag &= PARENB; //Parity Enable
    			options.c_cflag &= ~CSTOPB; //Send two stop bits, else one
    			options.c_cflag &= ~CSIZE; //Character size
    			options.c_cflag |= CS8; //8 bits
    			options.c_cflag &= ~CRTSCTS;
    			options.c_cflag |= CREAD | CLOCAL;  /* turn on READ & ignore ctrl lines*/
    			options.c_iflag &= ~(IXON | IXOFF | IXANY); /* turn off s/w flow ctrl*/

    			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    			options.c_oflag &= ~OPOST; /* make raw*/
    			options.c_cflag &= ~HUPCL;//NEW ADDITION, DISSABLE HANGUP ON 
    			if(tcsetattr(fd, TCSANOW, &options) < 0 && j==0) {
        			perror("First Attempt Error: Couldn't set term attributes");
        			goto redoattribute; j++; 
    			}
    			else if(tcsetattr(fd, TCSANOW, &options) < 0 && j>0) {
        			perror("Second Attempt Error: Couldn't set term attributes");
        			return -3; 
    			}
    			else{ 
    				// Fill Buffer with New Line Char
    				for(i=0; i<buf_siz; ++i)
						buffer[i]='\n';
    			}
    		}
		}
    }
return 0; 
}

int main(){ 
	int i,count_success=0;
	if(serialimu_connect()!=0)	printf("Error_in_serialimu_connect()\n"); 
		else	printf("Success_in_serialimu_connect()\n");

// For IMU reset 
//	if(reset_imu()!=0)	printf("Error_in_reset_imu()\n"); 
//		else	printf("Success_in_reset_imu()\n"); 
//	sleep(1);


// For Config setting for position
	if(config_imu_pos()!=0)	printf("Error_in_config_imu_pos()\n"); 
		else	printf("Success_in_config_imu_pos\n");


// For data capture of position data
	for (i=0;i<1;i++) {
		if(data_imu_pos()!=0)	printf("Error_in_data_imu()\n"); 
		else	count_success++; 
	}
	if (count_success==i) printf("Data collection successful");

// For COnfig to Orientation data output mode – Euler angles (12 bytes)
//	if(config_imu_orien()!=0)	printf("Error_in_config_imu_orien()\n"); 
//		else	printf("Success_in_config_imu_orien\n");

// For data capture of position data
//	for (i=0;i<1;i++) {
//		if(data_imu_pos()!=0)	printf("Error_in_data_imu()\n"); 
//		else	count_success++; 
//	}
//	if (count_success==i) printf("Data collection successful");


	fclose(fp);
	close(fd);
	return 0; 
}
