/******************************************************************************* 

This is the code for getting position data from Xsens Mti-g.

It has config mode to configure the sensor to position data.

Then you could record the position in latitude, longitude, altitude onto a 

Datafile\send over the serial port. After recieving the data it also does a checksum.



Author: Chida Matada

Dated: 140630

For compilation



sudo gcc -o test imu_lcm.c

sudo ./test



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

int fd, fdSG; /* File descriptor for the port */

int buf_siz=23;

char buffer[23]; 

char tempHolder[6],accxHolder[7],accyHolder[7],acczHolder[7],gyrxHolder[7],gyryHolder[7],gyrzHolder[7],rollHolder[11],pitchHolder[11],yawHolder[11],latHolder[11],longHolder[11],altHolder[5],velxHolder[7],velyHolder[7],velzHolder[7];

char sendBuffer[]="AccX:00.0000 AccY:00.0000 AccZ:00.0000 GyrX:00.0000 GyrY:00.0000 GyrZ:00.0000 Roll:0000.000000 Pitch:0000.000000 Yaw:0000.000000 Lat:0000.000000 Long:0000.000000 Alt:00.00 VelX:00.0000 VelY:00.0000 VelZ:00.0000 Temp:000.00 \r\n";  

char readBuffer[10];

int statwrite;



// struct timeval now,start;



char msgconfig[5],msgconfigack[5],msgout[7],msgoutack[5],msgoutsettings[9],msgoutsettingsack[5],msgmeas[5],msgmeasack[5];

//FILE *fp; 





// The function for getting the position data from the IMU. 

// Make sure to run the config once before you run this.

int data_imu_pos(){

    char data_pos[69],crc,data[80],data_pos1[69];

    int length=69,t=0,i,k=0,j=0,l,i1,j1,test,d0,d1,d2,d3,dp0,dp1,dp2,dp3;

//    time_t rawtime;

//    struct tm *timeinfo;

    unsigned int value,crc_val;

    union {

	int i;float f;

    } hextofloat;

    float temp,accx,accy,accz,gyrx,gyry,gyrz,roll,pitch,yaw,latitude,longitude,altitude,velx,vely,velz;

// The standard position message structure.

    data_pos[0]=0xFA;data_pos[1]=0xFF;data_pos[2]=0x32;data_pos[3]=0x40;



//First compare with the known standard message which the IMU sends.

// if you want longer recording make sure to increase the value in thw while loop.



    while(k<200000){ 

repeat1:	read(fd,buffer,sizeof(buffer)); 

/*		gettimeofday(&now,NULL);

		time(&rawtime);

		timeinfo=localtime(&rawtime); */

		printf("a \n");

		for(i = 0; i<23; ++i){ 

			printf("b \n");

			if(buffer[i]!='\n'){

				printf("c \n");

//				fprintf(fp,"%X ",buffer[i] & 0xff); 				

				data[j]=buffer[i] & 0xff;

				j++;

// make chunks of 50 bytes of data.

				if(j>79) {

					printf("d \n");

					j=0;

					for (i1=0;i1<80-length;i1++){

						printf("e \n");

						if (data[i1]==data_pos[0] && data[i1+1]==data_pos[1] && data[i1+2]==data_pos[2] && data[i1+3]==data_pos[3]) {

						printf("f \n");

						// If the message structure matches then copy the message contents.

										for(j1=4; j1<length; j1++)

											printf("g \n");

											data_pos[j1]=data[j1+i1] & 0xFF;

						}

					}

				}

			}

		}		

		for(i=0; i<23; ++i)

			buffer[i]='\n'; 

		printf("h \n");

		k++;



// Copy the contents into another array.

		printf("i \n");		

		for(i=0; i<length; ++i)

		data_pos1[i]=data_pos[i];

// Do the Checksum

		printf("j \n");

		crc=(data_pos[1] & 0xFF);

		for(i=2; i<length; ++i)

			crc+=(data_pos[i] & 0xFF);

//		crc=1;

		//crc_val = (crc & 0xFF);



// TO make avoid residual values add zeroes into it.

		printf("k \n");

		for(i=4; i<length; ++i)

		data_pos[i]=0;



// If checksum is found valid then convert the hex values to IEEE float values.

// Get the Lat, Lon, Alt and write them into a Binary Datafile for now.

		if (crc==0) {

			printf("l \n");

			//Bytes 4:7 are temperature.

			value = (data_pos1[7] & 0xFF) | ((data_pos1[6] & 0xFF)<<8) | ((data_pos1[5] & 0xFF)<<16) | ((data_pos1[4] & 0xFF)<<24);

			hextofloat.i = value;

			temp = hextofloat.f;

			//Bytes 8:19 are accelerations. 4 for each of the X,Y and Z directions in m/s^2, IEEE float format

			value = (data_pos1[11] & 0xFF) | ((data_pos1[10] & 0xFF)<<8) | ((data_pos1[9] & 0xFF)<<16) | ((data_pos1[8] & 0xFF)<<24);

			hextofloat.i = value;

			accx = hextofloat.f;

			value = (data_pos1[15] & 0xFF) | ((data_pos1[14] & 0xFF)<<8) | ((data_pos1[13] & 0xFF)<<16) | ((data_pos1[12] & 0xFF)<<24);

			hextofloat.i = value;

			accy = hextofloat.f;

			value = (data_pos1[19] & 0xFF) | ((data_pos1[18] & 0xFF)<<8) | ((data_pos1[17] & 0xFF)<<16) | ((data_pos1[16] & 0xFF)<<24);

			hextofloat.i = value;

			accz = hextofloat.f;

			//Bytes 20:31 are rate of turns. 4 for each of the X,Y and Z axes in rad/s, IEEE float format

			value = (data_pos1[23] & 0xFF) | ((data_pos1[22] & 0xFF)<<8) | ((data_pos1[21] & 0xFF)<<16) | ((data_pos1[20] & 0xFF)<<24);

			hextofloat.i = value;

			gyrx = hextofloat.f;

			value = (data_pos1[27] & 0xFF) | ((data_pos1[26] & 0xFF)<<8) | ((data_pos1[25] & 0xFF)<<16) | ((data_pos1[24] & 0xFF)<<24);

			hextofloat.i = value;

			gyry = hextofloat.f;

			value = (data_pos1[31] & 0xFF) | ((data_pos1[30] & 0xFF)<<8) | ((data_pos1[29] & 0xFF)<<16) | ((data_pos1[28] & 0xFF)<<24);

			hextofloat.i = value;

			gyrz = hextofloat.f;

			//Bytes 32:43 are orientation angles. 4 for each of the roll, pitch, and yaw in degrees, IEEE float format

			value = (data_pos1[35] & 0xff) | ((data_pos1[34] & 0xff)<<8) | ((data_pos1[33] & 0xff)<<16) | ((data_pos1[32] & 0xff)<<24);

			hextofloat.i = value;

			roll=hextofloat.f;

			value = (data_pos1[39] & 0xff) | ((data_pos1[38] & 0xff)<<8) | ((data_pos1[37] & 0xff)<<16) | ((data_pos1[36] & 0xff)<<24);

			hextofloat.i = value;

			pitch=hextofloat.f;

			value = (data_pos1[43] & 0xff) | ((data_pos1[42] & 0xff)<<8) | ((data_pos1[41] & 0xff)<<16) | ((data_pos1[40] & 0xff)<<24);

			hextofloat.i = value;

			yaw=hextofloat.f;

			//Bytes 44:55 are GPS values. 4 for each of the latitude,longitude and altitude. Latitude and longitude are in degrees, altitude is in meters with respect to the LLAWGS84 model of the earth's surface, IEEE float format

			value = (data_pos1[47] & 0xff) | ((data_pos1[46] & 0xff)<<8) | ((data_pos1[45] & 0xff)<<16) | ((data_pos1[44] & 0xff)<<24);

			hextofloat.i = value;

			latitude=hextofloat.f;

			value = (data_pos1[51] & 0xff) | ((data_pos1[50] & 0xff)<<8) | ((data_pos1[49] & 0xff)<<16) | ((data_pos1[48] & 0xff)<<24);

			hextofloat.i = value;

			longitude=hextofloat.f;

			value = (data_pos1[55] & 0xff) | ((data_pos1[54] & 0xff)<<8) | ((data_pos1[53] & 0xff)<<16) | ((data_pos1[52] & 0xff)<<24);

			hextofloat.i = value;

			altitude=hextofloat.f;

			//Bytes 56:67 are velocities. 4 for each of the X,Y and Z directions in m/s, IEEE float format

			value = (data_pos1[59] & 0xFF) | ((data_pos1[58] & 0xFF)<<8) | ((data_pos1[57] & 0xFF)<<16) | ((data_pos1[56] & 0xFF)<<24);

			hextofloat.i = value;

			velx = hextofloat.f;

			value = (data_pos1[63] & 0xFF) | ((data_pos1[62] & 0xFF)<<8) | ((data_pos1[61] & 0xFF)<<16) | ((data_pos1[60] & 0xFF)<<24);

			hextofloat.i = value;

			vely = hextofloat.f;

			value = (data_pos1[67] & 0xFF) | ((data_pos1[66] & 0xFF)<<8) | ((data_pos1[65] & 0xFF)<<16) | ((data_pos1[64] & 0xFF)<<24);

			hextofloat.i = value;

			velz = hextofloat.f;

			printf("m \n");

			sprintf(accxHolder,"%+07.4f",accx);

			sprintf(accyHolder,"%+07.4f",accy);

			sprintf(acczHolder,"%+07.4f",accz);

			sprintf(gyrxHolder,"%+07.4f",gyrx);

			sprintf(gyryHolder,"%+07.4f",gyry);

			sprintf(gyrzHolder,"%+07.4f",gyrz);

			sprintf(rollHolder,"%+011.6f",roll);

			sprintf(pitchHolder,"%+011.6f",pitch);

			sprintf(yawHolder,"%+011.6f",yaw);

			sprintf(latHolder,"%+011.6f",latitude);

			sprintf(longHolder,"%+011.6f",longitude);

			sprintf(altHolder,"%+05.2f",altitude);

			sprintf(velxHolder,"%+07.4f",velx);

			sprintf(velyHolder,"%+07.4f",vely);

			sprintf(velzHolder,"%+07.4f",velz);

			sprintf(tempHolder,"%+06.2f",temp);

			printf("n \n");

			for (i=0;i<sizeof(accxHolder);i++) {

				sendBuffer[5+i] = accxHolder[i];

			}

			for (i=0;i<sizeof(accyHolder);i++) {

				sendBuffer[18+i] = accyHolder[i];

			}

			for (i=0;i<sizeof(acczHolder);i++) {

				sendBuffer[31+i] = acczHolder[i];

			}

			for (i=0;i<sizeof(gyrxHolder);i++) {

				sendBuffer[44+i] = gyrxHolder[i];

			}

			for (i=0;i<sizeof(gyryHolder);i++) {

				sendBuffer[57+i] = gyryHolder[i];

			}

			for (i=0;i<sizeof(gyrzHolder);i++) {

				sendBuffer[70+i] = gyrzHolder[i];

			}

			for (i=0;i<sizeof(rollHolder);i++) {

				sendBuffer[83+i] = rollHolder[i];

			}

			for (i=0;i<sizeof(pitchHolder);i++) {

				sendBuffer[101+i] = pitchHolder[i];

			}

			for (i=0;i<sizeof(yawHolder);i++) {

				sendBuffer[117+i] = yawHolder[i];

			}

			for (i=0;i<sizeof(latHolder);i++) {

				sendBuffer[133+i] = latHolder[i];

			}

			for (i=0;i<sizeof(longHolder);i++) {

				sendBuffer[150+i] = longHolder[i];

			}

			for (i=0;i<sizeof(altHolder);i++) {

				sendBuffer[166+i] = altHolder[i];

			}

			for (i=0;i<sizeof(velxHolder);i++) {

				sendBuffer[177+i] = velxHolder[i];

			}

			for (i=0;i<sizeof(velyHolder);i++) {

				sendBuffer[190+i] = velyHolder[i];

			}

			for (i=0;i<sizeof(velzHolder);i++) {

				sendBuffer[203+i] = velzHolder[i];

			}

			for (i=0;i<sizeof(tempHolder);i++) {

				sendBuffer[216+i] = tempHolder[i];

			}

			printf("o \n");

			goto end1;

		}

// Remove residual values.

		printf("p \n");

		for(i=0; i<length; ++i)

			data_pos1[i]=0; 	

    }// end of while

printf("q \n");

end1:    return 0;

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

            if (buffer[i]==msgconfigack[0] && buffer[i+1]==msgconfigack[1] && buffer[i+2]==msgconfigack[2] && buffer[i+3]==msgconfigack[3]) {

//		printf("Config Successful\n");

		br=1;

	    }

        }

    if (br==1 || t>30) break;

    }

    if (t>30 && br!=1) goto redo;

    sleep(1);



// Change the IMU output settings.

// Also check for acknowledge from the IMU.

redo0:

    br=0;t=0;

    statwrite=write(fd,&msgoutsettings,9);

    if (statwrite!=9) {

	perror("Outpu.t settings unsuccessful");

	statwrite=write(fd,&msgoutsettings,9);

        if (statwrite!=9) perror("Output settings unsuccessful");

	}

    while(1){

	t++;

	read(fd,buffer,sizeof(buffer)); 

	for (i=0;i<sizeof(buffer)-3;i++) {

            if (buffer[i]==msgoutsettingsack[0] && buffer[i+1]==msgoutsettingsack[1] && buffer[i+2]==msgoutsettingsack[2] && buffer[i+3]==msgoutsettingsack[3]) {

//						printf("Position output mode successful\n");

						br=1;

	    }

        }

    if (br==1 || t>30) break;

    }

    if (t>30 && br!=1) goto redo0;
    sleep(1);

// Change the IMU output mode to Position.

// Also check for acknowledge from the IMU to make sure it is in position output mode.

redo1:

    br=0;t=0;

    statwrite=write(fd,&msgout,7);

    if (statwrite!=7) {

	perror("Output mode unsuccessful");

	statwrite=write(fd,&msgout,7);

        if (statwrite!=7) perror("Output mode unsuccessful");

	}

    while(1){

	t++;

	read(fd,buffer,sizeof(buffer)); 

	for (i=0;i<sizeof(buffer)-3;i++) {

            if (buffer[i]==msgoutack[0] && buffer[i+1]==msgoutack[1] && buffer[i+2]==msgoutack[2] && buffer[i+3]==msgoutack[3]) {

//						printf("Position output mode successful\n");

						br=1;

	    }

        }

    if (br==1 || t>30) break;

    }

    if (t>30 && br!=1) goto redo1;

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





// For the linux configuration of the serial to USB port

int serialimu_connect(){

    int i,j=0;

    // Open file to write to 

/*    fp=fopen("DataFile","w"); 

    if(fp==NULL){

    	perror("First Attempt Error: Data File Not Created/Opened"); 

    	fp=fopen("DataFile","w"); 

    	if(fp==NULL){

    		perror("Second Attempt Error: Data File Not Created/Opened"); 

    		return -1; // Unsuccessful - Can't Open File to write to 

    	}

    } */

     /*Setup USB serial connection*/

//    else{

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

//    }

return 0; 

}



int SGConnect(){ 

	fdSG = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fdSG == -1){ 

		printf("Can't Connect to Speedgoat USB1. Exiting Program...\n"); 

		return -1; 

	} 

	else{ 

		struct termios options2; 

		if(tcgetattr(fdSG,&options2)<0){ 

			printf("Can't get SG Serial Attributes. Exiting Program...\n"); 

			return -1; 

		}

		else{ 

			cfsetospeed(&options2,B115200); 

			cfsetispeed(&options2,B115200); 

			options2.c_cflag &= ~CSTOPB; 

			options2.c_cflag &= ~CSIZE; 

			options2.c_cflag |= CS8; 

			options2.c_cflag |= (CREAD|CLOCAL);	

			if(tcsetattr(fdSG,TCSANOW,&options2)<0){ 

				printf("Can't set SG Serial Attributes. Exiting Program...\n");

				return -1;  

			}

			else{

				fcntl(fdSG,F_SETFL,0); 

				return 0; 

			}

		}

	}

		

}



int main(){ 

    int i,count_success=0,i2,repeatwp=0,stopbit=0; 

         char placeHolder[1];	

	//Send to config mode

    msgconfig[0]=250;msgconfig[1]=255;msgconfig[2]=48;msgconfig[3]=00;msgconfig[4]=209;

	//Config mode acknowledge

    msgconfigack[0]=0xFA;msgconfigack[1]=0xFF;msgconfigack[2]=0x31;msgconfigack[3]=00;msgconfigack[4]=0xD0;

	

	//Send to output temp

    msgout[0]=0xFA;msgout[1]=0xFF;msgout[2]=0xD0;msgout[3]=0x02;msgout[4]=00;msgout[5]=0x37;msgout[6]=0xF8;

	//Get acknowledge for sending to position and orientation mode

    msgoutack[0]=0xFA;msgoutack[1]=0xFF;msgoutack[2]=0xD1;msgoutack[3]=00;msgoutack[4]=0x30;

	//Adjust settings

	msgoutsettings[0]=0xFA;msgoutsettings[1]=0xFF;msgoutsettings[2]=0xD2;msgoutsettings[3]=0x04;msgoutsettings[4]=0x80;msgoutsettings[5]=0x00;msgoutsettings[6]=0x0C;msgoutsettings[7]=0x44;msgoutsettings[8]=0x5B;

	//Get acknowledge for settings

	msgoutsettingsack[0]=0xFA;msgoutsettingsack[1]=0xFF;msgoutsettingsack[2]=0xD3;msgoutsettingsack[3]=0x00;msgoutsettingsack[4]=0x2E;

	//Send to measurement mode

    msgmeas[0]=250;msgmeas[1]=255;msgmeas[2]=16;msgmeas[3]=00;msgmeas[4]=241;

	//Get acknowledge for sending to measurement mode.

    msgmeasack[0]=250;msgmeasack[1]=255;msgmeasack[2]=0x11;msgmeasack[3]=00;msgmeasack[4]=0xF0;

	
	gettimeofday(&start,NULL);

	

	//printf("Enter the number of times:");

	//scanf("%d",&repeatwp);

	if(serialimu_connect()!=0)	printf("Error_in_serialimu_connect()\n"); 

//		else	printf("Success_in_serialimu_connect()\n");



// For IMU reset 

//	if(reset_imu()!=0)	printf("Error_in_reset_imu()\n"); 

//		else	printf("Success_in_reset_imu()\n"); 

//	sleep(1);



// For Config setting for position

	if(config_imu_pos()!=0)	printf("Error_in_config_imu_pos()\n"); 

		else	printf("Success_in_config_imu_pos\n");



//	fprintf(fp," time,latitude,longitude,altitude,roll,pitch,yaw\n");

	if(SGConnect()==0)

		printf("Connected To Speedgoat\n"); 

	else{ 

		printf("Failed to Connect to SG \n"); 

		return -1; 

	}



while(1){



// For data capture of position data

	printf("Capturing GPS Data\n"); 	

		if(data_imu_pos()!=0)	printf("Error_in_data_imu()\n"); 

		else{ 

/*			read(fdSG,&readBuffer,sizeof(readBuffer)); 

			printf("Recv Buffer: %s\n",readBuffer); 

			if(readBuffer[0]=='S' && readBuffer[1]=='G' && readBuffer[2]=='B'){ 

				placeHolder[0]=readBuffer[4];

				stopbit = atoi(placeHolder); 

				printf("STOPBIT: %d",stopbit); 

				if(stopbit==1)

					goto STOPHERE; 	

			} */

			printf("Sending: %s\n",sendBuffer); 

			write(fdSG,sendBuffer,sizeof(sendBuffer)); 

			printf("Message Sent.\n");

			

			tcflush(fdSG,TCIOFLUSH);  

		}



	//if (count_success==i) printf("Data collection successful\n");

	//count_success=0;

	

}



STOPHERE: 

	close(fdSG); 

//	fclose(fp);

	close(fd);

	return 0; 

}

