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

char longHolder[11],latHolder[11],rollHolder[11],pitchHolder[11],yawHolder[11];

char sendBuffer[]="GPS Lat:0000.000000 Long:0000.000000 Roll:0000.000000 Pitch:0000.000000 Yaw:0000.000000 \r\n";  

char readBuffer[10];

int statwrite;



struct timeval now,start;



    char msgconfig[5],msgconfigack[5],msgout[7],msgoutack[5],msgmeas[5],msgmeasack[5],msgout1[7],msgout1e[9],msgoutack1e[5];

FILE *fp; 





// The function for getting the position data from the IMU. 

// Make sure to run the config once before you run this.

int data_imu_pos(){

    char data_pos[31],crc,data[50],data_pos1[31];

    int t=0,i,k=0,j=0,l,i1,j1,test;

    time_t rawtime;

    struct tm *timeinfo;

    unsigned int value;

    union {

	int i;float f;

    } hextofloat;

    float latitude,longitude,altitude,roll,pitch,yaw;

// The standard position message structure.

    data_pos[0]=0xFA;data_pos[1]=0xFF;data_pos[2]=0x32;data_pos[3]=0x1A;



//First compare with the known standard message which the IMU sends.

// if you want longer recording make sure to increase the value in thw while loop.



    while(k<200000){ 

repeat1:	read(fd,buffer,sizeof(buffer)); 

		gettimeofday(&now,NULL);

		time(&rawtime);

		timeinfo=localtime(&rawtime);

		for(i = 0; i<23; ++i){ 

			if(buffer[i]!='\n'){

//				fprintf(fp,"%X ",buffer[i] & 0xff); 				

				data[j]=buffer[i] & 0xff;

				j++;

// make chunks of 50 bytes of data.

				if(j>49) {

					j=0;

					for (i1=0;i1<50-30;i1++){

						if (data[i1]==data_pos[0] && data[i1+1]==data_pos[1] && data[i1+2]==data_pos[2] && data[i1+3]==data_pos[3]) {

						// If the message structure matches then copy the message contents.

										for(j1=4; j1<31; j1++)

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

		for(i=0; i<31; ++i)

		data_pos1[i]=data_pos[i];



// Do the Checksum

		crc=data_pos[1]& 0xFF;

		for(i=2; i<31; ++i)

			crc+=data_pos[i];

//		crc=1;



// TO make avoid residual values add zeroes into it.

		for(i=4; i<31; ++i)

		data_pos[i]=0;



// If checksum is found valid then convert the hex values to IEEE float values.

// Get the Lat, Lon, Alt and write them into a Binary Datafile for now.

		if (crc==0) {

			

			

			value = (data_pos1[7] & 0xff) | ((data_pos1[6] & 0xff)<<8) | ((data_pos1[5] & 0xff)<<16) | ((data_pos1[4] & 0xff)<<24);

			hextofloat.i = value;

			roll=hextofloat.f;

			if ((roll<(-180) && roll>180) ) goto repeat1;

//			printf("%s",asctime(timeinfo));

//			printf("roll= %f ",roll);

			value = (data_pos1[11] & 0xff) | ((data_pos1[10] & 0xff)<<8) | ((data_pos1[9] & 0xff)<<16) | ((data_pos1[8] & 0xff)<<24);

			hextofloat.i = value;

			pitch=hextofloat.f;

			if ((pitch<(-180) && pitch>180)) goto repeat1;

//			printf("pitch= %f ",pitch);

			value = (data_pos1[15] & 0xff) | ((data_pos1[14] & 0xff)<<8) | ((data_pos1[13] & 0xff)<<16) | ((data_pos1[12] & 0xff)<<24);

			hextofloat.i = value;

			yaw=hextofloat.f;

			if ((yaw<(-180) && yaw>180) || (yaw<(1) && yaw>-1 )) goto repeat1;

//			printf("yaw= %f \n",yaw);

						

			

			value = (data_pos1[19] & 0xff) | ((data_pos1[18] & 0xff)<<8) | ((data_pos1[17] & 0xff)<<16) | ((data_pos1[16] & 0xff)<<24);

			hextofloat.i = value;

			latitude=hextofloat.f;

			if ((latitude<(-90) && latitude>90) || (latitude<(1) && latitude>-1) ) goto repeat1 ;

//			printf("%s",asctime(timeinfo));

//			printf("lat= %f ",latitude);

			value = (data_pos1[23] & 0xff) | ((data_pos1[22] & 0xff)<<8) | ((data_pos1[21] & 0xff)<<16) | ((data_pos1[20] & 0xff)<<24);

			hextofloat.i = value;

			longitude=hextofloat.f;

			if ((longitude<(-180) && longitude>180) || (longitude<(1) && longitude>-1)) goto repeat1;

//			printf("long= %f ",longitude);

			value = (data_pos1[27] & 0xff) | ((data_pos1[26] & 0xff)<<8) | ((data_pos1[25] & 0xff)<<16) | ((data_pos1[24] & 0xff)<<24);

			hextofloat.i = value;

			altitude=hextofloat.f;

			if ((altitude<(-200) && altitude>1000) || (altitude<(1) && altitude>-1 )) goto repeat1;

//			printf("alti= %f \n",altitude);		

			test=fprintf(fp,"%f %f %f %f %f %f %f \r",(float)(now.tv_sec-start.tv_sec)+(float)(now.tv_usec-start.tv_usec)/1000000,latitude,longitude,altitude,roll,pitch,yaw); 

//			printf("\n %f \n",(float)(now.tv_sec-start.tv_sec)+(float)(now.tv_usec-start.tv_usec)/1000000);

//			printf("\n Latitude= %f Longitude= %f Altitude= %f roll= %f pitch= %f yaw= %f %s\n",latitude,longitude,altitude,roll,pitch,yaw,asctime(timeinfo)); 

			if (test>60){ 

				sprintf(latHolder,"%+011.6f",latitude); 

				sprintf(longHolder,"%+011.6f",longitude);  

				sprintf(rollHolder,"%+011.6f",roll);

				sprintf(pitchHolder,"%+011.6f",pitch);

				sprintf(yawHolder,"%+011.6f",yaw);

				// Latitude 

				sendBuffer[8]=latHolder[0]; 

				sendBuffer[9]=latHolder[1]; 

				sendBuffer[10]=latHolder[2];

				sendBuffer[11]=latHolder[3]; 

				sendBuffer[12]=latHolder[4]; 

				sendBuffer[13]=latHolder[5];

				sendBuffer[14]=latHolder[6]; 

				sendBuffer[15]=latHolder[7]; 

				sendBuffer[16]=latHolder[8];

				sendBuffer[17]=latHolder[9]; 

				sendBuffer[18]=latHolder[10];

			

				// Longitude 

				sendBuffer[25]=longHolder[0]; 

				sendBuffer[26]=longHolder[1];  

				sendBuffer[27]=longHolder[2]; 

				sendBuffer[28]=longHolder[3];   

				sendBuffer[29]=longHolder[4]; 

				sendBuffer[30]=longHolder[5];   

				sendBuffer[31]=longHolder[6]; 

				sendBuffer[32]=longHolder[7];   

				sendBuffer[33]=longHolder[8];

				sendBuffer[34]=longHolder[9]; 

				sendBuffer[35]=longHolder[10];


				
				// Roll

				sendBuffer[42]=rollHolder[0];

				sendBuffer[43]=rollHolder[1];

				sendBuffer[44]=rollHolder[2];

				sendBuffer[45]=rollHolder[3];

				sendBuffer[46]=rollHolder[4];

				sendBuffer[47]=rollHolder[5];

				sendBuffer[48]=rollHolder[6];

				sendBuffer[49]=rollHolder[7];

				sendBuffer[50]=rollHolder[8];

				sendBuffer[51]=rollHolder[9];

				sendBuffer[52]=rollHolder[10];



				// Pitch

				sendBuffer[60]=pitchHolder[0];

				sendBuffer[61]=pitchHolder[1];

				sendBuffer[62]=pitchHolder[2];

				sendBuffer[63]=pitchHolder[3];

				sendBuffer[64]=pitchHolder[4];

				sendBuffer[65]=pitchHolder[5];

				sendBuffer[66]=pitchHolder[6];

				sendBuffer[67]=pitchHolder[7];

				sendBuffer[68]=pitchHolder[8];

				sendBuffer[69]=pitchHolder[9];

				sendBuffer[70]=pitchHolder[10];





				// Yaw

				sendBuffer[76]=yawHolder[0];

				sendBuffer[77]=yawHolder[1];

				sendBuffer[78]=yawHolder[2];

				sendBuffer[79]=yawHolder[3];

				sendBuffer[80]=yawHolder[4];

				sendBuffer[81]=yawHolder[5];

				sendBuffer[82]=yawHolder[6];

				sendBuffer[83]=yawHolder[7];

				sendBuffer[84]=yawHolder[8];

				sendBuffer[85]=yawHolder[9];

				sendBuffer[86]=yawHolder[10];


				goto end1;

			}

// Remove residual values.

			for(i=0; i<19; ++i)

				data_pos1[i]=0; 	

		}

    }// end of while

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

	

	//Send to position and orientation mode

    msgout[0]=0xFA;msgout[1]=01;msgout[2]=0xD0;msgout[3]=02;msgout[4]=00;msgout[5]=0x14;msgout[6]=0x19;

	//Get acknowledge for sending to position and orientation mode

    msgoutack[0]=0xFA;msgoutack[1]=01;msgoutack[2]=0xD1;msgoutack[3]=00;msgoutack[4]=0x2E;

	//Send to measurement mode

    msgmeas[0]=250;msgmeas[1]=255;msgmeas[2]=16;msgmeas[3]=00;msgmeas[4]=241;

	//Get acknowledge for sending to measurement mode.

    msgmeasack[0]=250;msgmeasack[1]=255;msgmeasack[2]=0x11;msgmeasack[3]=00;msgmeasack[4]=0xF0;

	

	//Send to orientation mode

    msgout1[0]=250;msgout1[1]=01;msgout1[2]=0xD0;msgout1[3]=02;msgout1[4]=00;msgout1[5]=04;msgout1[6]=0x29;

	//Send to euler mode

    msgout1e[0]=250;msgout1e[1]=01;msgout1e[2]=0xD2;msgout1e[3]=04;msgout1e[4]=00;msgout1e[5]=00;msgout1e[6]=0x00;msgout1e[7]=05;msgout1e[8]=0x24;

	//Acknowledge for sending to euler mode

    msgoutack1e[0]=250;msgoutack1e[1]=01;msgoutack1e[2]=0xD3;msgoutack1e[3]=00;msgoutack1e[4]=0x2C; 

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



	fprintf(fp," time,latitude,longitude,altitude,roll,pitch,yaw\n");

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

			read(fdSG,&readBuffer,sizeof(readBuffer)); 

			printf("Recv Buffer: %s\n",readBuffer); 

			if(readBuffer[0]=='S' && readBuffer[1]=='G' && readBuffer[2]=='B'){ 

				placeHolder[0]=readBuffer[4];

				stopbit = atoi(placeHolder); 

				printf("STOPBIT: %d",stopbit); 

				if(stopbit==1)

					goto STOPHERE; 	

			}

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

	fclose(fp);

	close(fd);

	return 0; 

}

