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

int buf_siz=63;

char buffer[63]; 

char tempHolder[5],latHolder[11],longHolder[11],altHolder[5],velxHolder[7],velyHolder[7],velzHolder[5],rollHolder[11],pitchHolder[11],yawHolder[11],gyrxHolder[7],gyryHolder[7],gyrzHolder[7],accxHolder[7],accyHolder[7],acczHolder[7];

char sendBuffer[]="Temp:10.00 Lat:0000.000000 Long:0000.000000 Alt:00.00 VelX:00.0000 VelY:00.0000 VelZ:00.0000 Roll:0000.000000 Pitch:0000.000000 Yaw:0000.000000 GyrX:00.0000 GyrY:00.0000 GyrZ:00.0000 AccX:00.0000 AccY:00.0000 AccZ:00.0000 \r\n";  

char readBuffer[10];

int statwrite;



struct timeval now,start;



    char msgconfig[5],msgconfigack[5],msgout[7],msgoutack[5],msgoutsettings[9],msgoutsettingsack[5],msgmeas[5],msgmeasack[5];

FILE *fp; 





// The function for getting the position data from the IMU. 

// Make sure to run the config once before you run this.

int data_imu_pos(){

    char data_pos[69],crc,data[88],data_pos1[69];

    int t=0,i,k=0,j=0,l,i0,i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12,i13,i14,i15,i16,j1,test,d0,d1,d2,d3,dp0,dp1,dp2,dp3;

    time_t rawtime;

    struct tm *timeinfo;

    unsigned int value;

    union {

	int i;float f;

    } hextofloat;

    float crcheck,temp,accx,accy,accz,gyrx,gyry,gyrz,velx,vely,velz,latitude,longitude,altitude,roll,pitch,yaw;

// The standard position message structure.

    data_pos[0]=0xFA;data_pos[1]=0xFF;data_pos[2]=0x32;data_pos[3]=0x40;



//First compare with the known standard message which the IMU sends.

// if you want longer recording make sure to increase the value in thw while loop.



    while(k<200000){ 

repeat1:	read(fd,buffer,sizeof(buffer)); 

		gettimeofday(&now,NULL);

		time(&rawtime);

		timeinfo=localtime(&rawtime);

		for(i = 0; i<61; ++i){ 

			if(buffer[i]!='\n'){

//				fprintf(fp,"%X ",buffer[i] & 0xff); 				

				data[j]=buffer[i] & 0xff;

				j++;

// make chunks of 90 bytes of data.

				if(j>89) {

					j=0;

					for (i0=0;i0<88-30;i0++){

						if (data[i0]==data_pos[0] && data[i0+1]==data_pos[1] && data[i0+2]==data_pos[2] && data[i0+3]==data_pos[3]) {

						printf("valid packet found");

						// If the message structure matches then copy the message contents.

										for(j1=0; j1<69; j1++)

											data_pos[j1]=data[j1+i0] & 0xFF;

						}

					}

				}

			}

		}		

		for(i=0; i<61; ++i)

			buffer[i]='\n'; 

		k++;



// Copy the contents into another array.		

		for(i=0; i<69; ++i)

		data_pos1[i]=data_pos[i];



// Do the Checksum

		crc=data_pos[1]& 0xFF;

		for(i=2; i<69; ++i)

			crc+=data_pos[i];

//		crc=1;

		hextofloat.i = crc;

		crcheck = hextofloat.f;

		printf("crc check:%5.2f",crcheck);





// TO make avoid residual values add zeroes into it.

		for(i=4; i<69; ++i)

		data_pos[i]=0;



// If checksum is found valid then convert the hex values to IEEE float values.

// Get the Lat, Lon, Alt and write them into a Binary Datafile for now.

		if (crc==0) {

			

			

			//Bytes 4:5 will be temperature

			value = (data_pos1[5] & 0xFF) | ((data_pos1[4] & 0xFF)<<8);

			//temperature data is fomatted as a 16 bit two complement number - last byte represents data after the comma

			temp = value/256;

			//Bytes 6:17 are accelerations. 4 for each of the X,Y and Z directions in m/s^2, IEEE float format

			value = (data_pos1[9] & 0xFF) | ((data_pos1[8] & 0xFF)<<8) | ((data_pos1[7] & 0xFF)<<16) | ((data_pos1[6] & 0xFF)<<24);

			hextofloat.i = value;

			accx = hextofloat.f;

			value = (data_pos1[13] & 0xFF) | ((data_pos1[12] & 0xFF)<<8) | ((data_pos1[11] & 0xFF)<<16) | ((data_pos1[10] & 0xFF)<<24);

			hextofloat.i = value;

			accy = hextofloat.f;

			value = (data_pos1[17] & 0xFF) | ((data_pos1[16] & 0xFF)<<8) | ((data_pos1[15] & 0xFF)<<16) | ((data_pos1[14] & 0xFF)<<24);

			hextofloat.i = value;

			accz = hextofloat.f;

			//Bytes 18:29 are rate of turns. 4 for each of the X,Y and Z axes in rad/s, IEEE float format

			value = (data_pos1[21] & 0xFF) | ((data_pos1[20] & 0xFF)<<8) | ((data_pos1[19] & 0xFF)<<16) | ((data_pos1[18] & 0xFF)<<24);

			hextofloat.i = value;

			gyrx = hextofloat.f;

			value = (data_pos1[25] & 0xFF) | ((data_pos1[24] & 0xFF)<<8) | ((data_pos1[23] & 0xFF)<<16) | ((data_pos1[22] & 0xFF)<<24);

			hextofloat.i = value;

			gyry = hextofloat.f;

			value = (data_pos1[29] & 0xFF) | ((data_pos1[28] & 0xFF)<<8) | ((data_pos1[27] & 0xFF)<<16) | ((data_pos1[26] & 0xFF)<<24);

			hextofloat.i = value;

			gyrz = hextofloat.f;

			//Bytes 30:41 are orientation angles. 4 for each of the roll, pitch, and yaw in degrees, IEEE float format

			value = (data_pos1[33] & 0xff) | ((data_pos1[32] & 0xff)<<8) | ((data_pos1[31] & 0xff)<<16) | ((data_pos1[30] & 0xff)<<24);

			hextofloat.i = value;

			roll=hextofloat.f;

			if ((roll<(-180) && roll>180) ) goto repeat1;

			value = (data_pos1[37] & 0xff) | ((data_pos1[36] & 0xff)<<8) | ((data_pos1[35] & 0xff)<<16) | ((data_pos1[34] & 0xff)<<24);

			hextofloat.i = value;

			pitch=hextofloat.f;

			if ((pitch<(-180) && pitch>180)) goto repeat1;

			value = (data_pos1[41] & 0xff) | ((data_pos1[40] & 0xff)<<8) | ((data_pos1[39] & 0xff)<<16) | ((data_pos1[38] & 0xff)<<24);

			hextofloat.i = value;

			yaw=hextofloat.f;

			if (yaw<(-180) && yaw>180) goto repeat1;

			////Bytes 42:53 are GPS values. 4 for each of the latitude,longitude and altitude. Latitude and longitude are in degrees, altitude is in meters with respect to the LLAWGS84 model of the earth's surface, IEEE float format

			value = (data_pos1[45] & 0xff) | ((data_pos1[44] & 0xff)<<8) | ((data_pos1[43] & 0xff)<<16) | ((data_pos1[42] & 0xff)<<24);

			hextofloat.i = value;

			latitude=hextofloat.f;

			if (latitude<(-90) && latitude>90) goto repeat1;

			value = (data_pos1[49] & 0xff) | ((data_pos1[48] & 0xff)<<8) | ((data_pos1[47] & 0xff)<<16) | ((data_pos1[46] & 0xff)<<24);

			hextofloat.i = value;

			longitude=hextofloat.f;

			if (longitude<(-180) && longitude>180) goto repeat1;

			value = (data_pos1[53] & 0xff) | ((data_pos1[52] & 0xff)<<8) | ((data_pos1[51] & 0xff)<<16) | ((data_pos1[50] & 0xff)<<24);

			hextofloat.i = value;

			altitude=hextofloat.f;

			if (altitude<(-200) && altitude>1000) goto repeat1;

			//Bytes 54:65 are velocities. 4 for each of the X,Y and Z directions in m/s, IEEE float format

			value = (data_pos1[57] & 0xFF) | ((data_pos1[56] & 0xFF)<<8) | ((data_pos1[55] & 0xFF)<<16) | ((data_pos1[54] & 0xFF)<<24);

			hextofloat.i = value;

			velx = hextofloat.f;

			value = (data_pos1[61] & 0xFF) | ((data_pos1[60] & 0xFF)<<8) | ((data_pos1[59] & 0xFF)<<16) | ((data_pos1[58] & 0xFF)<<24);

			hextofloat.i = value;

			vely = hextofloat.f;

			value = (data_pos1[65] & 0xFF) | ((data_pos1[64] & 0xFF)<<8) | ((data_pos1[63] & 0xFF)<<16) | ((data_pos1[62] & 0xFF)<<24);

			hextofloat.i = value;

			velz = hextofloat.f;




			sprintf(tempHolder,"%+05.2f",temp);

			sprintf(latHolder,"%+011.6f",latitude);

			sprintf(longHolder,"%+011.6f",longitude);

			sprintf(altHolder,"%+05.2f",altitude);

			sprintf(velxHolder,"%+07.4f",velx);

			sprintf(velyHolder,"%+07.4f",vely);

			sprintf(velzHolder,"%+07.4f",velz);

			sprintf(rollHolder,"%+011.6f",roll);

			sprintf(pitchHolder,"%+011.6f",pitch);

			sprintf(yawHolder,"%+011.6f",yaw);

			sprintf(gyrxHolder,"%+07.4f",gyrx);

			sprintf(gyryHolder,"%+07.4f",gyry);

			sprintf(gyrzHolder,"%+07.4f",gyrz);

			sprintf(accxHolder,"%+07.4f",accx);

			sprintf(accyHolder,"%+07.4f",accy);

			sprintf(acczHolder,"%+07.4f",accz);

			/*sendBuffer[5] = tempHolder[0];
			sendBuffer[6] = tempHolder[1];
			sendBuffer[7] = tempHolder[2];
			sendBuffer[8] = tempHolder[3];
			sendBuffer[9] = tempHolder[4];*/

			for (i1=0;i1<sizeof(tempHolder);i1++) {

				sendBuffer[5+i1]=tempHolder[i1];

			}

			for (i2=0;i2<sizeof(latHolder);i2++) {

				sendBuffer[15+i2]=latHolder[i2];
				
			}

			for (i3=0;i3<sizeof(longHolder);i3++) {

				sendBuffer[32+i3]=longHolder[i3];
				
			}

			for (i4=0;i4<sizeof(altHolder);i4++) {

				sendBuffer[48+i4]=altHolder[i4];
				
			}

			for (i5=0;i5<sizeof(velxHolder);i5++) {

				sendBuffer[59+i5]=velxHolder[i5];
				
			}

			for (i6=0;i6<sizeof(velyHolder);i6++) {

				sendBuffer[72+i6]=velyHolder[i6];
				
			}

			for (i7=0;i7<sizeof(velzHolder);i7++) {

				sendBuffer[85+i7]=longHolder[i7];
				
			}

			for (i8=0;i8<sizeof(rollHolder);i8++) {

				sendBuffer[98+i8]=rollHolder[i8];
				
			}

			for (i9=0;i9<sizeof(pitchHolder);i9++) {

				sendBuffer[116+i9]=pitchHolder[i9];
				
			}

			for (i10=0;i10<sizeof(yawHolder);i10++) {

				sendBuffer[132+i10]=yawHolder[i10];
				
			}

			for (i11=0;i11<sizeof(gyrxHolder);i11++) {

				sendBuffer[149+i11]=gyrxHolder[i11];
				
			}

			for (i12=0;i12<sizeof(gyryHolder);i12++) {

				sendBuffer[162+i12]=gyryHolder[i12];
				
			}

			for (i13=0;i13<sizeof(gyrzHolder);i13++) {

				sendBuffer[175+i13]=gyrzHolder[i13];
				
			}

			for (i14=0;i14<sizeof(accxHolder);i14++) {

				sendBuffer[188+i14]=accxHolder[i14];
				
			}

			for (i15=0;i15<sizeof(accyHolder);i15++) {

				sendBuffer[201+i15]=accyHolder[i15];
				
			}

			for (i16=0;i16<sizeof(acczHolder);i16++) {

				sendBuffer[214+i16]=longHolder[i16];
				
			}

// Remove residual values.

			for(i=0; i<55; ++i)

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





// Change the IMU output mode.

// Also check for acknowledge from the IMU.

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




// Change the IMU output settings.

// Also check for acknowledge from the IMU.

redo3:

    br=0;t=0;

    statwrite=write(fd,&msgoutsettings,9);

    if (statwrite!=9) {

	perror("Output settings unsuccessful");

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

    if (t>30 && br!=1) goto redo3;

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

			cfsetospeed(&options2,B9600); 

			cfsetispeed(&options2,B9600); 

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

	

	//set output mode

    msgout[0]=0xFA;msgout[1]=0x01;msgout[2]=0xD0;msgout[3]=0x02;msgout[4]=0x00;msgout[5]=0x37;msgout[6]=0xF6;

    //expected output mode acknowledge message

    msgoutack[0]=0xFA;msgoutack[1]=0x01;msgoutack[2]=0xD1;msgoutack[3]=0x00;msgoutack[4]=0x2E;

    //change output mode settings

    msgoutsettings[0]=0xFA;msgoutsettings[1]=0x01;msgoutsettings[2]=0xD2;msgoutsettings[3]=0x04;msgoutsettings[4]=0x10;msgoutsettings[5]=0x00;msgoutsettings[6]=0x0C;msgoutsettings[7]=0x44;msgoutsettings[8]=0xC9;

    //expected output mode settings acknowledge message

    msgoutsettingsack[0]=0xFA;msgoutsettingsack[1]=0x01;msgoutsettingsack[2]=0xD3;msgoutsettingsack[3]=0x00;msgoutsettingsack[4]=0x2C;



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

