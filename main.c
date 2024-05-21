//******************************************************************************
//  OT15_Base_Test  
//  USB Serial to OT15 SPI terminal interface program.
//  for use with MAX32630FTHR on MAX30134_OT15_EVKIT_A and similar boards.
//
//  last edit date 30 SEP 2020  DJE.
//  - added FIFO read with decoded tags and Code to value conversions.
//  - updated help response.
//  - added GPIO1 output to use as scope trigger
//  - removed Test Mode code
//  - Fix EIS Offset Calc,  21 Dec 2020
//******************************************************************************

#include "mbed.h"
#include "math.h"
#include "max32630fthr.h"
#include "USBSerial.h"

#define BS 8
#define CR 13

#define ECHO_WRITE 1        //echos write commands with a read.
#define DEBUG 0
//******************************************************************************
//  Hardware Configuration
//
//  SCLK        --> P5_0
//  MOSI        --> P5_1
//  MISO        --> P5_2
//  OT15_CSB    --> P5_6
//  
//  
//  OT15_INTB   --> P5_5  FIFO_A_FULL (OT15)
//  GPIO1       --> P3_5  GPIO 
//  
//******************************************************************************
Timer      mytimer;
void my_delay_ms(int n); 

void my_delay_us(int n)
{
     /*  int i,j=0;
       for(i=0;i<10000;i++)
        {
         
         //SSS.write(0);
         
            }
      
            //pc.printf("");
        */  int begin,end;
          mytimer.reset();      
           mytimer.start();
          begin=mytimer.read_us();
             
             do{
               end=mytimer.read_us();
               } while(end-begin <n);
           mytimer.stop();    
}
void my_delay_ms(int n)
{
          int begin,end;
          mytimer.reset();      
           mytimer.start();
          begin=mytimer.read_ms();
             
             do{
               end=mytimer.read_ms();
               } while(end-begin <n);
           mytimer.stop();    
}

//******************************************************************************
//               init Feather Boared Hardware
//******************************************************************************
MAX32630FTHR pegasus(MAX32630FTHR::VIO_1V8);  

//Configure serial ports 
Serial db(P2_1, P2_0);  // Hardware serial debug port over DAPLink, 9600 defult baudrate
USBSerial pc;  // Virtual serial port over USB

SPI spi_bus(P5_1,P5_2,P5_0);        //P5_1 -> MOSI,  P5_2 -> MISO, P5_0 -> SCLK

DigitalOut OT15_cs(P5_6,1);         // P5_6 chip select for OT15
DigitalOut GPIO1(P3_5,1);           // P3_5 trigger out for scope.

//DigitalIn  OT15_INTB(P5_5, OpenDrain);  //FIFO_A_FULL or DATA_RDY interupt.
//DigitalIn  OT02_INTB(P5_5, OpenDrain);  //EFIT 
int interrupt_flag;
InterruptIn  OT15_INTB(P5_5);
Ticker HeartBeat_timer;
Ticker polling_timer;


DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
char OT15_FIFO_data_1[30000][3];
bool polling_flag = false;

void HeartBeat_ticker_callback(){ // MBED LED Heart beat    
    led2=!led1;       
    led1=!led1;   
 }
 
 void polling_ticker_callback(){ // MBED LED Heart beat    
    polling_flag = true; 
 }
 
//******************************************************************************
//                       OT15 low level read / write
//******************************************************************************

void OT15_write_reg(int reg, int val){      //writes one byte to reg address.    
        OT15_cs = 0;                        // select OS61
        spi_bus.write(reg);                 // write register address of byte to read
        spi_bus.write(0x00);                // write 'write command'  [7] = 0 --> write, [6:0] dont care
        spi_bus.write(val);                 // write data, bits [7:0]        
        OT15_cs = 1;                        // release OS61
        
}

int OT15_read_reg(int reg, char data[]){    //reads byte at reg address add     
        OT15_cs = 0;                        // select OT15
        spi_bus.write(reg);                 // write register address of byte to read
        data[0] = spi_bus.write(0x80);      // write 'read command' [7] = 1 --> read, [6:0] dont care
        data[1] = spi_bus.write(0x00);      // read reg value        
        OT15_cs = 1;                        // release OT15 
        return (int)data[1];       
}

int OT15_read_reg(int reg){                 //reads byte at reg address add     
        char data[2];
        OT15_cs = 0;                        // select OT15
        spi_bus.write(reg);                 // write register address of byte to read
        data[0] = spi_bus.write(0x80);      // write 'read command' [7] = 1 --> read, [6:0] dont care
        data[1] = spi_bus.write(0x00);      // read reg value        
        OT15_cs = 1;                        // release OT15              
        return   (int)data[1];       
}


int OT15_read_FIFO(char FIFO_data[][3]){  // reads 24 bit 'words' from FIFO
        char data[4];
        int data_reg = 0x0E;            // FIFO Data register
        int of_reg = 0x0C;              // FIFO Overflow Counter register followed by offset reg and read pointer reg
        //int overflow;   
        int fifo_count;       
        int i;
        
        //check if fifo has new data
        //read fifo OF and Data Counters.
        OT15_cs = 0;                            // select OT15
        spi_bus.write(of_reg);                  // write register address of first byte to read 
        data[0] = spi_bus.write(0x80);          // write 'read command' [7] = 1, [6:0] 'dont care bits'
        data[1] = spi_bus.write(0xff);          // read bits [7:0]  FIFO Overflow Counter
        data[2] = spi_bus.write(0xff);          // read bits [7:0]  FIFO Data counter          
        OT15_cs = 1;                            // release OT15  
        
        
        //overflow = (int)data[1];
        fifo_count = +((int)(data[1]&0x80)*2)  + (int)data[2];
        
        if(fifo_count){//only read fifo if new data avalible 
            // set up FIFO data burst read
            OT15_cs = 0;                        // select OT15
            spi_bus.write(data_reg);            // write register address of byte to read
            data[0] = spi_bus.write(0x80);      // read command [7] = 1, [6:0] dont care
            for(i=0;i<fifo_count;i++){
                FIFO_data[i][0] = spi_bus.write(0xff);     // read bits [23:16] (MSB)
                FIFO_data[i][1] = spi_bus.write(0xff);     // read bits [15:8]
                FIFO_data[i][2] = spi_bus.write(0xff);     // read bits [7:0]   (LSB)
            }
            OT15_cs = 1;                        // release OT15   
           // pc.printf("FIFO count[%3d] \r\n",fifo_count);
           // for(i=0;i<fifo_count;i++){
           //     pc.printf("[%02X][%02X][%02X]\r\n", FIFO_data[i][0],FIFO_data[i][1],FIFO_data[i][2]);
           // }    
        }//end if (fifo_count)

        return fifo_count;
}

// *****************************************************************************
// convert_temperature()    sends convert command to OT15 device
//                     
// *****************************************************************************

void convert_temperature(){   // set convert bit to start conversion
    
    OT15_write_reg(0x10, 0x10);    //clear fifo
    OT15_write_reg(0x60, 0xC1);    // set temperature configuration
    OT15_write_reg(0x80, 0x00);    //set DC mode
    OT15_write_reg(0x83, 0x01);    // manual convert
    OT15_write_reg(0x83, 0x00);    // manual convert
    OT15_write_reg(0x83, 0x01);    // manual convert        
    
}

//******************************************************************************
// get_temperature()        read temperature from OT15 device FIFO register
//                 
// returns                  double temperature in oC
//******************************************************************************

double get_temperature(){
    char data[8][3];
    double T=0;
    int count;
    int i;
    int  FIFO_read_count;
    OT15_write_reg(0x60, 0xC0);  // clear temperature configuration
    FIFO_read_count=OT15_read_FIFO(data);     // Read temperature from FIFO, 3 byte words
    
    for(i=0;i<FIFO_read_count;i++)
     {  
       // pc.printf("%02x, %02x, %02x \r\n",data[i][0],data[i][1],data[i][2]); 
        if(data[i][0]=0xc0)
            {     
                  
                count = (int)(data[i][1]*256 + data[i][2]);
                if (count >= 32768)count = count - 65536;     // 2s comp
                T = (double)count*0.005; 
            }
    }
    return T;  
}
//*************************************************************************
void OS15_detect_enable()
{
     char data[64];                         

     OT15_read_reg(0x02,data);   //read the status register  ,clear register
                                                                 
                                                                    
     if((data[1]&0x01)==0x01)
         {
            pc.printf("\r\n WE1 resister has insert\r\n");     
            pc.printf("**********************************\r\n");   
            interrupt_flag=1;                                                                                
         }
     if((data[1]&0x10)==0x10)
         {
            pc.printf("\r\n WE2 resister has insert\r\n");     
            pc.printf("**********************************\r\n");   
            interrupt_flag=1;                                                                                
         }    
} 

void OS15_ADC_Read()
{
     char data[64];
     int ADC_count;
     int FIFO_read_count;
     int tag;
     double Temperature_T;
     double Current1;
     double Current2;
     double RE1;
     double CE1;
     double WE1;
     double WE2;
     double WO1;
     double Resister;
     int x;
     int i;
     int Vref;
     char OT15_FIFO_data[256][3]; 
    do{    

        OT15_read_reg(0x00,data);   //read the status register  ,clear register
        OT15_read_reg(0x01,data);   //read the status register  ,clear register
        OT15_read_reg(0x02,data);   //read the status register  ,clear register
        OT15_read_reg(0x03,data);   //read the status register  ,clear register
        OT15_read_reg(0x04,data);   //read the status register  ,clear register
        OT15_read_reg(0x83,data);                                        
      }while((data[1]&0x01)==0x01);     //wait the ADC finish                             

      

//*******************************
                                FIFO_read_count = OT15_read_FIFO(OT15_FIFO_data); 
                             //   pc.printf("FiFO count===%d\r\n",FIFO_read_count) ;
                                for(i=0;i<FIFO_read_count;i++)  
                                         {        
                                                //  pc.printf("%02X,%02X,%02X    ",OT15_FIFO_data[i][0],OT15_FIFO_data[i][1],OT15_FIFO_data[i][2]);
                                                  tag = (((OT15_FIFO_data[i][0])&0x0F)<<4) + (((OT15_FIFO_data[i][1])&0xF0)>>4); 
                                                  if((tag&0xf0)==0xc0)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            if (ADC_count >= 32768)ADC_count = ADC_count - 65536;     // 2s comp
                                                            Temperature_T = (double)ADC_count*0.005; 
                                                            pc.printf("Temp=%2.1f ",Temperature_T); 
                                                          }   
                                                  if((tag&0xf0)==0x0)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            Current1= (double)ADC_count*0.0000152588*2000-0;//(0.1*2000);
                                                            pc.printf("cur1=%4.0fnA ",Current1); 
                                                          }   
                                                  if((tag&0xf0)==0x10)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            Current2= (double)ADC_count*0.0000152588*2000-0;//(0.1*2000);
                                                            pc.printf("cur2=%4.0fnA ",Current2); 
                                                          }   
                                                                                                                    
                                                   x = (OT15_read_reg(0x68)&0x06)>>1;
                                                        switch(x){
                                                                  case 0: Vref = 1536;break;
                                                                  case 1: Vref = 2048;break;
                                                                  case 2: Vref = 3072;break;
                                                                  case 3: Vref = 4096;break;
                                                                  }      
                                                   // pc.printf("                          VREF ==== %d mV\r\n",Vref);                                                                    
                                                   if(tag==0xd1)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WE1=%4.0fmV ",WE1); 
                                                          }   
                                                                                                                            
                                                    if(tag==0xd2)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            RE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("RE1=%4.0fmV ",RE1); 
                                                          }   
                                                   if(tag==0xd3)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            CE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("CE1=%4.0fmV ",CE1); 
                                                          }     
                                                   if(tag==0xd0)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WO1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WO1=%4.0fmV ",WO1); 
                                                          }     
                                                   if(tag==0xd5)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WE2= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WE2=%4.0fmV ",WE2); 
                                                          }                                                             
                                                                                                                                                                                     
                                                  
                                         } 
                                        Resister=(WE1-RE1)/ Current1;  
                                        pc.printf("Res1=%7.3f MOhm ",Resister);   
                                        Resister=(WE2-RE1)/ Current2;  
                                        pc.printf("Res2=%7.3f MOhm\r\n",Resister);





//*****************************
        interrupt_flag=1;                                                                                              

}             
//******************************************************************************
//
//                                  main()
//
//******************************************************************************


int main()
{
    
    // general 
    int i = 0;
    int j;
    int k;
    int add;   
    char data[64];   
    int delay = 25;             // temperature convert delay in ms
        
    // IO  
    char c;
    char rx_buff[64];
    char s[128];
    int n;
    int arg0;
    int arg1;
    int arg2; 
    int arg3;
    int arg4;    
    int rx_index = 0;
    int echo = 1; 
    int PA_count;
    
        
    //OT15

    char OT15_FIFO_data[256][3];      //array to save FIFO_data in 
                     //
    
    
    int FIFO_read_count;
    int x;                          //temp var for bit fiddle
    int y;
    int z;
    int Vref;
    char tag;
    int ADC_count;
    int ADC_compare;

    int AC_mode;
    int convert_type;
    double Sn_FSR;
    double Sn_offset;
    
    double EIS_offset;
    double EIS_FSR;
    double EIS_amp;
    double kcic_table[24] = {1.0,1.0,1.0,0.7298,0.8332,0.8831,0.8960,0.8992,0.9,0.9002,0.9003,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
    double EIS_real;
    double EIS_imag;
    double Temperature_T;
    double CE1;
    double RE1;
    double WE1;
    double WE2;
    double WO1;        
    double Current1;
    double Current2;
    double Resister;
    double Iadc1;
    double Iadc2;
    double Iadc3;
    double PA_deg;
    double correction_factor;
    
    double EIS_theta;
    double R,X,Z;
    int course;
    int fine;
    double frequencey_calibration;
    double y1;
    double y2;
    char dietype;
 
    int clk_divide;
    int DAC_code;
    double T;
    double Iadc;
    char continueconverte;
    char freqtrim1;

    //************* init ticker timer callback  ****************
    //HeartBeat_timer.attach(&HeartBeat_ticker_callback,1.0);    
    
    //*************  init serial ports  ************
    sprintf(s,"\r\nOT15 Python Terminal  V0.02\r\n");
    db.printf(s);
    wait(1);                            // wait 1 sec for USB Serial to start
    pc.printf(s);  
    // ************  SPI init  ******************
    spi_bus.frequency(1000000);         // 1 or 8 MHz    
    spi_bus.format(8,0);                // 8 bit words, mode 0 (POL = 0, PHA = 0)
    OT15_cs = 1;                        // set OSxx cs to high (not selected)
    
        
    OT15_read_reg(0xFF,data);           // address of part ID 
      
    pc.printf("<OT15_ID[%02X]>\r\n",data[1]); 
    OT15_write_reg(0x68,0x01);          //Enable REF    
   
  
//------------------------------------------------------------------------------
//                             Start Main Loop
//------------------------------------------------------------------------------
    
    
    rx_buff[0] = 0; 
    while (1){ 
        
         //  poll status registers if polling_flag true
        if(polling_flag){
            polling_flag = false;
            for(add=0;add<5;add++){
                data[add] = OT15_read_reg(add);
            }
            pc.printf("[ %02X %02X %02X %02X %02X ]\r\n",data[0],data[1],data[2],data[3],data[4]);
        }      

//------------------------------------------------------------------------------
//                   Test for Characters from PC and Process
//------------------------------------------------------------------------------        
            
        //test if PC sent some charaters   
        while(pc.readable()){  //characters in buffer,  get them                        
            rx_buff[rx_index] = pc.getc();
            if(echo)pc.putc(rx_buff[rx_index]);     //echo character  
            //pc.printf("<[%02x] %c i[%d]>",rx_buff[i],rx_buff[i],i); //echo charater
            
                            
            if(rx_buff[rx_index] == CR){
                if(echo)pc.printf("\r\n");
                rx_buff[++rx_index] = 0;
                //pc.printf("\n%s\n",rx_buff);
                rx_index=-1;  // because i++ at end of while give i=0 on next loop 
                
 
                arg0 = -1;
                arg1 = 0;
                arg2 = 0;
                
                n = sscanf(rx_buff, " %c %x %x %x %x %x", &c, &arg0, &arg1, &arg2, &arg3, &arg4);
                
                if(DEBUG)db.printf("%s",rx_buff);  //echo values read in
                
                //process input
                if(n > 0){//got input so process it
                    switch(c){
                        case 'n':
                        case 'N':     //write data to register 
                                OT15_write_reg(0x14, 0x01);   //reset the chip  ,maybe can set the chip into shutdown mode
                                OT15_write_reg(0x68, 0x03);   //set the VREF==2.048V
                                

                                                                
                                OT15_write_reg(0x1f, 0x03);   //set WE1 Detect Bias ==800mV
                                
                                OT15_write_reg(0x22, 0x06);   //set WE1  detect enable; detect current=80nA   
                                OT15_write_reg(0x2f, 0x06);   //set WE2  detect enable; detect current=80nA  
                                
                                OT15_read_reg(0x00,data);   //read the status register  ,clear register
                                OT15_read_reg(0x01,data);   //read the status register  ,clear register
                                OT15_read_reg(0x02,data);   //read the status register  ,clear register
                                OT15_read_reg(0x03,data);   //read the status register  ,clear register

                                OT15_INTB.fall(&OS15_detect_enable);   //fall    rise
                                
                                OT15_write_reg(0x07, 0x11);   //enable S1 detect, enable S2 detect
                                OT15_write_reg(0x95, 0x03);   //set interrupt active low  
                                pc.printf("wait for insert resister\r\n");
                               // pc.printf("***************************************\r");
                                interrupt_flag=0;
                                pc.printf("**** pls wait **********      \r\n");
                                do{
                                    pc.printf("*");
                                    my_delay_ms(400);                                                                                                    
                                    }while(interrupt_flag==0);
                                
 
                                 pc.printf("\r\n\r\n\r\n**********************************\r\ninterrupt end      \r\n");                               

                             /*                                   
                                do{
                                    pc.printf("wait for insert resister\r\n");

                                    OT15_read_reg(0x02,data);   //read the status register  ,clear register
                                    my_delay_ms(500);                                    
                                                                    
                                    } while((data[1]&0x01)==0);
                               
                                    pc.printf("the Resister has insert\r\n");     
                                    pc.printf("**********************************\r\n");                                                                                     
                                    pc.printf("**********************************\r\n");  
                                    pc.printf("**********************************\r\n");                                                                       
                                  */     
                                break;                                
                                                case 'i':
                        case 'I':     //convert channel1 DC +temperature
                                OT15_write_reg(0x14, 0x01);   //reset the chip  ,maybe can set the chip into shutdown mode
                                OT15_read_reg(0xfe,data); 
                                dietype=data[1];
                                pc.printf("datatype=%d\r\n",dietype);                       
                                for(continueconverte=0x0;continueconverte<0x50;continueconverte++)  //0x70
                                {

                                
                                OT15_write_reg(0x68, 0x03);   //set the VREF==2.048V
                                OT15_write_reg(0x1f, 0x04);   //config shared RE1 & CE1
                                
                                
                                OT15_write_reg(0x69, 0x80+continueconverte);   //set DACA== volatage  default is 0x640
                                OT15_write_reg(0x6A, 0x01);   //set DACA== volatage  default is 0x640
                                OT15_write_reg(0x6B, 0x70-(continueconverte/2));   //set DACB ===voltage   default is 0x640
                                OT15_write_reg(0x6C, 0x01);   //set DACB ===voltage   default is 0x640
  
                                OT15_write_reg(0x6D, 0x90+continueconverte);   //set DACC ===voltage   default is 0x640
                                OT15_write_reg(0x6E, 0x01);   //set DACC ===voltage   default is 0x640
                                                              
                                

                                
                                //channel 1
                                OT15_write_reg(0x20, 0xc4);   //set the S1_WE S1_CE switch   WE1==DACA   CE1==DACB                   c4
                                OT15_write_reg(0x21, 0x90);   //set the S1_WE S1_CE switch  0x90 for WE drive, 0x28 dor CE drive     90
                                OT15_write_reg(0x22, 0x08);   //set the OFFSET current mode                                          08
                                OT15_write_reg(0x23, 0xa1);   //set the S1 FSR  and OFFSET                                          a1
                                OT15_write_reg(0x24, 0x03);   //S1 conver time speed &  S1 conveter include or not                  01
                                //channel 2
                                OT15_write_reg(0x2d, 0xe4);   //set the S2_WE S2_CE switch  WE1==DACC   CE1==DACB                    e4
                                OT15_write_reg(0x2e, 0x90);   //set the S2_WE S2_CE switch  0x90 for WE drive, 0x28 dor CE drive     90
                                OT15_write_reg(0x2f, 0x08);   //set the OFFSET current mode                                          08
                                OT15_write_reg(0x30, 0xa1);   //set the S2 FSR  and OFFSET                                          a1
                                OT15_write_reg(0x31, 0x03);   //S2 conver time speed &  S2 conveter include or not                  01                                
                                
                                OT15_write_reg(0x54, 0x08);   //set Vsensor voltage amplifer==0.5
                                OT15_write_reg(0x55, 0x01);   //select system voltage for ADC
                                OT15_write_reg(0x56, 0x47);   //system ADC ===RE1  CE1  WE1   &  WE2
                                OT15_write_reg(0x57, 0x01);   //must need set up this,otherwise, the CE1 will be 0000
                                
                                OT15_write_reg(0x80, 0x41);    //channel1 is in DC mode                                
                                OT15_write_reg(0x60, 0xc1);    //add temperature in the list
                                OT15_write_reg(0x10, 0x10);    //clear FIFO
                                
                              
                                
                             //*************************interrupt enable
                                OT15_write_reg(0x06, 0x00);   //set the System ADC & Temperature ADC interrupt enable
                                OT15_write_reg(0x07, 0x80);   //set the channel1 & channel2 current ADC finish interrupt enable                                    
                                OT15_write_reg(0x05, 0x00);   //
                                OT15_write_reg(0x08, 0x00);   //  
                                OT15_write_reg(0x09, 0x00);   //

                                                                                         
                                OT15_read_reg(0x00,data);   //read the status register  ,clear register
                                OT15_read_reg(0x01,data);   //read the status register  ,clear register
                                OT15_read_reg(0x02,data);   //read the status register  ,clear register
                                OT15_read_reg(0x03,data);   //read the status register  ,clear register
                                OT15_read_reg(0x04,data);   //read the status register  ,clear register
                                
                                OT15_INTB.fall(&OS15_ADC_Read);   //fall    rise
                                
                                OT15_write_reg(0x95, 0x03);   //set interrupt active low  
 
                                OT15_write_reg(0x83, 0x01);  //   DIE_TYPE==0xFE register     if(DIE_TYPE == 129)|(DIE_TYPE == 145), need  convert2 times  else£¬ only 1 time 
                                if((dietype==129) ||(dietype==145))                                                    
                                    {
                                        OT15_write_reg(0x83, 0x00);  //   DIE_TYPE==0xFE register     if(DIE_TYPE == 129)|(DIE_TYPE == 145), need  convert2 times  else£¬ only 1 time 
                                        OT15_write_reg(0x83, 0x01);  //   DIE_TYPE==0xFE register     if(DIE_TYPE == 129)|(DIE_TYPE == 145), need  convert2 times  else£¬ only 1 time                                   
                                    }  
                             //*************************interrupt enable   

                                interrupt_flag=0;
                                do{

                                    my_delay_us(100);                                                                                                    
                                    }while(interrupt_flag==0);

                             
                                /*
                                do{     
                                        add=0x83;
                                        OT15_read_reg(add,data);
                                    }while((data[1]&0x01)==0x01);     //wait the ADC finish
                                
                                FIFO_read_count = OT15_read_FIFO(OT15_FIFO_data); 
                               // pc.printf("FiFO count===%d\r\n",FIFO_read_count) ;
                                for(i=0;i<FIFO_read_count;i++)  
                                         {        
                                                //  pc.printf("%02X,%02X,%02X    ",OT15_FIFO_data[i][0],OT15_FIFO_data[i][1],OT15_FIFO_data[i][2]);
                                                  tag = (((OT15_FIFO_data[i][0])&0x0F)<<4) + (((OT15_FIFO_data[i][1])&0xF0)>>4); 
                                                  if((tag&0xf0)==0xc0)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            if (ADC_count >= 32768)ADC_count = ADC_count - 65536;     // 2s comp
                                                            Temperature_T = (double)ADC_count*0.005; 
                                                            pc.printf("Temp=%2.1f ",Temperature_T); 
                                                          }   
                                                  if((tag&0xf0)==0x0)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            Current1= (double)ADC_count*0.0000152588*2000-0;//(0.1*2000);
                                                            pc.printf("cur1=%4.0fnA ",Current1); 
                                                          }   
                                                  if((tag&0xf0)==0x10)
                                                          {     
                                                            ADC_count = (int)(OT15_FIFO_data[i][1]*256 + OT15_FIFO_data[i][2]);
                                                            Current2= (double)ADC_count*0.0000152588*2000-0;//(0.1*2000);
                                                            pc.printf("cur2=%4.0fnA ",Current2); 
                                                          }   
                                                                                                                    
                                                   x = (OT15_read_reg(0x68)&0x06)>>1;
                                                        switch(x){
                                                                  case 0: Vref = 1536;break;
                                                                  case 1: Vref = 2048;break;
                                                                  case 2: Vref = 3072;break;
                                                                  case 3: Vref = 4096;break;
                                                                  }      
                                                   // pc.printf("                          VREF ==== %d mV\r\n",Vref);                                                                    
                                                   if(tag==0xd1)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WE1=%4.0fmV ",WE1); 
                                                          }   
                                                                                                                            
                                                    if(tag==0xd2)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            RE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("RE1=%4.0fmV ",RE1); 
                                                          }   
                                                   if(tag==0xd3)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            CE1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("CE1=%4.0fmV ",CE1); 
                                                          }     
                                                   if(tag==0xd0)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WO1= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WO1=%4.0fmV ",WO1); 
                                                          }     
                                                   if(tag==0xd5)
                                                          {     
                                                            ADC_count = (int)((OT15_FIFO_data[i][1] & 0x0f)*256 + OT15_FIFO_data[i][2]);
                                                            WE2= (double)ADC_count*0.000244140625*Vref*2;
                                                            pc.printf("WE2=%4.0fmV ",WE2); 
                                                          }                                                             
                                                                                                                                                                                     
                                                  
                                         } 
                                        Resister=(WE1-RE1)/ Current1;  
                                        pc.printf("Res1=%7.3f MOhm ",Resister);   
                                        Resister=(WE2-RE1)/ Current2;  
                                        pc.printf("Res2=%7.3f MOhm\r\n",Resister);      
                                        */                                    
                                   }         
                                break;     
                         
                       
                    }//end switch(c)
                }//if(n>0)             
            }//end if(CR)           
            if(rx_buff[rx_index] == BS){//backspace received, back up buffer pointer                                
                 if(rx_index>0)rx_index--;//remove last char from buffer if not at start.                 
            }else rx_index++;                                      
        }//end while(pc.redable())         
    }// end while(1) 
}// end main