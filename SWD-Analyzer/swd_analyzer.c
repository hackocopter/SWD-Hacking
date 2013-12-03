
/*
    SWD Analyzer v0.3
    
    Dec 2, 2013 cpldcpu
    based on code by chris
   
    http://www.mikrocontroller.net/topic/309185    
    
    Based on:
    http://sourceforge.net/apps/mediawiki/stm32primer2swd/index.php?title=Main_Page#SWD_Packet_Construction
    http://www.pjrc.com/arm/pdf/doc/ARM_debug.pdf
*/

#define VERSION_MINOR 3
#define VERSION_MAJOR 0

#include <stdio.h>
#include <string.h>
#include <stdint.h>

typedef unsigned char 		byte;
typedef unsigned short 		word;
typedef unsigned int 		dword;
typedef unsigned long long 	qword;

FILE             *inputfile;
int              samplecount,totalbits,failbits;
unsigned char    lastbyte;
int              firstedge;
double           timerorigin;
double           timestamps[64];
int              minperiod; // minimum clk period
int              verbose;
int              translateregacc;

#define     CLKMASK     0x02
#define     DATMASK     0x01
#define     SAMPLERATE  16000000

#define     SWD_RESETCOND   0xFFFFFFFFFFFFC000
#define     IDLETHRESHOLD   10               // in ms

unsigned int parity(unsigned int dat) 
{ 
    int ret=0;  
    for(;dat;dat>>=1) ret+=dat&1;  
    return ret&1;
}


qword lsb(char len, char offset, qword dat) 
{
 qword ret=0;
 char i;
 while(len--) if(dat&((qword)1<<(63-offset-len))) ret|=(qword)1<<len;
 return ret;
}


#define tblsize(tbl)	(sizeof(tbl)/sizeof(tbl[0]))

//char *dap_id[]={ "IDCODE","ABORT","CTRL/STAT","DP-CTRL","WIRE-CTRL","WIRE-CTRL","READ_RESENT","SELECT","BUFFER","BUFFER"};  // Debugport registers
char *dap_id[]={ "IDCODE","ABORT","CTRL/STAT","CTRL/STAT","SELECT","SELECT","READBUFFER","ERR (RDBUFFER)"};  // Debugport registers

char *ack_t []={ "ERROR(0)","OK   (1)","WAIT (2)","UNDEF(3)","FAULT(4)","UNDEF(5)","UNDEF(6)","ERROR(7)"};  // acknowledgements


char *ap_0 []={ "CSW - Control/Status", "TAR - Transfer Address" , "RESERVED", "DRW - Data R/W" };
char *ap_1 []={ "BD0 - Banked Data 0"  , "BD1 - Banked Data 1" , "BD2 - Banked Data 2" ,"BD3 - Banked Data 3" };
char *ap_ff[]={ "RESERVED","CFG - Configuration","BASE - Base Address","IDR - Identification"};

// start - DAP/AP - W/R - adr[2:3] - Parity - Stop - Park - Trn - ACK W - Ack R - ACK Busy - Trn - Data32 - Parity --
//   1		X		X		XX         X       	0		1	 Z     X         X        X       Z     X32        X

int swd(qword dataRE,qword dataFE) 
{
 byte header,ack,ret=0,t,addr,read,dap;
 
 unsigned  int dat,par;
 unsigned  int datre,parre; 
 char*id="";
 int       datavalid;

// Emulate MEM-AP states
 static         byte bank;
 static unsigned int TAR;
 static unsigned int LastBD;
 static unsigned int Addrinc;    // TAR Autoincrement  00=off 01=increment on each read/write (Bit 4 and 5 of CSW-Reg
 
 header = dataRE>>(64-8);
 dap	= lsb(1,1,dataRE);
 read 	= lsb(1,2,dataRE);
 addr	= lsb(2,3,dataRE); 
 //ack 	= lsb(3,8+1,dataFE);
 ack 	= lsb(3,8,dataFE);
 
 // Note: The target data is sampled on the falling edge of the _previous_ clock cycle. This
 // is to avoid glitches due to cheap logic analyzers that are unable to trigger on the clock 
 // signal. However, this method may fail at high SWD frequencies, when logic delays add up
 // to a sum equal to or greater than half a clock cycle.
 
 if (read==1)
 {          // Read from target Trn at end
    dat	= lsb(32,8+3,dataFE);       // one clock cycle earlier
    par	= lsb(1,8+3+32,dataFE);
  } else
 {         // write to target Trn between ack and data
    dat	= lsb(32,8+4+1,dataRE);
    par	= lsb(1,8+4+1+32,dataRE);
    datre=0;
    parre=0;
 }
 
 if (verbose>2)
 {
    int i;
    printf("@%fms: RE=",timestamps[0]);
    for (i=0; i<(8+4+1+32+1); i++) 
    {
        if (i==8||i==8+4||i==9||i==8+4+(read?33:1)) printf(" ");
        printf("%i",!!(dataRE&((long long)1<<(63-i))));        
    }
    printf("\n");
    printf("@%fms: FE=",timestamps[0]);
    for (i=0; i<(8+4+1+32+1); i++) 
    {
        if (i==8||i==8+4||i==9||i==8+4+(read?33:1)) printf(" ");
        printf("%i",!!(dataFE&((long long)1<<(63-i))));        
    }
    printf("\n");
 }
 
 if(!dap) 				id=dap_id[(addr<<1)|(!read)];
 else if(bank==0) 		id=ap_0 [addr];
 else if(bank==1) 		id=ap_1 [addr];
 else if(bank==0xf) 	id=ap_ff[addr];

 
    if (verbose>0)
    {
        printf("@%fms: %s-",timestamps[0],ack_t[ack]);
        
        datavalid=1;
        
        if (ack==7||ack==2||ack==4) datavalid=0;    // read acccess failed, data will not be valid
           
        if (datavalid)
        {
            printf("%s-%s at ADR=%i DAT=%.8x -", dap?"Access port":"Debug port ",read?"Read ":"Write",addr,dat);
         
            if (parity(dat)!=par) {printf("PAR!");} else {printf("   ");}
            printf(" %-12s ",id);  
            
        } else
        {
            printf("%s-%s at ADR=%i DAT=------------  ", dap?"Access port":"Debug port ",read?"Read ":"Write",addr);
        
            printf(" %-12s ",id);      
        }

          printf("\n");
    }
    
    //abort    
    if (ack==7)  return 8+4+1;
      
    // This is only true when "sticky overrun behavior" is not enabled.
    // TODO:Implement sticky
    
    if (ack==2||ack==4) return 8+4+1;
    
    // Emulate states
    // control register accecss
    
    if(!dap&&addr==2&&!read) bank=(dat>>4)&0xf; // bank selection for AP access , should i check parity error ??? 
    if (bank>1&&bank<15) printf("MEM-AP bank %i selection invalid?!?\n",bank);
    
    // TAR register write
    if (bank==0&&dap&&addr==1&&!read) TAR=dat;
    
    // BD access
    if (bank==1&&dap) LastBD=addr;
 
    // Access to CSW
    if (bank==0&&dap&&addr==0&&!read) 
    {
            // TAR Autoincrement  00=off 01=increment on each read/write (Bit 4 and 5 of CSW-Reg
            Addrinc=(dat>>4)&3;            
            if (verbose>1) printf("Addirc set to:%i%\n",Addrinc);            
     }
     
     
    if (translateregacc) {    
        // Write to banked register    
        if (bank==1&&dap&&!read) printf("@%fms:Write BD (0x%.8x)=0x%.8x\n",timestamps[0],(TAR&0xFFFFFFF0)+LastBD*4,dat);
        // Write via DRW
        if (bank==0&&dap&&addr==3&&!read) printf("@%fms:Write DRW(0x%.8x)=0x%.8x\n",timestamps[0],(TAR&0xFFFFFFFF),dat);
        // Read via DRW
        if (bank==0&&dap&&addr==3&&read) printf("@%fms:Read  DRW(0x%.8x)=0x%.8x\n",timestamps[0],(TAR&0xFFFFFFFF),dat);
        
        // Read from readbuffer
        if (addr==3&&!dap&&read) printf("@%fms:Read  Buf(0x%.8x)=0x%.8x\n",timestamps[0],(TAR&0xFFFFFFF0)+LastBD*4,dat);
        
        
     }

     // Access to DRW
    if (bank==0&&dap&&addr==3) 
    {
        if (Addrinc==1) TAR+=4;
    }   

     // 
    
 return 8+4+1+32+1;
}


/*
    Read next clock cycle from input file.
    
    1) if last byte was not clk low, wait for clk low
    2) wait for rising edge, sample
    3) wait for falling edge, sample
    
    return value
    
    -1       error
    Bit 1    SWDIO on rising edge
    Bit 0    SWDIO on falling edge    
*/

int getnextperiod(void)
{
    int bitsout=0;
    int bitcount=0;
    unsigned char nextbyte=0;
    int clk=0;
    
    // Wait for low
    while (lastbyte&CLKMASK)
    {
        lastbyte=fgetc(inputfile);
        samplecount++;
        if (feof(inputfile)) return -1;
    }

    
    while (bitcount<2)
    {
        clk++;
        nextbyte=fgetc(inputfile);
        samplecount++;
        if (feof(inputfile)) return -1;
        
        
        if ((lastbyte^nextbyte)&CLKMASK)        // detect clk transition
        {
        
            if (!firstedge) {
                firstedge=1;
                if (verbose>0) printf("@%f: (Samplenumber: %i) First CLK transition. Using this as origin.\n",(float)samplecount/SAMPLERATE,samplecount);
                timerorigin=(double)samplecount/SAMPLERATE;
            }
            bitsout=(bitsout<<1) | (!!(nextbyte&DATMASK));            
            bitcount++;        
            lastbyte=nextbyte;                      
        }            
    }
    
    if (clk<minperiod) {minperiod=clk;}
    
    return bitsout;
}

int main(int argc,char**argv) 
{ 
    verbose=2;
    translateregacc=1;

    // 4 print everything
    // 3 dump raw packet data if valid
    // 2 show reset and idle times as well
    // 1 clk data, invalid packets
    // 0 only valid packets (omit IDCODE hammering etc.)
        
    if (argc<2) 
    {
        printf("Fatal error: no input file.\n");        
        return -1;
    }
    if (!strcmp(argv[1],"--help"))
    {
        printf("SWD Protocol Analyzer v%i.%i\n\n",VERSION_MAJOR,VERSION_MINOR);
        printf("Usage: swd_analzyer [options] file...\n");
        printf("Options:\n");
        printf("  --help\t\tPrint this information\n");
        printf("\n\nThe input file contains a raw dump of the SWD bus traffic. Each sample is one byte SWDIO=bit 0, SWDCLK=bit 1.\n");
        return 0;
    }

    if (!(inputfile=fopen(argv[1],"r")))
    {
        printf("Fatal error: could not open input file.\n");
        return -1;
    }

    printf("SWD Protocol Analyzer v%i.%i\n\n",VERSION_MAJOR,VERSION_MINOR);
    printf("Analyzing File: %s\n",argv[1]);
    printf("=========================================================================\n");
    
    
    lastbyte=0;
    samplecount=0;
    totalbits=0;
    failbits=0;
    firstedge=0;
    minperiod=1<<30;
    
    
     int                     discardbits=64;
     unsigned long long      bufferRE=0,bufferFE=0;
     int                     inreset=0;   // In reset state?
     int                     totalerrorbits=0;
     int                     i;
     
     for (i=0; i<64; i++) timestamps[i]=0;
     
     while (1)
     {
        int inp;
        double oldtime,newtime;
        
        oldtime=timestamps[discardbits-1];
        
        while (discardbits-->0)
        {
            inp=getnextperiod();
            if (verbose>3) printf("%i.%i-",inp,discardbits);           
            bufferRE=(bufferRE<<1)|((inp&0x02)>>1);   // rising edge
            bufferFE=(bufferFE<<1)|(inp&0x01);   // falling edge            
            totalbits++;
            
            for (i=0; i<63; i++) timestamps[i]=timestamps[i+1];
            timestamps[63]=((double)samplecount/SAMPLERATE-timerorigin)*1000;            
        }
        newtime=timestamps[0];
        
        if ((newtime-oldtime)>IDLETHRESHOLD) 
        {
            if (verbose>1) printf("@%fms: Bus idle for %.4fms.\n",timestamps[0],(double)(newtime-oldtime));
        }
        
        if (inp==-1) break;
        
        
        // detect reset = 50 cycles while swdio=1
        if ((bufferRE&SWD_RESETCOND)==SWD_RESETCOND)
        {
            if (failbits) printf("Errorbits before reset: %i\n",failbits);
            totalerrorbits+=failbits;

            if (verbose>1) printf("@%fms: Reset found. Entering reset state.\n",timestamps[0]);
            discardbits=50;
            failbits=0;
            inreset=1;   
            continue;
        }
   
                // While in reset state only Idle states or JTAG-to-SWD are recognized.             
                // Quitting from reset state only with at least two idle states.
        if (inreset)
        {
                if ((bufferRE>>(64-16))==0x79E7)    // Found magic SWJ-DP code
                {
                    if (verbose>1) printf("@%fms: SWJ-DP Switch\n",timestamps[0]); 
                    discardbits=16;
                    continue;
                }
                
                // Nu-link sends 4 idle cycles
                // ST-Link sends 2 idle cycles
                  
                if (!(bufferRE>>63))        // found idle cycle. Strip all idle cycles and quit reset state
                {
                    int idlecycles=0;
                    unsigned long long buftemp=bufferRE;
                    while (!(buftemp>>63)) {buftemp<<=1; idlecycles++;}
                    
                      if (verbose>1) 
                      {     
                            printf("@%fms: %i Idle cycles. Quitting reset state.\n",timestamps[0],idlecycles); 
                      }
                      else
                      {
                            printf("@%fms: SWD Reset occured.\n",timestamps[0]); 

                      }
                      inreset=0;
                      discardbits=idlecycles;
                    
                    continue;
                }
                
                discardbits=1;  // Just another reset cycle
                continue;
        }        
   
        // start - DAP/AP - W/R - adr[2:3] - Parity - Stop - Park - Trn - ACK W - Ack R - ACK Busy - Trn - Data32 - Parity --
        //   1		X		X		XX         P       	0		1	 Z     X         X        X       Z     X32        X
   
        unsigned long header=(bufferRE>>(64-8));
  
        // Only process data if parity is correct and both start and stop bit are found and total packet timing is below threshold!
        // Header= 1abcdP01 where P=a^b^c^d
        
        
        if ((!parity(header&0x7c)) && ((header&0x83)==0x81) && ((timestamps[8+4+1-1]-timestamps[0])<IDLETHRESHOLD)) 
        {
              if (failbits) printf("Errorbits: %i\n",failbits);
              totalerrorbits+=failbits;
              failbits=0;
              
              if (verbose>3) printf("Valid Header %X close to: %i (%f s) - Park %i - errorbits: %i - payload %X/%X\n",header,samplecount,(float)samplecount/SAMPLERATE,!!(header&1),failbits,(unsigned int)(bufferRE>>24),(unsigned int)(bufferFE>>24));        
                        
              discardbits=swd(bufferRE,bufferFE);
              continue;                    
        }
               
        // No idea what is going on, lets just read another bit and increase biterror count.
        failbits++;
        discardbits=1;       
     }
   
    printf("===============================================================\n");
    printf("Final statistics:\n");
    printf("Total samples analyzed:%i\nTotal sample time:%fms\n",samplecount, ((float)samplecount/SAMPLERATE-timerorigin)*1000);
    printf("Total number of bits extracted: %i\n",totalbits/2);
    printf("Total number of bit errors:%i\nFraction of biterrors %f%%\n",totalerrorbits,(float)totalerrorbits*100/totalbits/2);
        
    printf("Shortest clock cycle in number of samples:%i\n",minperiod);
    printf("Maximum SWD CLK frequency: %f kHz\n",(float)SAMPLERATE/minperiod/1000);

    fclose(inputfile);
}

