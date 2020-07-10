#include "Arduino.h"
#include "RoboMotherMINI.h"
#include "ururuMD.h"
#include "MoveBase.h"
#include "Omuni4.h"
#include "Autoprocessing.h"
#include "daigaku.h"
#include "math.h"
#include "BTSerial.h"
#include "I_encoder.h"
#include "PID.h"
//Setup ****************************************************************************
#define Terminal Serial//Serial or BTSerial
#define BTSNAME "BAuto"  //Bluetooth Serial name
#define pointnum 8
#define L 394.5 //[mm]
#define R 51.0  //[mm]
#define E_range 15//Error range
//Don't care ***********************************************************************
#define x 0
#define y 1
#define END 2
#define red 1
#define blue 0
#define none 0
#define t_end 1
#define DRY 2
#define d_stby 3
//**********************************************************************************
//location**************************************************************************
uint8_t pnum=0;
//point[Field type(red or blue)][point number(0~)][x or y or END]
//End with point[Field type(red or blue)][point number(0~)][END]==1 
/*
double point[2][pointnum][3]=
{
  {//blue
  {0,0,0},
  {7450,0,0},
  {7450,800,1},//start
  {6900,800,0},
  {6900,3600,0},
  {7450,3600,1},//dry
  {7450,800,0},
  {0,0,1}//end
  },
  {//red
  {0,0,0},
  {-7450,0,0},
  {-7450,800,1},//start
  {-6900,800,0},
  {-6900,3600,0},
  {-7450,3600,1},//dry
  {-7450,800,0},
  {0,0,1}//end
  }
};
double point[2][pointnum][3]=
{
  {//blue
  {0,0,none},
  {3700,0,none},
  {3700,1140,t_end},//start end
  {4310,1140,d_stby},//dry stand by
  {4310,3940,DRY},//dry
  {3700,1140,none},//dry end
  {3700,800,none},
  {0,0,t_end}//end end
  },
  {//red
  {0,0,none},
  {-3700,0,none},
  {-3700,1140,t_end},//start end
  {-4350,1140,d_stby},//dry stand by
  {-4350,3500,DRY},//dry
  {-3700,3500,t_end},//dry end
  {-3700,800,none},
  {0,0,t_end}//end end
  }
};//test1
*/
double point[2][pointnum][3]=
{
  {//blue
  {0,0,none},
  {3700,0,none},
  {3700,1140,t_end},//start end
  {4310,1140,d_stby},//dry stand by
  {4310,3940,DRY},//dry
  {3700,1140,none},//dry end
  {3700,800,none},
  {0,0,t_end}//end end
  },
  {//red
  {0,0,none},
  {-3700,0,none},
  {-3700,1211.5,t_end},//start end
  {-4452.5,1211.5,d_stby},//dry stand by
  {-4452.5,3888.5,DRY},//dry
  {-3700,3888.5,t_end},//dry end
  {-3700,800,none},
  {0,0,t_end}//end end
  }
};//test
double location[2]={0,0};//x,y
uint8_t Fieldtype=red;
inline const char* ftype(){return (Fieldtype==red)?"red":"blue";}
//object****************************************************************************
Autoprocessing Auto;
Installation_encoder enc(&CAN0,CAN_INT);
PID pidx(8.5E-3,0,0,-3.0,3.0,0.02);
PID pidy(8.5E-3,0,0,-3.0,3.0,0.02);
Timer t[3];
//variable*************************************************************************
double Vx, Vy, Angular;
uint8_t sol=0b00000000,sw[SW_BYTE * 8]={0};
String printdata,solstatus;//debug
double speed[4]={0};
bool setgain=false,gsetup=false,reset=false,gp=false;
int8_t motorsel,gadd=0;
enum gain{KP,KI,KD};
enum gain gset;
uint8_t LED[2]={0};
//function*************************************************************************
void SPI_task(void *arg);
void gainsetup(MD *md[]);
bool soltask(Daigaku *solenoid);
void debug();
bool Start();
bool Dry(); 
bool End();

void SPI_task(void *arg)//SPI communication function
{
  //variable
  bool solt=false;
  //spi setup
  SPI.begin(SCLK, MISO, MOSI, -1);
  Daigaku solenoid(DEV10,MISO);//solenoid valve and switch
  CAN_Init();//CAN
  //SPI.beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));
  MD *md[4];//motor
  MoveBase *wheel;//motor
  for(int i=4; i<=7; i++){md[i-4]=new ururuMD(DEV[i],SPEED_MODE);}//motor
  wheel = new Omuni4(md);//motor
  for(int i=0;i<4;i++){((ururuMD*)md[i])->Kp=910.0;((ururuMD*)md[i])->Ki=5600.0;((ururuMD*)md[i])->Kd=0.1;}//default gain
  portTickType lt = xTaskGetTickCount();
  while(1)
  {
    solenoid.shift();
    for(int i=0;i<8;i++)printf("%d",solenoid.sw_status[i]);//sw[i]=solenoid.sw_status[i];
    printf("/%f,%f\n",location[x],location[y]);
    enc.readlocation(location);
    enc.communication();
    vTaskDelayUntil(&lt,1/portTICK_RATE_MS);
  }
}

void setup() {
  Serial.begin(115200);
  BTSerial.begin(BTSNAME);

  pinMode(POWER_CONTROL,OUTPUT);

  Auto.setfunction("starttask",Start);
  Auto.setfunction("drytask",Dry);
  Auto.setfunction("endtask",End);
  Auto.setfunction(debug);
  Auto.stop=true;

  printdata=solstatus= " ";
  xTaskCreatePinnedToCore(SPI_task,"spi_task",8096,NULL,1,NULL,0);
  // put your setup code here, to run once:
}

void loop() 
{
  Auto.run();
  delay(10);
  // put your main code here, to run repeatedly:
}
void gainsetup(MD *md[])
{
  switch(gset)
  {
    case KP:
      ((ururuMD*)md[motorsel])->Kp+=gadd;
      break;
    case KI:
      ((ururuMD*)md[motorsel])->Ki+=gadd;
      break;
    case KD:
      ((ururuMD*)md[motorsel])->Kd+=gadd*0.01;
      break;
  }
  gadd=0;
  for(int i=0;i<4;i++)
  {
    if(i==0)printdata=String("md["+String(i)+"]/"+String(((ururuMD*)md[i])->Kp)+"/"+String(((ururuMD*)md[i])->Ki)+"/"+String(((ururuMD*)md[i])->Kd)+"/\n\r");
    else printdata=String(printdata+"md["+String(i)+"]/"+String(((ururuMD*)md[i])->Kp)+"/"+String(((ururuMD*)md[i])->Ki)+"/"+String(((ururuMD*)md[i])->Kd)+"/\n\r");
  }
  if(setgain){for(int i=0;i<4;i++){((ururuMD*)md[i])->SetGain();}setgain=false;}
  gp=true;
}

bool soltask(Daigaku *solenoid)
{
  static bool sol3=false;
  solstatus="sol";
  solenoid->send_data[7]=LED[0];
  solenoid->send_data[6]=LED[1];
  for(int i=0;i<8;i++)
  {
    switch((int)(~sol)&1<<i)
    {
      case 0b10000000:solenoid->send_data[0]=0;break;
      case 0b01000000:solenoid->send_data[1]=0;break;
      case 0b00100000:solenoid->send_data[2]=0;sol3=false;break;
      case 0b00010000:solenoid->send_data[3]=0;break;
      case 0b00001000:solenoid->send_data[4]=0;break;
      case 0b00000100:solenoid->send_data[5]=0;break;
      //case 0b00000010:solenoid->send_data[6]=0;break;
      //case 0b00000001:solenoid->send_data[7]=0;break;
    }
  }
  for(int i=0;i<8;i++)
  {
    switch((int)sol&1<<i)
    {
      case 0b10000000:solenoid->send_data[0]=1;solstatus=String(solstatus+",1");break;
      case 0b01000000:solenoid->send_data[1]=1;solstatus=String(solstatus+",2");break;
      case 0b00100000:solenoid->send_data[2]=1;solstatus=String(solstatus+",3");sol3=true;break;
      case 0b00010000:solenoid->send_data[3]=1;solstatus=String(solstatus+",4");break;
      case 0b00001000:solenoid->send_data[4]=1;solstatus=String(solstatus+",5");break;
      case 0b00000100:solenoid->send_data[5]=1;solstatus=String(solstatus+",6");break;
      //case 0b00000010:solenoid->send_data[6]=1;solstatus=String(solstatus+",7");break;
      //case 0b00000001:solenoid->send_data[7]=1;solstatus=String(solstatus+",8");break;
    }
  }
  solstatus=String(solstatus+"/");
  return sol3;
}
void debug()
{
  //variable
  static int sdata=0;//Serial data
  static bool sol1=false;
  //location update
  enc.readlocation(location);
  //control
  if(Fieldtype==red)LED[1]=1;
  else LED[1]=0;
  if(sw[7]&&sol1==false){sol1=true;LED[0]=~(LED[0]^0);}
  if(sol1)
  {
    if(t[2].stand_by(0.5)){sol=~(sol^0b01111111);sol1=false;}
  }
  if(Serial.available())sdata = Serial.read();//usb
  else if(BTSerial.available())sdata = BTSerial.read();//bluetooth
  else sdata=0;
  if(sw[6])sdata='q';
  switch(sdata)
  {
    //field setup
    case 'b':Fieldtype=blue;break;
    case 'r':Fieldtype=red;break;
    case '^':enc.setlocation((float)point[Fieldtype][pnum][x]-location[x],(float)point[Fieldtype][pnum][y]-location[y]);break;
    //Autoprocessing switch
    case 'o':Auto.stop=true;break;//Stop processing
    case 'f':Auto.stop=false;break;
    //emergency stop
    case 'e':
      if(em==true)em=false;
      else em=true;
      break;
    //all reset
    case 'q':reset=true;break;
    //gain setup
    case 'S':setgain=true;break;//gain write
    case '0':motorsel=0;break;//select MD[0]
    case '1':motorsel=1;break;//select MD[1]
    case '2':motorsel=2;break;//select MD[2]
    case '3':motorsel=3;break;//select MD[3]
    case 'P':gset=KP;break;//Kp gain setup mode
    case 'I':gset=KI;break;//ki gain setup mode
    case 'D':gset=KD;break;//Kd gain setup mode
    case ',':gadd=+1;break;//add to gain 
    case '.':gadd=-1;break;//subtract to gain
    case 'g'://gain setup mode switch
      if(gsetup==true)gsetup=false;
      else gsetup=true;
      break;
    //move
    case 'w':Vy+=0.1;break;
    case 's':Vy-=0.1;break;
    case 'a':Vx-=0.1;break;
    case 'd':Vx+=0.1;break;
    case 'z':Angular-=0.1;break;
    case 'x':Angular+=0.1;break;
    case '/':sol=0b11111111;break;//all solenoid valve open
    case ' ':Vx=Vy=Angular=0.0;break;//stop
    default :break;
  }
  if(reset)sol1=false;
  if(Auto.stand_by(0.05))//Displayed at 0.05 second intervals
  {
    /*
    if(gp&&gsetup)
    {
      Serial.print(printdata);
      Serial.printf("MD[%d]-------------------------------------\n\r",motorsel);
      //Terminal.print(printdata);
      printdata=" ";
      gp=false;
    }
    Terminal.printf("/em%d/fild %s/task %s/Vx=%3.2lf,Vy=%3.2lf,%3.2lf,%d(%.1lf,%.1lf)/NOW(%.1lf,%.1lf)/",em,ftype(),Auto.status(),Vx,Vy,Angular,pnum,point[Fieldtype][pnum][x],point[Fieldtype][pnum][y],location[x],location[y]);
    Terminal.print(solstatus);
    //for(int i=0;i<4;i++)Terminal.printf("md[%d]=%.2f[rps]/",i,speed[i]);
    for(int i=0;i<8;i++)Terminal.printf("%d",sw[i]);
    Terminal.printf("test(%.2f,%.2f)",(float)point[Fieldtype][pnum][x]-location[x],(float)point[Fieldtype][pnum][y]-location[y]);
    Terminal.printf("\n\r");*/
  }
  if(em)e_stop(e_OFF);
  else e_stop(e_ON);
}
//Auto processing*******************************************************************
bool Start()
{
  bool end=false;
  if((location[x]>point[Fieldtype][pnum][x]-E_range&&location[x]<point[Fieldtype][pnum][x]+E_range)&&(location[y]>point[Fieldtype][pnum][y]-E_range&&location[1]<point[Fieldtype][pnum][y]+E_range))
  {
    if(point[Fieldtype][pnum][END]==t_end)end=true;
    pnum++;
    Vx=Vy=Angular=0.0;
  }
  else
  {
    Vx=pidx.output(point[Fieldtype][pnum][x],location[x]);
    Vy=pidy.output(point[Fieldtype][pnum][y],location[y]);
  }
  return end;
}
bool Dry()
{
  static bool fin=false;
  bool end=false;
  if(((location[x]>point[Fieldtype][pnum][x]-E_range&&location[x]<point[Fieldtype][pnum][x]+E_range)&&(location[y]>point[Fieldtype][pnum][y]-E_range&&location[1]<point[Fieldtype][pnum][y]+E_range))||fin)
  {
    if(point[Fieldtype][pnum][END]==t_end)end=true;
    if(fin){enc.setlocation((float)point[Fieldtype][pnum][x]-location[x],(float)point[Fieldtype][pnum][y]-location[y]);fin=false;sol=0b11000000;}
    if(point[Fieldtype][pnum][END]==DRY)sol=0b10000000;
    pnum++;
    Vx=Vy=Angular=0.0;
  }
  else
  {
    if(point[Fieldtype][pnum][END]==d_stby)
    {
      sol=0b11000000;
    }
    Vx=pidx.output(point[Fieldtype][pnum][x],location[x]);
    if(point[Fieldtype][pnum][END]==DRY)//dry
    {
      static bool set=false;
      if(sw[0]==1)set=true;
      
      if(set)
        Vy=1.0;
      else
        Vy=-1.0;
      if(sw[1]==1)fin=true;
    }
    else
      Vy=pidy.output(point[Fieldtype][pnum][y],location[y]);
  }
  return end;
} 
bool End()
{
  bool end=false;
  if((location[x]>point[Fieldtype][pnum][x]-E_range&&location[x]<point[Fieldtype][pnum][x]+E_range)&&(location[y]>point[Fieldtype][pnum][y]-E_range&&location[1]<point[Fieldtype][pnum][y]+E_range))
  {
    if(point[Fieldtype][pnum][END]==t_end)end=true;
    pnum++;
    Vx=Vy=Angular=0.0;
  }
  else
  {
    Vx=pidx.output(point[Fieldtype][pnum][x],location[x]);
    Vy=pidy.output(point[Fieldtype][pnum][y],location[y]);
  }
  return end;
}