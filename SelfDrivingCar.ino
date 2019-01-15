#include <LBLE.h>
#include <LBLEPeriphral.h>
#include <Servo.h>
#include <LWiFi.h>
#define centerTrigPin 2
#define centerEchoPin 3
#define leftTrigPin 4
#define leftEchoPin 5
#define rightTrigPin 6
#define rightEchoPin 7
#define r_Pin 8
#define g_Pin 16
#define b_Pin 17
#define leftEnable 10
#define leftpin1 11
#define leftpin2 12
#define rightpin1 13
#define rightpin2 14
#define rightEnable 15
#define SSID "CSIE-WLAN-Sparq"
#define PASSWD "wificsie"
#define TCP_IP "192.168.209.31"
#define TCP_PORT 5000
int value=0;
typedef struct{ // -1 牆  1路   0擋
  int up;
  int down;
  int left;
  int right;
  }node;

node maze[8][8];
char orientation;
bool reach_End = false;
bool route_Set = false;

int last_X = -10;
int last_Y = -10;
int current_X = -10;
int current_Y = -10;
int end_X = 0;
int end_Y = 0;
int start_X;
int start_Y;
float l_value[10]; 
float r_value[10];
float c_value[10];
float d_value[9];
float c_dis,l_dis,r_dis;
int adjust_point = 0;


WiFiClient wifiClient;
static char buf[32];
int i = 0;
int j = 1;
void setup() {
  //intialize
  orientation = 'S';
  initialize();
  
  //wifi connect
  Serial.begin(115200);
  while (!Serial);
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(SSID, PASSWD);
  }
  wifiClient.connect(TCP_IP, TCP_PORT);    
  Serial.println("connect wifi");
  Serial.print("status:");
  Serial.println(status);
  delay(1000);
  wifiClient.write("join Carrr B");
  //pin set
  pinMode(r_Pin,OUTPUT);
  pinMode(g_Pin, OUTPUT);
  pinMode(b_Pin, OUTPUT);
  pinMode(rightpin1,OUTPUT);
  pinMode(rightpin2,OUTPUT);
  pinMode(leftpin1,OUTPUT);
  pinMode(leftpin2,OUTPUT);
  pinMode(rightEnable,OUTPUT);
  pinMode(leftEnable,OUTPUT);
  digitalWrite(rightEnable,HIGH);
  digitalWrite(leftEnable,HIGH);
  analogWrite(r_Pin,0);
  analogWrite(g_Pin,0);
  analogWrite(b_Pin,255);
  
}

void loop() {
      if(value==0){
      current_X=-10;
      current_Y=-10;
      char start_Area;
      char end_Area;
      if(!reach_End){
        wifiClient.write("position");
        while (wifiClient.available()){
          buf[i++] = wifiClient.read();
          delayMicroseconds(10);    
        }     
        if (i != 0) {
          buf[i] = '\0';
          if(buf[0]=='p'){
            current_X = buf[9]-'0';
            current_Y = buf[11]-'0';
          }
        }
        i = 0;
        Serial.print("position    ");
        Serial.print(current_X);
        Serial.print("  ");
        Serial.println(current_Y);  
        if(current_X<=7&&current_X>=0&&current_Y<=7&&current_Y>=0){
          if(end_X==current_X&&end_Y==current_Y){
            reach_End = true;
            wifiClient.write("send-to CCCCC go");
            value = 2;              
          }
          if(!route_Set){
            start_X = current_X;
            start_Y = current_Y;
            start_Area = Area_Check(start_X,start_Y);
            end_Area = Area_Check(end_X,end_Y);   
            Route_Arrange(start_Area,end_Area);
            route_Set = true;  
          }
          else{
            Route_Check();
            Route_Travel();
          }   
        }
      }
    } 
    delay(500);
}

float get_dis(int trig,int echo){
  float distance;
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  distance = pulseIn(echo,HIGH,5000000);
  return distance/29/2;
}
void initialize() {  
  maze[0][0].up = 1;
  maze[0][0].down = -1;
  maze[0][0].left = -1;
  maze[0][0].right = 1; 

  maze[0][1].up = 1;
  maze[0][1].down = 1;
  maze[0][1].left = -1;
  maze[0][1].right = -1;

  maze[0][2].up = 1;
  maze[0][2].down = 1;
  maze[0][2].left = -1;
  maze[0][2].right = -1;

  maze[0][3].up = 1;
  maze[0][3].down = 1;
  maze[0][3].left = -1;
  maze[0][3].right = -1;

  maze[0][4].up = -1;
  maze[0][4].down = 1;
  maze[0][4].left = -1;
  maze[0][4].right = 1;

  maze[0][5].up = 1;
  maze[0][5].down = -1;
  maze[0][5].left = -1;
  maze[0][5].right = 1;

  maze[0][6].up = 1;
  maze[0][6].down = 1;
  maze[0][6].left = -1;
  maze[0][6].right = -1;

  maze[0][7].up = -1;
  maze[0][7].down = 1;
  maze[0][7].left = -1;
  maze[0][7].right = -1;

  maze[1][0].up = 1;
  maze[1][0].down = -1;
  maze[1][0].left = 1;
  maze[1][0].right = -1;

  maze[1][1].up = -1;
  maze[1][1].down = 1;
  maze[1][1].left = -1;
  maze[1][1].right = 1;

  maze[1][2].up = 1;
  maze[1][2].down = -1;
  maze[1][2].left = -1;
  maze[1][2].right = 1;

  maze[1][3].up = 1;
  maze[1][3].down = 1;
  maze[1][3].left = -1;
  maze[1][3].right = -1;

  maze[1][4].up = -1;
  maze[1][4].down = 1;
  maze[1][4].left = 1;
  maze[1][4].right = -1;

  maze[1][5].up = 1;
  maze[1][5].down = -1;
  maze[1][5].left = 1;
  maze[1][5].right = 1;

  maze[1][6].up = 1;
  maze[1][6].down = 1;
  maze[1][6].left = -1;
  maze[1][6].right = -1;

  maze[1][7].up = -1;
  maze[1][7].down = 1;
  maze[1][7].left = -1;
  maze[1][7].right = 1;

  maze[2][0].up = 1;
  maze[2][0].down = -1;
  maze[2][0].left = -1;
  maze[2][0].right = -1;

  maze[2][1].up = -1;
  maze[2][1].down = 1;
  maze[2][1].left = 1;
  maze[2][1].right = -1;

  maze[2][2].up = 1;
  maze[2][2].down = -1;
  maze[2][2].left = 1;
  maze[2][2].right = 1;

  maze[2][3].up = 1;
  maze[2][3].down = 1;
  maze[2][3].left = -1;
  maze[2][3].right = -1;

  maze[2][4].up = 1;
  maze[2][4].down = 1;
  maze[2][4].left = -1;
  maze[2][4].right = -1;

  maze[2][5].up = -1;
  maze[2][5].down = 1;
  maze[2][5].left = 1;
  maze[2][5].right = 1;

  maze[2][6].up = -1;
  maze[2][6].down = -1;
  maze[2][6].left = -1;
  maze[2][6].right = 1;

  maze[2][7].up = -1;
  maze[2][7].down = -1;
  maze[2][7].left = 1;
  maze[2][7].right = 1;

  maze[3][0].up = 1;
  maze[3][0].down = -1;
  maze[3][0].left = -1;
  maze[3][0].right = 1;

  maze[3][1].up = 1;
  maze[3][1].down = 1;
  maze[3][1].left = -1;
  maze[3][1].right = -1;

  maze[3][2].up = -1;
  maze[3][2].down = 1;
  maze[3][2].left = 1;
  maze[3][2].right = 1;

  maze[3][3].up = 1;
  maze[3][3].down = -1;
  maze[3][3].left = -1;
  maze[3][3].right = 1;

  maze[3][4].up = -1;
  maze[3][4].down = 1;
  maze[3][4].left = -1;
  maze[3][4].right = 1;

  maze[3][5].up = 1;
  maze[3][5].down = -1;
  maze[3][5].left = 1;
  maze[3][5].right = 1;

  maze[3][6].up = -1;
  maze[3][6].down = 1;
  maze[3][6].left = 1;
  maze[3][6].right = -1;

  maze[3][7].up = -1;
  maze[3][7].down = -1;
  maze[3][7].left = 1;
  maze[3][7].right = -1;

  maze[4][0].up = -1;
  maze[4][0].down = -1;
  maze[4][0].left = 1;
  maze[4][0].right = 1;

  maze[4][1].up = 1;
  maze[4][1].down = -1;
  maze[4][1].left = -1;
  maze[4][1].right = -1;

  maze[4][2].up = -1;
  maze[4][2].down = 1;
  maze[4][2].left = 1;
  maze[4][2].right = 1;

  maze[4][3].up = -1;
  maze[4][3].down = -1;
  maze[4][3].left = 1;
  maze[4][3].right = 1;

  maze[4][4].up = -1;
  maze[4][4].down = -1;
  maze[4][4].left = 1;
  maze[4][4].right = -1;

  maze[4][5].up = 1;
  maze[4][5].down = -1;
  maze[4][5].left = 1;
  maze[4][5].right = 1;

  maze[4][6].up = -1;
  maze[4][6].down = 1;
  maze[4][6].left = -1;
  maze[4][6].right = 1;

  maze[4][7].up = -1;
  maze[4][7].down = -1;
  maze[4][7].left = -1;
  maze[4][7].right = 1;

  maze[5][0].up = 1;
  maze[5][0].down = -1;
  maze[5][0].left = 1;
  maze[5][0].right = -1;

  maze[5][1].up = -1;
  maze[5][1].down = 1;
  maze[5][1].left = -1;
  maze[5][1].right = -1;

  maze[5][2].up = 1;
  maze[5][2].down = 1;
  maze[5][2].left = 1;
  maze[5][2].right = 1;

  maze[5][3].up = 1;
  maze[5][3].down = 1;
  maze[5][3].left = 1;
  maze[5][3].right = -1;

  maze[5][4].up = 1;
  maze[5][4].down = 1;
  maze[5][4].left = -1;
  maze[5][4].right = -1;

  maze[5][5].up = -1;
  maze[5][5].down = 1;
  maze[5][5].left = 1;
  maze[5][5].right = 1;

  maze[5][6].up = -1;
  maze[5][6].down = -1;
  maze[5][6].left = 1;
  maze[5][6].right = 1;

  maze[5][7].up = -1;
  maze[5][7].down = -1;
  maze[5][7].left = 1;
  maze[5][7].right = 1;

  maze[6][0].up = 1;
  maze[6][0].down = -1;
  maze[6][0].left = -1;
  maze[6][0].right = -1;

  maze[6][1].up = -1;
  maze[6][1].down = 1;
  maze[6][1].left = -1;
  maze[6][1].right = 1;

  maze[6][2].up = -1;
  maze[6][2].down = -1;
  maze[6][2].left = 1;
  maze[6][2].right = 1;

  maze[6][3].up = 1;
  maze[6][3].down = -1;
  maze[6][3].left = -1;
  maze[6][3].right = -1;

  maze[6][4].up = 1;
  maze[6][4].down = 1;
  maze[6][4].left = -1;
  maze[6][4].right = -1;

  maze[6][5].up = -1;
  maze[6][5].down = 1;
  maze[6][5].left = 1;
  maze[6][5].right = 1;
  
  maze[6][6].up = 1;
  maze[6][6].down = -1;
  maze[6][6].left = 1;
  maze[6][6].right = 1;

  maze[6][7].up = -1;
  maze[6][7].down = 1;
  maze[6][7].left = 1;
  maze[6][7].right = -1;

  maze[7][0].up = 1;
  maze[7][0].down = -1;
  maze[7][0].left = -1;
  maze[7][0].right = -1;

  maze[7][1].up = 1;
  maze[7][1].down = 1;
  maze[7][1].left = 1;
  maze[7][1].right = -1;

  maze[7][2].up = 1;
  maze[7][2].down = 1;
  maze[7][2].left = 1;
  maze[7][2].right = -1;

  maze[7][3].up = 1;
  maze[7][3].down = 1;
  maze[7][3].left = -1;
  maze[7][3].right = -1;

  maze[7][4].up = -1;
  maze[7][4].down = 1;
  maze[7][4].left = -1;
  maze[7][4].right = -1;

  maze[7][5].up = -1;
  maze[7][5].down = -1;
  maze[7][5].left = 1;
  maze[7][5].right = -1;

  maze[7][6].up = 1;
  maze[7][6].down = -1;
  maze[7][6].left = 1;
  maze[7][6].right = -1;

  maze[7][7].up = -1;
  maze[7][7].down = 1;
  maze[7][7].left = -1;
  maze[7][7].right = -1;
}
char Area_Check(int x,int y) {
  if(x==0){
    if(y==0){return 'A';}
    else if(y==1){return 'A';}
    else if(y==2){return 'A';}
    else if(y==3){return 'A';}
    else if(y==4){return 'A';}
    else if(y==5){return 'G';}
    else if(y==6){return 'G';}
    else if(y==7){return 'G';}
  }
  else if(x==1){
    if(y==0){return 'A';}
    else if(y==1){return 'A';}
    else if(y==2){return 'A';}
    else if(y==3){return 'A';}
    else if(y==4){return 'A';}
    else if(y==5){return 'G';}
    else if(y==6){return 'G';}
    else if(y==7){return 'G';}
  }
  else if(x==2){
    if(y==0){return 'A';}
    else if(y==1){return 'A';}
    else if(y==2){return 'H';}
    else if(y==3){return 'H';}
    else if(y==4){return 'H';}
    else if(y==5){return 'H';}
    else if(y==6){return 'F';}
    else if(y==7){return 'G';}
  }
  else if(x==3){
    if(y==0){return 'B';}
    else if(y==1){return 'B';}
    else if(y==2){return 'H';}
    else if(y==3){return 'I';}
    else if(y==4){return 'I';}
    else if(y==5){return 'H';}
    else if(y==6){return 'F';}
    else if(y==7){return 'G';}
  }
  else if(x==4){
    if(y==0){return 'B';}
    else if(y==1){return 'J';}
    else if(y==2){return 'H';}
    else if(y==3){return 'I';}
    else if(y==4){return 'I';}
    else if(y==5){return 'H';}
    else if(y==6){return 'E';}
    else if(y==7){return 'E';}
  }
  else if(x==5){
    if(y==0){return 'B';}
    else if(y==1){return 'B';}
    else if(y==2){return 'H';}
    else if(y==3){return 'H';}
    else if(y==4){return 'H';}
    else if(y==5){return 'H';}
    else if(y==6){return 'E';}
    else if(y==7){return 'E';}
  }
  else if(x==6){
    if(y==0){return 'C';}
    else if(y==1){return 'C';}
    else if(y==2){return 'C';}
    else if(y==3){return 'D';}
    else if(y==4){return 'D';}
    else if(y==5){return 'D';}
    else if(y==6){return 'E';}
    else if(y==7){return 'E';}
  }
  else if(x==7){
    if(y==0){return 'C';}
    else if(y==1){return 'C';}
    else if(y==2){return 'C';}
    else if(y==3){return 'C';}
    else if(y==4){return 'C';}
    else if(y==5){return 'D';}
    else if(y==6){return 'E';}
    else if(y==7){return 'E';}
  }
  else{return 'X';}   
}
void Route_Arrange(char start_Area,char end_Area){
  if(start_Area=='A'){
    if(end_Area=='A'){ maze[1][2].right = 0;}
    else if(end_Area=='B'){maze[2][2].up = 0; maze[3][2].right = 0;}
    else if(end_Area=='C'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0; maze[0][0].right = 0;
      if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}  
      else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
    }
    else if(end_Area=='D'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0;
      if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
    }
    else if(end_Area=='E'){maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;
      if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
      else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
    }
    else if(end_Area=='F'){maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].right = 0;}
    else if(end_Area=='G'){maze[2][2].right = 0; maze[2][5].right = 0;
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
    }
    else if(end_Area=='H'){
      if((end_X==2&&end_Y==2)||(end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[4][5].right = 0;
      }
    }
    else if(end_Area=='I'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].up = 0;}
    else if(end_Area=='J'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].right = 0;}
  }
  else if(start_Area=='B'){
    if(end_Area=='A'){maze[2][2].up = 0; maze[3][2].right = 0;}
    else if(end_Area=='B'){maze[3][1].up = 0;}
    else if(end_Area=='C'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].up = 0;
      if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;} 
      else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;} 
    }
    else if(end_Area=='D'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0;maze[5][5].left = 0;
      if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
    }
    else if(end_Area=='E'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;
      if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
      else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
    }
    else if(end_Area=='F'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0;}
    else if(end_Area=='G'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].right = 0;
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
    }
    else if(end_Area=='H'){
      if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[4][5].right = 0;
      }
    }
    else if(end_Area=='I'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].up = 0;}
    else if(end_Area=='J'){maze[3][2].left = 0; maze[4][2].right = 0;}
  }
  else if(start_Area=='C'){
    if(end_Area!='C'){
      if((start_X==6&&start_Y==0)||(start_X==6&&start_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(start_X==7&&start_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4)){maze[7][2].down = 0;}
      else if(start_X==7&&start_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      
      if(end_Area=='A'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0;}
      else if(end_Area=='B'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].up = 0;}
      else if(end_Area=='D'){maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].left = 0;
        if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
        else {maze[6][5].right = 0;}
      }
      else if(end_Area=='E'){maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].left = 0; maze[5][2].down = 0;
        if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
        else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
      }
      else if(end_Area=='F'){maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].left = 0;}
      else if(end_Area=='G'){maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0; maze[2][5].right = 0;
        if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
        else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
      }
      else if(end_Area=='H'){
        if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)){
          maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0; maze[2][5].left = 0; maze[2][5].right = 0;
        }
        else if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
          maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[3][5].left = 0;
        }
      }
      else if(end_Area=='I'){maze[5][2].left = 0; maze[5][3].up = 0;}
      else if(end_Area=='J'){maze[5][2].up = 0; maze[4][2].left = 0;}
    }
    if(end_Area=='C'){
      maze[6][2].left = 0;
      if(((start_X==6&&start_Y==0)||(start_X==6&&start_Y==1))&&(end_X==7&&end_Y==0)){maze[7][1].up = 0;}
      else if(((start_X==6&&start_Y==0)||(start_X==6&&start_Y==1))&&((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4))){maze[7][1].down = 0;maze[7][2].left = 0;}
      else if(((start_X==6&&start_Y==0)||(start_X==6&&start_Y==1))&&((end_X==6&&end_Y==2)||(end_X==7&&end_Y==1)||(end_X==7&&end_Y==2))){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(((start_X==6&&start_Y==0)||(start_X==6&&start_Y==1))&&((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1))){maze[6][1].right = 0;}
      
      else if(((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4))&&((end_X==6&&end_Y==2)||(end_X==7&&end_Y==2))){maze[7][2].down = 0;}
      else if(((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4))&&(end_X==7&&end_Y==1)){maze[7][2].left = 0; maze[7][1].down = 0; maze[7][1].left = 0;}
      else if(((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4))&&((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1))){maze[7][2].left = 0; maze[7][1].down = 0;}
      else if(((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4))&&(end_X==7&&end_Y==0)){maze[7][2].left = 0; maze[7][1].left = 0;}
      else if(((start_X==7&&start_Y==3)||(start_X==7&&start_Y==4))&&((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4))){maze[7][3].down = 0;}
      
      else if((start_X==7&&start_Y==0)&&((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4))){maze[7][1].left = 0;maze[7][2].left = 0;}
      else if((start_X==7&&start_Y==0)&&((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1))){maze[7][1].up = 0;}
      else if((start_X==7&&start_Y==0)&&((end_X==6&&end_Y==2)||(end_X==7&&end_Y==1)||(end_X==7&&end_Y==2))){maze[7][1].left = 0; maze[7][2].up = 0;}
      
      
      else if(((start_X==6&&start_Y==2)||(start_X==7&&start_Y==1)||(start_X==7&&start_Y==2))&&((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1))){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(((start_X==6&&start_Y==2)||(start_X==7&&start_Y==1)||(start_X==7&&start_Y==2))&&(end_X==7&&end_Y==0)){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if(((start_X==6&&start_Y==2)||(start_X==7&&start_Y==2))&&((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4))){maze[7][2].down = 0;}      
      else if((start_X==7&&start_Y==1)&&((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4))){maze[7][1].down = 0; maze[7][1].left = 0; maze[7][2].left = 0;}
      else if(((start_X==6&&start_Y==2)||(start_X==7&&start_Y==1)||(start_X==7&&start_Y==2))&&((end_X==6&&end_Y==2)||(end_X==7&&end_Y==1)||(end_X==7&&end_Y==2))){maze[7][1].down = 0; maze[7][1].left = 0; maze[7][2].up = 0;}
    }
  }
  else if(start_Area=='D'){
    if(end_Area!='D'){
      if(start_X==7&&start_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
      if(end_Area=='A'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0;}
      else if(end_Area=='B'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0;maze[5][5].left = 0;}
      else if(end_Area=='C'){maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].left = 0;
        if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
        else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
        else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
        else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
      }
      else if(end_Area=='E'){maze[5][5].down = 0; maze[4][5].left = 0;
        if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
        else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
      }
      else if(end_Area=='F'){maze[5][5].down = 0; maze[4][5].up = 0; maze[3][5].left = 0;}
      else if(end_Area=='G'){maze[5][5].down = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[2][5].down = 0;
        if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
        else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
      }
      else if(end_Area=='H'){
        if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==5)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)){
          maze[2][2].left = 0; maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].down = 0;
        }
        else if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)){
          maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[4][2].down = 0; maze[3][2].down = 0; maze[3][2].left = 0;
        }
      }
      else if(end_Area=='I'){maze[5][3].down = 0;maze[5][5].left = 0;}
      else if(end_Area=='J'){maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0;maze[5][5].left = 0;}
    }
    else if(end_Area=='D'){
      maze[6][5].left = 0;
    }
  }
  else if(start_Area=='E'){
    if(end_Area!='E'){
      if((start_X==4&&start_Y==7)||(start_X==5&&start_Y==7)||(start_X==6&&start_Y==7)){maze[6][6].right = 0;}
      else if((start_X==7&&start_Y==6)||(start_X==7&&start_Y==7)){maze[6][6].up = 0;}
      if(end_Area=='A'){maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;}
      else if(end_Area=='B'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;}
      else if(end_Area=='C'){maze[5][2].left = 0; maze[5][2].down = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].left = 0;
        if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
        else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
        else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
        else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
      }
      else if(end_Area=='D'){maze[5][5].down = 0; maze[4][5].left = 0;
        if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
        else {maze[6][5].right = 0;}
      }
      else if(end_Area=='F'){maze[4][5].right = 0; maze[3][5].left = 0;}
      else if(end_Area=='G'){maze[4][5].right = 0; maze[3][5].up = 0; maze[2][5].down = 0;
        if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
        else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
      }
      else if(end_Area=='H'){
        if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
          maze[3][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].left = 0; maze[5][5].right = 0;
        }
        else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)){
          maze[2][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;
        }
      }
      else if(end_Area=='I'){maze[5][3].down = 0; maze[4][5].left = 0; maze[5][5].right = 0;}
      else if(end_Area=='J'){maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].left = 0; maze[5][5].right = 0;}
    }
    else if(end_Area=='E'){
      maze[4][6].down = 0; 
      if(((start_X==4&&start_Y==7)||(start_X==5&&start_Y==7)||(start_X==6&&start_Y==7))&&((end_X==4&&end_Y==6)||(end_X==5&&end_Y==6)||(end_X==6&&end_Y==6))){maze[6][6].right = 0;}
      else if(((start_X==4&&start_Y==7)||(start_X==5&&start_Y==7)||(start_X==6&&start_Y==7))&&((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7))){maze[6][6].left = 0;}
      else if(((start_X==4&&start_Y==7)||(start_X==5&&start_Y==7)||(start_X==6&&start_Y==7))&&((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7))){maze[6][7].down = 0;}

      else if(((start_X==4&&start_Y==6)||(start_X==5&&start_Y==6)||(start_X==6&&start_Y==6))&&((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7))){maze[6][6].right = 0;}
      else if(((start_X==4&&start_Y==6)||(start_X==5&&start_Y==6)||(start_X==6&&start_Y==6))&&((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7))){maze[6][6].up = 0;}
      else if(((start_X==4&&start_Y==6)||(start_X==5&&start_Y==6)||(start_X==6&&start_Y==6))&&((end_X==4&&end_Y==6)||(end_X==5&&end_Y==6)||(end_X==6&&end_Y==6))){maze[6][6].up = 0; maze[6][6].right = 0;}

      else if(((start_X==7&&start_Y==6)||(start_X==7&&start_Y==7))&&((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7))){maze[6][6].left = 0;}
      else if(((start_X==7&&start_Y==6)||(start_X==7&&start_Y==7))&&((end_X==4&&end_Y==6)||(end_X==5&&end_Y==6)||(end_X==6&&end_Y==6))){maze[6][6].up = 0;}
      else if(((start_X==7&&start_Y==6)||(start_X==7&&start_Y==7))&&((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7))){maze[7][6].left = 0;}
    }  
  }
  else if(start_Area=='F'){
    if(end_Area=='A'){maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].right = 0;}
    else if(end_Area=='B'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0;}
    else if(end_Area=='C'){maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].left = 0;
      if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
      else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
    }
    else if(end_Area=='D'){maze[5][5].down = 0; maze[4][5].up = 0; maze[3][5].left = 0;
      if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
    }
    else if(end_Area=='E'){maze[4][5].right = 0; maze[3][5].left = 0;
      if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
      else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
    }
    else if(end_Area=='F'){maze[3][6].down = 0;}
    else if(end_Area=='G'){maze[3][5].right = 0; maze[2][5].down = 0;
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
    }
    else if(end_Area=='H'){
      if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[3][5].left = 0; maze[5][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].up = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)){
        maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[4][2].left = 0;
      }
    }
    else if(end_Area=='I'){maze[3][5].left = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].down = 0;}
    else if(end_Area=='J'){maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0; maze[3][2].down = 0; maze[4][2].right = 0;}
  }
  else if(start_Area=='G'){
    if(end_Area!='G'){
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
      if(end_Area=='A'){maze[2][2].right = 0; maze[2][5].right = 0;maze[1][5].up = 0; maze[0][0].right = 0;}
      else if(end_Area=='B'){maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].right = 0;}
      else if(end_Area=='C'){maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0; maze[2][5].right = 0;
        if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
        else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
        else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
        else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
      }
      else if(end_Area=='D'){maze[5][5].down = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[2][5].down = 0;
        if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
        else {maze[6][5].right = 0;}
      }
      else if(end_Area=='E'){maze[4][5].right = 0; maze[3][5].up = 0; maze[2][5].down = 0;
        if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
        else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
      }
      else if(end_Area=='F'){maze[3][5].right = 0; maze[2][5].down = 0;}   
      else if(end_Area=='H'){
        if((end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==5)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==2)){
          maze[2][5].down = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].left = 0; maze[5][2].left = 0; maze[5][2].right = 0;
        }
        else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)){
          maze[2][5].right = 0; maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[4][2].right = 0;
        }
      }
      else if(end_Area=='I'){maze[2][5].down = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].down = 0;}
      else if(end_Area=='J'){maze[2][2].left = 0; maze[2][5].right = 0; maze[3][2].down = 0; maze[4][2].right = 0;}
    }
    else if(end_Area=='G'){
        maze[1][5].right = 0;
        if(((start_X==1&&start_Y==6)||(start_X==1&&start_Y==7)||(start_X==2&&start_Y==7)||(start_X==3&&start_Y==7))&&((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7))){maze[1][5].right = 0;}
        else if(((start_X==1&&start_Y==6)||(start_X==1&&start_Y==7)||(start_X==2&&start_Y==7)||(start_X==3&&start_Y==7))&&((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7))){maze[1][6].down = 0;}
        else if(((start_X==0&&start_Y==5)||(start_X==0&&start_Y==6)||(start_X==0&&start_Y==7))&&((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7))){maze[1][5].right = 0;}
        else if(((start_X==0&&start_Y==5)||(start_X==0&&start_Y==6)||(start_X==0&&start_Y==7))&&((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7))){maze[0][5].right = 0;}
    }
  }
  else if(start_Area=='H'){
    if(end_Area=='A'){
      if((start_X==2&&start_Y==2)||(start_X==3&&start_Y==2)||(start_X==4&&start_Y==2)||(start_X==5&&start_Y==2)||(start_X==5&&start_Y==3)||(start_X==5&&start_Y==4)||(start_X==5&&start_Y==5)){
        maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[5][5].right = 0;
      }
      else if((start_X==2&&start_Y==3)||(start_X==2&&start_Y==4)||(start_X==2&&start_Y==5)||(start_X==3&&start_Y==5)||(start_X==4&&start_Y==5)){
        maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[4][5].right = 0;
      }
    }
    else if(end_Area=='B'){
      if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[3][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[4][5].right = 0;
      }
    }
    else if(end_Area=='C'){
      if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)){
          maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].up = 0; maze[2][5].left = 0; maze[2][5].right = 0;
        }
        else if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
          maze[5][2].left = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[3][5].left = 0;
        }
    }
    else if(end_Area=='D'){
      if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==5)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)){
        maze[2][2].left = 0; maze[2][2].right = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].down = 0;
      }
      else if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)){
        maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].left = 0; maze[4][2].down = 0; maze[3][2].down = 0; maze[3][2].left = 0;
      }
    }
    else if(end_Area=='E'){
      if((end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[3][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].left = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[2][2].right = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[4][5].right = 0;
      }
    }
    else if(end_Area=='F'){
      if((end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)){
        maze[3][5].left = 0; maze[5][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].up = 0; maze[5][5].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)){
        maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[4][2].left = 0;
      }
    }
    else if(end_Area=='G'){
      if((end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==5&&end_Y==5)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==2)){
        maze[2][5].down = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].left = 0; maze[5][2].left = 0; maze[5][2].right = 0;
      }
      else if((end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==3&&end_Y==2)||(end_X==4&&end_Y==2)){
        maze[2][5].right = 0; maze[2][2].left = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[4][2].right = 0;
      }
    }
    else if(end_Area=='H'){}
    else if(end_Area=='I'){
      if((end_X==5&&end_Y==3)||(end_X==5&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==3&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)){
        maze[5][3].up = 0; maze[5][2].right = 0; maze[4][2].down = 0; maze[3][2].down = 0; maze[2][2].left = 0; maze[2][3].up = 0;
      }
      else if((end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==2&&end_Y==5)||(end_X==2&&end_Y==4)){
        maze[5][3].down = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[2][5].left = 0; maze[2][4].down = 0;
      }
    }
    else if(end_Area=='J'){
      if((end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].left = 0; maze[4][5].up = 0;
      }
      else if((end_X==3&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)){
        maze[4][2].right = 0; maze[3][2].down = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[3][5].right = 0;
      }
    }
  }
  else if(start_Area=='I'){
    if(end_Area=='A'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].up = 0;}
    else if(end_Area=='B'){maze[3][2].left = 0; maze[4][2].down = 0; maze[5][2].right = 0; maze[5][3].up = 0;}
    else if(end_Area=='C'){maze[5][2].left = 0; maze[5][3].up = 0;
      if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
      else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}  
    }
    else if(end_Area=='D'){maze[5][3].down = 0;maze[5][5].left = 0;
      if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
    }
    else if(end_Area=='E'){maze[5][3].down = 0; maze[4][5].left = 0; maze[5][5].right = 0;
      if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
      else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
    }
    else if(end_Area=='F'){maze[3][5].left = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].down = 0;}
    else if(end_Area=='G'){maze[2][5].down = 0; maze[3][5].up = 0; maze[4][5].up = 0; maze[5][5].right = 0; maze[5][3].down = 0;
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
    }
    else if(end_Area=='H'){
      if((end_X==5&&end_Y==3)||(end_X==5&&end_Y==2)||(end_X==4&&end_Y==2)||(end_X==3&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)){
        maze[5][3].up = 0; maze[5][2].right = 0; maze[4][2].down = 0; maze[3][2].down = 0; maze[2][2].left = 0; maze[2][3].up = 0;
      }
      else if((end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)||(end_X==4&&end_Y==5)||(end_X==3&&end_Y==5)||(end_X==2&&end_Y==5)||(end_X==2&&end_Y==4)){
        maze[5][3].down = 0; maze[5][5].right = 0; maze[4][5].up = 0; maze[3][5].up = 0; maze[2][5].left = 0; maze[2][4].down = 0;
      }
    }
    else if(end_Area=='I'){maze[4][3].right = 0;}
    else if(end_Area=='J'){maze[4][2].left = 0;  maze[5][2].right = 0; maze[5][3].up = 0;}
  }
  else if(start_Area=='J'){
    if(end_Area=='A'){maze[2][2].up = 0; maze[3][2].down = 0; maze[4][2].right = 0;}
    else if(end_Area=='B'){maze[3][2].left = 0; maze[4][2].right = 0;}
    else if(end_Area=='C'){maze[5][2].up = 0; maze[4][2].left = 0;
      if((end_X==6&&end_Y==0)||(end_X==6&&end_Y==1)){maze[7][1].down = 0;maze[7][2].up = 0;}
      else if(end_X==7&&end_Y==0){maze[7][1].left = 0;maze[7][2].up = 0;}
      else if((end_X==7&&end_Y==3)||(end_X==7&&end_Y==4)){maze[7][2].down = 0;}
      else if((end_X==7&&end_Y==1)){maze[7][2].up = 0;}
    }
    else if(end_Area=='D'){maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0;maze[5][5].left = 0;
      if(end_X==7&&end_Y==5){maze[6][5].down = 0;}
      else {maze[6][5].right = 0;}
    }
    else if(end_Area=='E'){maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[4][5].left = 0; maze[5][5].right = 0;
      if((end_X==4&&end_Y==7)||(end_X==5&&end_Y==7)||(end_X==6&&end_Y==7)){maze[6][6].right = 0;}
      else if((end_X==7&&end_Y==6)||(end_X==7&&end_Y==7)){maze[6][6].up = 0;}
    }
    else if(end_Area=='F'){maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].right = 0; maze[3][2].down = 0; maze[4][2].right = 0;}
    else if(end_Area=='G'){maze[2][2].left = 0; maze[2][5].right = 0; maze[3][2].down = 0; maze[4][2].right = 0;
      if((end_X==1&&end_Y==6)||(end_X==1&&end_Y==7)||(end_X==2&&end_Y==7)||(end_X==3&&end_Y==7)){maze[1][5].left = 0;}
      else if((end_X==0&&end_Y==5)||(end_X==0&&end_Y==6)||(end_X==0&&end_Y==7)){maze[1][5].up = 0;}
    }
    else if(end_Area=='H'){
      if((end_X==4&&end_Y==2)||(end_X==5&&end_Y==2)||(end_X==5&&end_Y==3)||(end_X==5&&end_Y==4)||(end_X==5&&end_Y==5)||(end_X==4&&end_Y==5)){
        maze[4][2].left = 0; maze[5][2].right = 0; maze[5][3].left = 0; maze[5][5].right = 0; maze[4][5].left = 0; maze[4][5].up = 0;
      }
      else if((end_X==3&&end_Y==2)||(end_X==2&&end_Y==2)||(end_X==2&&end_Y==3)||(end_X==2&&end_Y==4)||(end_X==2&&end_Y==5)||(end_X==3&&end_Y==5)){
        maze[4][2].right = 0; maze[3][2].down = 0; maze[2][2].left = 0; maze[2][5].left = 0; maze[3][5].up = 0; maze[3][5].right = 0;
      }
    }
    else if(end_Area=='I'){maze[4][2].left = 0;  maze[5][2].right = 0; maze[5][3].up = 0;}
  }
}
void Route_Travel(){
  if(orientation=='N'){
    if(maze[current_X][current_Y].up==1){if(j==1){gostraight(); last_X=current_X;  last_Y=current_Y;}}
    else if(maze[current_X][current_Y].left==1){TurnLeft(); orientation='W'; j=1;}
    else if(maze[current_X][current_Y].right==1){TurnRight(); orientation='E'; j=1;}
    else if(maze[current_X][current_Y].down==1){goback(); rotate(); orientation='S'; j=1;}
  }  
  else if(orientation=='S'){
    if(maze[current_X][current_Y].down==1){if(j==1){gostraight();last_X=current_X;  last_Y=current_Y;}}
    else if(maze[current_X][current_Y].right==1){TurnLeft(); orientation='E'; j=1;}
    else if(maze[current_X][current_Y].left==1){TurnRight(); orientation='W'; j=1;} 
    else if(maze[current_X][current_Y].up==1){goback(); rotate(); orientation='N';} 
  }
  else if(orientation=='W'){
    if(maze[current_X][current_Y].left==1){if(j==1){gostraight();last_X=current_X;  last_Y=current_Y;}}
    else if(maze[current_X][current_Y].down==1){TurnLeft(); orientation='S'; j=1;}
    else if(maze[current_X][current_Y].up==1){TurnRight(); orientation='N'; j=1;}
    else if(maze[current_X][current_Y].right==1){goback(); rotate(); orientation='E';}
  }
  else if(orientation=='E'){
    if(maze[current_X][current_Y].right==1){if(j==1){gostraight();last_X=current_X;  last_Y=current_Y;}}
    else if(maze[current_X][current_Y].up==1){TurnLeft(); orientation='N'; j=1;}
    else if(maze[current_X][current_Y].down==1){TurnRight(); orientation='S'; j=1;} 
    else if(maze[current_X][current_Y].left==1){goback(); rotate(); orientation='W';}
  }
}
void Route_Check(){
  if((current_X==last_X)&&(current_Y==last_Y)){
    j*=-1;
  }
  else{j=1;}
}
void TurnLeft(){
  int k = 0;
  while(k<10){ 
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(90);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(200);
    k++;
  }   
}
void TurnRight(){
  int k = 0;
    while(k<10){ 
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,HIGH);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,HIGH);
      delay(70);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);
      k++;
    }    
}
/*void turnright(){ 
  while(c_dis<35){
    int k = 0;
    while(k<10){ 
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,HIGH);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,HIGH);
      delay(80);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);
      dis_record(k);
      k++;
    }
    turn_adjust(l_value);
    c_dis = get_dis(centerTrigPin,centerEchoPin);
    while(c_dis>1000){c_dis = get_dis(centerTrigPin,centerEchoPin);}
  } 
}
void turnrightreverse(int j){
  if(l_value[j+1]<l_value[j]){adjust_point = 9 - (j+1);}
  else if(l_value[j]<l_value[j+1]){adjust_point = 9 - (j);}
  else{adjust_point = 9 - (j+1);}
  for(int i = adjust_point;i>0;--i){
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(80);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
}
void dis_record(int n){
  l_dis = get_dis(leftTrigPin,leftEchoPin);
  while(l_dis>1000){
    l_dis = get_dis(leftTrigPin,leftEchoPin);
  }
  l_value[n] = l_dis;
}
void turn_adjust(float u_value[]){
    for(int i =0;i<9;i++){
      d_value[i] = u_value[i+1]-u_value[i];
      if(d_value[i]<0){d_value[i]*=-1;}
    }
    findmin(d_value);
}
void findmin(float uu_value[]){
  float min_value = 1000;
  int min_point = 0;
  for(int i =0;i<9;++i){  
    if(uu_value[i]<min_value){min_value = uu_value[i];min_point = i; }
  }
  turnrightreverse(min_point);
}
void turnleft(){
  while(c_dis<30){
  int k = 0;
  while(k<10){ 
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(80);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
    dis_recordl(k);
    k++;
  }
  turn_adjustl(r_value);
  c_dis = get_dis(centerTrigPin,centerEchoPin);
  while(c_dis>1000){c_dis = get_dis(centerTrigPin,centerEchoPin);} 
}
}
void turnleftreverse(int j){
  if(r_value[j+1]<r_value[j]){adjust_point = 9 - (j+1);}
  else if(r_value[j]<r_value[j+1]){adjust_point = 9 - (j);}
  else{adjust_point = 9 - (j+1);}
  for(int i = adjust_point;i>0;--i){
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,HIGH);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,HIGH);
    delay(120);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
}
void dis_recordl(int n){
  r_dis = get_dis(leftTrigPin,leftEchoPin);
  while(l_dis>1000){
    r_dis = get_dis(leftTrigPin,leftEchoPin);
  }
  r_value[n] = r_dis;
}
void turn_adjustl(float u_value[]){
    for(int i =0;i<9;i++){
      d_value[i] = u_value[i+1]-u_value[i];
      if(d_value[i]<0){d_value[i]*=-1;}
    }
    findminl(d_value);
}
void findminl(float uu_value[]){
  float min_value = 1000;
  int min_point = 0;
  for(int i =0;i<9;++i){  
    if(uu_value[i]<min_value){min_value = uu_value[i];min_point = i; }
  }
  turnleftreverse(min_point);
}*/
void gostraight(){
  int y = 0;
  while(y<8)
  {
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,HIGH);
    delay(90); 
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100); 
    straight_adjust();
    c_dis = get_dis(centerTrigPin,centerEchoPin);
    while(c_dis>1000){
      c_dis = get_dis(centerTrigPin,centerEchoPin);
    }
    while((c_dis<30)&&(c_dis>12)){
      digitalWrite(rightpin1,HIGH);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,HIGH);
      delay(50); 
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);
      straight_adjust();
      c_dis = get_dis(centerTrigPin,centerEchoPin);
    }
    if(c_dis<=12){break;}
    y++;
  } 
}
void straight_adjust(){
  l_dis = get_dis(leftTrigPin,leftEchoPin);
  r_dis = get_dis(rightTrigPin,rightEchoPin);
  while(l_dis>1000){
    l_dis = get_dis(leftTrigPin,leftEchoPin);
  }
  while(r_dis>1000){
    r_dis = get_dis(rightTrigPin,rightEchoPin);
  }
  if(l_dis<30 && l_dis>5){
    if(l_dis<11){
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,HIGH);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,HIGH);
      delay(50);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);
    }
    else if (l_dis>12){
      digitalWrite(rightpin1,HIGH);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,HIGH);
      digitalWrite(leftpin2,LOW);
      delay(50);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);  
    }
  }
  else if(r_dis<30 && r_dis>5){
    if(r_dis<11){
      digitalWrite(rightpin1,HIGH);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,HIGH);
      digitalWrite(leftpin2,LOW);
      delay(50);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);
    }
    else if (r_dis>12){    
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,HIGH);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,HIGH);
      delay(50);
      digitalWrite(rightpin1,LOW);
      digitalWrite(rightpin2,LOW);
      digitalWrite(leftpin1,LOW);
      digitalWrite(leftpin2,LOW);
      delay(100);  
    }
  }
  else if((l_dis<10)&&(l_dis>5)){
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,HIGH);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,HIGH);
    delay(60);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
  else if((r_dis<9)&&(r_dis>5)){
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(60);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
  else if(l_dis<4){
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,HIGH);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,HIGH);
    delay(70);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
  else if(r_dis<4){
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(70);
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
  }
}
void goback(){
  c_dis = get_dis(centerTrigPin,centerEchoPin);
  while(c_dis>1000){
    c_dis = get_dis(centerTrigPin,centerEchoPin);
  }
  while(c_dis<15){
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,HIGH);
    digitalWrite(leftpin1,HIGH);
    digitalWrite(leftpin2,LOW);
    delay(50); 
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    digitalWrite(leftpin1,LOW);
    digitalWrite(leftpin2,LOW);
    delay(100);
    c_dis = get_dis(centerTrigPin,centerEchoPin);
  }
}
void rotate(){
  digitalWrite(rightpin1,HIGH);
  digitalWrite(rightpin2,LOW);
  digitalWrite(leftpin1,HIGH);
  digitalWrite(leftpin2,LOW);
  delay(600);
  digitalWrite(rightpin1,LOW);
  digitalWrite(rightpin2,LOW);
  digitalWrite(leftpin1,LOW);
  digitalWrite(leftpin2,LOW);
  delay(100); 
  digitalWrite(rightpin1,LOW);
  digitalWrite(rightpin2,HIGH);
  digitalWrite(leftpin1,HIGH);
  digitalWrite(leftpin2,LOW);
  delay(100); 
  digitalWrite(rightpin1,LOW);
  digitalWrite(rightpin2,LOW);
  digitalWrite(leftpin1,LOW);
  digitalWrite(leftpin2,LOW);
  delay(100);
  digitalWrite(rightpin1,HIGH);
  digitalWrite(rightpin2,LOW);
  digitalWrite(leftpin1,HIGH);
  digitalWrite(leftpin2,LOW);
  delay(600);
  digitalWrite(rightpin1,LOW);
  digitalWrite(rightpin2,LOW);
  digitalWrite(leftpin1,LOW);
  digitalWrite(leftpin2,LOW);
  delay(100); 
}
void SetEndpoint(int endX,int endY){
  if((endX==0)&&(endY==7)){
    end_X = 0;
    end_Y = 0;
  }
  else if((endX==0)&&(endY==0)){
    end_X = 7;
    end_Y = 0;
  }
  else if((endX==7)&&(endY==0)){
    end_X = 7;
    end_Y = 7;
  }
  else if((endX==7)&&(endY==7)){
    end_X = 0;
    end_Y = 7;
  }
}
