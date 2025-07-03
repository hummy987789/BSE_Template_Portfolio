# 3 Joint Robotic Arm
My BSE project was a 3 joint robotic arm. It has a claw and 3 joint to move in 3 axes and pick things up. It moves using servos. It can be controlled with a controll as well. All of these components are wired into the nanoshield, which is connected to the Arduino Nano. This is the core of the project and tells everything what to do.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Sohum B | Wissahickon Middle School | Mechanical Engineering | Rising 8th Grader

**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
  
# Final Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE



# Second Milestone

<iframe width="560" height="315" src="https://www.youtube.com/embed/8TKmikUfqGI?si=DmWbcdOfI9txb8os" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 

# First Milestone

<iframe width="560" height="315" src="https://www.youtube.com/embed/gr2-A26rm-8?si=lNMZZyKzSOa0R-Nx" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


My first milestone was to build up a good amount of the base robot. I feel I achieved this goal as I have built roughly 3/4s of it. It works with servos connected to it acting as it joints. It rotates to move the arm. Some of my surprises were definitely how fragile the parts were. My biggest challenge was definitely breaking parts. I broke the right finger twice! For my next milestone, I hope to achieve the goal of coding and completely building my baseline project.

# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
/*
 * This code applies to cokoino mechanical arm
 * Through this link you can download the source code:
 * https://github.com/Cokoino/CKK0006
 * Company web site:
 * http://cokoino.com/
 *                                     ________
 *                         ----|servo4| 
 *                        |            --------
 *                    |servo3|   
 *                        |
 *                        |
 *                    |servo2|
 *                        |
 *                        |
 *                  ___________
 *                  |  servo1 |
 *         ____________________
 *         ____________________
 * Fanctions:
 * arm.servo1.read();   //read the servo of angle
 * arm.servo2.read();
 * arm.servo3.read();
 * arm.servo4.read();
 * 
 * arm.servo1.write(0);   //servo run
 * arm.servo2.write(0);
 * arm.servo3.write(0);
 * arm.servo4.write(0);
 * 
 * arm.left(speed);    //perform the action 
 * arm.right(speed);
 * arm.up(speed);
 * arm.down(speed);
 * arm.open(speed);
 * arm.close(speed);
 * 
 * arm.captureAction();    //capture the current action,return pointer array
 * arm.do_action(int *p,int speed);  //P is a pointer to the array
 * 
 * arm.JoyStickL.read_x(); //Returns joystick numerical
 * arm.JoyStickL.read_y();
 * arm.JoyStickR.read_x();
 * arm.JoyStickR.read_y();
 */
#include "src/CokoinoArm.h"
#define buzzerPin 9

CokoinoArm arm;
int xL,yL,xR,yR;

const int act_max=170;    //Default 10 action,4 the Angle of servo
int act[act_max][4];    //Only can change the number of action
int num=0,num_do=0;
///////////////////////////////////////////////////////////////
void turnUD(void){
  if(xL!=512){
    if(0<=xL && xL<=100){arm.up(10);digitalWrite(8,HIGH);return;}
    if(900<xL && xL<=1024){arm.down(10);digitalWrite(8,LOW);return;} 
    if(100<xL && xL<=200){arm.up(20);digitalWrite(8,HIGH);return;}
    if(800<xL && xL<=900){arm.down(20);digitalWrite(8,LOW);return;}
    if(200<xL && xL<=300){arm.up(25);digitalWrite(8,HIGH);return;}
    if(700<xL && xL<=800){arm.down(25);digitalWrite(8,LOW);return;}
    if(300<xL && xL<=400){arm.up(30);digitalWrite(8,HIGH);return;}
    if(600<xL && xL<=700){arm.down(30);digitalWrite(8,LOW);return;}
    if(400<xL && xL<=480){arm.up(35);digitalWrite(8,HIGH);return;}
    if(540<xL && xL<=600){arm.down(35);digitalWrite(8,LOW);return;} 
    }
}
///////////////////////////////////////////////////////////////
void turnLR(void){
  Serial.println(yL);
  if(yL!=512){
    if(0<=yL && yL<=100){arm.right(0);digitalWrite(9,HIGH);return;}
    if(900<yL && yL<=1024){arm.left(0);digitalWrite(9,LOW);return;}  
    if(100<yL && yL<=200){arm.right(5);digitalWrite(9,HIGH);return;}
    if(800<yL && yL<=900){arm.left(5);digitalWrite(9,LOW);return;}
    if(200<yL && yL<=300){arm.right(10);digitalWrite(9,HIGH);return;}
    if(700<yL && yL<=800){arm.left(10);digitalWrite(9,LOW);return;}
    if(300<yL && yL<=400){arm.right(15);digitalWrite(9,HIGH);return;}
    if(600<yL && yL<=700){arm.left(15);digitalWrite(9,LOW);return;}
    if(400<yL && yL<=480){arm.right(20);digitalWrite(9,HIGH);return;}
    if(540<yL && yL<=600){arm.left(20);digitalWrite(9,LOW);return;}
  }
}
///////////////////////////////////////////////////////////////
void turnCO(void){
  if(xR!=504){
    if(0<=xR && xR<=100){arm.close(0);digitalWrite(10,HIGH);return;}
    if(900<xR && xR<=1024){arm.open(0);digitalWrite(10,LOW);return;} 
    if(100<xR && xR<=200){arm.close(5);digitalWrite(10,HIGH);return;}
    if(800<xR && xR<=900){arm.open(5);digitalWrite(10,LOW);return;}
    if(200<xR && xR<=300){arm.close(10);digitalWrite(10,HIGH);return;}
    if(700<xR && xR<=800){arm.open(10);digitalWrite(10,LOW);return;}
    if(300<xR && xR<=400){arm.close(15);digitalWrite(10,HIGH);return;}
    if(600<xR && xR<=700){arm.open(15);digitalWrite(10,LOW);return;}
    if(400<xR && xR<=480){arm.close(20);digitalWrite(10,HIGH);return;}
    if(540<xR && xR<=600){arm.open(20);digitalWrite(10,LOW);return;} 
    }
}
///////////////////////////////////////////////////////////////
void date_processing(int *x,int *y){
  if(abs(512-*x)>abs(512-*y))
    {*y = 512;}
  else
    {*x = 512;}
}
///////////////////////////////////////////////////////////////
void buzzer(int H,int L){
  while(yR<420){
    digitalWrite(buzzerPin,HIGH);
    delayMicroseconds(H);
    digitalWrite(buzzerPin,LOW);
    delayMicroseconds(L);
    yR = arm.JoyStickR.read_y();
    }
  while(yR>600){
    digitalWrite(buzzerPin,HIGH);
    delayMicroseconds(H);
    digitalWrite(buzzerPin,LOW);
    delayMicroseconds(L);
    yR = arm.JoyStickR.read_y();
    }
}
///////////////////////////////////////////////////////////////
void C_action(void){
  if(yR>800){
    int *p;
    p=arm.captureAction();
    for(char i=0;i<4;i++){
    act[num][i]=*p;
    p=p+1;     
    }
    num++;
    num_do=num;
    if(num>=act_max){
      num=0;
      buzzer(600,400);
      }
    while(yR>600){yR = arm.JoyStickR.read_y();}
    //Serial.println(act[0][0]);
  }
}
///////////////////////////////////////////////////////////////
void Do_action(void){
  if(yR<220){
    buzzer(200,300);
    for(int i=0;i<num_do;i++){
      arm.do_action(act[i],15);
      }
    num=0;
    while(yR<420){yR = arm.JoyStickR.read_y();}
    for(int i=0;i<2000;i++){
      digitalWrite(buzzerPin,HIGH);
      delayMicroseconds(200);
      digitalWrite(buzzerPin,LOW);
      delayMicroseconds(300);        
    }
  }
}
///////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  //arm of servo motor connection pins
  arm.ServoAttach(4,5,6,7);
  arm.servo1.write(190);   //servo run
  arm.servo2.write(90);
  arm.servo3.write(122.5);
  arm.servo4.write(0);
  //arm of joy stick connection pins : xL,yL,xR,yR
  arm.JoyStickAttach(A0,A1,A2,A3);
  pinMode(buzzerPin,OUTPUT);
pinMode(8, OUTPUT);}
///////////////////////////////////////////////////////////////
void loop() {
  xL = arm.JoyStickL.read_x();
  yL = arm.JoyStickL.read_y();
  xR = arm.JoyStickR.read_x();
  yR = arm.JoyStickR.read_y();
  Serial.print("xL:");
  Serial.println(xL);
  Serial.println(xR);
  Serial.println(yL);
  Serial.println(yR);
  date_processing(&xL,&yL);
  date_processing(&xR,&yR);
  turnUD();
  turnLR();
  turnCO();
  C_action();
  Do_action();
}



```

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
