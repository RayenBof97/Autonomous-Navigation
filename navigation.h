#ifndef navigation.h
#define navigation.h
#include "navigation.h"
#include "state.h"
#include <SimpleTimer.h>
#include <Math.h>
#include "state.h"
#include <arduino.h>
#define in_left1 5 
#define in_left2 6
#define in_right1 9
#define in_right2 10
//PID coefficient 
#define positionKp 0
#define positionKd 0
#define velocityKp 0
#define velocityKi 0
//Coefficient to balance between two wheels
#define Kb 0.05
//distance per tick 
#define distancePerTick 0
//distance between the center of the robot and the encoder's wheel
#define L 15




class Navigation {
public :
    double positionLeft, positionRight, lastPositionLeft, lastPositionRight, velocityLeft, velocityRight;
    long counterLeft;
    long counterRight;
    state currentState;
    SimpleTimer timer;  
    
    Navigation(){
        positionLeft = positionRight = lastPositionLeft = lastPositionRight = velocityLeft = velocityRight = 0;
        encoderInit();
        timer.setInterval(20,interuptServiceRoutine);
    }

    void encoderInit(){
        attachInterrupt(digitalPinToInterrupt(enc_left_A),ReadEncoder_ISR_left,CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_left_B),ReadEncoderB_left,CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_right_A),ReadEncoder_ISR_right,CHANGE);
        attachInterrupt(digitalPinToInterrupt(enc_right_B),ReadEncoderB_right,CHANGE);
    }

    void interruptServiceRoutine(){
        calcualteVelocity();
        currentState.odometry(positionleft,positionright);
    }

    //this function is called each 20ms to calcuate the velocity of each wheel .
    void calculateVelocity (){
        positionLeft = counterLeft * distancePerTick;
        positionRight= counterRight * distancePerTick; 
        velocityLeft = (positionLeft - lastPositionLeft)/20 ;
        velocityRight = (positionRight-lastPositionRight)/20;
        lastPositionLeft = positionLeft ;
        lastPositionRight = positionRight ;
    }

    //to calculate the ramp of the velocity . the ramp is linear function where the slope is the (acceleration critique).
    float calculateRamp (int time){
        float ramp = 255/3000 * time  ;
        ramp = constraint(ramp,-255,255);
        return ramp ;
    }

    void move (float goal){
    int time_change=15 ,now ,lastTime =0, firstTime ; 
    float positionError ,  dPosition ,lastPositionError ,velocityCommand ,velocityError ,iVelocity ,balanceError,output ;
    int firsttime = millis() ;
    // goal 
    while (goal != 0.5*(positionLeft+positionRight)){
        lastTime = millis();

        // calculating the pd terms of position
        positionError = goal - 0.5 *(positionLeft+positionRight); 
        dPosition = (positionError - lastPositionError)/time_change ;
        lastPositionError = positionError ;

        // the velocity command is gonna be the minimum between pid result of Position error and the ramp input
        velocityCommand = (positionKp * positionError + positionKd * dPosition) ;
          
        float ramp = calculateRamp(now-firsttime);
        velocityCommand = min(velocityCommand,ramp);


        //calculating the pi terms of velocity
        velocityError = velocityCommand - 0.5*(velocityRight+velocityLeft);
        iVelocity += velocityError * time_change ;
        iVelocity = constrain(iVelocity,-255,255) ;
        
        // the pwm of motors(output) is gonna be the pi of the velocity 
        output = velocityKp *velocityError + velocityKi * iVelocity ;
        output = constrain (output,-170,170) ;

        // before giving the pwm to both motors we gotta balance them so the robot goes straight 
        balanceError = positionLeft - positionRight ;
        setMotorsPower(constrain(output+Kb*balanceError+47,-255,255) , constrain(output-Kb*balanceError+47,-255,255)); 

        //updating time variable and calculating the velocity
        now = millis();        
        delay(time_change-now+lastTime) ;
    }
}


    void rotate (double goal){
        int time_change ,now ,lastTime ,firsttime; 
        float positionError , iPosition, dPosition ,lastPositionError ,velocityCommand ,velocityError ,iVelocity,dVelocity ,lastVelocityError,balanceError ;
        goal = goal/360 * 2 * M_PI * L ;
        int firsttime = millis() ;
        now = firsttime ;
        while (goal != 0.5*(positionLeft-positionRight)){
            time_change = 20 ;

            // calculating the pid terms of position
            positionError = goal - 0.5 (positionleft-positionright); 
            iPosition += positionError * time_change ;
            dPosition = (positionError - lastPositionError)/time_change ;
            lastPositionError = positionError ;


            // the velocity command is gonna be the minimum between pid result of Position error and the ramp input
            velocityCommand = (positionKp * positionError + positionKi * iPosition + positionKd * dPosition) ;
            ramp = calculateRamp(now-firsttime) ;
            velocityCommand = min(velocityCommand,ramp);


            //calculating the pid terms of velocity
            VelocityError = velocityCommand - 0.5*(velocityRight-velocityLeft)
            iVelocity += velocityError * time_change ;
            iVelocity = constarint(ivelocity,-280 ,280) ;
            dVelocity = (velocityError - lastVelocityError)/time_change ;
            lastVelocityError = velocityError ;


            // the pwm of motors(output) is gonna be the pid of the velocity 
            output = velocityKp *velocityError + velocityKi * ivelocity + velocityKd * dvelocity
            output = constraint (output,-255,255) ;


            // before giving the pwm to both motors we gotta balance them so the robot goes straight 
            balanceError = positionLeft + positionRight ;
            setMotorsPower(output-Kb*balanceError , -output-Kb*balanceError); 

            //updating time variable and calculating the velocity
            now = millis();
            delay(20-now+lastTime) ;
            lastTime = now ; 
        }
    }


    void setMotorsPower(int power_a, int power_b) {
        // Constrain power to between -255 and 255
        power_a = constrain(power_a, -255, 255);
        power_b = constrain(power_b, -255, 255);

        // Left motor direction
        if ( power_a < 0 ) {
            analogWrite(in_left2, 0);
            analogWrite(in_left1, power_a);
        } else {
            analogWrite(in_left2, power_a);
            analogWrite(in_left1, 0);
        }

        // Right motor direction
        if ( power_b < 0 ) {
            analogWrite(in_right1, 0);
            analogWrite(in_right2, power_b);
        } else {
            analogWrite(in_right1, power_b);
            analogWrite(in_right2, 0);
        }
    }

    //encoder reading functions 
    void ReadEncoder_ISR_left(){
        int a= digitalRead(enc_left_A);
        int b= digitalRead(enc_left_B);
        if(a^b){
            counterLeft ++;
        }
        else{
            counterLeft--;
        }
    }

    void ReadEncoderB_left(){
        int a= digitalRead(enc_left_A);
        int b= digitalRead(enc_left_B);
        if(!(a^b)){
            counterLeft++;
        }
        else{
            counterLeft--;
        }
    }

    void ReadEncoder_ISR_right(){
        int a= digitalRead(enc_right_A);
        int b= digitalRead(enc_right_B);
        if(a^b){
            counterRight++;
        }
        else{
            counterRight--;
        }
    }
    
    void ReadEncoderB_right(){
        int a= digitalRead(enc_right_A);
        int b= digitalRead(enc_right_B);
        if(!(a^b)){
            counterRight ++;
        }
        else{
            counterRight--;
        }
    }
    /*
    void orientate(double goal){
        angle = angle - theta ;
        rotate(angle);
    }

    void robot_locate (double xDestination , double yDestination){
        xCurrent = r cos(theta) ;
        yCurrent = r sin(theta);
        float angle = (xDestination*xCurrent + yCurrent*yDestination)/(sqrt(xDestination*xDestination+yDestination*yDestination)*sqrt(xCurrent*xCurrent+yDestination*yDestination));
        distance = sqrt(pow(xDestination-xCurrent,2)+pow(yDestination-yCurrent,2));
        rotate(angle);
        move(distance);
    }
    */
}
#endif