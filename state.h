#ifndef state.h 
#define state.h
#define entreAxe 0
class state{
public :
    double x ;
    double y ; 
    double theta;
    double lastDistance;
    
    state(){x = y = theta = lastDistance = 0;}

    void odometry (float positionLeft , float positionRight){
        float distance = 0.5 * (positionLeft + positionRight);
        theta += (positionLeft-positionRight)/entreAxe ;
        float velocity = distance - lastDistance ;
        lastDistance = distance ; 
        float deltaX = -velocity*sin(theta) ;
        float deltaY = velocity*cos(theta);
        x += deltaX ;
        y += deltaY ;
    }
}

#endif