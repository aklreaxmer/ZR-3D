 float opponentState[12];
float destination[3];
float myPos[3]; 
float myVel[3];
float spot[3]; //where to be for pic
float direction[3]; //direction to point for pic
float zeroPOI[3]; 
float onePOI[3];
float twoPOI[3];
float currPOI[3]; //POI desired at moment
float uZone[3];
float myState[12];
float target[3];

float innerZone, outerZone, uploadZone; 

int picCount, poiID;
int outerTime;
int blastTime, time;

bool isOff;

bool    firstTime;              //is it first time calling mvt fn for this destination?
bool    areWeThereYet;          //have you arrived at the target location yet?
float    forces[3];              //the forces to be applied to this call of the function
float    initPos[3];             //the position of the sphere in the first call of the fn
float    halfPos[3];             //the position halfway between initpos and the destination
float    travelledVector[3];     //the vector from initpos to the sphere this call of the fn
float    halfwayVector[3];       //the vector from initpos to halfpos
float    targetVector[3];        //the vector from initpos to the destination


void init() 
{
	target[0] = 0.3f;
    target[1] = 0.4f;
    target[2] = 0.3f;
    
    picCount = 0;
    
    innerZone = 0.35f;
    outerZone = 0.45f;
    uploadZone = 0.6f;
    
    uZone[0]=-0.515f;
    uZone[1]=0.0f;
    uZone[2]=0.0f;
    
    isOff = false;
}

void loop()
{
    time = api.getTime();
    blastTime = game.getNextFlare();

    api.getMyZRState(myState);
    api.getOtherZRState(opponentState);
    
    myPos[0]=myState[0];
    myPos[1]=myState[1];
    myPos[2]=myState[2];
    
    myVel[0]=myState[3];
    myVel[1]=myState[4];
    myVel[2]=myState[5];

    if (game.getNextFlare()< 6 && game.getNextFlare()!=-1)
    {
        if (game.getNextFlare()>=3)
        {
            api.setPositionTarget(myPos);
        }
        
        else if(game.getNextFlare() < 2 && game.getNextFlare()!=-1)
        {
            DEBUG(("POWERING OFF"));
            picCount =0;
            game.turnOff();
            isOff = true;
        }    
      
    } 
    
    else if(game.getNextFlare()== -1 && isOff)
    {
        game.turnOn();
        DEBUG(("WE ON"));
        isOff = false;
    }
    
	else if(picCount==0 && game.getNextFlare() == -1)
	{   //take pics of closest POI inner Zone
	   takeOPic();
	   DEBUG(("Opic    "));
	}
	
	else if(picCount==1 && (game.getNextFlare()==-1 || game.getNextFlare()>25)&&api.getTime()<220)
	{
	   takeIPic();
	   DEBUG(("IPic    "));
	}
	
	else if(picCount == 2 || (game.getNextFlare()<=27&& picCount==1)|| (api.getTime()<220&&picCount==1))
	{
	   uploadPics();
	   DEBUG(("Up   "));
	}

    //DEBUG(("Time: %i, Flare: %i," , time, blastTime));
	
}


void moveTo(float x, float y, float z)
{
    target[0] = x;
    target[1] = y;
    target[2] = z;
    
    stopAtFastest(target);

}

void takeIPic()
{
    getClosestPOILoc();
    getSpot(innerZone);
    moveTo(spot[0], spot[1], spot[2]);
    getPOIVec();
   
   if(nearEnough(myPos, spot, 0.015f)&& api.getTime()>= (outerTime+3))
    {
        game.takePic(poiID);
        DEBUG(("Inner pic successfull :)"));
        picCount++;
    }
}

void takeOPic()
{
    getClosestPOILoc();
    getSpot(outerZone);
    moveTo(spot[0], spot[1], spot[2]);
    getPOIVec();
   
   if(nearEnough(myPos, spot, 0.015f))
    {
        game.takePic(poiID);
        outerTime = api.getTime();
        DEBUG(("Outer pic successfull :)"));
        picCount++;
    }
}

float dist(float pt1[],float pt2[])
{
    return sqrtf(powf(pt1[0]-pt2[0],2) + powf(pt1[1]- pt2[1],2) +
    powf(pt1[2] - pt2[2],2));
}

// currPOI contains closest POI location
void getClosestPOILoc()
{
    
    game.getPOILoc(zeroPOI, 0);
    game.getPOILoc(onePOI, 1);
    game.getPOILoc(twoPOI, 2);
    
    if((dist(myPos, zeroPOI)<=dist(myPos, onePOI))&&((dist(myPos, zeroPOI)<=dist(myPos, twoPOI))))
    {
        copyArray(zeroPOI, currPOI, 3);
        poiID=0;
    }
    
    else if(dist(myPos, onePOI)<=dist(myPos, twoPOI))
    {   
        copyArray(onePOI, currPOI, 3);
        poiID=1;
    }  
    
    else
    {
        copyArray(twoPOI, currPOI, 3);
        poiID=2;
    }
}

void uploadPics()
{
    getSpot(uploadZone);
	moveTo(spot[0], spot[1], spot[2]);
    if (nearEnough(myPos,spot, 0.04f))
    {
        game.uploadPic();
        DEBUG(("Uploaded"));
        picCount=0;
	}
}

void getSpot(float zone)
{
    copyArray(currPOI, spot, 3);
    mathVecNormalize(spot, 3);
    scalarMultiple(spot, zone, 3);
}

void getPOIVec()
{
    copyArray(currPOI, direction, 3);
    mathVecNormalize(direction, 3);
    scalarMultiple(direction, -1.0f, 3);
    api.setAttitudeTarget(direction); 
}

void copyArray(float from[], float to[], int x)
{
    for(int i=0; i<x; i++)
        to[i] = from[i];
}

bool nearEnough(float pos1[], float pos2[], float howClose)
{
    return dist(pos1, pos2) <= howClose;
}

void scalarMultiple(float vector[], float scalar, int size)
{
    for(int i = 0; i < size; i++)
        vector[i] = scalar*vector[i];
}

void stopAtFastest(float target[3])
{
    if(areWeThereYet)
    {
        api.setPositionTarget(target);
        return;
    }
        
    if(firstTime)
    {
        for(int i = 0; i < 3; i++)
        {
            initPos[i] = myPos[i];
            halfPos[i] = (target[i] - myPos[i])/2 + myPos[i];
        }
            

        mathVecSubtract(targetVector, target, initPos, 3);
        mathVecSubtract(halfwayVector, halfPos, initPos, 3);
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    }
    else
    {
        mathVecSubtract(travelledVector, myPos, initPos, 3);
        if(mathVecMagnitude(travelledVector, 3) >= mathVecMagnitude(halfwayVector, 3))
        {
            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(targetVector, 3)) < 0.1f)
            {
                api.setPositionTarget(target);
                firstTime = true;
                areWeThereYet = true;
                
            }
            else
            {
                for(int i = 0; i < 3; i++)
                    forces[i] = targetVector[i] * -1;
                
                mathVecNormalize(forces, 3);

            }
        }
    }
    api.setForces(forces);
    
}

