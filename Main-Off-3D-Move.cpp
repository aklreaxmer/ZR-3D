float   opponentState[12];
float   destination[3];
float   myPos[3]; 
float   myVel[3];
float   spot[3]; //where to be for pic
float   direction[3]; //direction to point for pic
float   zeroPOI[3]; 
float   onePOI[3];
float   twoPOI[3];
float   currPOI[3]; //POI desired at moment
float   uZone[3];
float   myState[12];
float   destination[3];

float   innerZone, outerZone, uploadZone; 

int     picCount, poiID;
int     outerTime;
int     blastTime, time;

bool    isOff;

bool    firstTime;
bool    areWeThereYet;
bool    firstTimeWp;

float   MAX_ACCEL_ONE_AXIS;
float   forces[3];
float   initPos[3];
float   initVel[3];
float   myAcc[3];
float   myForce[3];
float   prevVel[3];
float   ourState[13];
float   travelledVector[3];
float   wpToTar[3];
float   initToWp[3];
float   currToWp[3];
float   maxVelPerpTar[3];
float   maxVelParTar[3];
float   myVelPerpTar[3];
float   myVelParTar[3];
float   myAccPerpTar[3];
float   myAccParTar[3];
float   dirParTar[3];
float   myForcePerpTar[3];
float   myForceParTar[3];
float   dblMyForcePerpTar[3];
float   dblMyForceParTar[3];
float   switchPerpPos[3];
float   switchParPos[3];
float   switchPerpVector[3];
float   switchParVector[3]; 
float   switchVectorAway[3];
float   predictedVelWp[3];
float   xProdMyVelWpToTar[3];
float   xProdMyAccWpToTar[3];
float   xProdMyForceWpToTar[3];
float   goalAccPerpTar[3];
float   predictedSpeedWpPerp;
float   predictedSpeedWpPar;
float   xProdPredVelWpToTar[3];
float   mySpeed;
float   mySpeedPerpTar;
float   mySpeedParTar;
float   myAccMagPerpTar;
float   myAccMagParTar;
float   myForceMagPerpTar;
float   myForceMagParTar;
float   maxSpeedParTar;
float   maxSpeedPerpTar;
float   maxAccOneComp;

void init() 
{
    MAX_ACCEL_ONE_AXIS = 0.05f / 4.3f;

    firstTime = true;
    areWeThereYet = false;
    firstTimeWp = true;

    for(int i = 0; i < 13; i++)
    {
        if(i < 3)
        {
            myVel[i] = 0.0f;
            initVel[i] = 0.0f;
            myPos[i] = 0.0f;
        }
        ourState[i] = 0.0f;
    }

	destination[0] = 0.3f;
    destination[1] = 0.4f;
    destination[2] = 0.3f;
    
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
    destination[0] = x;
    destination[1] = y;
    destination[2] = z;
    
    stopAtFastest(destination);

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

void stopAtFastest(float destination[3])
{
    if(areWeThereYet)
    {
        api.setPositionTarget(destination);
        return;
    }
        
    if(firstTime)
    {
        for(int i = 0; i < 3; i++)
        {
            initPos[i] = myPos[i];
            halfPos[i] = (destination[i] - myPos[i])/2 + myPos[i];
        }
            

        mathVecSubtract(targetVector, destination, initPos, 3);
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
                api.setPositionTarget(destination);
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

