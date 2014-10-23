/*
* Moves the satellite to a provided target location, with
* thrusters firing at full power the enitre trip. Stops the
* satellite at the target.
*/

void stopAtFastest(float target[3]) {

    if(areWeThereYet) {
        api.setPositionTarget(target);
        return;
    }
        
    if(firstTime) {
        for(int i = 0; i < 3; i++) { //if it is, sets initPos to the position and halfPos to the halfway point of the sphere
            initPos[i] = myPos[i];
            initVel[i] = myVel[i];
            halfPos[i] = (target[i] - myPos[i])/2 + myPos[i];
        }
            
        mathVecSubtract(targetVector, target, initPos, 3);  //subtracts initpos from the destination to get the vector from initpos to the destination
        mathVecSubtract(halfwayVector, halfPos, initPos, 3);//   ''       ''     ''  halfpos         ''  ''  ''   ''     ''   ''    '' halfpos
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else { //not first time calling function
        mathVecSubtract(travelledVector, myPos, initPos, 3);    //subtracts initpos from myent pos to get the vector from initpos to myent pos
        if(mathVecMagnitude(travelledVector, 3) >= mathVecMagnitude(halfwayVector, 3)) {
            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(targetVector, 3)) < 0.1f) {
                api.setPositionTarget(target);
                firstTime = true;
                areWeThereYet = true;
                
                DEBUG(("%f", (float)api.getTime()));
            } else {
                for(int i = 0; i < 3; i++)
                    forces[i] = targetVector[i] * -1;
                
                mathVecNormalize(forces, 3);
            }
        }
    }
    api.setForces(forces);
}


/*
 * Moves the satellite towards the provided target, with
 * thrusters firing at full power the entire trip. Makes
 * no attempt to stop at the target.
*/

void moveTowardFastest(float target[]) {
    if(areWeThereYet)
        return;
        
    if(firstTime) {
        for(int i = 0; i < 3; i++)
            initPos[i] = myPos[i];
            
        mathVecSubtract(targetVector, target, initPos, 3); 
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else {
        mathVecSubtract(travelledVector, myPos, initPos, 3);
        if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(targetVector, 3)) < 0.1f) {
            firstTime = true;
            areWeThereYet = true;
            
        }
    }
    api.setForces(forces);
}
/*
 * Moves the satellite towards the provided target, with
 * thrusters firing only the percentage of the time 
 * specified by the power variable. Makes no attempt to
 * stop at the target. Power should be a decimal between
 * 0 and 1, with 1 being equivalent to calling
 * moveTowardFastest() and 0 resulting in no movement.
*/

void moveTowardVariable(float target[], float power) {
    if(areWeThereYet)
        return;
    
    if(firstTime) {
        for(int i = 0; i < 3; i++) {
            initPos[i] = myPos[i];
            halfPos[i] = power * (target[i] - myPos[i]) + myPos[i];
        }

        mathVecSubtract(targetVector, target, initPos, 3);  
        mathVecSubtract(halfwayVector, halfPos, initPos, 3);
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else {
        mathVecSubtract(travelledVector, myPos, initPos, 3);
        if(mathVecMagnitude(travelledVector, 3) >= mathVecMagnitude(halfwayVector, 3)) {
            for(int i = 0; i < 3; i++)
                forces[i] = 0;
        }
    }
    api.setForces(forces);
}

void moveToWaypoint(float target[3], float waypoint[3]) {
    if(areWeThereYet)   {
        api.setPositionTarget(target);
        return;
    }

    if(firstTime) {
        for(int i = 0; i < 3; i++) {
            initPos[i] = myPos[i];
            prevVel[i] = myVel[i];
            initVel[i] = myVel[i];
        }

        mathVecSubtract(travelledVector, myPos, initPos, 3);
        mathVecSubtract(initToWp, waypoint, initPos, 3);
        mathVecSubtract(wpToTar, target, waypoint, 3);

        for(int i = 0; i < 3; i++) {
            forces[i] = initToWp[i];
            dirParTar[i] = wpToTar[i];

        }

        mathVecNormalize(forces, 3);
        mathVecNormalize(dirParTar, 3);

        for(int i = 0; i < 3; i++) {
            myForce[i] = forces[i];
            switchPerpVector[i] = 3 * wpToTar[i] / 4;
        }

        mathVecAdd(switchPerpPos, waypoint, switchPerpVector, 3);

        mySpeedParTar = mathVecInner(myVel, wpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        mySpeedPerpTar = mathVecMagnitude(xProdMyVelWpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        // mathVecSubtract(mySpeedPerpTar, myVel, mySpeedParTar, 3);


        for(int i = 0; i < 3; i++)
            predictedVelWp[i] = sqrtf(powf(initVel[i], 2) + 2 * MAX_ACCEL_ONE_AXIS * mathVecMagnitude(initToWp, 3));

        mathVecCross(xProdPredVelWpToTar, predictedVelWp, wpToTar);

        predictedSpeedWpPar = mathVecInner(predictedVelWp, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
        predictedSpeedWpPerp = mathVecMagnitude(xProdPredVelWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

        maxSpeedParTar = sqrtf(2 * MAX_ACCEL_ONE_AXIS * mathVecMagnitude(wpToTar, 3));

        if(predictedSpeedWpPar > maxSpeedParTar)
            maxSpeedPerpTar = maxSpeedParTar;

        else
            maxSpeedPerpTar = predictedSpeedWpPar;

        firstTime = false;

    }

    else {
        mathVecSubtract(myAcc, myVel, prevVel, 3);

        mathVecCross(xProdMyVelWpToTar, myVel, wpToTar);
        mathVecCross(xProdMyAccWpToTar, myAcc, wpToTar);

        mySpeedParTar = mathVecInner(myVel, wpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        mySpeedPerpTar = mathVecMagnitude(xProdMyVelWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

        myAccMagParTar = mathVecInner(myAcc, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
        myAccMagPerpTar = mathVecMagnitude(xProdMyAccWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

        for(int i = 0; i < 3; i++) {
            myVelParTar[i] = mySpeedParTar * dirParTar[i];
            myVelPerpTar[i] = myVel[i] - myVelParTar[i];
            myAccParTar[i] = myAccMagParTar * dirParTar[i];
            myAccPerpTar[i] = myAcc[i] - myAccParTar[i];

        }

        if(mathVecMagnitude(initToWp, 3) - mathVecMagnitude(travelledVector, 3) > 0.05f) {
            mathVecSubtract(travelledVector, myPos, initPos, 3);

            for(int i = 0; i < 3; i++)
                forces[i] = initToWp[i];

            mathVecNormalize(forces, 3);

            for(int i = 0; i < 3; i++)
                myForce[i] = forces[i];

            mathVecCross(xProdMyForceWpToTar, myForce, wpToTar);

            myForceMagParTar = mathVecInner(myForce, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
            myForceMagPerpTar = mathVecMagnitude(xProdMyForceWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

            for(int i = 0; i < 3; i++) {
                myForceParTar[i] = myForceMagParTar * dirParTar[i];
                dblMyForceParTar[i] = 2 * myForceParTar[i];
                dblMyForcePerpTar[i] = 2 * myForcePerpTar[i];

            }

            mathVecSubtract(myForcePerpTar, forces, myForceParTar, 3);

            if(mySpeedParTar > maxSpeedParTar)
                mathVecSubtract(forces, forces, myForceParTar, 3);

            if(mySpeedPerpTar > maxSpeedPerpTar)
                mathVecSubtract(forces, forces, myForcePerpTar, 3);

        } else {
            if(firstTimeWp) {


                for(int i = 0; i < 3; i++) {
                    switchParVector[i] = myVel[i] / (-2 * MAX_ACCEL_ONE_AXIS  + waypoint[i] + target[i]) / 2;
                    goalAccPerpTar[i] = myVelPerpTar[i] / (-2 * mathVecMagnitude(wpToTar, 3));
                }


                mathVecAdd(switchParPos, switchParVector, waypoint, 3);

            }

            for(int i = 0; i < 3; i++)
                forces[i] = wpToTar[i];

            mathVecNormalize(forces, 3);

            for(int i = 0; i < 3; i++)
                myForce[i] = forces[i];

            mathVecCross(xProdMyForceWpToTar, myForce, wpToTar);

            myForceMagParTar = mathVecInner(myForce, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
            myForceMagPerpTar = mathVecMagnitude(xProdMyForceWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(switchParVector, 3)) > 0.05f)
                mathVecSubtract(forces, forces, dblMyForceParTar, 3);

            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(switchPerpVector, 3)) > 0.05f)
                mathVecSubtract(forces, forces, dblMyForcePerpTar, 3);

            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(wpToTar, 3)) > 0.05f) {
                for (int i = 0; i < 3; i++)
                    forces[i] = 0;
                areWeThereYet = true;
                api.setPositionTarget(target);
                return;
            }
        }
    }   

    api.setForces(forces);
    // DEBUG(("Forces: %f %f %f\nmaxSpeedParTar: %f\nmySpeedParTar: %f\nmaxSpeedPerpTar: %f\nmySpeedPerpTar: %f", forces[0], forces[1], forces[2], maxSpeedParTar, mySpeedParTar, maxSpeedPerpTar, mySpeedPerpTar));
    for(int i = 0; i < 3; i++) {
        myForce[i] = forces[i];
        prevVel[i] = myVel[i];
    }

}

/*
 * The satellite’s angular velocity must start at less than 2.3°/s.
 * Rotate the satellite >90° along about the Z axis for 2D. 
 * Do not attempt to rotate faster than 80°/s
 */
void spinForMemoryPack(float memoryPackPos[]) {

    api.getMyZRState(ourState); 
    float zRVel = ourState[11]; 

    float myPos[3];
    myPos[0] = ourState[0];
    myPos[1] = ourState[1];
    myPos[2] = ourState[2];
  
    float distToMemPack = dist(memoryPackPos, myPos);

        if (zRVel < 1.4f && zRVel > 0.04f && distToMemPack < 0.05f) {
            float rVel[3];
            rVel[0] = 0.0f;
            rVel[1] = 0.0f;
            rVel[2] = 1.0f; //in radians
            api.setAttRateTarget(rVel); 
        }
        else {    
            api.setPositionTarget(memoryPackPos); 
            float rVel[3];
            rVel[0] = 0.0f;
            rVel[1] = 0.0f;
            rVel[2] = 0.0f;
            api.setAttRateTarget(rVel);  
        }
}

void moveToShadowZone() {

    //this needs to be global, and init it in init()
    float shadowPos[3]; 
    //init that with the right position, and account for red or blue side

    //update this
    api.getMyZRState(ourState); 

    if (!needsWaypoint())
        stopAtFastest(shadowPos); 
    else {

        float waypoint[3]; 
        findWaypoint[waypoint]; 

        moveToWaypoint(shadowPos, waypoint);
    }
}
void findWaypoint(float waypoint[]) {

    //update this
    game.getMyZRState(ourState); 

    //TODO: ireallyneedtolearnmoremath

}

bool needsWaypoint() {
    float ourPos[3]; 
    ourPos[0] = ourState[0]; 
    ourPos[1] = ourState[1]; 
    ourPos[2] = ourState[2]; 

    float centerOfAsteroid[3]; 
    centerOfAsteroid[0] = 0; 
    centerOfAsteroid[1] = 0;
    centerOfAsteroid[2] = 0;

    float distance = getDistBetweenLineAndPoint(centerOfAsteroid, ourPos, shadowPos); 

    //rad of ast = .2
    //rad of danger zone = .25
    //rad of sph = .11
    //zoom for error = .1
    return distance < (.2 + .25 + .11 + .1)
}

float getDistBetweenLineAndPoint(float indPoint[], float pt1OnLine[], float pt2OnLine[]) {

    //---first, find the vector from pt1 and pt2---//
    float vector[]; 
    vector[0] = pt1OnLine[0] - pt2OnLine[0]; 
    vector[1] = pt1OnLine[1] - pt2OnLine[1]; 
    vector[2] = pt1OnLine[2] - pt2OnLine[2]; 

    //---now find the mag of that vector---//
    float magnitute = dist(pt1OnLine, pt2OnLine);

    //---Direction vector of line---//
    float unitVector[]; 
    unitVector[0] = vector[0]/magnitute;
    unitVector[1] = vector[1]/magnitute; 
    unitVector[2] = vector[2]/magnitute;  

    //---dot product unitVector * vector---//
    float dotProduct = (unitVector[0] * vector[0] +
                        unitVector[1] * vector[1] + 
                        unitVector[2] * vector[2] ); 

    //---find and return distance---// 
    return sqrtf((powf(magnitute, 2) - powf(dotProduct, 2))); 
}


void lookAtPOIFromZone(int zoneID) {

    //update this each time this gets called in loop
    api.getMyZRState(ourState); 

    float innerPos, outerPos; 

    innerPos = ourState[1] > 0 ? 0.26f : -0.26f; 
    outerPos = ourState[1] > 0 ? 0.425f : -0.425f; 

    float xTargetPos; 
    xTargetPos = zoneID == INNER_ZONE_ID ? innerPos : outerPos; 

    float lookingPosition[3]; 

    lookingPosition[0] =  xTargetPos; 

    //---no need to change these---//
    lookingPosition[1] = ourState[1]; 
    lookingPosition[2] = ourState[2]; 

    api.setPositionTarget(lookingPosition); 

    float noRotation[3];
    noRotation[0] = 0.0f;
    noRotation[1] = 0.0f; 
    noRotation[2] = 0.0f; 

    api.setAttRateTarget(noRotation); 

}

