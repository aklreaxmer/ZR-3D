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
            initPos[i] = currPos[i];
            initVel[i] = currVel[i];
            halfPos[i] = (target[i] - currPos[i])/2 + currPos[i];
        }
            
        mathVecSubtract(targetVector, target, initPos, 3);  //subtracts initpos from the destination to get the vector from initpos to the destination
        mathVecSubtract(halfwayVector, halfPos, initPos, 3);//   ''       ''     ''  halfpos         ''  ''  ''   ''     ''   ''    '' halfpos
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else { //not first time calling function
        mathVecSubtract(travelledVector, currPos, initPos, 3);    //subtracts initpos from current pos to get the vector from initpos to current pos
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
            initPos[i] = currPos[i];
            
        mathVecSubtract(targetVector, target, initPos, 3); 
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else {
        mathVecSubtract(travelledVector, currPos, initPos, 3);
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
            initPos[i] = currPos[i];
            halfPos[i] = power * (target[i] - currPos[i]) + currPos[i];
        }

        mathVecSubtract(targetVector, target, initPos, 3);  
        mathVecSubtract(halfwayVector, halfPos, initPos, 3);
    
        for(int i = 0; i < 3; i++)
            forces[i] = targetVector[i];
        
        mathVecNormalize(forces, 3);
        
        firstTime = false;
        
    } else {
        mathVecSubtract(travelledVector, currPos, initPos, 3);
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
            initPos[i] = currPos[i];
            prevVel[i] = currVel[i];
            initVel[i] = currVel[i];
        }

        mathVecSubtract(travelledVector, currPos, initPos, 3);
        mathVecSubtract(initToWp, waypoint, initPos, 3);
        mathVecSubtract(wpToTar, target, waypoint, 3);

        for(int i = 0; i < 3; i++) {
            forces[i] = initToWp[i];
            dirParTar[i] = wpToTar[i];

        }

        mathVecNormalize(forces, 3);
        mathVecNormalize(dirParTar, 3);

        for(int i = 0; i < 3; i++) {
            currForce[i] = forces[i];
            switchPerpVector[i] = 3 * wpToTar[i] / 4;
        }

        mathVecAdd(switchPerpPos, waypoint, switchPerpVector, 3);

        currSpeedParTar = mathVecInner(currVel, wpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        currSpeedPerpTar = mathVecMagnitude(xProdCurrVelWpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        // mathVecSubtract(currSpeedPerpTar, currVel, currSpeedParTar, 3);


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
        mathVecSubtract(currAcc, currVel, prevVel, 3);

        mathVecCross(xProdCurrVelWpToTar, currVel, wpToTar);
        mathVecCross(xProdCurrAccWpToTar, currAcc, wpToTar);

        currSpeedParTar = mathVecInner(currVel, wpToTar, 3)/mathVecMagnitude(wpToTar, 3);
        currSpeedPerpTar = mathVecMagnitude(xProdCurrVelWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

        currAccMagParTar = mathVecInner(currAcc, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
        currAccMagPerpTar = mathVecMagnitude(xProdCurrAccWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

        for(int i = 0; i < 3; i++) {
            currVelParTar[i] = currSpeedParTar * dirParTar[i];
            currVelPerpTar[i] = currVel[i] - currVelParTar[i];
            currAccParTar[i] = currAccMagParTar * dirParTar[i];
            currAccPerpTar[i] = currAcc[i] - currAccParTar[i];

        }

        if(mathVecMagnitude(initToWp, 3) - mathVecMagnitude(travelledVector, 3) > 0.05f) {
            mathVecSubtract(travelledVector, currPos, initPos, 3);

            for(int i = 0; i < 3; i++)
                forces[i] = initToWp[i];

            mathVecNormalize(forces, 3);

            for(int i = 0; i < 3; i++)
                currForce[i] = forces[i];

            mathVecCross(xProdCurrForceWpToTar, currForce, wpToTar);

            currForceMagParTar = mathVecInner(currForce, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
            currForceMagPerpTar = mathVecMagnitude(xProdCurrForceWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

            for(int i = 0; i < 3; i++) {
                currForceParTar[i] = currForceMagParTar * dirParTar[i];
                dblCurrForceParTar[i] = 2 * currForceParTar[i];
                dblCurrForcePerpTar[i] = 2 * currForcePerpTar[i];

            }

            mathVecSubtract(currForcePerpTar, forces, currForceParTar, 3);

            if(currSpeedParTar > maxSpeedParTar)
                mathVecSubtract(forces, forces, currForceParTar, 3);

            if(currSpeedPerpTar > maxSpeedPerpTar)
                mathVecSubtract(forces, forces, currForcePerpTar, 3);

        } else {
            if(firstTimeWp) {


                for(int i = 0; i < 3; i++) {
                    switchParVector[i] = currVel[i] / (-2 * MAX_ACCEL_ONE_AXIS  + waypoint[i] + target[i]) / 2;
                    goalAccPerpTar[i] = currVelPerpTar[i] / (-2 * mathVecMagnitude(wpToTar, 3));
                }


                mathVecAdd(switchParPos, switchParVector, waypoint, 3);

            }

            for(int i = 0; i < 3; i++)
                forces[i] = wpToTar[i];

            mathVecNormalize(forces, 3);

            for(int i = 0; i < 3; i++)
                currForce[i] = forces[i];

            mathVecCross(xProdCurrForceWpToTar, currForce, wpToTar);

            currForceMagParTar = mathVecInner(currForce, wpToTar, 3) / mathVecMagnitude(wpToTar, 3);
            currForceMagPerpTar = mathVecMagnitude(xProdCurrForceWpToTar, 3) / mathVecMagnitude(wpToTar, 3);

            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(switchParVector, 3)) > 0.05f)
                mathVecSubtract(forces, forces, dblCurrForceParTar, 3);

            if(fabsf(mathVecMagnitude(travelledVector, 3) - mathVecMagnitude(switchPerpVector, 3)) > 0.05f)
                mathVecSubtract(forces, forces, dblCurrForcePerpTar, 3);

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
    // DEBUG(("Forces: %f %f %f\nmaxSpeedParTar: %f\ncurrSpeedParTar: %f\nmaxSpeedPerpTar: %f\ncurrSpeedPerpTar: %f", forces[0], forces[1], forces[2], maxSpeedParTar, currSpeedParTar, maxSpeedPerpTar, currSpeedPerpTar));
    for(int i = 0; i < 3; i++) {
        currForce[i] = forces[i];
        prevVel[i] = currVel[i];
    }

}

void moveToShadowZone() {

    //update this
    api.getMyZRState(ourState); 

    if (ourState[0] >= 0.0f && ourState[1] <= -0.2f) {
        
        float waypoint[3];
        waypoint[0] = 0.0f;
        waypoint[1] = -0.4f;
        waypoint[2] = 0.0f;  

        moveToWaypoint(waypoint, shadowCenter); 

     } else if (ourState[0] >= 0.0f && ourState[1] >= 0.2f) {

       float waypoint[3];
       waypoint[0] = 0.0f;
       waypoint[1] = 0.4f; 
       waypoint[2] = 0.0f; 
        
       moveToWaypoint(waypoint, shadowCenter); 
     } else {  //easy /Q1 or Q2
        stopAtFastest(shadowCenter);
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

    float currPos[3];
    currPos[0] = ourState[0];
    currPos[1] = ourState[1];
    currPos[2] = ourState[2];
  
    float distToMemPack = dist(memoryPackPos, currPos);

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

