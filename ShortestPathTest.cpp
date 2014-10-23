
//shortest path is stored in target
void findShortestPath(float startPos[], float endPos[], float target[3])
{
     float vecToStart[3] = startPos;
     float vecToEnd[3] = endPos;
     mathVecNormalize(vecToStart, 3);
     mathVecNormalize(vecToEnd, 3);
     float vecBetween[3];
     mathVecSubtract(vecBetween, vecToStart, vecToEnd, 3);
     
     float theta = acosf(mathVecInner(vecToStart, vecToEnd) / ( sqrtf(vecToStart[0]*vecToStart[0]) + 
        vecToStart[1]*vecToStart[1] + vecToStart[2]*vecToStart[2] )  +
        sqrtf(vecToEnd[0]*vecToEnd[0] + vecToEnd[1]*vecToEnd[1] + vecToEnd[2]*vecToEnd[2]));
     DEBUG(("theta: " + theta));

    target[0] = endPos[0];
    target[1] = endPos[1];
    target[2] = endPos[2];
     if(theta <= 90)
       return target;
     else
     {
        float midpoint[3];
        midpoint[0] = vecBetween[0] / 2;
        midpoint[1] = vecBetween[1] / 2;
        midpoint[2] = vecBetween[2] / 2;
        int i = 0;
        while(theta <= 90)
        {
            scalarMultiple(target, i);
            i += .01;
            theta = acosf(mathVecInner(target, vecToEnd) / ( sqrtf(target[0]*target[0]) + 
                target[1]*target[1] + target[2]*target[2] )  +
                sqrtf(vecToEnd[0]*vecToEnd[0] + vecToEnd[1]*vecToEnd[1] + vecToEnd[2]*vecToEnd[2]));
            DEBUG(("new theta: " + theta));
        }
        return target;
     }
}