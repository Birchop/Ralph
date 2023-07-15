#include "gaitEngine.h"

gaitEngine::gaitEngine(Leg* allLegs[])
  : allLegs(allLegs) {
}

void gaitEngine::move(float stride, float strafe, float yaw) {
  move(gait, stride, strafe, yaw, stanceWidth, adduction, ground, clearance, increment);
}

void gaitEngine::move(float stride, float strafe, float yaw, int increment) {
  move(gait, stride, strafe, yaw, stanceWidth, adduction, ground, clearance, increment);
}

void gaitEngine::move(float stride, float strafe, float yaw, float ground, int increment) {
  move(gait, stride, strafe, yaw, stanceWidth, adduction, ground, clearance, increment);
}

void gaitEngine::move(Gait gait, float stride, float strafe, float yaw) {
  move(gait, stride, strafe, yaw, stanceWidth, adduction, ground, clearance, increment);
}

void gaitEngine::move(Gait gait, float stride, float strafe, float yaw, float ground) {
  move(gait, stride, strafe, yaw, stanceWidth, adduction, ground, clearance, increment);
}

void gaitEngine::move(Gait gait, float stride, float strafe, float yaw, float stanceWidth, float adduction, float ground, float clearance, int increment) {

  float hStride = stride / 2;
  float hStrafe = strafe / 2;
  float qStride = stride / 4;
  float qStrafe = strafe / 4;
  float hInc = increment / 2;
  float qInc = increment / 2;
  float defX[6] = {80, 80, 0, 0, -80, -80};
  float defY[6] = {
    -154.5f - stanceWidth,
    154.5f + stanceWidth,
    -184.5f - stanceWidth,
    184.5f + stanceWidth,
    -154.5f - stanceWidth,
    154.5f + stanceWidth
  };
  float defaultX[6];
  float defaultY[6];

  yaw = yaw  * PI / 180;

  float xMin[6];
  float yMin[6];

  float xMax[6];
  float yMax[6];

  float xMaxNoRotation[6];
  float yMaxNoRotation[6];



  // Calculate front & rear adduction
  adduction = adduction * PI / 180;
  defaultX[0] = defX[0] * cos(adduction) - defY[0] * sin(adduction);
  defaultY[0] = defX[0] * sin(adduction) + defY[0] * cos(adduction);

  defaultX[1] = defX[1] * cos(-adduction) - defY[1] * sin(-adduction);
  defaultY[1] = defX[1] * sin(-adduction) + defY[1] * cos(-adduction);

  defaultX[2] = defX[2];
  defaultY[2] = defY[2];

  defaultX[3] = defX[3];
  defaultY[3] = defY[3];

  defaultX[4] = defX[4] * cos(-adduction) - defY[4] * sin(-adduction);
  defaultY[4] = defX[4] * sin(-adduction) + defY[4] * cos(-adduction);

  defaultX[5] = defX[5] * cos(adduction) - defY[5] * sin(adduction);
  defaultY[5] = defX[5] * sin(adduction) + defY[5] * cos(adduction);

  float zArc[increment];

  switch (gait) {
    case tri:
      if (j > 1) {
        j = 0;
      };
      for (int i = 0; i < 6; i++) { // Calculate start & end x,y coordinates | apply strafe, stride & yaw
        xMin[i] = defaultX[i] - hStride;
        xMaxNoRotation[i] = defaultX[i] + hStride;

        yMin[i] = defaultY[i] - hStrafe;
        yMaxNoRotation[i] = defaultY[i] + hStrafe;
        if (yaw != 0) {
          xMax[i] = xMaxNoRotation[i] * cos(yaw) - yMaxNoRotation[i] * sin(yaw);
          yMax[i] = yMaxNoRotation[i] * sin(yaw) + xMaxNoRotation[i] * cos(yaw);
        } else {
          xMax[i] = xMaxNoRotation[i];
          yMax[i] = yMaxNoRotation[i];
        }
      }
      for (int i = 0; i < increment; i++) { //Start step loop
        float xDiff[6];
        float yDiff[6];
        for (int k = 0; k < 6; k++) {//Calculate difference between start & end x,y coordinates - current increment
          xDiff[k] = (xMin[k] > xMax[k] ? -1 : 1) * (abs(xMin[k] - xMax[k]) / increment) * i;
          yDiff[k] = (yMin[k] > yMax[k] ? -1 : 1) * (abs(yMin[k] - yMax[k]) / increment) * i;
        }

        if (i <= hInc) {
          zArc[i] = ground + ((clearance / hInc) * i);
        } else {
          int t = increment - i;
          zArc[i] = ground + ((clearance / hInc) * t);
        }

        float xDest[6];
        float yDest[6];
        //calculate current iteration x,y coordinates
        xDest[0] = (j == 0) ? xMax[0] - xDiff[0] : xMin[0] + xDiff[0];
        xDest[1] = (j == 1) ? xMin[1] + xDiff[1] : xMax[1] - xDiff[1];
        xDest[2] = (j == 1) ? xMin[2] + xDiff[2] : xMax[2] - xDiff[2];
        xDest[3] = (j == 0) ? xMax[3] - xDiff[3] : xMin[3] + xDiff[3];
        xDest[4] = (j == 0) ? xMin[4] + xDiff[4] : xMax[4] - xDiff[4];
        xDest[5] = (j == 0) ? xMin[5] + xDiff[5] : xMax[5] - xDiff[5];

        yDest[0] = (j == 0) ? yMax[0] - yDiff[0] : yMin[0] + yDiff[0];
        yDest[1] = (j == 1) ? yMin[1] + yDiff[1] : yMax[1] - yDiff[1];
        yDest[2] = (j == 1) ? yMin[2] + yDiff[2] : yMax[2] - yDiff[2];
        yDest[3] = (j == 0) ? yMax[3] - yDiff[3] : yMin[3] + yDiff[3];
        yDest[4] = (j == 0) ? yMin[4] + yDiff[4] : yMax[4] - yDiff[4];
        yDest[5] = (j == 0) ? yMin[5] + yDiff[5] : yMax[5] - yDiff[5];


        //move legs
        allLegs[0]->moveLegGlobal(xDest[0], yDest[0], (j == 0) ? zArc[i] : ground);
        allLegs[1]->moveLegGlobal(xDest[1], yDest[1], (j == 0) ? ground : zArc[i]);
        allLegs[2]->moveLegGlobal(xDest[2], yDest[2], (j == 0) ? ground : zArc[i]);
        allLegs[3]->moveLegGlobal(xDest[3], yDest[3], (j == 0) ? zArc[i] : ground);
        allLegs[4]->moveLegGlobal(xDest[4], yDest[4], (j == 0) ? zArc[i] : ground);
        allLegs[5]->moveLegGlobal(xDest[5], yDest[5], (j == 0) ? ground : zArc[i]);
        /*Serial.println((j==0) ? "FL| xDest: " + String(xDest[0]) + "  yDest: " + String(yDest[0]) + " Z: " + String(zArc[i]) : "FL| xDest: " + String(xDest[0]) + "  yDest: " + String(yDest[0]) + " Z: " + String(ground));
          Serial.println((j==0) ? "FR| xDest: " + String(xDest[1]) + "  yDest: " + String(yDest[1]) + " Z: " + String(ground) : "FR| xDest: " + String(xDest[1]) + "  yDest: " + String(yDest[1]) + " Z: " + String(zArc[i]));
          Serial.println((j==0) ? "ML| xDest: " + String(xDest[2]) + "  yDest: " + String(yDest[2]) + " Z: " + String(ground) : "ML| xDest: " + String(xDest[2]) + "  yDest: " + String(yDest[2]) + " Z: " + String(zArc[i]));
          Serial.println((j==0) ? "MR| xDest: " + String(xDest[3]) + "  yDest: " + String(yDest[3]) + " Z: " + String(zArc[i]) : "MR| xDest: " + String(xDest[3]) + "  yDest: " + String(yDest[3]) + " Z: " + String(ground));
          Serial.println((j==0) ? "RL| xDest: " + String(xDest[4]) + "  yDest: " + String(yDest[4]) + " Z: " + String(zArc[i]) : "RL| xDest: " + String(xDest[4]) + "  yDest: " + String(yDest[4]) + " Z: " + String(ground));
          Serial.println((j==0) ? "RR| xDest: " + String(xDest[5]) + "  yDest: " + String(yDest[5]) + " Z: " + String(ground) : "RR| xDest: " + String(xDest[5]) + "  yDest: " + String(yDest[5]) + " Z: " + String(zArc[i]));
        */
      }
      j = 1 - j;
      break;

    case trot:
      if (j > 2) {
        j = 0;
      }

      for (int i = 0; i < 2; i++) { // Calculate start & end x,y coordinates | apply strafe, stride & yaw | Pair 1 | Full lift and drop
        int t = (j == 0) ? 0 : ((j == 1) ? 2 : 1); //Cycle legs, need to use the correct defaultX/Y coords for each leg. Probably a better way of doing this..
        int k = (j == 0) ? 5 : ((j == 1) ? 3 : 4);
        xMin[i] = defaultX[(i == 0) ? t : k] - hStride;
        xMaxNoRotation[i] = defaultX[(i == 0) ? t : k] + hStride;

        yMin[i] = defaultY[(i == 0) ? t : k] - hStrafe;
        yMaxNoRotation[i] = defaultY[(i == 0) ? t : k] + hStrafe;
        if (yaw != 0) {
          xMax[i] = xMaxNoRotation[i] * cos(yaw) - yMaxNoRotation[i] * sin(yaw);
          yMax[i] = yMaxNoRotation[i] * sin(yaw) + xMaxNoRotation[i] * cos(yaw);
        } else {
          xMax[i] = xMaxNoRotation[i];
          yMax[i] = yMaxNoRotation[i];
        }
      }

      for (int i = 2; i < 4; i++) { // Calculate start & end x,y coordinates | apply strafe, stride & yaw | Pair 2 | Ending backstroke
        int t = (j == 0) ? 2 : ((j == 1) ? 1 : 0);
        int k = (j == 0) ? 3 : ((j == 1) ? 4 : 5);
        xMin[i] = defaultX[(i == 2) ? t : k] - hStride;
        xMaxNoRotation[i] = defaultX[(i == 2) ? t : k];

        yMin[i] = defaultY[(i == 2) ? t : k] - hStrafe;
        yMaxNoRotation[i] = defaultY[(i == 2) ? t : k];
        if (yaw != 0) {
          xMax[i] = xMaxNoRotation[i] * cos(yaw) - yMaxNoRotation[i] * sin(yaw);
          yMax[i] = yMaxNoRotation[i] * sin(yaw) + xMaxNoRotation[i] * cos(yaw);
        } else {
          xMax[i] = xMaxNoRotation[i];
          yMax[i] = yMaxNoRotation[i];
        }
      }

      for (int i = 4; i < 6; i++) { // Calculate start & end x,y coordinates | apply strafe, stride & yaw | Pair 3 | Starting backstroke
        int t = (j == 0) ? 1 : ((j == 1) ? 0 : 2);
        int k = (j == 0) ? 4 : ((j == 1) ? 5 : 3);
        xMin[i] = defaultX[(i == 4) ? t : k];
        xMaxNoRotation[i] = defaultX[(i == 4) ? t : k] + hStride;

        yMin[i] = defaultY[(i == 4) ? t : k];
        yMaxNoRotation[i] = defaultY[(i == 4) ? t : k] + hStrafe;
        if (yaw != 0) {
          xMax[i] = xMaxNoRotation[i] * cos(yaw) - yMaxNoRotation[i] * sin(yaw);
          yMax[i] = yMaxNoRotation[i] * sin(yaw) + xMaxNoRotation[i] * cos(yaw);
        } else {
          xMax[i] = xMaxNoRotation[i];
          yMax[i] = yMaxNoRotation[i];
        }
      }

      Leg* legs[6]; //pointers and ternary ops.. Legs are paired up (FL, RR | ML, MR | FR, RL) and cycled through, lifting the legs assigned to [0] & [1] whilst moving the body forward with legs [2],[3],[4],[5] at 1/2 the speed of the lifting legs.
      legs[0] = (j == 0) ? allLegs[0] : ((j == 1) ? allLegs[2] : allLegs[1]);
      legs[1] = (j == 0) ? allLegs[5] : ((j == 1) ? allLegs[3] : allLegs[4]);

      legs[2] = (j == 0) ? allLegs[2] : ((j == 1) ? allLegs[1] : allLegs[0]);
      legs[3] = (j == 0) ? allLegs[3] : ((j == 1) ? allLegs[4] : allLegs[5]);

      legs[4] = (j == 0) ? allLegs[1] : ((j == 1) ? allLegs[0] : allLegs[2]);
      legs[5] = (j == 0) ? allLegs[4] : ((j == 1) ? allLegs[5] : allLegs[3]);

      for (int i = 0; i < increment; i++) { //Start step loop
        float xDiff[6];
        float yDiff[6];
        for (int k = 0; k < 6; k++) {//Calculate difference between  x,y coordinates for current increment
          xDiff[k] = (xMin[k] > xMax[k] ? -1 : 1) * (abs(xMin[k] - xMax[k]) / increment) * i;
          yDiff[k] = (yMin[k] > yMax[k] ? -1 : 1) * (abs(yMin[k] - yMax[k]) / increment) * i;
        }

        if (i <= hInc) {
          zArc[i] = ground + ((clearance / hInc) * i);
        } else {
          int t = increment - i;
          zArc[i] = ground + ((clearance / hInc) * t);
        }

        float xDest[6];
        float yDest[6];
        xDest[0] = xMin[0] + xDiff[0];
        xDest[1] = xMin[1] + xDiff[1];
        xDest[2] = xMax[2] - xDiff[2];
        xDest[3] = xMax[3] - xDiff[3];
        xDest[4] = xMax[4] - xDiff[4];
        xDest[5] = xMax[5] - xDiff[5];

        yDest[0] = yMin[0] + yDiff[0];
        yDest[1] = yMin[1] + yDiff[1];
        yDest[2] = yMax[2] - yDiff[2];
        yDest[3] = yMax[3] - yDiff[3];
        yDest[4] = yMax[4] - yDiff[4];
        yDest[5] = yMax[5] - yDiff[5];

        legs[0]->moveLegGlobal(xDest[0], yDest[0], zArc[i]);
        legs[1]->moveLegGlobal(xDest[1], yDest[1], zArc[i]);
        legs[2]->moveLegGlobal(xDest[2], yDest[2], ground);
        legs[3]->moveLegGlobal(xDest[3], yDest[3], ground);
        legs[4]->moveLegGlobal(xDest[4], yDest[4], ground);
        legs[5]->moveLegGlobal(xDest[5], yDest[5], ground);
      }
      j++;

      break;

    case ripple:
      // Calculate start & end x,y coordinates | apply strafe, stride & yaw | Leg 1 | Full lift and drop
      xMin[0] = defaultX[0] - hStride;
      xMaxNoRotation[0] = defaultX[0] + hStride;

      yMin[0] = defaultY[0] - hStrafe;
      yMaxNoRotation[0] = defaultY[0] + hStrafe;
      if (yaw != 0) {
        xMax[0] = xMaxNoRotation[0] * cos(yaw) - yMaxNoRotation[0] * sin(yaw);
        yMax[0] = yMaxNoRotation[0] * sin(yaw) + xMaxNoRotation[0] * cos(yaw);
      } else {
        xMax[0] = xMaxNoRotation[0];
        yMax[0] = yMaxNoRotation[0];
      }

      break;

    case wave:

      break;

    default:
      return;
      break;
  }
}

void gaitEngine::setAdduction(float newAdduction) {
  adduction = newAdduction;
}

void gaitEngine::setIncrement(int newIncrement) {
  increment = newIncrement;
}

void gaitEngine::setStanceWidth(float newStanceWidth) {
  stanceWidth = newStanceWidth;
}

void gaitEngine::setGround(float newGround) {
  ground = newGround;
}

void gaitEngine::setClearance(float newClearance) {
  clearance = newClearance;
}

void gaitEngine::setGait(Gait newGait) {
  gait = newGait;
}
