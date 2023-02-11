//StartPositions() MUST be called in setup or to run once prior to gait beginning
void startPositions() {
  inverseKinematics(tri1,-20,75,-95,60);
  delay(500);
  inverseKinematics(tri2,20,75,-95,60);
  delay(500);
}

// Movement Gaits

/*strideLength = length of a step 
  strideWidth = distance from body 
  strideLift = how high the body is off the ground
  velocity = servo degrees/sec*/
  
//Tri  



  void triWalk(float strideLength, float strideWidth, float strideLift, float velocity) {
  
  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;

  float back = (strideLength/2) - strideLength;
  float mid = (strideLength/2) - (strideLength/2) + 5;
  float ward = strideLength/2;
  
  //Step 1;
  inverseKinematics(tri1, mid, strideWidth, strideLift + 45, velocity, true);   //lift tri 1
  inverseKinematics(tri2, back, strideWidth, strideLift, velocity, false); //move tri 2 - body moving
  inverseKinematics(tri1, ward, strideWidth, strideLift, velocity, false);   //drop tri 1
  

  //Step 2;
  inverseKinematics(tri2, mid, strideWidth, strideLift + 45, velocity, true); //lift tri 2
  inverseKinematics(tri1, back, strideWidth, strideLift, velocity, false); //move tri 1 - body moving
  inverseKinematics(tri2, ward, strideWidth, strideLift, velocity, false); //drop tri 2
  }

  void biWalk(float strideLength, float strideWidth, float strideLift, float velocity) {
  
  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;

  float back = (strideLength/2) - strideLength;
  float mid = (strideLength/2) - (strideLength/2) + 5;
  float ward = strideLength/2;
  
  //Step 1;
  inverseKinematics(pair1, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 1
  inverseKinematics(pair2, back, strideWidth, strideLift, velocity, true); //move pair 2 - body moving
  inverseKinematics(pair3, back, strideWidth, strideLift, velocity, true); //move pair 3 - body moving
  inverseKinematics(pair1, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 2
  inverseKinematics(pair1, ward, strideWidth, strideLift, velocity, true);   //drop pair 1
  inverseKinematics(pair1, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 2

  //Step 2;
  inverseKinematics(pair2, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 1
  inverseKinematics(pair3, back, strideWidth, strideLift, velocity/1.5, true); //move pair 2 - body moving
  inverseKinematics(pair1, back, strideWidth, strideLift, velocity/3, true); //move pair 3 - body moving
  inverseKinematics(tri1, ward, strideWidth, strideLift, velocity, true);   //drop pair 1
  
  }
