/*****************************************************************************-
 *                        Nav.ino
 *                        
 *     Navigations using routes in the "routes.ino" file
 *****************************************************************************/

//const float SEEK_FPS = 2.0;
//const float TURN_FPS = 3.0;

// Seek states
const int SEEK_SETTLE = 0;
const int SEEK_FWD = 1;
int seekState = SEEK_SETTLE;

// Plot states
const int PLOT_SETTLE = 0;
const int PLOT_LEFT = 1;
const int PLOT_RIGHT = 2;
int plotState = PLOT_SETTLE;

// Circle states
const int CIRCLE_ORIENT = 0;
const int CIRCLE_TURN = 2;
int circleState = CIRCLE_ORIENT;

const float STEP_ERROR = -42.42;
int originalStepStringPtr = 0;
struct loc savedOrientationLoc;
struct loc savedPositionLoc;
String stepString = "";
int numLen = 0;
int loopX = 0;
boolean isSaveOrientation = false;
boolean isSavePosition = false;
boolean isFixOrientation = false;
long endStandTime = 0L;
double routeStandKph = 0.0D;
float barrelXAxis = 0.0;
char axisC = ' ';
double dDiff = 0.0D;

float targetBearing = 0.0;
float targetDistance = 0.0;
double degreesPerTick;
double turnStartBearing;
int startTurnTick;
double startOrientation = 0.0;
struct loc startLoc;
int jumpTicksEnd = 0;
float jumpCompKph = 0.0;

float coArray[50];
int coPtr = 0;
float coDist = 0.0;
float coHCorrection = 0.0;



/*****************************************************************************-
 *  steerRoute() called every loop (200/sec).
 *          This is called as the last step in aTp7() for steering.
 *            1. Check if target reached.
 *            2. Set currentLoc.
 *            3. Adjust steering.
 *****************************************************************************/
void steerRoute() {
  boolean isNewRouteStep = false;
  timeRun = timeMilliseconds - timeStart;
  targetWKphRight = targetWKphLeft = targetWKph;

  // See of we need to move to the next route step.
  switch (routeCurrentAction) {
    case 'E':
    case 'L':
    case 'N':
    case 'D':
      isNewRouteStep = true;
      break;

    case 'G':
      if (isTargetReached()) isNewRouteStep = true;
      else steerTarget();
      break;

    case 'K':  // Lock
      if (isStartReceived) {
        isNewRouteStep = true;
        setHeading(rangeAngle(startOrientation));
        currentLoc = startLoc;
        timeStart = timeMilliseconds;
        routeKph = 0.0;
        isRunReady = true;
      }
      else {
        routeKph = 0.0;
      }
      break;

    case 'T': // Turn
      if (isTargetReached()) isNewRouteStep = true;
      else turn();
      break;

    default:
      isRouteInProgress = false;
      sprintf(message, "Illegal step: %d", routeStepPtr);
      break;
  } // end switch()

  // Move to new action if current action is done.
  if (isNewRouteStep) { // Move to next action in list.
    interpretRouteLine(getNextStepString());
    sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction);
    log(message);
//    sendXMsg(SEND_MESSAGE, message);

  }
}



/*****************************************************************************-
 *  interpretRouteLine()
 *      Called every time the end criterion for a route step is reached.
 *      Read the new route step and set the values.
 ************************************************************************/
boolean interpretRouteLine(String ss) {
  float aDiff;
  char char1;

  stepString = ss;
  isSaveOrientation = isSavePosition = isFixOrientation = false;
  originalStepStringPtr = 0;
  Serial.print(stepString);  Serial.print(":   ");
  routeCurrentAction = stepString.charAt(0);
  stepString = stepString.substring(1);
  originalStepStringPtr++;

//  (*currentValSet).v = 2.0;  // restore speed correction.

  switch (routeCurrentAction) {

    case 'D':  // Decelerate
      decelKph = readNum();
      Serial.print(decelKph); Serial.print("   ");
      if (decelKph == STEP_ERROR) return false;
      isDecelActive = true;
      isDecelPhase = false;
      break;

    case 'F':  // Fini
      isRouteInProgress = false;
      break;

    case 'G':  // Go to the next waypoint
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      routeKph = routeScriptKph = readNum();
      Serial.print(routeKph);  Serial.print("   ");
      if (routeKph == STEP_ERROR) return false;
      isDecelPhase = false;
      setTarget();
      break;

     case 'K': // Lock bearing at start
      char1 = stepString.charAt(0);
      if (char1 == 'S') isLockStand = true;
      else if (char1 == 'R') isLockStand = false;
      else return false;
      stepString = stepString.substring(1);
      startLoc = readLoc();
      Serial.print(startLoc.x); Serial.print("  "); Serial.print(startLoc.y); Serial.print("   ");
      if (startLoc.y == STEP_ERROR) return false;
      startOrientation = readNum();
      Serial.print(startOrientation); Serial.print("   ");
      if ((startOrientation < -180.0D) || (startOrientation > 180.0)) return false;
//      startGyroDrift();
      isStartReceived = false;
      break;

    case 'N':  // Name of route
      stripWhite();
      routeTitle = stepString;
      break;

    case 'T':  // Turn at radius ending at waypoint.
      targetLoc = readLoc();
      Serial.print(targetLoc.x); Serial.print("  "); Serial.print(targetLoc.y); Serial.print("   ");
      if (targetLoc.y == STEP_ERROR) return false;
      setTarget();
      aDiff = rangeAngle(targetBearing - imu.gHeading);
      isRightTurn = aDiff > 0.0;

      routeKph = routeScriptKph = readNum();
      Serial.print(routeKph);  Serial.print("   ");
      if (routeKph == STEP_ERROR) return false;
      turnRadius = readNum();
      Serial.print(turnRadius);  Serial.print("   ");
      if (turnRadius == STEP_ERROR) return false;
      endTangentDegrees = readNum();
      Serial.print(endTangentDegrees);  Serial.print("   ");
      if (endTangentDegrees == STEP_ERROR) return false;

      pivotBearing = endTangentDegrees + (isRightTurn ?  90.0 :  -90.0);
      pivotBearing = rangeAngle(pivotBearing);
      //      if (isRightTurn) {
      pivotLoc.x = targetLoc.x + (sin(pivotBearing * DEG_TO_RAD) * turnRadius);
      pivotLoc.y = targetLoc.y + (cos(pivotBearing * DEG_TO_RAD) * turnRadius);
      //      } else {
      //        pivotLoc.x = targetLoc.x + (sin(pivotBearing * DEG_TO_RAD) * turnRadius);
      //        pivotLoc.y = targetLoc.y - (cos(pivotBearing * DEG_TO_RAD) * turnRadius);
      //      }
      Serial.println();
      sprintf(message, "       turn = %s ", (isRightTurn ? "R" : "L"));
//      sendBMsg(SEND_MESSAGE, message);
      Serial.print(message);
      sprintf(message, "      pivotLoc.: %5.2f,%5.2f", pivotLoc.x, pivotLoc.y);
//      sendBMsg(SEND_MESSAGE, message);
      Serial.print(message);
      setTarget();
      break;

    default:
      Serial.println("Step error: Illegal command.");
      return false;
  }
  Serial.println();
  //  sprintf(message, "Step %d: %c", routeStepPtr, routeCurrentAction); isNewMessage = true;
  return true;
}



//#define RAD_TURN 10.0
#define RAD_TURN 4.0
#define END_STEER 0.5
/*****************************************************************************-
 *  steerTarget() Find the correct heading to the target and adjust the
 *               wheel speeds to turn toward the target.  As tp approaches
 *               the target, use the originalTargetBearing.
 ************************************************************************/
void steerTarget() {
  double speedAdjustment;

  if (targetDistance < 1.0) {
    speedAdjustment = 0.0;
  } else {
    double aDiff = rangeAngle(targetBearing - imu.gHeading);
    double d = (aDiff > 0.0) ? 1.0 : -1.0;

    speedAdjustment = (wKph / RAD_TURN) * 0.64 * d;

    // Reduce adjustment proportionally if less than X degrees.
    if (abs(aDiff) < 5.0) {
      speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
    }

    // Reduce speed adjustment as speed increases
    if (tpKph > 11.0) speedAdjustment *= 0.3;
    else if (tpKph > 8.0) speedAdjustment *= 0.5;
    else if (tpKph > 5.0) speedAdjustment *= 0.7;
  }
  targetWKphRight = targetWKph - speedAdjustment;
  targetWKphLeft = targetWKph + speedAdjustment;
}


/*****************************************************************************-
 *  steerHeading() Adjust the wheel speeds to turn toward the 
 *                 targetHeading. 
 ***********************************************************************/
void steerHeading() {
  double speedAdjustment;

  double aDiff = rangeAngle(targetBearing - imu.gHeading);
  double d = (aDiff > 0.0) ? 1.0 : -1.0;

  speedAdjustment = (wKph / RAD_TURN) * 0.64 * d;

  // Reduce adjustment proportionally if less than X degrees.
  if (abs(aDiff) < 5.0) {
    speedAdjustment = (abs(aDiff) / 5.0) * speedAdjustment;
  }

  // Reduce speed adjustment as speed increases
  if (tpKph > 11.0) speedAdjustment *= 0.3;
  else if (tpKph > 8.0) speedAdjustment *= 0.5;
  else if (tpKph > 5.0) speedAdjustment *= 0.7;
    
  targetWKphRight = targetWKph - speedAdjustment;
  targetWKphLeft = targetWKph + speedAdjustment;
}



/*****************************************************************************-
 *  turn() Turn with a given radius.
 ************************************************************************/
void turn() {
  float speedAdjustment, headingError;
  float xDist, yDist, radiusAngle, targetTurnHeading;

  float d = isRightTurn ? 1.0 : -1.0;
  float radiusDiff = (wKph / turnRadius) * 0.54 * d;

  if (isRightTurn) {
    xDist = (currentLoc.x - pivotLoc.x);
    yDist = (currentLoc.y - pivotLoc.y);
    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
    targetTurnHeading = rangeAngle(radiusAngle + 90.0);
    headingError = rangeAngle(imu.gHeading - targetTurnHeading);
  } else {
    xDist = (currentLoc.x - pivotLoc.x);
    yDist = (currentLoc.y - pivotLoc.y);
    radiusAngle = atan2(xDist, yDist) * RAD_TO_DEG;
    targetTurnHeading = rangeAngle(radiusAngle - 90.0);
    headingError = rangeAngle(imu.gHeading - targetTurnHeading);
  }

  float radiusError = sqrt((xDist * xDist) + (yDist * yDist)) - turnRadius;
  float radiusAdjustment = radiusError * 0.3 * d;
  float headingAdjustment = -headingError * 0.03;
   speedAdjustment = radiusDiff + headingAdjustment + radiusAdjustment;

  targetWKphRight = targetWKph - speedAdjustment;
  targetWKphLeft = targetWKph + speedAdjustment;
}

/*****************************************************************************-
 *  getRouteKph() Limit acceleration to prevent too rapid acceleration--
 *                particularly when starting from zero from the stand.
 *                Also starts deceleration at end of step.
 *                Returns true if is decelerating and reached target Kph.
 ************************************************************************/
float getRouteKph() {
  float stopDistance;

  switch (routeCurrentAction) {
    case 'C':
      routeKph = routeScriptKph;
      break;
      
    case 'G':
    case 'T':
      // Check to see if we need to start the decel phase.
      if (isDecelActive && !isDecelPhase) {
        stopDistance = (tpKph * tpKph) / 5.58;
        if (targetDistance <= stopDistance) {
          isDecelPhase = true;
        }
      }

      if (isDecelPhase) {
        routeKph = tpKph - 5.0;
      } else {
        if (routeScriptKph < 0.0) {
          routeKph = routeScriptKph;
        } else { // Going forward.
          if (timeRun < 1000) {  // taking off from stand?
            routeKph = (((float) timeRun) / 200.0) + 0.5; // Accelerate to 5.5 in 1 sec
            if (routeKph > routeScriptKph) routeKph = routeScriptKph;
          } else { // Accelerate to target speed.
//            float inc = (routeScriptKph - tpKph) * 2.0;
//            inc = constrain(inc, -5.0, 5.0);
//            routeKph = tpKph + inc;
              routeKph = routeScriptKph;
          }
        }
      }
      break;

    case 'H':
    case 'J':
    default:
      routeKph = routeScriptKph;
      break;
   
  }
  return routeKph;
}



/*****************************************************************************-
 *  isTargetReached()  Return true if within 1 ft of target and is
 *                     moving away from the target.
 *                     Return true if at end of decel.
 ***********************************************************************/
boolean isTargetReached() {
  const int RETREAT_TIMES = 10;
  const int RETREAT_DISTANCE = 2.0;
  static double lastTargetDist = 10000.0D;
  static int timesReached = 0;  if (isDecelActive && isDecelPhase) {
    if (wKph <= 0.8) {
      isDecelActive = isDecelPhase = false;
      return true;
    }
  } else {
    setTarget();
    if (targetDistance < RETREAT_DISTANCE) {
      boolean isCloser = ((lastTargetDist - targetDistance) >= 0.0D);
      lastTargetDist = targetDistance;
      if (isCloser) { // Getting closer?
        timesReached = 0;
      } else {
        timesReached++;
        // Return true after Nth time.
        if (timesReached > RETREAT_TIMES) {
          timesReached = 0;
          return true;
        }
      }
    } else {
      timesReached = 0;
    }
  }
  return false;
}



/*****************************************************************************-
 *  setTarget() Set the new targetBearing and targetDistance from
 *              the currentLoc.
 ************************************************************************/
void setTarget() {
  double x =  targetLoc.x - currentLoc.x;
  double y = targetLoc.y - currentLoc.y;
  targetBearing = atan2(x, y) * RAD_TO_DEG;
  double xTargetDist = currentLoc.x - targetLoc.x;
  double yTargetDist = currentLoc.y - targetLoc.y;
  targetDistance = sqrt((xTargetDist * xTargetDist) + (yTargetDist * yTargetDist));
}



/*****************************************************************************-
 *  Script parsing routines
 ***********************************************************************/
double readNum() {
  stripWhite();
  double num = stepString.toFloat();
  stripNum();
  if (numLen == 0) return STEP_ERROR;
  return num;
}

struct loc readLoc() {
  struct loc locLoc = {STEP_ERROR, STEP_ERROR};
  stripWhite();
  locLoc.x = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  if (stepString.charAt(0) != ',') return locLoc;
  stepString = stepString.substring(1);
  //  stripWhite();
  double y = stepString.toFloat();
  stripNum();
  if (numLen == 0) return locLoc;
  locLoc.y = y;
  return locLoc;
}

void stripWhite() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c != ' ') && (c != '\t')) break;
    ptr++;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
}


void stripNum() {
  int ptr = 0;
  int len = stepString.length();
  while (true) {
    if (ptr >= len) break;
    char c = stepString.charAt(ptr);
    if ((c == '.') || (c == '-') || ((c >= '0') && (c <= '9'))) ptr++;
    else break;
  }
  originalStepStringPtr += ptr;
  stepString = stepString.substring(ptr);
  numLen = ptr;
}

char readChar() {
  stripWhite();
  if (stepString.length() == 0) return 0;
  char c = stepString.charAt(0);
  stepString = stepString.substring(1);
  return c;
}
