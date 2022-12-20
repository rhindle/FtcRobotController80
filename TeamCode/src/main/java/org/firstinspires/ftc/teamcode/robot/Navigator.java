package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Navigator {

//   LinearOpMode opMode;
//   HardwareMap hardwareMap;
//   Gamepad gamepad1;
//   Gamepad gamepad2;
   Telemetry telemetry;
   Robot robot;
   Localizer localizer;
   Drivetrain drivetrain;

   public double v0, v2, v1, v3;
   double driveSpeed, driveAngle, rotate;
   double maxSpeed = 1;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   boolean useFieldCentricDrive = true;
   boolean useHeadingHold = true;
   long headingDelay = System.currentTimeMillis();

   public double targetX, targetY, targetRot;
   public int navigate = 0;
   boolean accurate = true;
   int navStep = 0;
   private ElapsedTime navTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
   int lastStep = 0;

   Position testPosition = null;

   /* Constructor */
   public Navigator(Robot robot){//}, Localizer localizer,Drivetrain drivetrain){
      construct(robot);//, localizer, drivetrain);
   }

   void construct(Robot robot){//}, Localizer localizer, Drivetrain drivetrain){
//      this.opMode = opMode;
//      this.hardwareMap = opMode.hardwareMap;
//      this.gamepad1 = opMode.gamepad1;
//      this.gamepad2 = opMode.gamepad2;
      this.robot = robot;
      this.telemetry = robot.telemetry;
      this.localizer = robot.localizer;
      this.drivetrain = robot.drivetrain;
   }

   public void init() {
      v0 = 0.0;
      v1 = 0.0;
      v2 = 0.0;
      v3 = 0.0;
      storedHeading = localizer.returnGlobalHeading();
   }

   public void loop() {

      // State machine-ish.  Needs embetterment.  This will determine motor powers then set them
      if (navigate == 1) {
         autoDrive();
      }
      else if (navigate == 2) {
         scriptedNavigation();
      }
      else {
         userDrive();
      }

      drivetrain.setDrivePowers(v0, v1, v2, v3);
   }


   // type, x, y, rot
   NavActions[] autoScript2 =  new NavActions[] {
      new NavActions(Actions.MOVEACCURATE, 0, 0, 0),
      new NavActions(Actions.MOVEACCURATE, 24, -20, -90),
      new NavActions(Actions.PAUSE,500),
      new NavActions(Actions.MOVETRANSITION,24, 0, 90),
      new NavActions(Actions.MOVETRANSITION,48, 0, 90),
      new NavActions(Actions.MOVEACCURATE,48, 24, 90),
      new NavActions(Actions.PAUSE,1000),
      new NavActions(Actions.MOVETRANSITION,48, 4, 90),
      new NavActions(Actions.MOVEACCURATE,24, 0, 0),
      new NavActions(Actions.PAUSE,1000),
      new NavActions(Actions.MOVEACCURATE,0,0,0),
      new NavActions(Actions.ENDROUTINE),
   };

   // Simple state machine for scripted navigation actions
   public void scriptedNavigation() {

      // reset the timer each new step (could be used to time out actions;
      // is used for delays)
      if (navStep != lastStep) {
         navTime.reset();
         lastStep=navStep;
      }

      Actions currentAction = autoScript2[navStep].action;

      telemetry.addData("NavStep", navStep);

      // accurate position
      if (currentAction == Actions.MOVEACCURATE) {
         targetX=autoScript2[navStep].parameter1;
         targetY=autoScript2[navStep].parameter2;
         targetRot=autoScript2[navStep].parameter3;
         accurate=true;
         autoDrive();
      }
      // transitional position
      else if (currentAction == Actions.MOVETRANSITION) {
         targetX=autoScript2[navStep].parameter1;
         targetY=autoScript2[navStep].parameter2;
         targetRot=autoScript2[navStep].parameter3;
         accurate=false;
         autoDrive();
      }
      // delay
      else if (currentAction == Actions.PAUSE) {
         if (navTime.milliseconds()>=autoScript2[navStep].parameter1) navStep++;
      }
      // end the navigation
      else if (currentAction == Actions.ENDROUTINE) {
         navigate=0;
      }
   }

   // Determine motor speeds when under automatic control
   public void autoDrive () {
      double distance, deltaX, deltaY, deltaRot, pDist, pRot, navAngle;
//      v0 = 0.0;
//      v2 = 0.0;
//      v1 = 0.0;
//      v3 = 0.0;
      deltaX = targetX - localizer.xPos;  // error in x
      deltaY = targetY - localizer.yPos;  // error in y
      telemetry.addData("DeltaX", JavaUtil.formatNumber(deltaX, 2));
      telemetry.addData("DeltaY", JavaUtil.formatNumber(deltaY, 2));
      deltaRot = getError(targetRot);  // error in rotation
      distance = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));  // distance (error) from xy destination

      //exit criteria if destination has been adequately reached
      if (accurate==false && distance<2) {  // no rotation component here
         navStep++;
         return;
      }
      if (distance<0.5 && Math.abs(deltaRot)<0.2) {
         navStep++;
         return;
      }

      navAngle = Math.toDegrees(Math.atan2(deltaY,deltaX));  // angle to xy destination (vector when combined with distance)
      // linear proportional at 12", minimum 0.025
      pDist = Math.max(Math.min(distance/12,1),0.025);
      if (accurate==false) pDist = 1;  // don't bother with proportional when hitting transitional destinations
      // linear proportional at 15°, minimum 0.025
      pRot = Math.max(Math.min(Math.abs(deltaRot)/15,1),0.025)*Math.signum(deltaRot)*-1;

      telemetry.addData("NavDistance", JavaUtil.formatNumber(distance, 2));
      telemetry.addData("NavAngle", JavaUtil.formatNumber(navAngle, 2));
      telemetry.addData("NavRotation", JavaUtil.formatNumber(deltaRot, 2));
      telemetry.addData("pDist", JavaUtil.formatNumber(pDist, 2));
      telemetry.addData("pRot", JavaUtil.formatNumber(pRot, 2));

      navAngle -= localizer.globalHeading;  // need to account for how the robot is oriented
      double autoSpeed = pDist * 1;  // 1 here is maxspeed; could be turned into a variable
      // the following adds the mecanum X, Y, and rotation motion components for each wheel
      v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + pRot;
      v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + pRot;
      v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - pRot;
      v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - pRot;

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   // Determine motor speeds when under driver control
//   public void userDrive (double driveSpeed, double driveAngle, double rotate) {
   public void userDrive () {
      driveSpeed = Math.pow(driveSpeed, 1);
      v0 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v2 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v1 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) - rotate;
      v3 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) - rotate;

/*      // scale so average motor speed is not more than maxSpeed
      double averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3)));
      averageValue = averageValue / maxSpeed;
      if (averageValue > 1) {
         v0 /= averageValue;
         v2 /= averageValue;
         v1 /= averageValue;
         v3 /= averageValue;
      }*/

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   // Get heading error
   public double getError(double targetAngle) {
      double robotError;
      // calculate error in -179 to +180 range  (
      //robotError = targetAngle - getHeading();
      robotError = targetAngle - robot.returnImuHeading();
      while (robotError > 180)  robotError -= 360;
      while (robotError <= -180) robotError += 360;
      return robotError;
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

   public void beginScriptedNav() {
      navigate=2;
      navStep=0;
      navTime.reset();
   }

   public void beginAutoDrive() {
      navigate=1;
   }

   public void setTargetToZeroPosition() {
      targetX=0;
      targetY=0;
      targetRot=0;
   }

   public void cancelAutoNavigation() {
      navigate=0;
      storedHeading = localizer.returnGlobalHeading();
   }

   public void setTargetToCurrentPosition() {
      targetX = Math.round(localizer.xPos);
      targetY = Math.round(localizer.yPos);
      targetRot = Math.round(localizer.returnGlobalHeading());
   }

   public void setDeltaHeading() {
      deltaHeading = storedHeading;
   }

   public void toggleFieldCentricDrive() {
      useFieldCentricDrive = !useFieldCentricDrive;
   }

   public void toggleHeadingHold() {
      useHeadingHold = !useHeadingHold;
      storedHeading = localizer.returnGlobalHeading();
   }

   public void setTargetByDelta(double X, double Y, double R) {
      targetX += X;
      targetY += Y;
      targetRot += R;
   }

   public void setTargetByDeltaRelative(double X, double Y, double R) {
      double rot = localizer.globalHeading;
//      slamraRobotPose.X = sX + (rX*Math.cos(Math.toRadians(sR)) - rY*Math.sin(Math.toRadians(sR)));
//      slamraRobotPose.Y = sY + (rX*Math.sin(Math.toRadians(sR)) + rY*Math.cos(Math.toRadians(sR)));
      targetX = targetX + (X * Math.cos(Math.toRadians(rot)) - Y * Math.sin(Math.toRadians(rot)));
      targetY = targetY + (X * Math.sin(Math.toRadians(rot)) + Y * Math.cos(Math.toRadians(rot)));
      targetRot += R;
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;
      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;
      }
      // Modify for Hold Angle
      if (useHeadingHold) {
         // Correct the heading if not currently being controlled
         if (headingDelay <= System.currentTimeMillis()) {  // shouldn't need to check if == 0
            this.rotate = getError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
         }
      }
   }

   public void handleRotate(double rotate) {
      // overall plan here is to deal with IMU latency
      if (rotate != 0) {
         storedHeading = localizer.returnGlobalHeading();
         headingDelay = System.currentTimeMillis() + 250;  // going to ignore the possibility of overflow
      } else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = localizer.returnGlobalHeading();
      }
   }

//   public enum Actions {
//      MOVEACCURATE,
//      MOVETRANSITION,
//      PAUSE,
//      ENDROUTINE
//   }

//   public class NavActions {
//      Actions action;
//      double parameter1;
//      double parameter2;
//      double parameter3;
//
//      public NavActions(Actions a, double x, double y, double r) {
//         action = a;
//         parameter1 = x;
//         parameter2 = y;
//         parameter3 = r;
//      }
//      public NavActions(Actions a) {
//         action = a;
//         parameter1 = 0;
//         parameter2 = 0;
//         parameter3 = 0;
//      }
//      public NavActions(Actions a, double p) {
//         action = a;
//         parameter1 = p;
//         parameter2 = 0;
//         parameter3 = 0;
//      }
//   }

}