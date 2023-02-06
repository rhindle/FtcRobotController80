package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Tools.Position;

public class Localizer2 {

   Telemetry telemetry;
   Robot robot;

   DcMotorEx odoXL, odoY, odoXR;
   public long encoderY, encoderXL, encoderXR;
   long encoderY0, encoderXL0, encoderXR0;
   double odoHeading, odoHeading0;
   double imuHeading, imuHeading0;
   public double globalHeading, globalHeading0;
   public double yPos, xPos;
   public boolean useFusedHeading = true;

   Position odoRobotOffset = new Position (2.25,0,0);          // map odo to robot (so it holds turn position better)
   public Position odoFieldStart = new Position (-36,63,-90);  // field start position [blue right slot]

   Position odoRawPose = new Position (0,0,0);                 // original calculation of position before transforms applied
   Position odoRobotPose = new Position ();                             // odo mapped to robot position (minor change)
   Position odoFinalPose = new Position ();                             // odo mapped to field
   Position odoFieldOffset = new Position ();                           // transform from initial position (more relevant for slamra!)
   public Position robotPosition = new Position ();

   private static final double eTicksPerInch = 82300 / 48.0;
   private static final double eTicksPerRotate = 169619;

   //2022-12-22 Measured 10 rot: 846684,850800; 845875,851775; 845127,845543; 848073,850867
   //                            169748.4		169765		169067		169894		==>  169618.6

   public Localizer2(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
      this.robot = robot;
      this.telemetry = robot.telemetry;
   }

   public void init() {
      if (!robot.useODO) {
         robotPosition.X = 0;   // done this way to not break the link back to Navigator class
         robotPosition.Y = 0;
         return;
      }
      odoY = robot.motor0B;
      odoXR = robot.motor1B;
      odoXL = robot.motor2B;

      odoY.setDirection(DcMotorEx.Direction.FORWARD);
      odoXL.setDirection(DcMotorEx.Direction.REVERSE);
      odoXR.setDirection(DcMotorEx.Direction.REVERSE);
      odoY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      odoXL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      odoXR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

      encoderY0 = odoY.getCurrentPosition();
      encoderXL0 = odoXL.getCurrentPosition();
      encoderXR0 = odoXR.getCurrentPosition();
      imuHeading0 = robot.returnImuHeading(true);
      odoHeading0 = getOdoHeading();
      globalHeading0 = imuHeading0;

      // odo start position is 0,0,0; imu should also read 0.  odoRawPose is already 0,0,0
      updateOdoRobotPose();
      setOdoFinalPose();
      setOdoFieldOffset();
   }

   public void loop() {
      if (!robot.useODO) {
         imuHeading = robot.returnImuHeading();
         globalHeading = imuHeading;
         robotPosition.R = globalHeading;
         return;
      }

      /* Update encoder readings */
      encoderY = odoY.getCurrentPosition();
      encoderXL = odoXL.getCurrentPosition();
      encoderXR = odoXR.getCurrentPosition();

      /* Update heading */
      imuHeading = robot.returnImuHeading();
      odoHeading = getOdoHeading();
      globalHeading = fusedHeading();

      /* Calculate position */
      updateXY();
      odoRawPose = new Position(xPos, yPos, globalHeading);
      updateOdoRobotPose();
      setOdoFinalPose();

      /* Update robot position */
      robotPosition.X = odoFinalPose.X;   // done this way to not break the link back to Navigator class
      robotPosition.Y = odoFinalPose.Y;   // (creating a new object messes things up)
      robotPosition.R = odoFinalPose.R;

   }

   private double fusedHeading() {
      /* Don't fuse if the flag isn't set */
      if (!useFusedHeading) return imuHeading;
      /* Use imuHeading only if it's settled */
      if (Math.abs(Functions.normalizeAngle(imuHeading - imuHeading0)) < 0.5) return imuHeading;
      /* Otherwise fuse it with odoHeading data */
      return Functions.normalizeAngle(globalHeading0 + (odoHeading - odoHeading0));
   }

   public void toggleUseFusedHeading() {
      useFusedHeading = !useFusedHeading;
   }

   /* Get heading from the odometry ... accuracy varies :-(  */
   private double getOdoHeading() {
      double diffX;
      diffX = encoderXR - encoderXL;
      diffX = diffX % eTicksPerRotate;
      diffX = diffX / eTicksPerRotate * 360;
      return Functions.normalizeAngle(diffX);
   }

   public double returnOdoHeading() {
      return odoHeading;
   }

   public double returnGlobalHeading() {
      return globalHeading;
   }

   /* Get XY position data from odometry wheels */
   private void updateXY () {
      // this function could use some cleanup!
      double deltaEncX, deltaEncY;
      double myHeading;

      //deltaEncX = (encoderXL - encoderXL0) / eTicksPerInch;  //for single encoder; no heading would be possible
      deltaEncX = (encoderXL + encoderXR - encoderXL0 - encoderXR0) / 2.0 / eTicksPerInch;
      deltaEncY = (encoderY - encoderY0) / eTicksPerInch;

      myHeading = getAvgHeading(globalHeading0, globalHeading);

      telemetry.addData ("My Average Heading", myHeading);

      xPos = xPos + deltaEncX * Math.cos(Math.toRadians(myHeading));
      yPos = yPos + deltaEncX * Math.sin(Math.toRadians(myHeading));

      xPos = xPos + deltaEncY * Math.sin(Math.toRadians(myHeading));
      yPos = yPos - deltaEncY * Math.cos(Math.toRadians(myHeading));

      /* Store current values for next loop */
      encoderXL0 = encoderXL;
      encoderY0 = encoderY;
      encoderXR0 = encoderXR;
      imuHeading0 = imuHeading;
      odoHeading0 = odoHeading;
      globalHeading0 = globalHeading;
   }

   /* Calculate average of two headings */
   public double getAvgHeading (double firstHeading, double secondHeading) {
      double robotHeading;
      /* Find the difference between them; based on sampling rate, assume large values wrapped */
      robotHeading = Functions.normalizeAngle(secondHeading - firstHeading);
      robotHeading /= 2;
      robotHeading += firstHeading;
      return Functions.normalizeAngle(robotHeading);
   }

   void updateOdoRobotPose() {
      //pos1 = odoRawPose, pos2 = odoRobotOffset
      odoRobotPose = transformPosition(odoRawPose, odoRobotOffset);
   }

   void setOdoFinalPose() {
      //pos1 = odoFieldOffset, pos2 = odoRobotPose
      odoFinalPose = transformPosition(odoFieldOffset, odoRobotPose);
   }

   void setOdoFieldOffset() {
      Position fS = odoFieldStart;
      Position rP = odoRobotPose;
      double offsetR = fS.R - rP.R;
      odoFieldOffset = new Position (
              (fS.X - (rP.X*Math.cos(Math.toRadians(offsetR)) - rP.Y*Math.sin(Math.toRadians(offsetR)))),
              (fS.Y - (rP.X*Math.sin(Math.toRadians(offsetR)) + rP.Y*Math.cos(Math.toRadians(offsetR)))),
              (offsetR)
      );
   }

   Position transformPosition(Position pos1, Position pos2) {
      return new Position(
              (pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R)))),
              (pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R)))),
              (pos1.R + pos2.R)
      );
   }

   public void addTeleOpTelemetry() {
      telemetry.addData("raw__", odoRawPose.toString(2));
      telemetry.addData("robot", odoRobotPose.toString(2));
      telemetry.addData("final", odoFinalPose.toString(2));
   }
}