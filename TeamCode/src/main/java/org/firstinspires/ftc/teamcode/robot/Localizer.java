package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Localizer {

   LinearOpMode opMode;
//   HardwareMap hardwareMap;
//   Gamepad gamepad1;
//   Gamepad gamepad2;
   Telemetry telemetry;
   Robot robot;

   DcMotorEx odoXL, odoY, odoXR;
   public long encoderY, encoderXL, encoderXR;
   long encoderY0, encoderXL0, encoderXR0;
   double odoHeading, odoHeading0;
   double imuHeading, imuHeading0;
   public double globalHeading, globalHeading0;
   public double yPos, xPos;

   private static double eTicksPerInch = 82300 / 48;
   private static double eTicksPerRotate = 171500; //171738.8; //170000;

   /* Constructor */
   public Localizer(LinearOpMode opMode, Robot robot){
      construct(opMode, robot);
   }

   void construct(LinearOpMode opMode, Robot robot){
      this.opMode = opMode;
//      this.hardwareMap = opMode.hardwareMap;
//      this.gamepad1 = opMode.gamepad1;
//      this.gamepad2 = opMode.gamepad2;
      this.telemetry = opMode.telemetry;
      this.robot = robot;
   }

   public void loop() {
      encoderY = odoY.getCurrentPosition();
      encoderXL = odoXL.getCurrentPosition();
      encoderXR = odoXR.getCurrentPosition();

      imuHeading = robot.returnImuHeading();
      odoHeading = getOdoHeading();

      globalHeading = imuHeading;  // for now, assume IMU is best data

      updateXY();
   }

   public void init() {
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
   }

   // get heading from the odometry; accuracy varies :-(
   private double getOdoHeading() {
      double diffX;
      diffX = encoderXR - encoderXL;
      diffX = diffX % eTicksPerRotate;
      diffX = diffX / eTicksPerRotate * 360;
      if (diffX > 180) diffX -= 360;
      if (diffX < -180) diffX += 360;
      return diffX;
   }

   public double returnOdoHeading() {
      return odoHeading;
   }

   public double returnGlobalHeading() {
      return globalHeading;
   }
   // Get XY position data from odometry wheels
   private void updateXY () {
      double deltaEncX, deltaEncY, avgHeading;
      double myHeading;

      //deltaEncX = (encoderXL - encoderXL0) / eTicksPerInch;
      deltaEncX = (encoderXL + encoderXR - encoderXL0 - encoderXR0) / 2.0 / eTicksPerInch;
      deltaEncY = (encoderY - encoderY0) / eTicksPerInch;

      //myHeading = globalHeading;
      // Future - figure out average heading.  Challenge is the wrap.  Maybe use getError, /2, add to heading???
      //myHeading = getAvgHeading(odoHeading0,odoHeading);
      myHeading = getAvgHeading(imuHeading0, imuHeading);

      telemetry.addData ("My Average Heading", myHeading);

      xPos = xPos + deltaEncX * Math.cos(Math.toRadians(myHeading));
      yPos = yPos + deltaEncX * Math.sin(Math.toRadians(myHeading));

      xPos = xPos + deltaEncY * Math.sin(Math.toRadians(myHeading));
      yPos = yPos - deltaEncY * Math.cos(Math.toRadians(myHeading));

      encoderXL0 = encoderXL;
      encoderY0 = encoderY;
      encoderXR0 = encoderXR;
      imuHeading0 = imuHeading;
      odoHeading0 = odoHeading;
   }

   // average of two headings
   public double getAvgHeading (double firstHeading, double secondHeading) {
      double robotHeading;

      // find the difference between them
      robotHeading = secondHeading - firstHeading;

      // based on sampling rate, assume large values wrapped
      if (robotHeading > 180) robotHeading -= 360;
      if (robotHeading <= -180) robotHeading += 360;

      robotHeading /= 2;

      robotHeading += firstHeading;

       /* // make them positive angles first
        if (firstHeading < 0) firstHeading += 360;
        if (secondHeading < 0) secondHeading += 360;

        robotHeading = (secondHeading + firstHeading) / 2; */

      // then recalculate to -179 to +180 range
      while (robotHeading > 180)  robotHeading -= 360;
      while (robotHeading <= -180) robotHeading += 360;

      return robotHeading;
   }
}
