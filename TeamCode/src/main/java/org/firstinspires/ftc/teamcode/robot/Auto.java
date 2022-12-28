package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto {

//   LinearOpMode opMode;
//   HardwareMap hardwareMap;
//   Gamepad gamepad1;
//   Gamepad gamepad2;
   Telemetry telemetry;
   Robot robot;
   Navigator navigator;
   public Controls controls;

   /* Constructor */
   public Auto(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
//      this.opMode = opMode;
//      this.hardwareMap = opMode.hardwareMap;
//      this.gamepad1 = opMode.gamepad1;
//      this.gamepad2 = opMode.gamepad2;
      this.robot = robot;
      this.telemetry = robot.opMode.telemetry;
      this.navigator = robot.navigator;
      controls = new Controls(robot);
   }

   public boolean driveTo(double X, double Y, double R) {
      return driveTo(X,Y,R,1,0);
   }
   public boolean driveTo(double X, double Y, double R, int accuracy) {
      return driveTo(X,Y,R,accuracy,0);
   }
//    private void driveTo(double X, double Y, double R, long timeout) {
//        driveTo(X,Y,R,1,timeout);
//    }

   public boolean driveTo(double X, double Y, double R, int accuracy, long timeout) {
      boolean enableTimeout = true;
      boolean success = false;
      if (!robot.opMode.opModeIsActive()) return false;    //exit right away if stopped
      if (timeout == 0) enableTimeout = false;
      timeout += System.currentTimeMillis();  //this is the future time at which to exit

      navigator.setTargetAbsolute(X,Y,R);
      navigator.setAccuracy(accuracy);
      navigator.setAutoDrive(true);

      while (robot.opMode.opModeIsActive()) {
         telemetry.addData ("Timeout", System.currentTimeMillis() - timeout);
         if (enableTimeout && timeout < System.currentTimeMillis()) break;
         robotLoop();
         if (navigator.isOnTargetByAccuracy()) {
            success = true;
            break;
         }
      }

      navigator.setAutoDrive(false);
      navigator.setUseHoldPosition(success);  // hold the position only if it made it there!
      return success;
   }

   public void delay(long ms) {
      delay(ms,false);
   }

   public void delay(long ms, boolean blocking) {
      if (blocking) {
         robot.opMode.sleep(ms);
      } else {
         ms += System.currentTimeMillis();
         while (robot.opMode.opModeIsActive() && ms > System.currentTimeMillis()) {
            robotLoop();
         }
      }
   }

   public void robotLoop() {
      if (robot.opMode.opModeIsActive()) {
         robot.loop();               // Clears bulk data and reads IMU
         robot.buttonMgr.loop();           // Processes digital controller input
         robot.localizer.loop();           // Updates odometry X, Y, Rotation
         robot.sensors.loop();       // Update distance sensors, etc.

         addTelemetryLoopStart();

         controls.loop();            // Acts on user controls
         navigator.loop();           // Automatic navigation actions

         addTelemetryLoopEnd();
         telemetry.update();
      }
   }

   private void addTelemetryLoopStart() {
      telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(Support.calculateLoopTime(), 0));
      telemetry.addData("heading", JavaUtil.formatNumber(robot.returnImuHeading(),2));
      telemetry.addData("rangeL", String.format("%.01f in", robot.sensors.distL));
      telemetry.addData("rangeM", String.format("%.01f in", robot.sensors.distM));
      telemetry.addData("rangeR", String.format("%.01f in", robot.sensors.distR));
   }

   private void addTelemetryLoopEnd() {
      telemetry.addData("r (magnitude)", controls.DriveSpeed);
      telemetry.addData("robotAngle", controls.DriveAngle);
      telemetry.addData("rotate", controls.Rotate);
      telemetry.addData("storedHeading", JavaUtil.formatNumber(navigator.storedHeading, 2));
      telemetry.addData("deltaHeading", JavaUtil.formatNumber(navigator.deltaHeading, 2));
//      telemetry.addData("error", JavaUtil.formatNumber(currentError, 2));
      telemetry.addData("v0", JavaUtil.formatNumber(navigator.v0, 2));
      telemetry.addData("v1", JavaUtil.formatNumber(navigator.v2, 2));
      telemetry.addData("v2", JavaUtil.formatNumber(navigator.v1, 2));
      telemetry.addData("v3", JavaUtil.formatNumber(navigator.v3, 2));
      telemetry.addData("rot about Z", JavaUtil.formatNumber(robot.returnImuHeading(), 2));
      telemetry.addData("odo Heading", JavaUtil.formatNumber(robot.localizer.returnOdoHeading(), 2));
      telemetry.addData("Target X", navigator.targetX);
      telemetry.addData("Target Y", navigator.targetY);
      telemetry.addData("Target Rot", navigator.targetRot);
      telemetry.addData("OdoY", robot.localizer.encoderY);
      telemetry.addData("OdoXL", robot.localizer.encoderXL);
      telemetry.addData("OdoXR", robot.localizer.encoderXR);
      telemetry.addData("X", JavaUtil.formatNumber(robot.localizer.xPos, 2));
      telemetry.addData("Y", JavaUtil.formatNumber(robot.localizer.yPos, 2));
   }
}
