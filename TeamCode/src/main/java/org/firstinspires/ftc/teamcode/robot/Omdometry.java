package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Om.AngleMath;
import org.firstinspires.ftc.teamcode.robot.Om.Vector2;
import org.firstinspires.ftc.teamcode.robot.Om.Vector3;
import org.firstinspires.ftc.teamcode.robot.Om.VectorMath;
import org.firstinspires.ftc.teamcode.robot.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Tools.Position;

public class Omdometry {
   int lastLeftYPos, lastRightYPos, lastXPos;
   double cumulativeDistance = 0;

   double odoAngle = 0;
   double cumulativeOdoAngle = 0; //TESTING
   double lastImuAngle =  0;

   Telemetry telemetry;
   Robot robot;
   private static final double getSettings_ticksPerInch = 82300 / 48.0;
   private static final double getSettings_ticksPerRotation = 169619;
   DcMotorEx XWheel, rightYWheel, leftYWheel;

   //Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90);
   public Vector3 startPosition = new Vector3(0,0,-90);
   double offset = -startPosition.Z;
   double imuAngle = startPosition.Z;
   public Vector3 currentPosition = startPosition;
   Vector3 interimPosition;  //LK new
   double avgAngle =  0;     //LK new
   double lastOdoAngle =  0; //LK new

   public Omdometry(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
      this.robot = robot;
      this.telemetry = robot.telemetry;
   }

   public void init() {
      XWheel = robot.motor0B;
      rightYWheel = robot.motor1B;
      leftYWheel = robot.motor2B;

      XWheel.setDirection(DcMotorEx.Direction.FORWARD);
      leftYWheel.setDirection(DcMotorEx.Direction.REVERSE);
      rightYWheel.setDirection(DcMotorEx.Direction.REVERSE);
      onStart();
//      XWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      leftYWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      rightYWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//      lastXPos = XWheel.getCurrentPosition();
//      lastLeftYPos = leftYWheel.getCurrentPosition();
//      lastRightYPos = rightYWheel.getCurrentPosition();
//      imuHeading0 = robot.returnImuHeading(true);
//      odoHeading0 = getOdoHeading();
//      globalHeading0 = imuHeading0;
//
//      // odo start position is 0,0,0; imu should also read 0.  odoRawPose is already 0,0,0
//      updateOdoRobotPose();
//      setOdoFinalPose();
//      setOdoFieldOffset();
   }
   double getImuAngle() {
      double angle = robot.returnImuHeading();
      angle -= offset;
      imuAngle = AngleMath.scaleAngle(angle);
      return imuAngle;
   }
   /* Calculate average of two headings */
   public double getAvgHeading (double firstHeading, double secondHeading) {  //LK new
      double robotHeading;
      /* Find the difference between them; based on sampling rate, assume large values wrapped */
      robotHeading = AngleMath.scaleAngle(secondHeading - firstHeading);
      robotHeading /= 2;
      robotHeading += firstHeading;
      return AngleMath.scaleAngle(robotHeading);
   }

//   public Odometry(PositionTracker parent, OdometrySettings settings, OdometryHardware hardware) {
//      super(parent, "odometry");
//      setConfig(settings, hardware);
//   }
//
//   public Odometry(PositionTracker parent) {
//      super(parent, "odometry");
//      //TODO make default
//      setConfig(OdometrySettings.makeForOdoBot(), OdometryHardware.makeForOdoBot(parent.parent.opMode.hardwareMap));
//   }

   private double getAngleFromDiff(int leftYDiff, int rightYDiff){
//      return (leftYDiff - rightYDiff) * 360 / getSettings_ticksPerRotation;
      return (leftYDiff - rightYDiff) * -360 / getSettings_ticksPerRotation;    // LK this went the wrong direction!  !!!!!!!!!!!!!!!
   }

//   @Override
   public void onRun() {

      telemetry.addData("y odo dist", (leftYWheel.getCurrentPosition() + rightYWheel.getCurrentPosition()) / 2.0);
      telemetry.addData("cumulativeDistance", cumulativeDistance);

      int currLeftY = leftYWheel.getCurrentPosition();
      int currRightY = rightYWheel.getCurrentPosition();
      int currX = XWheel.getCurrentPosition();

      telemetry.addData("left y", currLeftY);
      telemetry.addData("right y", currRightY);
      telemetry.addData("middle x", currX);

      int leftYDiff = currLeftY - lastLeftYPos;
      int rightYDiff = currRightY - lastRightYPos;
      int XDiff = currX - lastXPos;

      telemetry.addData("left y diff", leftYDiff);
      telemetry.addData("right y diff", rightYDiff);
      telemetry.addData("x", XDiff);

//      Vector3 pos = parent.getCurrentPosition();
      Vector3 pos =currentPosition;

      odoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + odoAngle);
      cumulativeOdoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + cumulativeOdoAngle);

//      double imuAng = parent.getImuAngle();
      double imuAng = getImuAngle();
      boolean imuAccurate = Math.abs(imuAng - lastImuAngle) < 0.5;
      if(imuAccurate){
         odoAngle = imuAng;
      }

      telemetry.addData("odo angle0", odoAngle);
      //odoAngle = imuAng;  //LK suspect the odoAngle is a problem so override?
      //avgAngle = getAvgHeading(lastOdoAngle, odoAngle);  //LK
      lastOdoAngle = odoAngle;  //:L
      lastImuAngle = imuAng;


      telemetry.addData("imu accurate", imuAccurate);
      telemetry.addData("odo angle", odoAngle);
      telemetry.addData("cumulative angle", cumulativeOdoAngle);

      double angle = odoAngle;

      double XMove = ((XDiff) / getSettings_ticksPerInch);
      double YMove = ((leftYDiff + rightYDiff) / (2 * getSettings_ticksPerInch));

      telemetry.addData("x move", XMove);
      telemetry.addData("y move", YMove);

      cumulativeDistance += YMove;

//      parent.addPositionTicket(Odometry.class, new PositionTicket(VectorMath.translateAsVector2(pos.withZ(angle), XMove, YMove), new Vector2(XMove, YMove)));
      currentPosition = VectorMath.translateAsVector2(pos.withZ(angle), XMove, YMove);
//      interimPosition = VectorMath.translateAsVector2(pos.withZ(avgAngle), XMove, YMove);  // LK new
//      currentPosition = interimPosition.withZ(angle);   // LK new

      lastLeftYPos = currLeftY;
      lastRightYPos = currRightY;
      lastXPos = currX;
   }

//   @Override
//   public void onBeanLoad() {
//
//   }

//   @Override
   public void onInit() {
   }

//   @Override
   public void onStart() {
      XWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftYWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightYWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // run without encoder needed so it doesn't break the robot lift/sweep
      XWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftYWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightYWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      lastXPos = XWheel.getCurrentPosition();
      lastLeftYPos = leftYWheel.getCurrentPosition();
      lastRightYPos = rightYWheel.getCurrentPosition();

//      odoAngle = parent.getCurrentPosition().Z;
//      lastImuAngle = parent.getImuAngle();
      odoAngle = currentPosition.Z;
      lastImuAngle = getImuAngle();
   }

//   @Override
//   public void onStop() {
//   }
//
//   public void lower(){
//      getHardware().leftYServo.setPosition(getSettings().leftYServoDown);
//      getHardware().rightYServo.setPosition(getSettings().rightYServoDown);
//      getHardware().XWheelServo.setPosition(getSettings().XServoDown);
//   }
//
//   public void raise(){
//      getHardware().leftYServo.setPosition(getSettings().leftYServoUp);
//      getHardware().rightYServo.setPosition(getSettings().rightYServoUp);
//      getHardware().XWheelServo.setPosition(getSettings().XServoUp);
//   }
}