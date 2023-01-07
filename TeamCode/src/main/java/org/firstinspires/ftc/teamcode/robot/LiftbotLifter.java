package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftbotLifter {

    Telemetry telemetry;
    Robot robot;

    private DcMotorEx motorLiftLeft, motorLiftRight;
    private Servo servoLeft, servoRight, servoGrab;

    //turn servos
    double turnServoMinPosition       = 0.064;
    double turnServoMaxPosition       = 0.93;
    //double turnServoStartPosition     = 0.75; //no!
    double rightTurnServoOffset       = 0; //TODO return to final
    //grabber servo
    double grabberServoOpenPos        = 0.9;
    double grabberServoWideOpenPos    = 0.834;
    double grabberServoClosePos       = 1.0;
    //motor
    double minRegisterVal             = 0.05;
    int maxDownLiftSpeed              = 150;
    int maxUpLiftSpeed                = 150;
    int minLiftPosition               = 0;
    int maxLiftPosition               = 3200;
    int tolerance                     = 20;

    int[] depositHeight               = {350, 1150, 2060};
    int[] stackHeight                 = {0, 160, 270, 390, 500};
    int grabClearance                 = 400;
    int depositDrop                   = 200;
    double turnServoGrabPos           = 0.926;
    double turnServoDepositPos        = 0.286;
    double turnServoStartPos          = turnServoGrabPos;

    /* Constructor */
    public LiftbotLifter(Robot robot){
        construct(robot);
    }

    void construct(Robot robot){
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }

    public void init() {
        motorLiftLeft = robot.motor0B;
        motorLiftRight = robot.motor1B;
        servoLeft = robot.servo0B;
        servoRight = robot.servo2B;
        servoGrab = robot.servo4B;
        initMotors();
        initServos();

        setGrabServo(grabberServoClosePos);
        setVFBservos(turnServoStartPos);

    }

    public void loop() {
        
    }

    public void setDrivePowers (double m0) {
        motorLiftLeft.setPower(m0);
        motorLiftRight.setPower(m0);
    }

    public void stopMotors() {
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);
    }

    public void initMotors () {
        stopMotors();

        motorLiftLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorLiftRight.setDirection(DcMotorEx.Direction.REVERSE);

        motorLiftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLiftLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLiftRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLiftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLiftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void initServos () {
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoRight.setDirection(Servo.Direction.REVERSE);
        servoGrab.setDirection(Servo.Direction.FORWARD);
    }

    public void setVFBservos (double goTo) {
        servoLeft.setPosition(goTo);
        servoRight.setPosition(goTo+rightTurnServoOffset);
    }

    public void setGrabServo (double goTo) {
        servoGrab.setPosition(goTo);
    }

    public void setLiftHeight (int goTo) {
        stopMotors();
        motorLiftLeft.setTargetPosition(goTo);
        motorLiftRight.setTargetPosition(goTo);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePowers(1);
    }

    /* private void controlMast() {
        int slowPoint = 2000;

        mastPositionCurrent = motorMast.getCurrentPosition();
        //upper limits
        if (controlMastPower > 0 && digitalMastHigh.getState()) controlMastPower = 0;
        if (controlMastPower > 0 && mastPositionCurrent > mastPositionMax) controlMastPower = 0;
        //lower limits
        if (controlMastPower < 0 && digitalMastLow.getState()) controlMastPower = 0;
        if (controlMastPower < 0 && mastPositionCurrent < mastPositionMin) controlMastPower = 0;
        //slow down
        if (controlMastPower > 0 && (mastPositionMax - mastPositionCurrent) < slowPoint) {
            controlMastPower = controlMastPower * (mastPositionMax - mastPositionCurrent) / slowPoint;
            controlMastPower = Math.max(0.05, controlMastPower);
        }
        if (controlMastPower < 0 && (mastPositionCurrent - mastPositionMin) < slowPoint) {
            controlMastPower = controlMastPower * (mastPositionCurrent - mastPositionMin) / slowPoint;
            controlMastPower = Math.min(-0.05, controlMastPower);
        }
        //hold if power is 0
        if (controlMastPower == 0) {
            if (!flagMastHolding) {
                motorMast.setPower(0);
                motorMast.setTargetPosition(mastPositionCurrent);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(0.05);
                flagMastHolding = true;
                mastPositionHold = mastPositionCurrent;
            }
            telemetry.addData("Mast Holding At", mastPositionHold);
        }
        //if power isn't 0 make mast go that speed
        if (controlMastPower != 0) {
            if (flagMastHolding){
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flagMastHolding = false;
            }
            motorMast.setPower(controlMastPower);
            telemetry.addData("Mast Power", controlMastPower);
        }
    } */
}
