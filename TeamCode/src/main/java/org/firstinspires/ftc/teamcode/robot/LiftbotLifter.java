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
    private int liftTargetPosition;
    int safeToOpenDelay = 1000;
    long safeToOpenTime = System.currentTimeMillis();

    double grabOpenRequested = -1;
    boolean deferredHold = false;
    double liftPositionRequested = -1;
    int stateMachineType = -1;
    int stateMachineStep = -1;
    long stateMachineTimeLimit = System.currentTimeMillis();
    long stateMachineTaskLimit = System.currentTimeMillis();
    long stateMachineDelay = System.currentTimeMillis();
    boolean liftHomed = false;
    double userLiftSpeed = 0;
    boolean isUnderManualControl = false;
    int stackedCone = 5;

    //turn servos
    final double turnServoMinPosition       = 0.064;
    final double turnServoMaxPosition       = 0.93;
    //double turnServoStartPosition     = 0.75; //no!
    double rightTurnServoOffset       = 0; //TODO return to final
    //grabber servo
    final double grabberServoOpenPos        = 0.9;
    final double grabberServoWideOpenPos    = 0.834;
    final double grabberServoClosePos       = 1.0;
    //motor
    double minRegisterVal             = 0.05;
    int maxDownLiftSpeed              = 150;
    int maxUpLiftSpeed                = 150;
    final int minLiftPosition               = 0;
    final int maxLiftPosition               = 3200;
    int tolerance                     = 20;

    final int[] depositHeight         = {350, 1150, 2060};
    final int[] stackHeight           = {0, 160, 270, 390, 500};
    final int grabClearance                 = 400;
    final int depositDrop                   = 200;
    final double turnServoGrabPos           = 0.926;
    final double turnServoDepositPos        = 0.286;
    final double turnServoUpPos             = 0.542;
    final double turnServoStartPos          = turnServoGrabPos;

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
        stateMachineMgr();
        delayedServoActions();
        spamTelemetry();
    }

    public void startStateMachine(int machine) {
        stopStateMachine();
        stateMachineStep = -1;
        stateMachineType = machine;
    }

    public void startStateMachine(LiftActions action) {
        startStateMachine(convertActionToMachine(action));
    }

    public int convertActionToMachine(LiftActions action)
    {
        switch (action) {
            case AUTOMATE_PREP_CAPTURE:     return 1;
            case AUTOMATE_CAPTURE:          return 2;
            case AUTOMATE_PREP_DEPOSIT_HI:  return 3;
            case AUTOMATE_PREP_DEPOSIT_MED: return 4;
            case AUTOMATE_PREP_DEPOSIT_LO:  return 5;
            case AUTOMATE_HOME:             return 0;
            default:                        return -1;
        }
    }

    public void stopStateMachine() {
        stateMachineMgr(true);
    }

    public void stateMachineMgr() {
        stateMachineMgr(false);
    }

    public void stateMachineMgr(boolean stop) {
        if (stateMachineType == -1) {     // This will mean no state machine running
            stateMachineStep = -1;        // this will indicate not started for all machines
            return;
        }
        if (stop) stateMachineStep = -2;  // this will be cancel for all machines
        switch (stateMachineType) {
            case 1:
                stateMachinePrepCapture();
                break;
            case 2:
                stateMachineCapture();
                break;
            case 3:  // high
                stateMachinePrepDeposit(2);
                break;
            case 4:  // medium
                stateMachinePrepDeposit(1);
                break;
            case 5:  // low
                stateMachinePrepDeposit(0);
                break;
            case 0:
                stateMachineHome();
                break;
            default:
                break;
        }
    }

    public void initStateMachine(long timeLimit, long task1Limit) {
        stateMachineTimeLimit = System.currentTimeMillis() + timeLimit;
        stateMachineTaskLimit = System.currentTimeMillis() + task1Limit;
        stateMachineStep = 1;
        stateMachineDelay = System.currentTimeMillis();
    }

    public boolean cancelStateMachine() {
        if (System.currentTimeMillis() > stateMachineTimeLimit) stateMachineStep = -2;  // cancel if machine time too long
        if (System.currentTimeMillis() > stateMachineTaskLimit) stateMachineStep = -2;  // cancel if task time too long

        if (stateMachineStep == -2) {   // cancel needed
            stopMotorsAndHold();
            grabOpenRequested = -1;
            stateMachineType = -1;
            stateMachineStep = -1;
            return true;
        }
        return false;
    }

    public void stateMachineHome() {
        // Lowers the lift to hit the limit switch, then raises it until it isn't hit
        if (stateMachineStep == -1) initStateMachine(5000,5000);
        if (cancelStateMachine()) return;

        if (stateMachineStep == 1) {                 // send it down
            if (!isLimitSwitchPressed()) {           // go down if not already pressing switch
                zeroLiftEncoders();
                setLiftHeight(-1000,0.25);
            }
            stateMachineStep++;
        }
        if (stateMachineStep == 2) {         // wait for limit switch to be pressed
            if (isLimitSwitchPressed()) {
                zeroLiftEncoders();
                setLiftHeight(200,0.1);   // head back up
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 3) {
            if (!isLimitSwitchPressed()) {
                zeroLiftEncoders();
                setLiftHeight(0,0.1);  //hold at zero
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 4) {         // all done
            liftHomed = true;
            stateMachineType = -1;
            stateMachineStep = -1;
        }
    }

    public void stateMachinePrepDeposit(int pole) {
        int height = depositHeight[pole];
        // Raise the lift, rotate the arm (safely)
        if (stateMachineStep == -1) initStateMachine(5000,5000);
        if (cancelStateMachine()) return;

        if (stateMachineStep == 1) {  // close the servo and raise the lift
            if (!isGrabServoClosed()) {             // this should already be closed, but in case it isn't...
                action(LiftActions.GRAB_CLOSE);
                stateMachineDelay = System.currentTimeMillis() + 500;
            }
            setLiftHeight(height);
            stateMachineStep++;
        }
        if (stateMachineStep == 2) {         // if the grabber is closed, safe to move the arm up (if it isn't already over)
            if (isStateMachineDelayFinished()) {
//                if (whereIsArm()!=turnServoDepositPos) {   // if arm not already over pole, go straight up
                if (!isServoAtPosition(whereIsArm(), turnServoDepositPos)) {
                    action(LiftActions.ARM_UP);
                }
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 3) {         // wait for lift to be in position and move arm level
            if (isLiftInTolerance()) {
//                if (whereIsArm()!=turnServoDepositPos) {
                if (!isServoAtPosition(whereIsArm(), turnServoDepositPos)) {
                    action(LiftActions.ARM_POLE);
                    stateMachineDelay = System.currentTimeMillis() + 500;
                }
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 4) {         // wait for arm to level
            if (isStateMachineDelayFinished()) stateMachineStep++;
        }

        if (stateMachineStep == 5) {         // all done
            stateMachineType = -1;
            stateMachineStep = -1;
        }
    }

    public void stateMachineCapture() {
        // Closes the grabber and raises the lift
        if (stateMachineStep == -1) initStateMachine(5000,5000);
        if (cancelStateMachine()) return;

        if (stateMachineStep == 1) {
            if (isServoAtPosition(servoGrab, grabberServoClosePos) || !isServoAtPosition(servoLeft, turnServoGrabPos)) {
                // if the grabber is already closed or the arm is not in the grab position, don't bother
                stateMachineStep = 4;  // skip to the end
            } else {
                action(LiftActions.GRAB_CLOSE);   // close the grabber and set a delay time for the lift
                stateMachineDelay = System.currentTimeMillis() + 500;
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 2) {         // wait for grab to be closed and raise lift
            if (isStateMachineDelayFinished()) {
                action(LiftActions.LIFT_RAISE_AFTER_CAPTURE);
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 3) {         // wait for lift to be in position (may be useful for other automation)
            if (isLiftInTolerance()) stateMachineStep++;
        }
        if (stateMachineStep == 4) {         // all done
            stateMachineType = -1;
            stateMachineStep = -1;
        }
    }

    public void stateMachinePrepCapture() {
        // Lowers the lift to 0, Rotates the arm into place, opens servo wide
        if (stateMachineStep == -1) initStateMachine(5000,5000);
        if (cancelStateMachine()) return;

        if (stateMachineStep == 1) {
            setLiftHeight(minLiftPosition);  // lift to floor
            action(LiftActions.ARM_FLOOR);   // arm to floor
            stateMachineStep++;
        }
        if (stateMachineStep == 2) {         // wait for arm to be in position to open grab
            if (isGrabberSafeToOpen() && isLiftInTolerance()) {
                action(LiftActions.GRAB_WIDEOPEN);
                stateMachineStep += 2;       // skip the next step
            } else if (isGrabberSafeToOpen()) {
                action(LiftActions.GRAB_OPEN);
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 3) {         // wait for lift to be in position to wide open grab
            if (isGrabberSafeToOpen() && isLiftInTolerance()) {
                action(LiftActions.GRAB_WIDEOPEN);
                stateMachineStep++;
            }
        }
        if (stateMachineStep == 4) {         // all done
            stateMachineType = -1;
            stateMachineStep = -1;
        }
    }

    public boolean isLimitSwitchPressed() {
        return !robot.sensors.getSwitch0B();
    }

    public boolean isLiftInTolerance(int pos) {
        return Math.abs(motorLiftLeft.getCurrentPosition() - pos) < tolerance;
    }

    public boolean isLiftInTolerance() {
        //return Math.abs(motorLiftLeft.getCurrentPosition() - liftTargetPosition) < tolerance;
        return isLiftInTolerance(liftTargetPosition);
    }

    public boolean isGrabberSafeToOpen() {
        return (System.currentTimeMillis() >= safeToOpenTime);
    }

    public boolean isGrabServoClosed () {
        //return (servoGrab.getPosition()==grabberServoClosePos);
        return isServoAtPosition(servoGrab, grabberServoClosePos);
    }

    public boolean isStateMachineDelayFinished() {
        return (System.currentTimeMillis() >= stateMachineDelay);
    }

    public double whereIsArm() {
        return (servoLeft.getPosition());
    }

    public boolean isServoAtPosition(Servo servo, double comparePosition) {
        return isServoAtPosition(servo.getPosition(), comparePosition);
    }

    public boolean isServoAtPosition(double servoPosition, double comparePosition) {
        return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
    }

    public void setDrivePowers (double m0) {
        motorLiftLeft.setPower(m0);
        motorLiftRight.setPower(m0);
    }

    public void stopMotors() {
        motorLiftLeft.setPower(0);
        motorLiftRight.setPower(0);
    }

    public void stopMotorsAndHold() {
        setLiftHeight(motorLiftLeft.getCurrentPosition(),0.5);
    }

    public void zeroLiftEncoders() {
        stopMotors();
        motorLiftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLiftRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void initMotors () {
        stopMotors();

        motorLiftLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorLiftRight.setDirection(DcMotorEx.Direction.REVERSE);

        motorLiftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftTargetPosition = 0;

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
//        if (servoLeft.getPosition()!=goTo) {
        if (!isServoAtPosition(servoLeft, goTo)) {
            action(LiftActions.GRAB_CLOSE);
            servoLeft.setPosition(goTo);
            servoRight.setPosition(goTo + rightTurnServoOffset);
            safeToOpenTime = System.currentTimeMillis() + safeToOpenDelay;
        }
    }

    public void delayedServoActions () {
        if (grabOpenRequested!=-1) setGrabServo(grabOpenRequested);  // simple handling of delayed servo opening
        if (deferredHold) {
            stopMotorsAndHold();
            deferredHold = false;
        };
    }

    public void toggleGrabServo () {
        if (isGrabServoClosed()) action(LiftActions.GRAB_OPEN);
        else action(LiftActions.GRAB_CLOSE);
    }

    public void setGrabServo (double goTo) {
        grabOpenRequested = -1;
//        if (servoGrab.getPosition()!=goTo) {
        if (!isServoAtPosition(servoGrab, goTo)) {
            if (goTo == grabberServoClosePos) {   // Closing should always be safe
                servoGrab.setPosition(goTo);
            } else if (isGrabberSafeToOpen()) {   // Opening is safe if enough time has passed
                servoGrab.setPosition(goTo);
            } else {
                grabOpenRequested = goTo;         // for delayed opening
            }
        }
    }

    public void cycleConeStack() {
        //first check if the grabber and arm are in suitable positions (not closed, in grab position)
        if (isServoAtPosition(servoGrab, grabberServoClosePos) || !isServoAtPosition(servoLeft, turnServoGrabPos)) return;
        stackedCone--;
        if (stackedCone < 0) stackedCone = 4;
        setLiftHeight(stackHeight[stackedCone],0.5);
    }

//    public void actionGrab() {setGrabServo(grabberServoClosePos);}
//    public void actionOpen() {setGrabServo(grabberServoOpenPos);}
//    public void actionWideOpen() {setGrabServo(grabberServoWideOpenPos);}

    public void action(LiftActions action) {
        switch (action) {
            case GRAB_OPEN:
                setGrabServo(grabberServoOpenPos);
                break;
            case GRAB_WIDEOPEN:
                setGrabServo(grabberServoWideOpenPos);
                break;
            case GRAB_CLOSE:
                setGrabServo(grabberServoClosePos);
                break;
            case GRAB_TOGGLE:
                toggleGrabServo();
                break;
            case ARM_FLOOR:
                setVFBservos(turnServoGrabPos);
                break;
            case ARM_POLE:
                setVFBservos(turnServoDepositPos);
                break;
            case ARM_START:
                setVFBservos(turnServoStartPos);
                break;
            case ARM_UP:
                setVFBservos(turnServoUpPos);
                break;
            case LIFT_LOWER_TO_DEPOSIT:
                setLiftHeight(liftTargetPosition-depositDrop, 0.5);
                break;
            case LIFT_RAISE_AFTER_CAPTURE:
                setLiftHeight(liftTargetPosition+grabClearance, 0.5);
                break;
            case LIFT_DOWNSTACK:
                cycleConeStack();
                break;
            case AUTOMATE_CAPTURE:
                startStateMachine(action);
                break;
            case AUTOMATE_PREP_CAPTURE:
                startStateMachine(action);
                break;
            case AUTOMATE_PREP_DEPOSIT_HI:
                startStateMachine(action);
                break;
            case AUTOMATE_PREP_DEPOSIT_MED:
                startStateMachine(action);
                break;
            case AUTOMATE_PREP_DEPOSIT_LO:
                startStateMachine(action);
                break;
            case AUTOMATE_HOME:
                startStateMachine(action);
                break;
            case CANCEL:
                stopStateMachine();
                stopMotors();
                grabOpenRequested = -1;
                break;
            default:
                break;
        }
    }

    public void setLiftHeight (int goTo) {
        setLiftHeight(goTo, 1);
    }

    public void setLiftHeight (int goTo, double pwr) {
        if (stateMachineType != 0) {    // limits ignored if homing
            if (goTo < minLiftPosition || goTo > maxLiftPosition) {  // something very wrong so bail
                stopMotors();
                return;
            }
        }
        liftTargetPosition = goTo;
        stopMotors();
        motorLiftLeft.setTargetPosition(liftTargetPosition);
        motorLiftRight.setTargetPosition(liftTargetPosition);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePowers(pwr);
    }

    public enum LiftActions {
        GRAB_OPEN,
        GRAB_WIDEOPEN,
        GRAB_CLOSE,
        GRAB_TOGGLE,
        ARM_FLOOR,
        ARM_POLE,
        ARM_START,
        ARM_UP,
        LIFT_LOWER_TO_DEPOSIT,
        LIFT_RAISE_AFTER_CAPTURE,
        LIFT_DOWNSTACK,
        AUTOMATE_CAPTURE,
        AUTOMATE_PREP_CAPTURE,
        AUTOMATE_PREP_DEPOSIT_HI,
        AUTOMATE_PREP_DEPOSIT_MED,
        AUTOMATE_PREP_DEPOSIT_LO,
        AUTOMATE_HOME,
        CANCEL
    }

    public void setUserDriveSettings(double driveSpeed) {
        if (driveSpeed == 0 && !isUnderManualControl) return;

        if (driveSpeed != 0) {
            int currentPos = motorLiftLeft.getCurrentPosition();
            //enforce upper limits
            if (driveSpeed > 0 && currentPos > maxLiftPosition) driveSpeed = 0;
            //enforce lower limits
            if (driveSpeed < 0 && currentPos < minLiftPosition) driveSpeed = 0;
            if (driveSpeed < 0 && isLimitSwitchPressed()) driveSpeed = 0;
        }

        if (driveSpeed == 0) {  // when it drops out of manual control, hold
            isUnderManualControl = false;
            //stopMotorsAndHold();
            stopMotors();
            deferredHold = true;
            return;
        }

        if (!isUnderManualControl) {
            isUnderManualControl = true;
            stopStateMachine();
            stopMotors();
            motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
        }

        setDrivePowers(driveSpeed);
    }

    public void spamTelemetry() {
        telemetry.addData("~~~~~~~~~~", "~~~~~~~~~~");
        telemetry.addData("manual control", isUnderManualControl);
        telemetry.addData("statemachine", stateMachineType);
        telemetry.addData("statestep", stateMachineStep);
        telemetry.addData("homed", liftHomed);
        telemetry.addData("liftposition", motorLiftLeft.getCurrentPosition());
        telemetry.addData("targetposition", liftTargetPosition);
        telemetry.addData("in tolerance", isLiftInTolerance());
        telemetry.addData("servoleft", servoLeft.getPosition());
        telemetry.addData("servoright", servoRight.getPosition());
        telemetry.addData("servograb", servoGrab.getPosition());
        telemetry.addData("cone stack", stackedCone);
        telemetry.addData("~~~~~~~~~~", "~~~~~~~~~~");
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
