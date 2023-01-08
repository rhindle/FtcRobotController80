package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    Telemetry telemetry;
    Robot robot;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;

    /* Constructor */
    public Drivetrain(Robot robot){
        construct(robot);
    }

    void construct(Robot robot){
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }

    public void init() {
        motorLF = robot.motor0;
        motorRF = robot.motor1;
        motorLR = robot.motor2;
        motorRR = robot.motor3;
        initMotors();
    }

    public void setDrivePowers (double m0, double m1, double m2, double m3) {
        motorLF.setPower(m0);
        motorRF.setPower(m1);
        motorLR.setPower(m2);
        motorRR.setPower(m3);
    }

    public void stopDriveMotors() {
        motorLF.setPower(0);
        motorRF.setPower(0);
        motorLR.setPower(0);
        motorRR.setPower(0);
    }

    public void initMotors () {
        if (!robot.reverseDrive) {
            motorLF.setDirection(DcMotorEx.Direction.REVERSE);
            motorLR.setDirection(DcMotorEx.Direction.REVERSE);
            motorRF.setDirection(DcMotorEx.Direction.FORWARD);
            motorRR.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            motorLF.setDirection(DcMotorEx.Direction.FORWARD);
            motorLR.setDirection(DcMotorEx.Direction.FORWARD);
            motorRF.setDirection(DcMotorEx.Direction.REVERSE);
            motorRR.setDirection(DcMotorEx.Direction.REVERSE);
        }

        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
