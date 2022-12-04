package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LKController {

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    //private FtcDashboard dashboard;
    private Telemetry telemetry;
    //private TelemetryPacket dashboardPacket;
    //private List<RobotPart> parts = new ArrayList<>();
    //public Canvas field;
    //public boolean autoBlue = true;

    public Object[] controlList = {};  //Object
    public ControlData[] controlData;

    ////////////////
    //constructors//
    ////////////////
    public LKController(LinearOpMode opMode){
        construct(opMode);

    }

    void construct(LinearOpMode opMode){
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        controlList = new Object[]{   // Object
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                gamepad1.dpad_left,
                gamepad1.dpad_right,
                gamepad1.a,
                gamepad1.b,
                gamepad1.x,
                gamepad1.y,
                gamepad1.start,
                gamepad1.back,
                gamepad1.left_bumper,
                gamepad1.right_bumper,
                gamepad1.left_stick_button,
                gamepad1.right_stick_button,
                gamepad2.dpad_up,
                gamepad2.dpad_down,
                gamepad2.dpad_left,
                gamepad2.dpad_right,
                gamepad2.a,
                gamepad2.b,
                gamepad2.x,
                gamepad2.y,
                gamepad2.start,
                gamepad2.back,
                gamepad2.left_bumper,
                gamepad2.right_bumper,
                gamepad2.left_stick_button,
                gamepad2.right_stick_button
        };

        //allocate for # objects
        controlData = new ControlData[controlList.length];

        for (int i = 0; i < controlList.length; i++) {
            //create object
            controlData[i] = new ControlData();
            //assign
            controlData[i].initData(controlList[i]);
        }
    }

    void updateAll()
    {
        for (ControlData i : controlData) {
            i.update();
        }
    }

    int getIndex(int controller, GPbuttons button){
        int index = 0;
        controller--;
        if (controller < 0 || controller > 1) controller = 0;
        index = controller * controlList.length / 2;
        index += button.ordinal();
        return index;
    }

    ControlData getAllData(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)];
    }

    boolean wasPressed(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)].pressed;
    }
    boolean wasReleased(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)].released;
    }
    boolean wasTapped(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)].tapped;
    }
    boolean isHeld(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)].held;
    }
    boolean isPressed(int controller, GPbuttons button) {
        return controlData[getIndex(controller, button)].lastStatus;
    }


    enum GPbuttons {  //must be in same order as controlList
        dpadUP,
        dpadDOWN,
        dpadLEFT,
        dpadRIGHT,
        A,
        B,
        X,
        Y,
        START,
        BACK,
        leftBUMPER,
        rightBUMPER,
        leftJoyStickBUTTON,
        rightJoyStickBUTTON;
    }

    class ControlData {
        Object source;  //object
        boolean lastStatus;
        long lastTime;
        boolean tapped;
        boolean doubleTapped;
        boolean held;
        boolean pressed;
        boolean released;

        public void setData(Object source, boolean last, long lastTime, boolean tapped, boolean doubleTapped, boolean held, boolean pressed, boolean released)
        {  //object
            this.source = source;
            this.lastStatus = last;
            this.lastTime = lastTime;
            this.tapped = tapped;
            this.doubleTapped = doubleTapped;
            this.held = held;
            this.pressed = pressed;
            this.released = released;
        }

        public void initData(Object source)  //object
        {
            this.source = source;
            lastStatus = false;
            lastTime = System.currentTimeMillis();
            tapped = false;
            doubleTapped = false;
            held = false;
            pressed = false;
            released = false;
        }

        public void update()
        {
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            boolean currentState = (boolean)source;
            if (lastStatus == false && currentState == true) {  // change from not pressed to pressed
                pressed = true;          // this will last for one loop!
                held = false;
                tapped = false;
                lastTime = currentTime;  // reset the time
            } else {
                pressed = false;
            }
            if (lastStatus == true && currentState == false) {  // change from pressed to not pressed
                released = true;         // this will last for one loop!
                held = false;
                if (deltaTime < 500) {
                    tapped = true;
                }
                lastTime = currentTime;  // reset the time
            } else {
                released = false;
            }
            if (lastStatus == true && currentState == true) {   // still held
                tapped = false;
                if (deltaTime >= 500) {
                    held = true;
                }
            }
            if (lastStatus == false && currentState == false) {  // still not held
                tapped = false;
                held = false;
            }
            //need something for doubleTap (put this off until proof-of-concept works)
            //after all the checks are done...
            lastStatus = currentState;
        }
    }

 /*   class GamepadButtonManager {
        boolean wasButtonPressed = false;
        long lastButtonRelease = System.currentTimeMillis();
        Gamepad gamepad;
        GPbuttons gamepadButton;
        double minSliderVal = 0.1;

        GamepadButtonManager(Gamepad gamepad, GPbuttons gamepadButton) {
            this.gamepad = gamepad;
            this.gamepadButton = gamepadButton;
        }

        GamepadButtonManager(GPbuttons gamepadButton) {
            this.gamepadButton = gamepadButton;
        }

        boolean getButtonHeld(Gamepad gamepad) {
            if (gamepad.start) {
                if (gamepadButton == GPbuttons.START) return true;
                return false;
            }
            if (gamepadButton == GPbuttons.A) return gamepad.a;
            if (gamepadButton == GPbuttons.B) return gamepad.b;
            if (gamepadButton == GPbuttons.X) return gamepad.x;
            if (gamepadButton == GPbuttons.Y) return gamepad.y;

            if (gamepadButton == GPbuttons.dpadUP) return gamepad.dpad_up;
            if (gamepadButton == GPbuttons.dpadDOWN) return gamepad.dpad_down;
            if (gamepadButton == GPbuttons.dpadLEFT) return gamepad.dpad_left;
            if (gamepadButton == GPbuttons.dpadRIGHT) return gamepad.dpad_right;

            if (gamepadButton == GPbuttons.leftJoyStickBUTTON) return gamepad.left_stick_button;
            if (gamepadButton == GPbuttons.rightJoyStickBUTTON) return gamepad.right_stick_button;
            if (gamepadButton == GPbuttons.leftBUMPER) return gamepad.left_bumper;
            if (gamepadButton == GPbuttons.rightBUMPER) return gamepad.right_bumper;


            if (gamepadButton == GPbuttons.BACK) return gamepad.back;

            if (getSliderValue(gamepad) > minSliderVal) return true;

            return false;
        }

        boolean getButtonHeld() {
            return getButtonHeld(gamepad);
        }

        boolean getButtonHeld(Gamepad gamepad, int time) {
            if (getButtonHeld(gamepad)) {
                return System.currentTimeMillis() - lastButtonRelease > time;
            } else lastButtonRelease = System.currentTimeMillis();
            return false;
        }

        boolean getButtonHeld(int time) {
            return getButtonHeld(gamepad, time);
        }

        boolean getButtonPressed(Gamepad gamepad) {
            if (getButtonHeld(gamepad)) {
                if (!wasButtonPressed) {
                    wasButtonPressed = true;
                    return true;
                }
            } else wasButtonPressed = false;
            return false;
        }

        boolean getButtonPressed() {
            return getButtonPressed(gamepad);
        }

        boolean getButtonReleased(Gamepad gamepad) {
            if (getButtonHeld(gamepad)) wasButtonPressed = true;
            else if (wasButtonPressed) {
                wasButtonPressed = false;
                return true;
            }
            return false;
        }

        boolean getButtonReleased() {
            return getButtonReleased(gamepad);
        }

        float getSliderValue(Gamepad gamepad) {
            if (gamepadButton == GPbuttons.leftJoyStickX) return gamepad.left_stick_x;
            if (gamepadButton == GPbuttons.leftJoyStickY) return gamepad.left_stick_y;
            if (gamepadButton == GPbuttons.rightJoyStickX) return gamepad.right_stick_x;
            if (gamepadButton == GPbuttons.rightJoyStickY) return gamepad.right_stick_y;

            if (gamepadButton == GPbuttons.leftTRIGGER) return gamepad.left_trigger;
            if (gamepadButton == GPbuttons.rightTRIGGER) return gamepad.right_trigger;
            if (gamepadButton == GPbuttons.combinedTRIGGERS)
                return gamepad.right_trigger - gamepad.left_trigger;

            return 0;
        }

        float getSliderValue() {
            return getSliderValue(gamepad);
        }

    }  */
}
