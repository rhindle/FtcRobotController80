package Ignore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LKControllerBad {

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private Telemetry telemetry;

    public Object[] controlList = {};  //Object
    public ControlData[] controlData;

    ////////////////
    //constructors//
    ////////////////
    public LKControllerBad(LinearOpMode opMode){
        construct(opMode);
    }

    void construct(LinearOpMode opMode){
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.telemetry = opMode.telemetry;

        //allocate for # objects based on GPbuttons enum
        controlData = new ControlData[GPbuttons.values().length * 2];
        //create objects and assign a index numbers
        for (int i = 0; i < GPbuttons.values().length * 2; i++) {
            controlData[i] = new ControlData();
            controlData[i].initData(i);
        }

    }

    void updateAll()
    {
        for (ControlData i : controlData) {
            i.update();
        }
    }

    int getIndex(int controller, GPbuttons button){
        //This converts an index of 0-27 based on the controller 1-2 and button 0-13
        if (controller < 1 || controller > 2) controller = 0; else controller--;
        return controller * GPbuttons.values().length + button.ordinal();
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

    enum GPbuttons {  //must match what is in getReading's switch block
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
        int index;
        GPbuttons name;
        boolean lastStatus;
        long lastTime;
        boolean tapped;
        boolean doubleTapped;
        boolean held;
        boolean pressed;
        boolean released;

        public void initData(int index)  //object
        {
            this.index = index;
            name = GPbuttons.values()[index % GPbuttons.values().length];
            lastStatus = false;
            lastTime = System.currentTimeMillis();
            tapped = false;
            doubleTapped = false;
            held = false;
            pressed = false;
            released = false;
        }

        boolean getReading(int index)
        {
            Gamepad gpad;
            if (index >= GPbuttons.values().length) {
                index -= GPbuttons.values().length;
                gpad = gamepad2;
            } else {
                gpad = gamepad1;
            }
            switch (GPbuttons.values()[index]) {
                //must match the elements in the GPbuttons enum
                case dpadUP:              return gpad.dpad_up;
                case dpadDOWN:            return gpad.dpad_down;
                case dpadLEFT:            return gpad.dpad_left;
                case dpadRIGHT:           return gpad.dpad_right;
                case A:                   return gpad.a;
                case B:                   return gpad.b;
                case X:                   return gpad.x;
                case Y:                   return gpad.y;
                case START:               return gpad.start;
                case BACK:                return gpad.back;
                case leftBUMPER:          return gpad.left_bumper;
                case rightBUMPER:         return gpad.right_bumper;
                case leftJoyStickBUTTON:  return gpad.left_stick_button;
                case rightJoyStickBUTTON: return gpad.right_stick_button;
                default:                  return false;//something bad happened
            }
        }

        public void update()
        {
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            boolean currentState = getReading(index);
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
}
