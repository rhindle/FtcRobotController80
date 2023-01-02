package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

   HardwareMap hardwareMap;
   Telemetry telemetry;
   Robot robot;

   public DistanceSensor sensor2MLeft = null;
   public DistanceSensor sensor2MMiddle = null;
   public DistanceSensor sensor2MRight = null;
   public DigitalChannel ledRED = null;
   public DigitalChannel ledGREEN = null;

   public double distL, distM, distR;
   private int distCounter = 0;
   private boolean readDistSensors = false;

   /* Constructor */
   public Sensors(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
      this.robot = robot;
      this.telemetry = robot.telemetry;
      this.hardwareMap = robot.hardwareMap;
   }

   public void init(){
      sensor2MLeft = hardwareMap.get(DistanceSensor.class, "2MdistL");
      sensor2MMiddle = hardwareMap.get(DistanceSensor.class, "2MdistM");
      sensor2MRight = hardwareMap.get(DistanceSensor.class, "2MdistR");
      ledRED = hardwareMap.get(DigitalChannel.class, "digital6B");
      ledGREEN = hardwareMap.get(DigitalChannel.class, "digital7B");
      ledRED.setMode(DigitalChannel.Mode.OUTPUT);
      ledGREEN.setMode(DigitalChannel.Mode.OUTPUT);
      ledRED.setState(true);
      ledGREEN.setState(true);
   }

   public void loop(){
      updateDistanceSensors();
   }

   private void updateDistanceSensors() {
      if (readDistSensors) {
         switch (distCounter) {
            case 0:
               distL = sensor2MLeft.getDistance(DistanceUnit.INCH);
               break;
            case 1:
               distM = sensor2MMiddle.getDistance(DistanceUnit.INCH);
               break;
            case 2:
               distR = sensor2MRight.getDistance(DistanceUnit.INCH);
               break;
            case 3:    // does it work better if we alternate?
               distM = sensor2MMiddle.getDistance(DistanceUnit.INCH);
               break;
         }
         distCounter++;
         if (distCounter > 3) distCounter = 0;
      } else {
         distL = -1;
         distM = -1;
         distR = -1;
      }
   }

   public void readDistSensors (boolean boo) {
      readDistSensors = boo;
      ledGREEN.setState(!readDistSensors);
   }
   public void readDistSensors () {
      readDistSensors = !readDistSensors;
      ledGREEN.setState(!readDistSensors);
   }

   public void setLedRED(boolean boo) {
      ledRED.setState(!boo);
   }

   public void setLedGREEN(boolean boo) {
      ledGREEN.setState(!boo);
   }
}
