package org.firstinspires.ftc.teamcode.robot.Tools;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Functions {

    private static ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private static double timeLoop;

    // Hypotenuse function I used in Blocky?
    public static double mathHypotenuse(float arg0, float arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }

    public static double normalizeAngle(double A) {
        // normalize angle A to -179 to +180 range
        while (A > 180) A -= 360;
        while (A <= -180) A += 360;
        return A;
    }

    public static double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }

}