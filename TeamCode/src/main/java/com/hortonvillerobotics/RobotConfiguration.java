package com.hortonvillerobotics;

public class RobotConfiguration {

    public static double wheelCircumference = 9.25;
    public static double turnDiameter = 14.5;
    public static double countsPerRotation = 1120;

    public static String[][] motors = {
            {"mtrLeftDrive","reverse"},
            {"mtrRightDrive","forward"},
            {"thisMotorShouldntWork","forward"}
    };
    public static String[][] servos = {

    };
    public static String[][] sensors = {

    };

}
