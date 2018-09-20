package com.hortonvillerobotics;

public class RobotConfiguration {

    public static double wheelCircumference = 9;
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
