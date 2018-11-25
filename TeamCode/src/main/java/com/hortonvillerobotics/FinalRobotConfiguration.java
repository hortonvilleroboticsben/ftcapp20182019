package com.hortonvillerobotics;

public class FinalRobotConfiguration extends RobotConfiguration {

    public double wheelCircumference = 3*Math.PI;
    public double turnDiameter = 17.9375;
    public double countsPerRotation = 560;

    private static String[][] motors = {
            {"mtrLeftDrive", "forward"},
            {"mtrRightDrive", "reverse"},
            {"mtrLift", "reverse"}
    };
    private static String[][] servos = {

    };
    private static String[][] sensors = {

    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
