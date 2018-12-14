package com.hortonvillerobotics;

public class FinalRobotConfiguration extends RobotConfiguration {

    private double wheelCircumference = 4.4*Math.PI;
    private double turnDiameter = 16.8;
    private double countsPerRotation = 560;

    public double getWheelCircumference(){return wheelCircumference;}
    public double getTurnDiameter(){return turnDiameter;}
    public double getCountsPerRotation(){return countsPerRotation;}

    private static String[][] motors = {
            {"mtrLeftDrive", "forward"},
            {"mtrRightDrive", "reverse"},
            {"mtrLift", "reverse"}
    };
    private static String[][] servos = {
            {"srvLock"}
    };
    private static String[][] sensors = {
            {"colorLeft", "0x3c"},
            {"colorRight", "0xcc"}
    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
