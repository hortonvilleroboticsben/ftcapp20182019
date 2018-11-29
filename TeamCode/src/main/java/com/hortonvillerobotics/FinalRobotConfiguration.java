package com.hortonvillerobotics;

public class FinalRobotConfiguration extends RobotConfiguration {

    private double wheelCircumference = 3*Math.PI;
    private double turnDiameter = 17.9375;
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

    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

}
