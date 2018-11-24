package com.hortonvillerobotics;

public class FinalRobotConfiguration extends RobotConfiguration {

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
