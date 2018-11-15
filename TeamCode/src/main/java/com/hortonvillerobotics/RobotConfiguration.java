package com.hortonvillerobotics;

public class RobotConfiguration {

    public double wheelCircumference = 9.25;
    public double turnDiameter = 14.5;
    public double countsPerRotation = 1120;

    private static String[][] motors = {
            {"mtrLeftDrive", "forward"},
            {"mtrRightDrive", "reverse"}
    };
    private static String[][] servos = {

    };
    private static String[][] sensors = {

    };

    public String[][] getMotors(){return motors;}
    public String[][] getServos(){return servos;}
    public String[][] getSensors(){return sensors;}

    protected static String[][] join2DStringArrays(String[][]... arrays) {
        int count = 0;
        for (String[][] s : arrays) {
            count += s.length;
        }
        String[][] retVal = new String[count][];
        for (int j = 0; j < arrays.length; j++) {
            for (int i = 0; i < retVal.length; i++) {
                retVal[i+j] = arrays[j][i];
            }
        }
        return retVal;
    }
}

