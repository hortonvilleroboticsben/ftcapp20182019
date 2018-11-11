package com.hortonvillerobotics;

public class DemoRobotConfiguration extends RobotConfiguration {
    public String[][] motors = {
            {"mtrLeftDrive","forward"},
            {"mtrRightDrive","reverse"},
            {"mtrArm", "forward"}
    };
    public String[][] servos = {
            {"srvLeft"},
            {"srvRight"}
    };
    public String[][] sensors = {

    };
}
