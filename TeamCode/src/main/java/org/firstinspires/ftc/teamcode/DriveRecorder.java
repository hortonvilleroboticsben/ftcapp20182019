package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import lib_6981.FileUtils;

public class DriveRecorder extends LinearOpMode {

    public static final double WHEEL_DIAMETER = 3.75;
    public static final double GEAR_RATIO = 1;
    public static final double TURN_DIAMETER = 18;
    public static final double ENC_COUNTS_PER_ROTATION = 1120;

    public static final String FILENAME = "opMode_Drive.java";

    public static double backDrive_Drive(double encCounts){
        return encCounts/ENC_COUNTS_PER_ROTATION*WHEEL_DIAMETER*Math.PI;
    }

    public static double backDrive_Turn(double encCounts){
        return encCounts/ENC_COUNTS_PER_ROTATION*WHEEL_DIAMETER*Math.PI/TURN_DIAMETER/Math.PI*360*Math.signum(getEncoderCounts(mtrLeftDrive));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FileUtils.writeToFile(FILENAME, "package org.firstinspires.ftc.teamcode;\n" +
                "\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                "\n" +
                "import lib_6981.FileUtils;\n" +
                "\n" +
                "public class opMode_Drive extends LinearOpMode {\n" +
                "public final Robot robot = Robot.getInstance(this);\n"+
                "    \n" +
                "    @Override\n" +
                "    public void runOpMode() throws InterruptedException {\n" +
                "       robot.initialize();\n"+
                "       while(!opModeIsActive){}\n");

        FileUtils.writeToFile(FILENAME, "    }\n" +
                "}\n");
    }
}
