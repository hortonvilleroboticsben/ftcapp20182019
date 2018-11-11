package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.RobotConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.hortonvillerobotics.FileUtils;

@TeleOp(name = "Recorder")
public class DriveRecorder extends LinearOpMode {

    public static final double WHEEL_DIAMETER = 3.75;
    public static final double GEAR_RATIO = 1;
    public static final double TURN_DIAMETER = 18;
    public static final double ENC_COUNTS_PER_ROTATION = 1120;
    public static final double STD_POWER = 0.2;
    public int lockA = -1;
    public int lockB = -1;
    Robot r;

    public static final String FILENAME = "/opMode_Drive.java";

    public double backDrive_Drive(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION*(WHEEL_DIAMETER*Math.PI)*100)/100.;
    }

    public double backDrive_Turn(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION*WHEEL_DIAMETER*Math.PI/TURN_DIAMETER/Math.PI*360*Math.signum(r.getEncoderCounts("mtrLeftDrive"))*100)/100;
    }


    public void runOpMode() throws InterruptedException {

        r = Robot.getInstance(this,new RobotConfiguration());
        r.initialize(this,new RobotConfiguration());

        FileUtils.writeToFile(FILENAME, "package org.firstinspires.ftc.teamcode;\n" +
                "\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                "\n" +
                "import com.hortonvillerobotics.Robot;\n" +
                "\n" +
                "public class opMode_Drive extends LinearOpMode {\n" +
                "public final Robot robot = Robot.getInstance(this);\n"+
                "    \n" +
                "    @Override\n" +
                "    public void runOpMode() throws InterruptedException {\n" +
                "       robot.initialize();\n"+
                "       while(!opModeIsActive()){}\n"+
                "\n");

        r.resetDriveEncoders();

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            lockA = lockA == -1 && lockB == -1 ? Math.abs(gamepad1.left_stick_y) >= 0.05 ? 1 : Math.abs(gamepad1.right_stick_x) >= 0.05 ? 0 : -1 : lockA;
            lockB = lockA == -1 && lockB == -1 ? Math.abs(gamepad2.left_stick_y) >= 0.05 ? 1 : Math.abs(gamepad2.right_stick_y) >= 0.05 ? 0 : -1 : lockB;

            if(lockA != -1)
                switch(lockA) {
                    case 1:
                        r.setDrivePower(gamepad1.left_stick_y, gamepad1.left_stick_y);
                        break;
                    case 0:
                        r.setDrivePower(gamepad1.right_stick_x, -gamepad1.right_stick_x);
                        break;
                }

            if(lockB != -1)
                switch (lockB){
                    case 1:
                        r.setDrivePower(gamepad2.left_stick_y, 0);
                        break;
                    case 0:
                        r.setDrivePower(0, gamepad2.right_stick_y);
                        break;
                }

            if(lockA != -1 && gamepad1.a && !gamepad1.start){
                switch(lockA){
                    case 0:
                        FileUtils.appendToFile(FILENAME, "      robot.Turn("+backDrive_Turn(-(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                        break;
                    case 1:
                        FileUtils.appendToFile(FILENAME, "      robot.Drive("+backDrive_Drive(Math.signum(r.getEncoderCounts("mtrLeftDrive"))*(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                        break;
                }
                r.resetDriveEncoders();
                lockA = -1;
            }

            if(lockB != -1 && gamepad2.a && !gamepad2.start&& !gamepad1.start){
                switch(lockB){
                    case 0:
                        FileUtils.appendToFile(FILENAME, "      robot.owTurn("+backDrive_Turn(-(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                        break;
                    case 1:
                        FileUtils.appendToFile(FILENAME, "      robot.owTurn("+backDrive_Drive(Math.signum(r.getEncoderCounts("mtrLeftDrive"))*(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                        break;
                }
                r.resetDriveEncoders();
                lockB = -1;
            }

            telemetry.addData("LEnc", r.getEncoderCounts("mtrLeftDrive"));
            telemetry.addData("REnc", r.getEncoderCounts("mtrRightDrive"));
            telemetry.addData("Lock", lockA);
            telemetry.update();

        }

        if(lockA != -1){
            switch(lockA){
                case 0:
                    FileUtils.appendToFile(FILENAME, "      robot.Turn("+backDrive_Turn(-(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                    break;
                case 1:
                    FileUtils.appendToFile(FILENAME, "      robot.Drive("+backDrive_Drive(Math.signum(r.getEncoderCounts("mtrLeftDrive"))*(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n");
                    break;
            }
            r.resetDriveEncoders();
            lockA = -1;
        }

        FileUtils.appendToFile(FILENAME, "    }\n" +
                "}\n");
    }
}
