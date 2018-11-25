package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.hortonvillerobotics.FileUtils;

@TeleOp(name = "Recorder")
public class DriveRecorder extends LinearOpMode {

    static final FinalRobotConfiguration f = new FinalRobotConfiguration();
    public static final double WHEEL_CIRCUMFERENCE = f.getWheelCircumference();
    public static final double GEAR_RATIO = 1;
    public static final double TURN_DIAMETER = f.getTurnDiameter();
    public static final double ENC_COUNTS_PER_ROTATION = f.getCountsPerRotation();
    public static final double STD_POWER = 0.4;
    public int lockA = -1;
    public int lockB = -1;
    Robot r = Robot.getInstance(this, f);;

    public static final String FILENAME = "/opMode_Drive.java";

    public double backDrive_Drive(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION*(WHEEL_CIRCUMFERENCE)*100)/100.;
    }

    public double backDrive_Turn(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION* WHEEL_CIRCUMFERENCE /TURN_DIAMETER/Math.PI*360*2*100)/100;
    }

    public Double backDrive_OWTurn(double encCounts){
        return Math.round((encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION* WHEEL_CIRCUMFERENCE /TURN_DIAMETER/Math.PI*360)*100)/100.;
    }


    public void runOpMode() throws InterruptedException {

        r.initialize(this, f);

        FileUtils.writeToFile(FILENAME, "package org.firstinspires.ftc.teamcode;\n" +
                "\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                "\n" +
                "import com.hortonvillerobotics.Robot;\n" +
                "import com.hortonvillerobotics.FinalRobotConfiguration;\n" +
                "\n" +
                "public class opMode_Drive extends LinearOpMode {\n" +
                "\tfinal Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());\n"+
                "\t\n" +
                "\t@Override\n" +
                "\tpublic void runOpMode() throws InterruptedException {\n" +
                "\t\trbt.initialize(this, new FinalRobotConfiguration());\n"+
                "\t\twhile(!opModeIsActive()){}\n"+
                "\n");

        r.resetDriveEncoders();

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            lockA = lockA == -1 && lockB == -1 ? Math.abs(gamepad1.left_stick_y) >= 0.05 ? 1 : Math.abs(gamepad1.right_stick_x) >= 0.05 ? 0 : -1 : lockA;
            lockB = lockA == -1 && lockB == -1 ? Math.abs(gamepad2.left_stick_y) >= 0.05 ? 1 : Math.abs(gamepad2.right_stick_y) >= 0.05 ? 0 : -1 : lockB;

            if(lockA != -1)
                switch(lockA) {
                    case 1:
                        r.setDrivePower(0.1 * gamepad1.left_stick_y, 0.1 * gamepad1.left_stick_y);
                        break;
                    case 0:
                        r.setDrivePower(0.1 * -gamepad1.right_stick_x, 0.1 * gamepad1.right_stick_x);
                        break;
                }

            if(lockB != -1)
                switch (lockB){
                    case 1:
                        r.setDrivePower( 0.1 * gamepad2.left_stick_y, 0);
                        break;
                    case 0:
                        r.setDrivePower(0, 0.1 * gamepad2.right_stick_y);
                        break;
                }

            if(lockA != -1 && gamepad1.right_bumper){
                switch(lockA){
                    case 0:
                        FileUtils.appendToFile(FILENAME, "\t\trbt.turn("+backDrive_Turn((Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n\t\trbt.pause(50);\n\n");
                        break;
                    case 1:
                        FileUtils.appendToFile(FILENAME, "\t\trbt.drive("+-backDrive_Drive(Math.signum(r.getEncoderCounts("mtrLeftDrive"))*(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n\t\trbt.pause(50);\n\n");
                        break;
                }
                r.resetDriveEncoders();
                lockA = -1;
            }

            if(lockB != -1 && gamepad2.right_bumper){
                switch(lockB){
                    case 1:
                        FileUtils.appendToFile(FILENAME, "\t\trbt.owTurn("+-backDrive_Turn(Math.abs(r.getEncoderCounts("mtrLeftDrive")))/4.+", "+STD_POWER*Math.signum(r.getEncoderCounts("mtrLeftDrive"))+");\n\t\trbt.pause(50);\n\n");
                        break;
                    case 0:
                        FileUtils.appendToFile(FILENAME, "\t\trbt.owTurn("+backDrive_Turn(Math.abs(r.getEncoderCounts("mtrRightDrive")))/4.+", "+STD_POWER*Math.signum(r.getEncoderCounts("mtrRightDrive"))+");\n\t\trbt.pause(50);\n\n");
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

        if(lockA != -1 && gamepad1.right_bumper){
            switch(lockA){
                case 0:
                    FileUtils.appendToFile(FILENAME, "\t\trbt.turn("+backDrive_Turn((Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n\n");
                    break;
                case 1:
                    FileUtils.appendToFile(FILENAME, "\t\trbt.drive("+backDrive_Drive(Math.signum(r.getEncoderCounts("mtrLeftDrive"))*(Math.abs(r.getEncoderCounts("mtrLeftDrive"))+Math.abs(r.getEncoderCounts("mtrRightDrive")))/2)+", "+STD_POWER+");\n\n");
                    break;
            }
            r.resetDriveEncoders();
            lockA = -1;
        }

        if(lockB != -1 && gamepad2.right_bumper){
            switch(lockB){
                case 1:
                    FileUtils.appendToFile(FILENAME, "\t\trbt.owTurn("+backDrive_Turn(Math.abs(r.getEncoderCounts("mtrLeftDrive")))/2.+", "+STD_POWER*-Math.signum(r.getEncoderCounts("mtrLeftDrive"))+");\n\n");
                    break;
                case 0:
                    FileUtils.appendToFile(FILENAME, "\t\trbt.owTurn("+-backDrive_Turn(Math.abs(r.getEncoderCounts("mtrRightDrive")))/2.+", "+STD_POWER*-Math.signum(r.getEncoderCounts("mtrRightDrive"))+");\n\n");
                    break;
            }
            r.resetDriveEncoders();
            lockB = -1;
        }

        FileUtils.appendToFile(FILENAME, "\t\trbt.pause(500);\n" +
                "\t}\n" +
                "}\n");
    }
}
