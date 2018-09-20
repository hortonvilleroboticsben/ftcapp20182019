package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.hortonvillerobotics.FileUtils;

@TeleOp(name = "Recorder")
public class DriveRecorder extends LinearOpMode {

    public static final double WHEEL_DIAMETER = 3.75;
    public static final double GEAR_RATIO = 1;
    public static final double TURN_DIAMETER = 18;
    public static final double ENC_COUNTS_PER_ROTATION = 1120;
    public static final double STD_POWER = 0.2;
    public int lock = -1;

    public static final String FILENAME = "/opMode_Drive.java";

    public double backDrive_Drive(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION*(WHEEL_DIAMETER*Math.PI)*100)/100.;
    }

    public double backDrive_Turn(double encCounts){
        return Math.round(encCounts/GEAR_RATIO/ENC_COUNTS_PER_ROTATION*WHEEL_DIAMETER*Math.PI/TURN_DIAMETER/Math.PI*360*Math.signum(getEncoderCount(mtrLeftDrive))*100)/100;
    }

    DcMotor mtrLeftDrive;
    DcMotor mtrRightDrive;

    void resetDriveEncoders(){
        mtrLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setDrivePower(double a, double b){
        mtrLeftDrive.setPower(a);
        mtrRightDrive.setPower(b);
    }

    double getEncoderCount(DcMotor m){
        return m.getCurrentPosition();
    }


    public void runOpMode() throws InterruptedException {

        mtrLeftDrive = hardwareMap.dcMotor.get("mtrLeftDrive");
        mtrRightDrive = hardwareMap.dcMotor.get("mtrRightDrive");
        mtrRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        resetDriveEncoders();

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            lock = lock == -1 ? Math.abs(gamepad1.left_stick_y) >= 0.05 ? 1 : Math.abs(gamepad1.right_stick_x) >= 0.05 ? 0 : -1 : lock;

            switch(lock) {
                case 1:
                    setDrivePower(gamepad1.left_stick_y, gamepad1.left_stick_y);
                    break;
                case 0:
                    setDrivePower(gamepad1.right_stick_x, -gamepad1.right_stick_x);
                    break;
            }

            if(lock != -1 && gamepad1.a){
                switch(lock){
                    case 0:
                        FileUtils.appendToFile(FILENAME, "      robot.Turn("+backDrive_Turn(-(Math.abs(getEncoderCount(mtrLeftDrive))+Math.abs(getEncoderCount(mtrRightDrive)))/2)+", "+STD_POWER+");\n");
                        break;
                    case 1:
                        FileUtils.appendToFile(FILENAME, "      robot.Drive("+backDrive_Drive(Math.signum(getEncoderCount(mtrLeftDrive))*(Math.abs(getEncoderCount(mtrLeftDrive))+Math.abs(getEncoderCount(mtrRightDrive)))/2)+", "+STD_POWER+");\n");
                        break;
                }
                resetDriveEncoders();
                lock = -1;
            }

            telemetry.addData("LEnc", getEncoderCount(mtrLeftDrive));
            telemetry.addData("REnc", getEncoderCount(mtrRightDrive));
            telemetry.addData("Lock", lock);
            telemetry.update();

        }

        if(lock != -1){
            switch(lock){
                case 0:
                    FileUtils.appendToFile(FILENAME, "      robot.Turn("+backDrive_Turn(-(Math.abs(getEncoderCount(mtrLeftDrive))+Math.abs(getEncoderCount(mtrRightDrive)))/2)+", "+STD_POWER+");\n");
                    break;
                case 1:
                    FileUtils.appendToFile(FILENAME, "      robot.Drive("+backDrive_Drive((-getEncoderCount(mtrLeftDrive)+getEncoderCount(mtrRightDrive))/2)+", "+STD_POWER+");\n");
                    break;
            }
            resetDriveEncoders();
            lock = -1;
        }

        FileUtils.appendToFile(FILENAME, "    }\n" +
                "}\n");
    }
}
