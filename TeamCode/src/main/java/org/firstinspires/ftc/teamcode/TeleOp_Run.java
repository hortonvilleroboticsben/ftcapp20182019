package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.Timer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TeleOp Competition", group="final")
public class TeleOp_Run extends LinearOpMode {
    Robot r = Robot.getInstance(this, new FinalRobotConfiguration());

    boolean isLocked = true;
    public final double LOCKCLOSED = 0.112;
    public final double LOCKOPEN = 0.536;
    boolean lockOS = false;
    double srvPos = 0;
//    double posLocked = 0, posOpen = 1;
    Telemetry.Item encVol;

    String srvColL = "srvColL", srvColR = "srvColR", srvLock = "srvLock";

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new FinalRobotConfiguration());
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setRunMode("mtrLift", DcMotor.RunMode.RUN_USING_ENCODER);
//        r.setRunMode("mtrCrane", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        r.setRunMode("mtrLin", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!opModeIsActive()){}

        encVol = telemetry.addData("LiftVelocity", r.calculateVelocity(() -> r.getEncoderCounts("mtrLift"), 50));
        encVol.setRetained(true);

        r.runParallel("", ()->{
            while(opModeIsActive()) {
                encVol.setValue(r.calculateVelocity(() -> r.getEncoderCounts("mtrLift"), 50));
            }
        });

        while(opModeIsActive()){

            //CONTROLLER 1
            //CONTROLLER 1

            r.setDrivePower(Math.abs(gamepad1.left_stick_y) >= 0.05 ? (gamepad1.right_bumper ? 0.3 : 1) * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) >= 0.05 ? (gamepad1.right_bumper ? 0.3 : 1) * gamepad1.right_stick_y : 0);
            r.setPower("mtrLift", gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);

            if(gamepad1.x && !lockOS){
                lockOS = true;
                isLocked=!isLocked;
            }else if(!gamepad1.x) lockOS = false;

            if(isLocked) {
                r.setServoPosition("srvLock",LOCKCLOSED);
            } else {
                r.setServoPosition("srvLock",LOCKOPEN);
            }

            //CONTROLLER 2
            //CONTROLLER 2

//            r.setPower("mtrCrane",gamepad2.left_stick_y);
//            r.setPower("mtrLin",gamepad2.right_stick_y);
//            if(gamepad2.right_trigger > .5) {
//                r.setServoPower(srvColL,1);
//                r.setServoPower(srvColR,-1);
//            } else if(gamepad2.right_bumper) {
//                r.setServoPower(srvColL,-1);
//                r.setServoPower(srvColR,1);
//            } else {
//                r.setServoPower(srvColL,0);
//                r.setServoPower(srvColR,0);
//            }

//            if (gamepad1.left_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
//                srvPos = srvPos < 1 ? srvPos + 0.008 : 1;
//                srvTimer.reset();
//            } else if (gamepad1.right_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
//                srvPos = srvPos > 0 ? srvPos - 0.008 : 0;
//                srvTimer.reset();
//            }


//            r.setServoPosition("srvLock", srvPos);


            //UPDATING TELEMETRY FOR THE USER
            telemetry.addData("isLocked", isLocked);
            telemetry.addData("srvLock Position", ((Servo) r.servos.get("srvLock")).getPosition());
            for(Object s : r.motors.keySet()){
                telemetry.addData((String) s, r.getPower((String)s));
                telemetry.addData((String) s + " enc", r.getEncoderCounts((String) s));
            }

            telemetry.update();
        }
    }
}
