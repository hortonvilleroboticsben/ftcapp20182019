package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.Timer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp", group="competition")
public class MasterTeleOp extends LinearOpMode {

    Robot r = Robot.getInstance(this, new FinalRobotConfiguration());

    ModernRoboticsI2cColorSensor c;

    double srvPos = 0;
    Timer srvTimer = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new FinalRobotConfiguration());
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setRunMode("mtrLift", DcMotor.RunMode.RUN_USING_ENCODER);
        while(!opModeIsActive()){}
        while(opModeIsActive()){

            //CONTROLLER 1 CONTROLS
            //CONTROLLER 1 CONTROLS

            r.setDrivePower(Math.abs(gamepad1.left_stick_y) >= 0.05 ? (gamepad1.right_bumper ? 0.3 : 0.72) * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) >= 0.05 ? (gamepad1.right_bumper ? 0.3 : 0.72) * gamepad1.right_stick_y : 0);
            r.setPower("mtrLift", gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
            for(Object s : r.motors.keySet()){
                telemetry.addData((String) s, r.getPower((String)s));
                telemetry.addData((String) s + " enc", r.getEncoderCounts((String) s));
            }

            if (gamepad1.left_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
                srvPos = srvPos < 1 ? srvPos + 0.008 : 1;
                srvTimer.reset();
            } else if (gamepad1.right_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
                srvPos = srvPos > 0 ? srvPos - 0.008 : 0;
                srvTimer.reset();
            }


            r.setServoPosition("srvLock", srvPos);


            //CONTROLLER 2 CONTROLS
            //CONTROLLER 2 CONTROLS

            r.setPower("mtrCrane",gamepad2.left_stick_y);




            //GIVING DRIVER FEEDBACK

            telemetry.addData("leftRed", r.getColorValue("colorLeft", "red"));
            telemetry.addData("leftBlue", r.getColorValue("colorLeft", "blue"));
            telemetry.addData("rightRed", r.getColorValue("colorRight", "red"));
            telemetry.addData("rightBlue", r.getColorValue("colorRight", "blue"));
            telemetry.addData("srvPos", srvPos);
            telemetry.update();
        }
    }
}
