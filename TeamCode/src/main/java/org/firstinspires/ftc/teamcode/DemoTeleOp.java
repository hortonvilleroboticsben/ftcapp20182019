package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.DemoRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.StateMachine;
import com.hortonvillerobotics.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DemoTeleOp",group = "Demo")
public class DemoTeleOp extends LinearOpMode {

    Robot r = Robot.getInstance(this, new DemoRobotConfiguration());
    StateMachine s = new StateMachine();
    boolean launching = false, launchOS = true;
    double rightPos = 1, leftPos = 0;
    Timer srvTimer = new Timer();


    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new DemoRobotConfiguration());

        r.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            r.setDrivePower((gamepad1.right_bumper ? 0.1 : 1)*-gamepad1.left_stick_y, -(gamepad1.right_bumper ? 0.1 : 1)*gamepad1.right_stick_y);

            r.setServoPosition("srvRight", rightPos);
            r.setServoPosition("srvLeft", leftPos);

            if (gamepad1.left_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
                rightPos = rightPos < 1 ? rightPos + 0.008 : 1;
                leftPos = leftPos > 0 ? leftPos - 0.008 : 0;
                srvTimer.reset();
            } else if (gamepad1.right_trigger >= 0.5 && srvTimer.getTimeElapsed() >= 20) {
                rightPos = rightPos > 0 ? rightPos - 0.008 : 0;
                leftPos = leftPos < 1 ? leftPos + 0.008 : 1;
                srvTimer.reset();
            }

            if (gamepad1.a && !gamepad1.start && launchOS) {
                launching = true;
                launchOS = false;
                s.reset();
            }

            if(!launching && !gamepad1.a && !gamepad1.start) launchOS = true;

            if (launching) {
                s.runStates(() -> {
                    r.resetEncoder("mtrArm");
                    s.incrementState();
                }, () -> {
                    r.setPower("mtrArm", -1);
                    if (r.hasMotorEncoderReached("mtrArm", -80)) {
                        r.setPower("mtrArm", 0);
                        r.resetEncoder("mtrArm");
                        s.incrementState();
                    }
                }, () -> {
                    r.initRunToTarget("mtrArm", 97, -.12);
                    if (r.hasMotorEncoderReached("mtrArm", 97)) {
                        r.setPower("mtrArm", 0);
                        r.setRunMode("mtrArm", DcMotor.RunMode.RUN_USING_ENCODER);
                        launching = false;
                        s.incrementState();
                    }
                });
            }

            telemetry.addData("encoder", r.getEncoderCounts("mtrArm"));
            telemetry.update();


        }

        r.setServoPosition("srvRight", 1);
        r.setServoPosition("srvLeft", 0);

    }
}
