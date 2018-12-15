package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;
import android.os.Looper;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.ActivityHolder;
import org.firstinspires.ftc.robotcontroller.internal.CameraPreview;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.cameraView;
import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.cp;

@Autonomous(name = "Autonomous", group = "competition")
public class MasterAutonomous extends LinearOpMode {

    public final double LOCKCLOSED = 0.112;
    public final double LOCKOPEN = 0.536;

    boolean crater = false;
    //    String blockPos = "";
    StateMachine s = new StateMachine();
    long startPause = 0;
    boolean pauseOS = false, n = false;

    @Override
    public void runOpMode() {

        Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
        rbt.initialize(this, new FinalRobotConfiguration());
        rbt.setServoPosition("srvLock", LOCKCLOSED);

//        FtcRobotControllerActivity.initCamera();


        while (!opModeIsActive()) {
            s.runStates(() -> {
                telemetry.addData("Crater Side", "A for Yes, B for No");
                telemetry.update();
                if ((gamepad1.a ^ gamepad1.b) && !gamepad1.start && !gamepad2.start) {
                    crater = gamepad1.a;
                    n = true;
                }
                if (n && !gamepad1.a && !gamepad1.b) {
                    n = false;
                    s.incrementState();
                }

            }/*, ()->{
                telemetry.addData("DEBUG:Mineral Position", "X for Left, Y for Center, B for Right");
                telemetry.update();
                if((gamepad1.x ^ gamepad1.y ^ gamepad1.b) && !gamepad1.start && !gamepad2.start){
                    blockPos = gamepad1.x ? "left" : gamepad1.y ? "center" : "right";
                    n = true;
                }
                if(n && !gamepad1.x && !gamepad1.y && !gamepad1.b) {
                    n = false;
                    s.incrementState();
                }
            }*/, () -> {
                telemetry.addData("Starting Pause", "DPad to Change, A to Confirm");
                telemetry.addData("Current Pause", startPause);
                telemetry.update();

                if (!pauseOS) {
                    if (gamepad1.dpad_up) {
                        startPause += 1000;
                        pauseOS = true;
                    } else if (gamepad1.dpad_down) {
                        startPause -= 1000;
                        pauseOS = true;
                    }
                } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) pauseOS = false;

                if (gamepad1.a && !gamepad1.start && !gamepad2.start) n = true;
                if (!gamepad1.a && n) s.incrementState();

            }, () -> {
                telemetry.addData("Ready To Go!", "");
                telemetry.update();
            });
        }

        rbt.runParallel("ScanPause",
                () -> rbt.pause(startPause),
                () -> rbt.getCameraCapture()
        );

        rbt.waitForFlag("ScanPause");

        rbt.setPower("mtrLift", -1);
        rbt.pause(750);

        rbt.setServoPosition("srvLock",LOCKOPEN);

        rbt.runParallel("ProcessLower",
                () -> {
                    rbt.runToTarget("mtrLift", 5975, .72, true);
                    while (!rbt.hasMotorEncoderReached("mtrLift", 5965));
                    rbt.setServoPosition("srvLock", LOCKCLOSED);
                    rbt.setPower("mtrLift", 0);

                    rbt.owTurn(13.0, 0.23);
                    rbt.pause(50);

                    rbt.owTurn(17.0, -0.23);
                    rbt.pause(50);

                    rbt.owTurn(-93.5, 0.23);
                    rbt.pause(50);

                    rbt.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rbt.setDrivePower(-0.08, -0.08);
                    while(opModeIsActive() && rbt.getPower("mtrLeftDrive") != 0 && rbt.getPower("mtrRightDrive") != 0){
                        if(rbt.getColorValue("colorLeft", "red") >= 5 || rbt.getColorValue("colorLeft", "blue") >= 5)
                                rbt.setPower("mtrLeftDrive", 0.0);
                        if(rbt.getColorValue("colorRight", "red") >= 5 || rbt.getColorValue("colorRight", "blue") >= 5)
                            rbt.setPower("mtrRightDrive", 0.0);
                    }
                    
                    rbt.drive(-5, 0.23);
                },
                ()->{
                    rbt.analyzePhotoData();
                    telemetry.addData("Decision",rbt.blockLocation[0]);
                    telemetry.update();
                }
        );
        rbt.waitForFlag("ProcessLower");

        switch (rbt.blockLocation[0]) {
            case "right":
                rbt.turn(-187, 0.23);
                rbt.pause(50);

                rbt.drive(17, 0.23);
                rbt.pause(50);

                rbt.drive(-15, 0.23);
                rbt.pause(50);

                rbt.turn(94, 0.23);
                rbt.pause(50);

                rbt.drive(7, 0.23);
                rbt.pause(50);
                break;
            case "center":
                rbt.turn(-148, 0.23);
                rbt.pause(50);

                rbt.drive(19, 0.23);
                rbt.pause(50);

                rbt.drive(-10, 0.23);
                rbt.pause(50);

                rbt.owTurn(-63.5, 0.23);
                rbt.pause(50);

                rbt.drive(12, 0.23);
                rbt.pause(50);
                break;
            case "left":
                rbt.turn(-116.5, 0.23);
                rbt.pause(50);

                rbt.drive(30, 0.23);
                rbt.pause(50);

                rbt.drive(-18, 0.23);
                rbt.pause(50);

                rbt.owTurn(-35, 0.23);
                rbt.pause(50);

                rbt.drive(3.75, 0.23);
                rbt.pause(50);
                break;
        }

        if (!crater) {
            rbt.drive(27.75, 0.23);
            rbt.pause(50);

            rbt.owTurn(136.5, -0.23);
            rbt.pause(50);

            rbt.drive(-18, 0.23);
            rbt.pause(50);

            rbt.pause(500);
        } else {
            rbt.drive(36, 0.23);
            rbt.pause(50);

            rbt.owTurn(45, -0.23);
            rbt.pause(50);

            rbt.owTurn(-90, 0.23);
            rbt.pause(50);
        }


    }
}
