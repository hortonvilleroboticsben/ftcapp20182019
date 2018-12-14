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

    boolean crater = false;
    //    String blockPos = "";
    StateMachine s = new StateMachine();
    long startPause = 0;
    boolean pauseOS = false, n = false;

    @Override
    public void runOpMode() {

        Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
        rbt.initialize(this, new FinalRobotConfiguration());

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

        rbt.runParallel("ProcessLower",
                () -> {
                    rbt.runToTarget("mtrLift", 5800, .72, true);
                    while (!rbt.hasMotorEncoderReached("mtrLift", 5790)) ;
                    rbt.setPower("mtrLift", 0);

                    rbt.owTurn(13.0, 0.23);
                    rbt.pause(50);

                    rbt.owTurn(17.0, -0.23);
                    rbt.pause(50);

                    rbt.owTurn(-93.5, 0.23);
                    rbt.pause(50);

                    rbt.runParallel("colorBack",()->{
                        while(opModeIsActive() && rbt.getColorValue("colorLeft", "red") < 130 || rbt.getColorValue("colorLeft", "blue") < 130){
                            rbt.setPower("mtrLeftDrive", -0.23);
                        }
                        rbt.setPower("mtrLeftDrive", 0.0);
                    },()->{
                        while(opModeIsActive() && rbt.getColorValue("colorRight", "red") < 130 || rbt.getColorValue("colorRight", "blue") < 130){
                            rbt.setPower("mtrRightDrive", -0.23);
                        }
                        rbt.setPower("mtrRightDrive", 0.0);
                    });
                    rbt.waitForFlag("colorBack");
                },
                ()->rbt.analyzePhotoData()
        );
        rbt.waitForFlag("ProcessLower");

        switch (rbt.blockLocation[0]) {
            case "right":
                rbt.turn(-183, 0.23);
                rbt.pause(50);

                rbt.drive(22, 0.23);
                rbt.pause(50);

                rbt.drive(-19, 0.23);
                rbt.pause(50);

                rbt.turn(90, 0.23);
                rbt.pause(50);

                rbt.drive(14, 0.23);
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

                rbt.drive(18, 0.23);
                rbt.pause(50);
                break;
            case "left":
                rbt.turn(-118, 0.23);
                rbt.pause(50);

                rbt.drive(30, 0.23);
                rbt.pause(50);

                rbt.drive(-18, 0.23);
                rbt.pause(50);

                rbt.owTurn(-35, 0.23);
                rbt.pause(50);

                rbt.drive(8.75, 0.23);
                rbt.pause(50);
                break;
        }

        if (!crater) {
            rbt.drive(22.75, 0.23);
            rbt.pause(50);

            rbt.owTurn(136.5, -0.23);
            rbt.pause(50);

            rbt.drive(-18, 0.23);
            rbt.pause(50);

            rbt.pause(500);
        } else {
            rbt.drive(23, 0.23);
            rbt.pause(50);

            rbt.owTurn(45, -0.23);
            rbt.pause(50);

            rbt.owTurn(-90, 0.23);
            rbt.pause(50);
        }


    }
}
