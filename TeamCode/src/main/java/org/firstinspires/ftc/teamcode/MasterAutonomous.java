package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.StateMachine;
import com.hortonvillerobotics.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous", group = "competition")
public class MasterAutonomous extends LinearOpMode {

    private final double LOCKCLOSED = 0.117;
    private final double LOCKOPEN = 0.55; //.536

    private final double SAFESPEED = .3;

    private Integer leftRed, leftBlue, rightRed, rightBlue;

    private boolean crater = false;
    //    String blockPos = "";
    private StateMachine s = new StateMachine();
    private long startPause = 0;
    private boolean pauseOS = false, n = false;

    @Override
    public void runOpMode() {

        Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
        rbt.initialize(this, new FinalRobotConfiguration());
        rbt.setServoPosition("srvLock", LOCKCLOSED);
        rbt.setRunMode("mtrLift", DcMotor.RunMode.RUN_USING_ENCODER);

//        FtcRobotControllerActivity.initCamera();


        while (!opModeIsActive() && !isStopRequested()) {
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

                startPause = startPause < 0 ? 0 : startPause > 30000 ? 30000 : startPause;

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
        sleep(100);
        rbt.setServoPosition("srvLock", LOCKOPEN);
        sleep(500);

        rbt.runParallel("ProcessLower",
                () -> {

                    rbt.setPower("mtrLift", 0.72);
                    sleep(2500);
                    while (rbt.calculateVelocity(() -> rbt.getEncoderCounts("mtrLift"), 50) > 50) ;
                    rbt.setServoPosition("srvLock", LOCKCLOSED);
                    rbt.runToTarget("mtrLift", -600, -.72);
//                    rbt.initRunDriveToTarget(2000,.5,2000,.5);
//                    while(!rbt.hasMotorEncoderReached("mtrLeftDrive", 1990));
//                    rbt.setDrivePower(0,0);

                    rbt.owTurn(13.0, SAFESPEED);
                    rbt.pause(50);

                    rbt.owTurn(14.0, -SAFESPEED);
                    rbt.pause(50);

                    rbt.owTurn(-15, SAFESPEED); //changed from -45
                    rbt.pause(50);

//                    rbt.drive(1,SAFESPEED);
//                    rbt.pause(50);

                    rbt.owTurn(15, SAFESPEED); //changed from 45
                    rbt.pause(50);

                    rbt.owTurn(-35, -SAFESPEED);
                    rbt.turn(-56.5, SAFESPEED);
                    rbt.pause(50);

                    //TODO THIS VALUE MAY NEED TO BE ALTERED FOR THE DISTANCE
                    rbt.drive(4, SAFESPEED);
                    rbt.pause(50);

                    rbt.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rbt.setDrivePower(0.1, 0.1);//0.07
                    Timer t1 = new Timer();
                    try {
                        while (opModeIsActive() && !t1.hasTimeElapsed(2000) && !(rbt.getPower("mtrLeftDrive") == 0 && rbt.getPower("mtrRightDrive") == 0)) {

                            leftBlue = rbt.getColorValue("colorLeft", "blue");
                            leftRed = rbt.getColorValue("colorLeft", "red");

                            rightBlue = rbt.getColorValue("colorRight", "blue");
                            rightRed = rbt.getColorValue("colorRight", "red");

                            telemetry.addData("leftRed", leftRed);
                            telemetry.addData("leftBlue", leftBlue);
                            telemetry.addData("rightRed", rightRed);
                            telemetry.addData("rightBlue", rightBlue);
                            telemetry.update();

                            if ((leftRed != null && leftRed >= 5) || (leftBlue != null && leftBlue >= 5))
                                rbt.setPower("mtrLeftDrive", 0.0);
                            if ((rightRed != null && rightRed >= 6) || (rightBlue != null && rightBlue >= 4))
                                rbt.setPower("mtrRightDrive", 0.0);
                        }
                    } catch (NullPointerException e) {
                        e.printStackTrace();
                        rbt.setDrivePower(0, 0);
                    }
//                    Log.d("MasterAutonomous","Left red: " + leftRed + " Left blue: " + leftBlue
//                    + " Right red: " + rightRed + " Right blue: " + rightBlue);
                    rbt.pause(50);
                    rbt.drive(5, 0.2);
                },
                () -> {
                    rbt.analyzePhotoData();
                    telemetry.addData("Decision", rbt.blockLocation[0]).setRetained(true);
//                    telemetry.addData("mtrLeft RunMode", ((DcMotor) rbt.motors.get("mtrLeftDrive")).getMode()).setRetained(true);
                    telemetry.update();
                }
        );
        rbt.waitForFlag("ProcessLower");

        if (!crater) {
            //TODO:Extend Mineral System and place the Team Marker if on non-crater side
//            rbt.runToTarget("mtrCollection", 1000,0.72);
//            rbt.setServoPower("srvColR", -1);
//            rbt.setServoPower("srvColL", 1);
//            sleep(250);
//            rbt.initRunToTarget("mtrCollection", 0, 0.5);
        }

        switch (rbt.blockLocation[0]) {
            case "right":

                rbt.turn(-35, SAFESPEED);
                rbt.pause(50);

                if(crater) {
                    rbt.drive(25.5, SAFESPEED);
                    rbt.drive(-10, SAFESPEED);

                    rbt.turn(117,SAFESPEED);
                    rbt.drive(55,SAFESPEED);
                    rbt.turn(40,SAFESPEED);

                    //place marker in this spot
                    //rbt.runToTarget("mtrCollection", 1000,0.72);
                    //rbt.setServoPower("srvColR", -1);
                    //rbt.setServoPower("srvColL", 1);
                    //sleep(250);
                    //rbt.initRunToTarget("mtrCollection", 0, 0.5);

                    //knock off cube with arm after dropping off marker???

                    sleep(2000);

                    rbt.turn(179,SAFESPEED);
                    rbt.drive(24,SAFESPEED);

                } else {
                    rbt.drive(23.5, SAFESPEED);
                    rbt.pause(50);
                    rbt.drive(-9, SAFESPEED);

                    rbt.turn(-50, -SAFESPEED);
                    rbt.drive(40, SAFESPEED);
                    rbt.turn(-35,SAFESPEED);
                    rbt.drive(6,SAFESPEED);

                }

                break;

            case "left":

                rbt.turn(38, SAFESPEED);
                rbt.pause(50);
                rbt.drive(25, SAFESPEED);
                rbt.drive(-12, SAFESPEED);

                if(crater) {

                    rbt.turn(45,SAFESPEED);
                    rbt.drive(36,SAFESPEED);
                    rbt.turn(45,SAFESPEED);
                    rbt.drive(3,SAFESPEED);

                    //place marker in this spot
                    //rbt.runToTarget("mtrCollection", 1000,0.72);
                    //rbt.setServoPower("srvColR", -1);
                    //rbt.setServoPower("srvColL", 1);
                    //sleep(250);
                    //rbt.initRunToTarget("mtrCollection", 0, 0.5);

                    sleep(2000);

                    rbt.turn(175,SAFESPEED);
                    rbt.drive(27.5,SAFESPEED);

                } else {

                    rbt.turn(-120, SAFESPEED);
                    rbt.drive(43, SAFESPEED);
                    rbt.turn(-45,SAFESPEED);
                    rbt.drive(20,SAFESPEED);

                }

                break;

            //case "center" and error guess
            default:

                rbt.pause(50);
                rbt.drive(20, SAFESPEED);
                rbt.drive(-8,SAFESPEED);

                if(crater) {

                    rbt.turn(85.5,SAFESPEED);
                    rbt.drive(45,SAFESPEED);
                    rbt.turn(45,SAFESPEED);
                    rbt.drive(5,SAFESPEED);

                    //place marker in this spot
                    //rbt.runToTarget("mtrCollection", 1000,0.72);
                    //rbt.setServoPower("srvColR", -1);
                    //rbt.setServoPower("srvColL", 1);
                    //sleep(250);
                    //rbt.initRunToTarget("mtrCollection", 0, 0.5);

                    sleep(2000);

                    rbt.turn(175,SAFESPEED);
                    rbt.drive(25,SAFESPEED);

                } else {

                    rbt.turn(-85, SAFESPEED);
                    rbt.drive(50, SAFESPEED);
                    rbt.turn(-40,SAFESPEED);
                    rbt.drive(8,SAFESPEED);

                }

                break;
        }


    }
}
