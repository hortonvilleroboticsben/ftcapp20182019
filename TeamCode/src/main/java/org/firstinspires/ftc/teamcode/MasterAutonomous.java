package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous",group = "competition")
public class MasterAutonomous extends LinearOpMode {


    String alliance = "blue";
    int startPosition = 1, routeNumber = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot rbt = Robot.getInstance(this,new FinalRobotConfiguration());
        rbt.initialize(this, new FinalRobotConfiguration());

        telemetry.addData("Alliance?","a: blue, b: red");
        updateTelemetry(telemetry);
        while(!gamepad1.a && !gamepad1.b);
        alliance = gamepad1.a ? "blue" : "red";
        while(gamepad1.a || gamepad1.b);
        telemetry.addData("Start Position?","a: 1, b: 2");
        updateTelemetry(telemetry);
        while(!gamepad1.a && !gamepad1.b);
        startPosition = gamepad1.a ? 1 : 2;
        while(gamepad1.a || gamepad1.b);
        telemetry.addData("Route Number?","a: 1, b: 2");
        updateTelemetry(telemetry);
        while(!gamepad1.a && !gamepad1.b);
        routeNumber = gamepad1.a ? 1 : 2;
        telemetry.addData("Ready to go","");
        updateTelemetry(telemetry);

        waitForStart();

        (new Runnable() {
            @Override
            public void run() {
                rbt.getCameraCapture();
                rbt.analyzePhotoData();
            }
        }).run();

    }
}
