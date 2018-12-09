package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "visiontest3",group = "test")
public class VisionTest3 extends LinearOpMode {

    Timer t = new Timer();

    @Override
    public void runOpMode() throws InterruptedException {

        Robot rbt = Robot.getInstance(this,new FinalRobotConfiguration());
        rbt.initialize(this,new FinalRobotConfiguration());

        waitForStart();

        t.reset();

        rbt.getCameraCapture();

        telemetry.addData("Capture time", t.getTimeElapsed());
        telemetry.update();

        t.reset();

        rbt.analyzePhotoData();

        telemetry.addData("Final block position", rbt.blockLocation[0]);
        updateTelemetry(telemetry);
        sleep(30000);
    }
}
