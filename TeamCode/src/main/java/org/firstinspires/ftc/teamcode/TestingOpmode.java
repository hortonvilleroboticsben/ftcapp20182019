package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "positionverification",group = "test")
public class TestingOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());

        rbt.initialize(this,new FinalRobotConfiguration());

        waitForStart();

        rbt.drive(24,0.4);

        rbt.turn(360,0.2);

        rbt.pause(500);

    }
}
