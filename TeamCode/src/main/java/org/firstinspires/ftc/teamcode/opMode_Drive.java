package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.FinalRobotConfiguration;

@Autonomous(name="testRight")
public class opMode_Drive extends LinearOpMode {
	final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
	
	@Override
	public void runOpMode() throws InterruptedException {
		rbt.initialize(this, new FinalRobotConfiguration());
		while(!opModeIsActive()){}

		rbt.drive(23.01, 0.2);
		rbt.pause(50);

		rbt.drive(-14.14, 0.2);
		rbt.pause(50);

		rbt.turn(83.0, 0.2);
		rbt.pause(50);

		rbt.drive(10.07, 0.2);
		rbt.pause(50);

		rbt.pause(500);
	}
}
