package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.FinalRobotConfiguration;

public class opMode_Drive extends LinearOpMode {
	final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
	
	@Override
	public void runOpMode() throws InterruptedException {
		rbt.initialize(this, new FinalRobotConfiguration());
		while(!opModeIsActive()){}

		rbt.drive(-28.36, 0.2);
		rbt.pause(50);

		rbt.owTurn(134.0, -0.2);
		rbt.pause(50);

		rbt.drive(-24.96, 0.2);
		rbt.pause(50);

		rbt.pause(500);
	}
}
