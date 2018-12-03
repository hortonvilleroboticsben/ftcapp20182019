package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.FinalRobotConfiguration;

@Autonomous(name="landing")
public class launchSequence extends LinearOpMode {
	final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
	
	@Override
	public void runOpMode() throws InterruptedException {
		rbt.initialize(this, new FinalRobotConfiguration());
		while(!opModeIsActive()){}

		rbt.owTurn(13.0, 0.2);
		rbt.pause(50);

		rbt.owTurn(17.0, -0.2);
		rbt.pause(50);

		rbt.owTurn(-93.5, 0.2);
		rbt.pause(50);

		rbt.turn(-178.0, 0.2);
		rbt.pause(50);

		rbt.pause(500);
	}
}
