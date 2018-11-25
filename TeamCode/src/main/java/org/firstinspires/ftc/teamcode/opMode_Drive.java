package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.FinalRobotConfiguration;

@Autonomous(name="testDrive")
public class opMode_Drive extends LinearOpMode {
	final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
	
	@Override
	public void runOpMode() throws InterruptedException {
		rbt.initialize(this, new FinalRobotConfiguration());
		while(!opModeIsActive()){}

		rbt.drive(17.92, 0.2);

		rbt.turn(115.0, 0.2);

		rbt.owTurn(56.25, -0.2);

		rbt.owTurn(-58.0, -0.2);

	}
}
