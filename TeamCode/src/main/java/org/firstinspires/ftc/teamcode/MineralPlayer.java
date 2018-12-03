package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.FinalRobotConfiguration;

@Autonomous(name="Minerals")
public class MineralPlayer extends LinearOpMode {
	final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
	boolean left,right,center;

	@Override
	public void runOpMode() throws InterruptedException {
		rbt.initialize(this, new FinalRobotConfiguration());
		while(!opModeIsActive()){}

		while(opModeIsActive()) {

			if(gamepad1.x || left) {
				left = true;
				rbt.turn(-77.0, 0.2);
				rbt.pause(50);

				rbt.drive(26.0, 0.2);
				rbt.pause(50);

				rbt.turn(-89.0, 0.2);
				rbt.pause(50);

				rbt.drive(15.85, 0.2);
				rbt.pause(50);

				rbt.drive(-16.67, 0.2);
				rbt.pause(50);

				rbt.turn(-102.0, 0.2);
				rbt.pause(50);

				rbt.drive(-14.17, 0.2);
				rbt.pause(50);

				rbt.pause(500);
				left=false;
			}
			if(gamepad1.y || center){
				center = true;
				rbt.owTurn(36.5, -0.2);
				rbt.pause(50);

				rbt.drive(20.45, 0.2);
				rbt.pause(50);

				rbt.owTurn(46.0, 0.2);
				rbt.pause(50);

				rbt.owTurn(-27.0, 0.2);
				rbt.pause(50);

				rbt.drive(-6.97, 0.2);
				rbt.pause(50);

				rbt.turn(-79.0, 0.2);
				rbt.pause(50);

				rbt.pause(500);
				center = false;
			}
			if(gamepad1.b || right){
				right = true;
				rbt.drive(22.23, 0.2);
				rbt.pause(50);

				rbt.drive(-16.69, 0.2);
				rbt.pause(50);

				rbt.turn(-97.0, 0.2);
				rbt.pause(50);

				rbt.drive(14.06, 0.2);
				rbt.pause(50);

				rbt.pause(500);
				right = false;
			}
		}

	}
}
