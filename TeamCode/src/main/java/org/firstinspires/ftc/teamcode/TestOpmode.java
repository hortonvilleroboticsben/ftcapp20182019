package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.RobotConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "CoreLibTest", group = "LinearOpMode")
public class TestOpmode extends LinearOpMode {

    public static final String TAG = "TestOpMode";

    @Override
    public void runOpMode() throws InterruptedException {

        try {

            final Robot robot = Robot.getInstance(this);
            robot.initialize(this);

            waitForStart();

            for(int i = 0; i < 4; i ++) {
                robot.owTurn(90,0.5);
                sleep(1000);
            }

            for(int i = 0; i < 4; i ++) {
                robot.owTurn(90,-0.5);
                sleep(1000);
            }

            for(int i = 0; i < 4; i ++) {
                robot.owTurn(-90,0.5);
                sleep(1000);
            }

            for(int i = 0; i < 3; i ++) {
                robot.owTurn(-90,-0.5);
                sleep(1000);
            }
            robot.owTurn(-90,-0.5);

            robot.finish();
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG,e.toString());
            telemetry.addData("Error",e);
        }

    }

}
