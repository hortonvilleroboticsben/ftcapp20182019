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

            robot.owTurn(90,.2);
            robot.pause(2000);
            robot.owTurn(90,-.2);
            robot.pause(2000);
            robot.owTurn(-90,.2);
            robot.pause(2000);
            robot.owTurn(-90,-.2);



            robot.finish();
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG,e.toString());
            telemetry.addData("Error",e);
        }

    }

}
