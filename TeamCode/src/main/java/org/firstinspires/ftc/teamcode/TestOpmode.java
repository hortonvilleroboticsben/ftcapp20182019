package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;
import android.util.Log;

import com.hortonvillerobotics.FileUtils;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.RobotConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;

import static android.provider.MediaStore.Files.FileColumns.MEDIA_TYPE_IMAGE;

@Autonomous(name = "CoreLibTest", group = "LinearOpMode")
public class TestOpmode extends LinearOpMode {

    public static final String TAG = "TestOpMode";

    @Override
    public void runOpMode() throws InterruptedException {

        try {

            final Robot robot = Robot.getInstance(this,new RobotConfiguration());
            robot.initialize(this,new RobotConfiguration());

            waitForStart();

            robot.owTurn(90,.4);
            robot.pause(2000);
            robot.owTurn(90,-.4);
            robot.pause(2000);
            robot.owTurn(-90,.4);
            robot.pause(2000);
            robot.owTurn(-90,-.4);

            robot.turn(90,.4);
            robot.turn(-90,.4);

            robot.drive(24, .4);
            robot.drive(-24,.4);

            robot.finish();
        } catch (Exception e) {
            e.printStackTrace();
            Log.e(TAG,e.toString());
            telemetry.addData("Error",e);
            telemetry.update();
        }

    }

}
