package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.DemoRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="GenericsTesting")
public class TestGenericsTeleOp extends LinearOpMode {

    DcMotor m;
    Robot r = Robot.getInstance(this, new DemoRobotConfiguration());

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new DemoRobotConfiguration());
//        m = hardwareMap.dcMotor.get("mtrLeftDrive");
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Motors", r.motors.keySet().toString());
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("mtrPower", r.getPower("mtrLeftDrive"));
            telemetry.update();
//            r.setPower("mtrLeftDrive", gamepad1.left_stick_y);
            r.setDrivePower(gamepad1.left_stick_y, gamepad1.right_stick_y);
//            m.setPower(gamepad1.left_stick_y);
//            r.setPower("mtrArm", gamepad1.dpad_up ? 0.4 : gamepad1.dpad_down ? -0.4 : 0);
        }
    }
}
