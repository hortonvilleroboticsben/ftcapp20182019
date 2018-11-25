package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp", group="final")
public class TeleOp_Run extends LinearOpMode {
    Robot r = Robot.getInstance(this, new FinalRobotConfiguration());

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new FinalRobotConfiguration());
        r.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setRunMode("mtrLift", DcMotor.RunMode.RUN_USING_ENCODER);
        while(!opModeIsActive()){}
        while(opModeIsActive()){
//            ((DcMotor) r.motors.get("mtrLeftDrive")).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r.setDrivePower(Math.abs(gamepad1.left_stick_y) >= 0.05 ? gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) >= 0.05 ? gamepad1.right_stick_y : 0);
            r.setPower("mtrLift", gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
            for(Object s : r.motors.keySet()){
                telemetry.addData((String) s, r.getPower((String)s));
            }

            telemetry.update();
        }
    }
}
