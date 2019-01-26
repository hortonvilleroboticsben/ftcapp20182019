package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="FIX")
public class TeleOpFix extends OpMode {

    Robot r = null;

    @Override
    public void init() {
        r = Robot.getInstance(this, new FinalRobotConfiguration());
    }

    double drivePowerScale;
    boolean lockOS = false;
    boolean isLocked = true;
    public final double LOCKCLOSED = 0.112;
    public final double LOCKOPEN = 0.536;

    @Override
    public void loop() {
        drivePowerScale = gamepad1.left_trigger >= .5 ? 0.25 : gamepad1.left_bumper ? 1 : 0.6;

        r.setDrivePower(Math.abs(gamepad1.left_stick_y) > 0.05 && !gamepad1.left_stick_button ? drivePowerScale * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) > 0.05 && !gamepad1.right_stick_button ? drivePowerScale * gamepad1.right_stick_y : 0);
//            r.setPower("mtrLift", gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);

            /*
            Lock Controls

            X (One Shot)    -       Toggle Lock/Unlock

            If Lock is left Unlocked, automatic Lock control is disabled.
            Re-Locking enables automatic Lock Control.
             */

        if(gamepad1.x && !lockOS){
            lockOS = true;
            isLocked=!isLocked;
        }else if(!gamepad1.x) lockOS = false;

        if(isLocked) {
            r.setServoPosition("srvLock",LOCKCLOSED);
        } else {
            r.setServoPosition("srvLock",LOCKOPEN);
        }

            /*
            Lift Controls

            DPad Up         -       Lift Move Up
            DPad Down       -       Lift Move Down

            Automatic Lock Controls

            Start of Lift Motion Unlocks the Lift
            End of Lift Motion Locks the Lift
            Disabled by manually Unlocking the Lift
            Enabled by manually Locking the Lift
            */

        if(isLocked && (gamepad1.dpad_up || gamepad1.dpad_down))r.setServoPosition("srvLock", LOCKOPEN);

        if(isLocked && !gamepad1.dpad_up && !gamepad1.dpad_down)r.setServoPosition("srvLock", LOCKCLOSED);

        r.setPower("mtrLift", gamepad1.dpad_up ? .72 : gamepad1.dpad_down ? -.72 : 0);


        telemetry.addData("LPower", r.getPower("mtrLeftDrive"));
        telemetry.addData("RPower", r.getPower("mtrRightDrive"));
        telemetry.addData("Lift Power", r.getPower("mtrLift"));
        telemetry.addData("isLocked", isLocked);
    }
}
