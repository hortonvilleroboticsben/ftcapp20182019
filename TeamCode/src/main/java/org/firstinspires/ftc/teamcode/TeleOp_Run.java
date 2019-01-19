package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.Timer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TeleOp Competition", group="final")
public class TeleOp_Run extends LinearOpMode {
    Robot r = Robot.getInstance(this, new FinalRobotConfiguration());

    boolean isLocked = true;
    public final double LOCKCLOSED = 0.112;
    public final double LOCKOPEN = 0.536;
    boolean lockOS = false;
    double drivePowerScale = 1;
    boolean conveyor = false;
    boolean conveyorOS = false;
    double conveyorSpeed = 0.5;
    Timer conveyorSpeedTimer = new Timer();
    Telemetry.Item encVol;

    String srvColL = "srvColL", srvColR = "srvColR", srvLock = "srvLock";

    @Override
    public void runOpMode() throws InterruptedException {
        r.initialize(this, new FinalRobotConfiguration());
        r.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setRunMode("mtrLift", DcMotor.RunMode.RUN_USING_ENCODER);
//        r.setRunMode("mtrCrane", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        r.setRunMode("mtrLin", DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!opModeIsActive()){}

        encVol = telemetry.addData("LiftVelocity", r.calculateVelocity(() -> r.getEncoderCounts("mtrLift"), 50));
        encVol.setRetained(true);

        r.runParallel("", ()->{
            while(opModeIsActive()) {
                encVol.setValue(r.calculateVelocity(() -> r.getEncoderCounts("mtrLift"), 50));
            }
        });

        while(opModeIsActive()){

            //CONTROLLER 1
            //CONTROLLER 1

            /*
            Tank Drive Controls
            Configured for Left-Handed Drivers


            Left Stick      -       Left Power
            Right Stick     -       Right Power
            L. Stick Button -       Cut Left Power
            R. Stick Button -       Cut Right Power
            Left Trigger    -       25% Drive Power
            Left Bumper     -       50% Drive Power
             */

            drivePowerScale = gamepad1.left_trigger >= 0.5 ? 0.25 : gamepad1.left_bumper ? 0.5 : 1;

            r.setDrivePower(Math.abs(gamepad1.left_stick_y) < 0.05 && !gamepad1.left_stick_button ? drivePowerScale * gamepad1.left_stick_y : 0, Math.abs(gamepad1.right_stick_y) < 0.05 && !gamepad1.right_stick_button ? drivePowerScale * gamepad1.right_stick_y : 0);
            r.setPower("mtrLift", gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);

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

            //CONTROLLER 2
            //CONTROLLER 2

            /*
            Collection Extension Control

            Left Stick Y    -       Move Collection System
            L. Stick Button -       Stop Collection System
            */

            r.setPower("mtrCollection", Math.abs(gamepad2.left_stick_y) < 0.05 && !gamepad2.left_stick_button ? 0 : gamepad2.left_stick_y);

            /*
            Deposition Extension Control

            Right Stick Y   -       Move Deposition System
            R. Stick Button -       Stop Deposition System
            */

            r.setPower("mtrDeposition", Math.abs(gamepad2.right_stick_y) < 0.05 && !gamepad2.right_stick_button ? 0 : gamepad2.right_stick_y);

            /*
            Collection System

            DPad Up         -       Output Minerals from Chamber
            DPad Down       -       Input Minerals to Chamber
            DPad Left       -       Output Minerals from Conveyor
            DPad Right      -       Input Minerals to Conveyor
            */

            if(gamepad2.dpad_up){
                r.setServoPower("srvColR", 1);
                r.setServoPower("srvColL", -1);
            }else if(gamepad2.dpad_down){
                r.setServoPower("srvColR", -1);
                r.setServoPower("srvColL", 1);
            }else{
                r.setServoPower("srvColR", 0);
                r.setServoPower("srvColL", 0);
            }

            if(gamepad2.dpad_right) r.setServoPower("srvFlick", -1);
            else if(gamepad2.dpad_left) r.setServoPower("srvFlick", 1);
            else r.setServoPower("srvFlick", 0);

            /*
            Conveyor System

            B (One Shot)        -       Toggle On/Off for the Conveyor Belt
            Y (Trigger 200ms)   -       Conveyor Speed Up (5%)
            A (Trigger 200ms)   -       Conveyor Speed Down (5%)

            Default speed at 50%
            */

            if(gamepad2.b && !gamepad2.start && !gamepad1.start && !conveyorOS){
                conveyor = true;
                conveyorOS = true;
            }else if(!gamepad2.b) conveyorOS = false;

            if(gamepad2.y && conveyorSpeedTimer.hasTimeElapsed(200)){
                conveyorSpeed+=0.05;
                conveyorSpeedTimer.reset();
            }else if(gamepad2.a && !gamepad2.start && !gamepad1.start && conveyorSpeedTimer.hasTimeElapsed(200)){
                conveyorSpeed-=0.05;
                conveyorSpeedTimer.reset();
            }

            r.setPower("mtrConveyor", conveyor ? (conveyorSpeed = conveyorSpeed > 1 ? 1 : conveyorSpeed < 0 ? 0 : conveyorSpeed) : 0);

            //UPDATING TELEMETRY FOR THE USER

            telemetry.addData("leftRed", r.getColorValue("colorLeft", "red"));
            telemetry.addData("leftBlue", r.getColorValue("colorLeft", "blue"));
            telemetry.addData("rightRed", r.getColorValue("colorRight", "red"));
            telemetry.addData("rightBlue", r.getColorValue("colorRight", "blue"));

            telemetry.addData("isLocked", isLocked);

            telemetry.addData("Conveyor Speed", conveyorSpeed);
            telemetry.addData("Conveyor Running", conveyor);

            for(Object s : r.motors.keySet()){
                telemetry.addData((String) s, r.getPower((String)s));
                telemetry.addData((String) s + " enc", r.getEncoderCounts((String) s));
            }

            telemetry.update();
        }
    }
}
