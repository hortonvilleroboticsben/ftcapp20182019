package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestingTeleOp")
public class TestingTeleOp extends LinearOpMode {

    DcMotor drvMotor;
    CRServo srvR;
    CRServo srvL;

    @Override
    public void runOpMode() throws InterruptedException {
        drvMotor = hardwareMap.dcMotor.get("mtrArm");
        srvR = hardwareMap.crservo.get("srvLeft");
        srvL = hardwareMap.crservo.get("srvRight");

        while(!opModeIsActive());
        while(opModeIsActive()){
            drvMotor.setPower(gamepad1.right_stick_y);

            srvL.setPower(gamepad1.y ? 1 : -1);
            srvR.setPower(gamepad1.y ? -1 : 1);
        }
    }
}
