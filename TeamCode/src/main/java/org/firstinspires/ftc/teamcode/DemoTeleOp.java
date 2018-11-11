package org.firstinspires.ftc.teamcode;

import com.hortonvillerobotics.DemoRobotConfiguration;
import com.hortonvillerobotics.Robot;
import com.hortonvillerobotics.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.xmlpull.v1.XmlPullParser;

@TeleOp(name="DemoTeleOp",group = "Demo")
public class DemoTeleOp extends LinearOpMode {

    Robot r;
    Servo srvL, srvR;
    DcMotor mtrArm;
    boolean launching = false;
    Timer t = new Timer();

    void parseXML(Object o){
//        XmlPullParser x = new Pull;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        r = Robot.getInstance(this, new DemoRobotConfiguration());
        r.initialize(this, new DemoRobotConfiguration());

        r.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        r.setServoPosition("srvRight", 1);
//        r.setServoPosition("srvLeft", 0);

        mtrArm = hardwareMap.dcMotor.get("mtrArm");
        srvR = hardwareMap.servo.get("srvLeft");
        srvL = hardwareMap.servo.get("srvRight");

        srvL.setPosition(1);
        srvR.setPosition(0);


        waitForStart();

        while(opModeIsActive()){
           r.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

           if(gamepad1.right_trigger >= 0.5) {
//               r.setServoPosition("srvRight", .9);
//               r.setServoPosition("srvLeft", .1);
               srvR.setPosition(.9);
               srvL.setPosition(.1);
           }else{
//               r.setServoPosition("srvRight", 1);
//               r.setServoPosition("srvLeft", 0);
               srvR.setPosition(1);
               srvL.setPosition(0);
           }



           if((gamepad1.a && !gamepad1.start && !launching) || launching){
               launching = true;
//               r.runToTarget("mtrArm", -50, 0.2, true);
//               if(Math.abs(r.motors.get("mtrArm").getCurrentPosition()) <= Math.abs(-50)) {
//                   launching = false;
//                   r.setPower("mtrArm", 0.0);
//               }
               mtrArm.setPower(1);
               if(t.hasTimeElapsed(500)){
                   mtrArm.setPower(0);
                   launching = false;
               }

           } else t.reset();

        }
    }
}
