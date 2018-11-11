package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.hortonvillerobotics.UVCCamera;

@Autonomous(name="VisionTest2")
public class VisionTest2 extends LinearOpMode {

    UVCCamera c;

    @Override
    public void runOpMode() throws InterruptedException {

        c = UVCCamera.getCamera(new UVCCamera.Callback() {
            @Override
            public Bitmap onFrame(Bitmap bm) {
                telemetry.addData("Pix(0,0):",bm.getPixel(0,0));
                return bm;
            }
        });

        while(!opModeIsActive());
        while(opModeIsActive()){
            c.start();
        }
        c.stop();
    }
}
