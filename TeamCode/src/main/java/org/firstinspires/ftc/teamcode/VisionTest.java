package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.media.MediaPlayer;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.View;

import com.hortonvillerobotics.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.ar.pl.Camera2_Preview;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Arrays;

import boofcv.android.ConvertBitmap;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

@TeleOp(name = "VisionTest", group = "Test")
public class VisionTest extends LinearOpMode {

    int pCount = 0;
    Camera c;
    boolean camOS = false;
    boolean pic = false;
    Bitmap b;
    Timer t = new Timer();
    long minElapsed = Long.MAX_VALUE;

    @Override
    public void runOpMode() throws InterruptedException {
        c = FtcRobotControllerActivity.cp.mCamera;
        c.setDisplayOrientation(90);
        c.setPreviewCallback(new Camera.PreviewCallback() {
            @Override
            public void onPreviewFrame(byte[] bytes, Camera camera) {
                if (pic) {
                    Camera.Parameters parameters = camera.getParameters();
                    Camera.Size size = parameters.getPreviewSize();
                    YuvImage image = new YuvImage(bytes, parameters.getPreviewFormat(),
                            size.width, size.height, null);
                    bytes = image.getYuvData();

                    b = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);

                    minElapsed = t.getTimeElapsed();//Math.min(minElapsed, t.getTimeElapsed());
                    pic = false;
                }
            }
        });

        c.startPreview();
        while (!opModeIsActive()) {
        }
        while (opModeIsActive()) {
            telemetry.addData("Elapsed Time", this.getRuntime());
            telemetry.addData("Counter", pCount);
            if (b != null) {
//                for(int i = 0; i < b.getHeight(); i++){
//                    for(int j = 0; j < b.getWidth(); j++){
//
//                    }
//                }
                Planar<GrayU8> img = ConvertBitmap.bitmapToPlanar(b, null, GrayU8.class, null);
                telemetry.addData("Img red portion", Arrays.toString(new int[]{img.bands[0].get(80, 80), img.bands[1].get(80, 80), img.bands[2].get(80, 80)}));
            }
            telemetry.addData("b", b == null ? "null" : b);
            telemetry.addData("timeElapsed", minElapsed);
            telemetry.update();


            if (gamepad1.right_trigger > 0.5 && !camOS) {
                camOS = true;
                pic = true;
                t.reset();
//                try{
//                c.takePicture(null,null, new Camera.PictureCallback(){
//                    @Override
//                    public void onPictureTaken(byte[] bytes, Camera camera) {
//                        Log.e("CHECKPOINT","Hit internal method");
//                        int[] arr = new int[bytes.length];
//                        for(int i = 0; i < bytes.length; i++) arr[i] = bytes[i];
//                        Bitmap b = Bitmap.createBitmap(arr, camera.getParameters().getPictureSize().width, camera.getParameters().getPictureSize().height, Bitmap.Config.RGB_565);
//                        pCount++;
//                    }
//                });}catch(Exception e){e.printStackTrace();}
            }
            if (gamepad1.right_trigger <= 0.5) camOS = false;

        }
    }

}
