package com.hortonvillerobotics;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;

import boofcv.alg.filter.binary.BinaryImageOps;
import boofcv.alg.filter.binary.Contour;
import boofcv.alg.filter.binary.ThresholdImageOps;
import boofcv.android.ConvertBitmap;
import boofcv.struct.ConnectRule;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_I32;

public class Robot<T extends RobotConfiguration> {
    //this comment line is just so I can push

    public static String TAG = "ROBOTCLASS";

    private static Robot currInstance;

    public Bitmap[] cameraSnapshots;

    public final String[] blockLocation = {"error"};

    Telemetry telemetry;


    public static <T extends RobotConfiguration> Robot getInstance(OpMode opMode, T config) {
        currInstance = currInstance == null ? new Robot<T>(opMode, config) : currInstance;
        currInstance.opMode = opMode;
        currInstance.config = config;
        return currInstance;
    }

    public Map<String, DcMotor> motors;
    public Map<String, HardwareDevice> servos;
    public Map<String, HardwareDevice> sensors;

    List<String> flags = new CopyOnWriteArrayList<>();
    public OpMode opMode = null;
    public T config = null;

    public interface Task {
        void executeTasks();
    }

    private Robot(OpMode opMode, T config) {
        initialize(opMode, config);
    }

    public void initialize(@NonNull OpMode opMode, T config) {

        motors = new HashMap<>();
        servos = new HashMap<>();
        sensors = new HashMap<>();
        cameraSnapshots = new Bitmap[3];

        for (String[] motorData : config.getMotors()) {
            try {
                String motorName = motorData[0];
                Log.i(TAG, "initialize: trying to add motor: " + motorName);
                DcMotor motor = (DcMotor) opMode.hardwareMap.get(motorName);

                motor.resetDeviceConfigurationForOpMode();
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (motorData[1].equals("reverse")) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                motors.put(motorName, motor);
            } catch (Exception e) {
                Log.e(TAG, "Failed to add motor: " + motorData[0]);
                e.printStackTrace();
            }
        }

        for (String[] servoData : config.getServos()) {
            try {
                String servoName = servoData[0];
                if (servoData.length > 1 && servoData[1].equalsIgnoreCase("continuous")) {
                    CRServo servo = (CRServo) opMode.hardwareMap.get(servoName);
                    servo.resetDeviceConfigurationForOpMode();
                    servos.put(servoName, servo);
                } else {
                    Servo servo = (Servo) opMode.hardwareMap.get(servoName);
                    servo.resetDeviceConfigurationForOpMode();
                    servos.put(servoName, servo);
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to add servo: " + servoData[0]);
                e.printStackTrace();
            }
        }

        for (String[] sensorData : config.getSensors()) {
            try {
                String sensorName = sensorData[0];
                if (sensorData.length > 1) {
                    HardwareDevice sensor = opMode.hardwareMap.get(sensorName);
                    sensor.resetDeviceConfigurationForOpMode();
                    sensors.put(sensorName, sensor);
                }
            } catch (Exception e) {
                Log.e(TAG, "Failed to add sensor: " + sensorData[0]);
                e.printStackTrace();
            }
        }

        telemetry = opMode.telemetry;

    }

    public void runParallel(@NonNull String endFlagName, @NonNull Task... tasks) {
        CountDownLatch l = new CountDownLatch(tasks.length);
        for (Task t : tasks) {
            new Thread(() -> {
                t.executeTasks();
                l.countDown();
            }).start();
        }
        new Thread(() -> {
            try {
                l.await();
                flags.add(endFlagName);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
                Log.e(TAG, "runParallel: Failed to await latch");
            }
        }).start();

    }

    public void waitForFlag(@NonNull String flagName) {
        boolean flagFound = false;
        while (!flagFound && opModeIsActive()) {
            for (String s : flags)
                flagFound |= s.equals(flagName);
        }
        flags.remove(flagName);
        System.out.println("Flag \"" + flagName + "\" hit.");
    }


    //----ROBOT UTILITY FUNCTIONS----//

    public void setPower(@NonNull String motorName, double power) {
        if (motors.get(motorName) != null) motors.get(motorName).setPower(power);
    }

    @Nullable
    public Integer getEncoderCounts(@NonNull String motorName) {
        return (motors.get(motorName) != null) ? motors.get(motorName).getCurrentPosition() : null;
    }

    @Nullable
    public Double getPower(@NonNull String motorName) {
        return (motors.get(motorName) != null) ? motors.get(motorName).getPower() : null;
    }

    public void resetEncoder(@NonNull String motorName) {
        if (motors.get(motorName) != null) {
            setRunMode(motorName, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(motorName, DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            Log.e(TAG, "resetEncoder: motor is null: " + motorName);
        }
    }

    public void setRunMode(@NonNull String motorName, DcMotor.RunMode runMode) {
        if (motors.get(motorName) != null) motors.get(motorName).setMode(runMode);
    }

    public void setTarget(@NonNull String motorName, int target) {
        if (motors.get(motorName) != null) {
            setRunMode(motorName, DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(motorName).setTargetPosition(target);
        }
    }

    public void runToTarget(@NonNull String motorName, int target, double power) {
        if (motors.get(motorName) != null) {
            setTarget(motorName, target);
            setPower(motorName, power);
        } else {
            Log.e(TAG, "runToTarget: failed to run motor: " + motorName + " to target: " + target + " at power: " + power);
        }
    }

    public void runToTarget(@NonNull String motorName, int target, double power, boolean reset) {
        if (motors.get(motorName) != null) {
            if (reset) resetEncoder(motorName);
            runToTarget(motorName, target, power);
        }
    }

    public void resetDriveEncoders() {
        resetEncoder("mtrLeftDrive");
        resetEncoder("mtrRightDrive");
    }

    public void setDrivePower(double lPow, double rPow) {
        setPower("mtrLeftDrive", lPow);
        setPower("mtrRightDrive", rPow);
    }

    public void setDriveEncoderTarget(int lTarget, int rTarget) {
        setTarget("mtrLeftDrive", lTarget);
        setTarget("mtrRightDrive", rTarget);
    }

    public void setDriveRunMode(DcMotor.RunMode rm) {
        setRunMode("mtrLeftDrive", rm);
        setRunMode("mtrRightDrive", rm);
    }

    public void runDriveToTarget(int lTarget, double lPow, int rTarget, double rPow) {
        runToTarget("mtrLeftDrive", lTarget, lPow);
        runToTarget("mtrRightDrive", rTarget, rPow);
    }

    public void runDriveToTarget(int lTarget, double lPow, int rTarget, double rPow, boolean reset) {
        if (reset) resetDriveEncoders();
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDriveEncoderTarget(lTarget, rTarget);
        setDrivePower(lPow, rPow);
    }

    public boolean opModeIsActive() {
        return opMode instanceof LinearOpMode && ((LinearOpMode) opMode).opModeIsActive();
    }

    public void setServoPosition(@NonNull String servoName, double position) {
        HardwareDevice servo = servos.get(servoName);
        if (servo != null) {
            if (servo instanceof Servo) {
                ((Servo) servo).setPosition(position);
            } else if (servo instanceof CRServo) {
                Log.e(TAG, "setServoPosition: tried setting position of CR servo: " + servoName);
            } else {
                Log.println(Log.ASSERT, TAG, "setServoPosition: non-servo object in servo map: " + servoName);
            }
        } else Log.e(TAG, "setServoPosition: servo is null: " + servoName);
    }

    public void setServoPower(@NonNull String servoName, double power) {
        HardwareDevice servo = servos.get(servoName);
        if (servo != null) {
            if (servo instanceof CRServo) {
                ((CRServo) servo).setPower(power);
            } else if (servo instanceof Servo) {
                Log.e(TAG, "setServoPower: tried setting power of non CR servo: " + servoName);
            } else {
                Log.println(Log.ASSERT, TAG, "setServoPower: non-servo object in servo map: " + servoName);
            }
        } else Log.e(TAG, "setServoPower: CR servo is null: " + servoName);
    }

    @Nullable
    public Boolean hasMotorEncoderReached(@NonNull String motorName, int encoderCount) {
        return (motors.get(motorName) != null) ? Math.abs(getEncoderCounts(motorName)) >= Math.abs(encoderCount) : null;
    }

    //----ROBOT FUNCTIONS BEGIN----//
    //----ROBOT FUNCTIONS BEGIN----//

    public void pause(long msec) {
        Timer t = new Timer();
        while (opModeIsActive() && !t.hasTimeElapsed(msec)) ;
    }

    //DRIVE WITH DEFAULT POWER OF 72%
    public void drive(double distance) {
        drive(distance, 0.72);
    }

    //DRIVE WITH SPECIFIED POWER
    public void drive(double distance, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double wheelRotations = distance / config.getWheelCircumference();
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation());
        Log.i(TAG, "drive: Target counts: " + targetEncoderCounts);

        runDriveToTarget(-targetEncoderCounts, power, -targetEncoderCounts, power, true);

        while (opModeIsActive()) {

            Log.d(TAG, "turn: current right count: " + mtrRightDrive.getCurrentPosition());
            Log.d(TAG, "turn: current left count: " + mtrLeftDrive.getCurrentPosition());

            opMode.telemetry.addData("target", targetEncoderCounts);
            opMode.telemetry.update();

            if (Math.abs(getEncoderCounts("mtrLeftDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrLeftDrive", 0);
            } else {
                setPower("mtrLeftDrive", power);
            }

            if (Math.abs(getEncoderCounts("mtrRightDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrRightDrive", 0);
            } else {
                setPower("mtrRightDrive", power);
            }

            if (getPower("mtrLeftDrive") == 0 && getPower("mtrRightDrive") == 0) break;
        }


        resetDriveEncoders();
        Log.v(TAG, "drive: Successfully drove to target of " + distance + " inches");

    }


    //TURN WITH DEFAULT POWER OF 40%
    public void turn(double degrees) {
        turn(degrees, 0.4);
    }

    //TURN WITH SPECIFIED POWER
    public void turn(double degrees, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double turnCircumference = config.getTurnDiameter() * Math.PI;
        double wheelRotations = (turnCircumference / config.getWheelCircumference()) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation());
        Log.i(TAG, "turn: Target counts: " + targetEncoderCounts);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (degrees > 0) {

            runDriveToTarget(targetEncoderCounts, power, -targetEncoderCounts, power, true);

        } else {

            runDriveToTarget(-targetEncoderCounts, power, targetEncoderCounts, power, true);

        }

        while (opModeIsActive()) {

            Log.d(TAG, "turn: current right count: " + mtrRightDrive.getCurrentPosition());
            Log.d(TAG, "turn: current left count: " + mtrLeftDrive.getCurrentPosition());

            if (Math.abs(getEncoderCounts("mtrLeftDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrLeftDrive", 0);
            } else {
                setPower("mtrLeftDrive", power);
            }

            if (Math.abs(getEncoderCounts("mtrRightDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrRightDrive", 0);
            } else {
                setPower("mtrRightDrive", power);
            }

            if (getPower("mtrLeftDrive") == 0 && getPower("mtrRightDrive") == 0) break;
        }

        mtrLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Log.v(TAG, "turn: Successfully turned to target of " + degrees + " degrees");
    }

    /* DEGREES CONTROLS THE DISTANCE OF THE TURN
     * THE SIGN OF DEGREES CONTROLS WHICH MOTOR
     * THE SIGN OF POWER CONTROLS THE DIRECTION OF THE MOTOR */
    public void owTurn(double degrees, double power) {
        double turnCircumference = 2 * config.getTurnDiameter() * Math.PI;
        double wheelRotations = (turnCircumference / config.getWheelCircumference()) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation() * Math.signum(power));

        Log.i(TAG, "owturn: Target counts: " + targetEncoderCounts);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean targetReached = false;

        if (degrees > 0) {

            if (power < 0) {
                Log.i(TAG, "owturn: power less than 0");
            }

            runDriveToTarget(0, 0, targetEncoderCounts, power, true);

            while (opModeIsActive() && Math.abs(getEncoderCounts("mtrRightDrive")) < Math.abs(targetEncoderCounts) - 20) {
                Log.d(TAG, "owturn: current right count: " + getEncoderCounts("mtrRightDrive"));
                Log.d(TAG, "owturn: current right power: " + getPower("mtrRightDrive"));
            }
            setDrivePower(0.0, 0.0);

        } else {

            if (power < 0) {
                Log.i(TAG, "owturn: power less than 0");
            }

            runDriveToTarget(targetEncoderCounts, power, 0, 0, true);

            while (opModeIsActive() && Math.abs(getEncoderCounts("mtrLeftDrive")) < Math.abs(targetEncoderCounts) - 20) {
                Log.d(TAG, "owturn: current left count: " + getEncoderCounts("mtrLeftDrive"));
                Log.d(TAG, "owturn: current left power: " + getPower("mtrLeftDrive"));
            }
            setDrivePower(0.0, 0.0);

        }
    }

    //METHOD TO BE CALLED UPON COMPLETION OF AN AUTONOMOUS PROGRAM IF ENCODERS ARE DESIRED TO BE RESET
    public void finish() {
        Log.i(TAG, "finish: entering finish phase");
        for (String m : motors.keySet()) resetEncoder(m);
    }

    //SEASON SPECIFIC FUNCTIONS
    //SEASON SPECIFIC FUNCTIONS

    //TODO implement this method
    static byte[][] out = new byte[3][];

    public void getCameraCapture() {

        CountDownLatch cdl = new CountDownLatch(1);
        final int[] i = new int [1];
        i[0] = 0;

        FtcRobotControllerActivity.cp.mCamera.setPreviewCallback((bytes, camera) -> {
            Log.i(TAG,"getCameraCapture: inside camera callback");
            Camera.Parameters parameters = camera.getParameters();
            parameters.getPreviewFormat();
            Camera.Size size = parameters.getPreviewSize();
            YuvImage image = new YuvImage(bytes, parameters.getPreviewFormat(),
                    size.width, size.height, null);
            ByteArrayOutputStream outS = new ByteArrayOutputStream();
            image.compressToJpeg(new Rect(0,0,size.width, size.height),100, outS);
            FileUtils.writeToFile("/"+i[0]+".jpg",outS.toByteArray());
            out[i[0]++] = outS.toByteArray();
            cdl.countDown();
        });
        try {
            cdl.await();
            FtcRobotControllerActivity.cp.mCamera.setPreviewCallback(null);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        for (int j = 0; j < out.length && opModeIsActive(); j++) {
            try {
                cameraSnapshots[j] = BitmapFactory.decodeByteArray(out[j], 0, out[j].length);
            } catch (Exception e) {
                Log.e(TAG,"getCameraCapture: something is null");
                e.printStackTrace();
            }
        }

    }

    public void analyzePhotoData() {
//        (new Thread(() -> {

//            Planar<GrayF32> layers = ConvertBufferedImage.convertFromPlanar(originalBufferedImage, null, true, GrayF32.class);
        for (Bitmap bitmap : cameraSnapshots) {
            if(!opModeIsActive()) break;
            if (bitmap == null) continue;
            Planar<GrayF32> layers = ConvertBitmap.bitmapToPlanar(bitmap, null, GrayF32.class, null);

            GrayF32 blueLayer = layers.getBand(2);
//            int width = 2 * (blueLayer.width / 5), height = blueLayer.height;
            int width = blueLayer.width/3-blueLayer.width/8, height = blueLayer.height;
            GrayF32 subImage = blueLayer.subimage(blueLayer.width/8, 0, blueLayer.width/3, blueLayer.height);

            GrayU8 thresh = new GrayU8(width, height),
                    dilated = new GrayU8(width, height),
                    eroded = new GrayU8(width, height);

            float threshold = 160;
            ThresholdImageOps.threshold(subImage, thresh, threshold, false);
            BinaryImageOps.erode8(thresh, 2, eroded);
            BinaryImageOps.dilate8(eroded, 2, dilated);

            GrayU8 g = dilated.clone();

//            for(int index = 0; index < g.data.length; index++) {
//                if(g.data[index] > 0) g.data[index] = (byte) 255;
//            }

            for(int x = 0; x < g.width; x++) {
                for(int y = 0; y < g.height; y++) {
                    if(g.get(x,y)>0) g.set(x,y,50);
                }
            }

//            for(byte b : g.data)Log.d("ImageData",""+b);


            Bitmap b = ConvertBitmap.grayToBitmap(g, Bitmap.Config.ARGB_8888);
            ByteArrayOutputStream outS = new ByteArrayOutputStream();
            b.compress(Bitmap.CompressFormat.JPEG, 100, outS);
            FileUtils.writeToFile("/"+3+".jpg", outS.toByteArray());


            List<Contour> contours =
                    BinaryImageOps.contour(dilated, ConnectRule.EIGHT, null);


            int requiredSize = 8000, numLarger = 0;
            Point2D_I32 p1 = null;
            for (Contour c : contours) {
                int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE, minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
                if (c.external.size() == 0) continue;
                for (Point2D_I32 p : c.external) {
                    if (p.x > maxX) maxX = p.x;
                    if (p.x < minX) minX = p.x;
                    if (p.y > maxY) maxY = p.y;
                    if (p.y < minY) minY = p.y;
                }
                int w = maxX - minX;
                int h = maxY - minY;
                int size = w * h;

                if (size > requiredSize) {

//                    FileUtils.appendToFile("/sizes.txt", size+"\r\n");

                    int avg_x = 0, avg_y = 0;
                    for (Point2D_I32 p : c.external) {
                        avg_x += p.x;
                        avg_y += p.y;
                    }
                    avg_x /= c.external.size();
                    avg_y /= c.external.size();

                    if (numLarger == 0) {
                        p1 = new Point2D_I32(avg_x, avg_y);
                        numLarger++;
                    } else if (numLarger == 1) {
                        numLarger++;
                        break;
                    }
                }
            }

            blockLocation[0] = (numLarger == 2) ? "right" : (numLarger == 1) ? (p1.y > height / 2) ? "left" : "center" : "error";
        }

//        })).run();
    }

    public void lift(double distance, double power) {
        DcMotor mtrLift = motors.get("mtrLift");
        double wheelRotations = distance / config.getWheelCircumference();
        int targetEncoderCounts = (int) (wheelRotations * config.getCountsPerRotation());
        Log.i(TAG, "drive: Target counts: " + targetEncoderCounts);

        runDriveToTarget(targetEncoderCounts, power, targetEncoderCounts, power, true);

        while (opModeIsActive()) {

            Log.d(TAG, "turn: current lift count: " + mtrLift.getCurrentPosition());

            if (Math.abs(getEncoderCounts("mtrLift")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrLift", 0);
            } else {
                setPower("mtrLift", power);
            }

            if (Math.abs(getEncoderCounts("mtrRightDrive")) >= Math.abs(targetEncoderCounts) - 20) {
                setPower("mtrRightDrive", 0);
            } else {
                setPower("mtrRightDrive", power);
            }

            if (getPower("mtrLeftDrive") == 0 && getPower("mtrRightDrive") == 0) break;
        }

        Log.v(TAG, "drive: Successfully drove to target of " + distance + " inches");

    }


}
