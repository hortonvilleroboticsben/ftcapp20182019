package com.hortonvillerobotics;

import android.support.annotation.Nullable;
import android.util.Log;

import com.hortonvillerobotics.RobotConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.wifi.RobotControllerAccessPointAssistant;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;

public class Robot <T extends RobotConfiguration>{
    //test

    public static String TAG = "ROBOT";

    private static Robot currInstance;


    public static <T extends RobotConfiguration> Robot getInstance (OpMode opMode, T config) {
        currInstance = currInstance == null ? new Robot<T>(opMode,config) : currInstance;
        currInstance.opMode = opMode;
        currInstance.config = config;
        return currInstance;
    }

    public Map<String, DcMotor> motors;
    public Map<String, Servo> servos;
    public Map<String, HardwareDevice> sensors;

    List<String> flags = new CopyOnWriteArrayList<>();
    public OpMode opMode = null;
    public T config = null;

    interface Task {
        void executeTasks();
    }

    private  Robot(OpMode opMode, T config) {
        initialize(opMode, config);
    }

    public void initialize (OpMode opMode, T config) {

        motors = new HashMap<>();
        servos = new HashMap<>();
        sensors = new HashMap<>();

        for (String[] motorData : config.getMotors()) {
            try {
                String motorName = motorData[0];
                DcMotor motor = (DcMotor) opMode.hardwareMap.get(motorName);

                motor.resetDeviceConfigurationForOpMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (motorData[1].equals("reverse")) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.put(motorName, motor);
            } catch (Exception e) {
                Log.e(TAG, "Failed to add motor: " + motorData[0]);
                e.printStackTrace();
            }
        }

        for (String[] servoData : config.getServos()) {
            try {
                String servoName = servoData[0];
                if(servoData.length == 1 || !servoData[1].toLowerCase().equals("cr")) {
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
                HardwareDevice sensor = opMode.hardwareMap.get(sensorName);
                sensor.resetDeviceConfigurationForOpMode();
                sensors.put(sensorName, sensor);
            } catch (Exception e) {
                Log.e(TAG, "Failed to add sensor: " + sensorData[0]);
                e.printStackTrace();
            }
        }

    }

    public void runParallel(String endTag, Task... tasks) {
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
                flags.add(endTag);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
                Log.e(TAG, "runParallel: Failed to await latch");
            }
        }).start();

    }

    public void waitForFlag(String flag) {
        boolean flagFound = false;
        while (!flagFound) {
            for (String s : flags)
                flagFound |= s.equals(flag);
        }
        flags.remove(flag);
        System.out.println("Flag \"" + flag + "\" hit.");
    }


    //----ROBOT UTILITY FUNCTIONS----//

    public void setPower(String m, double power) {
        if (motors.get(m) != null) motors.get(m).setPower(power);
    }

    @Nullable
    public int getEncoderCounts(String m) {
        return (motors.get(m) != null) ? motors.get(m).getCurrentPosition() : null;
    }

    @Nullable
    public double getPower(String m) {
        return (motors.get(m) != null) ? motors.get(m).getPower() : null;
    }

    public void resetEncoder(String m) {
        if (motors.get(m) != null) {
            setRunMode(m, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(m, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setRunMode(String m, DcMotor.RunMode rm) {
        if (motors.get(m) != null) motors.get(m).setMode(rm);
    }

    public void setTarget(String m, int target) {
        if (motors.get(m) != null) {
            setRunMode(m, DcMotor.RunMode.RUN_TO_POSITION);
            motors.get(m).setTargetPosition(target);
        }
    }

    public void runToTarget(String m, int target, double power) {
        if (motors.get(m) != null) {
            setTarget(m, target);
            setPower(m, power);
        }
    }

    public void runToTarget(String m, int target, double power, boolean reset) {
        if (motors.get(m) != null) {
            if (reset) resetEncoder(m);
            runToTarget(m, target, power);
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
        runToTarget("mtrLeftDrive", lTarget, lPow, reset);
        runToTarget("mtrRightDrive", rTarget, rPow, reset);
    }

    public boolean opModeIsActive() {
        return opMode instanceof LinearOpMode && ((LinearOpMode) opMode).opModeIsActive();
    }

    public void setServoPosition(String servo, double position){
        if(servos.get(servo) != null) servos.get(servo).setPosition(position);
    }

    @Nullable
    public boolean hasMotorEncoderReached(String m, int val){
        return (motors.get(m) != null) ? Math.abs(getEncoderCounts(m)) >= Math.abs(val) : null;
    }

    //----ROBOT FUNCTIONS BEGIN----//
    //----ROBOT FUNCTIONS BEGIN----//

    public void pause(long msec) {
        Timer t = new Timer();
        while (opModeIsActive() && !t.hasTimeElapsed(msec)) ;
    }

    public void drive(double distance) {
        drive(distance, 0.4);
    }

    public void drive(double distance, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double wheelRotations = distance / config.wheelCircumference;
        int targetEncoderCounts = (int) (wheelRotations * config.countsPerRotation);
        Log.i(TAG, "drive: Target counts: " + targetEncoderCounts);

        runDriveToTarget(targetEncoderCounts, power, targetEncoderCounts, power, true);

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
        Log.v(TAG, "drive: Successfully drove to target of " + distance + " inches");

    }


    //TURN WITH DEFAULT POWER
    public void turn(double degrees) {
        turn(degrees, 0.4);
    }

    //TURN WITH SPECIFIED POWER
    public void turn(double degrees, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");
        double turnCircumference = config.turnDiameter * Math.PI;
        double wheelRotations = (turnCircumference / config.wheelCircumference) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.countsPerRotation);
        Log.i(TAG, "turn: Target counts: " + targetEncoderCounts);
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (degrees > 0) {

            runDriveToTarget(-targetEncoderCounts, power, targetEncoderCounts, power, true);

        } else {

            runDriveToTarget(targetEncoderCounts, power, -targetEncoderCounts, power, true);

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


    public void owTurn(double degrees, double power) {
        double turnCircumference = 2 * config.turnDiameter * Math.PI;
        double wheelRotations = (turnCircumference / config.wheelCircumference) * (Math.abs(degrees) / 360);
        int targetEncoderCounts = (int) (wheelRotations * config.countsPerRotation * Math.signum(power));

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


    public void finish() {
        Log.i(TAG, "finish: entering finish phase");
//        for (DcMotor motor : motors.values()) {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
        for (String m : motors.keySet()) resetEncoder(m);
    }


}
