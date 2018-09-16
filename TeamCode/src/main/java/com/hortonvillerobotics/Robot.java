package com.hortonvillerobotics;
import android.util.Log;

import com.hortonvillerobotics.RobotConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;

public class Robot {

    public static String TAG = "ROBOT";

    private static Robot currInstance;

    public static Robot getInstance(LinearOpMode opMode) {
        currInstance = currInstance == null ? new Robot(opMode) : currInstance;
        Robot.opMode = opMode;
        return currInstance;
    }

    Map<String,DcMotor> motors;
    Map<String,Servo> servos;
    Map<String,HardwareDevice> sensors;

    List<String> flags = new CopyOnWriteArrayList<>();
    public static OpMode opMode = null;

    interface Task {
        void executeTasks();
    }

    private Robot(OpMode opMode) {
        initialize(opMode);
    }

    public void initialize(OpMode opMode) {

        motors = new HashMap<>();
        servos = new HashMap<>();
        sensors = new HashMap<>();

        for(String motorName : RobotConfiguration.motors) {
            try {
                DcMotor motor = (DcMotor) opMode.hardwareMap.get(motorName);

                motor.resetDeviceConfigurationForOpMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(motorName.equals("mtrLeftDrive")) {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                }

                motors.put(motorName, motor);
            } catch (Exception e) {
                Log.e(TAG,"Failed to add motor: " + motorName);
                e.printStackTrace();
            }
        }

        for(String servoName : RobotConfiguration.servos) {
            try {
                Servo servo = (Servo) opMode.hardwareMap.get(servoName);
                servo.resetDeviceConfigurationForOpMode();
                servos.put(servoName, servo);
            } catch (Exception e) {
                Log.e(TAG,"Failed to add servo: " + servoName);
                e.printStackTrace();
            }
        }

        for(String sensorName : RobotConfiguration.sensors) {
            try {
                HardwareDevice sensor = opMode.hardwareMap.get(sensorName);
                sensor.resetDeviceConfigurationForOpMode();
                sensors.put(sensorName, sensor);
            } catch (Exception e) {
                Log.e(TAG,"Failed to add sensor: " + sensorName);
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


    //----ROBOT FUNCTIONS BEGIN----//
    //----ROBOT FUNCTIONS BEGIN----//

    public void drive(double distance) {
        drive(distance,0.72);
    }
    public void drive(double distance, double power) {
        DcMotor mtrLeftDrive = motors.get("mtrLeftDrive"), mtrRightDrive = motors.get("mtrRightDrive");

        boolean nullMotorError = false;

        if(mtrLeftDrive == null) {
            Log.e(TAG,"Left Drive Motor is null");
            nullMotorError = true;
        }

        if(mtrRightDrive == null) {
            Log.e(TAG,"Right Drive Motor is null");
            nullMotorError = true;
        }

        //EXIT DRIVE IF EITHER MOTOR IS NULL
        if(nullMotorError) {
            opMode.telemetry.addData("Drive","Exiting drive due to null motor");
            return;
        }

        double wheelRotations = distance / RobotConfiguration.wheelCircumference;

        int targetEncoderCounts = (int)(wheelRotations * RobotConfiguration.countsPerRotation);
        Log.i(TAG,"drive: Target counts: "+targetEncoderCounts);

        mtrLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrLeftDrive.setTargetPosition(targetEncoderCounts);
        mtrLeftDrive.setPower(power);

        mtrRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrRightDrive.setTargetPosition(targetEncoderCounts);
        mtrRightDrive.setPower(power);

        boolean targetReached = false, leftReached = false, rightReached = false;

        while(((LinearOpMode)opMode).opModeIsActive() && !targetReached) {

            Log.d(TAG,"drive: current right count: " + mtrRightDrive.getCurrentPosition());
            Log.d(TAG,"drive: current left count: " + mtrLeftDrive.getCurrentPosition());

            if(Math.abs(mtrLeftDrive.getCurrentPosition()) >= targetEncoderCounts) {
                mtrLeftDrive.setPower(0);
                leftReached = true;
            } else {
                mtrLeftDrive.setPower(power);
            }

            if(Math.abs(mtrRightDrive.getCurrentPosition()) >= targetEncoderCounts) {
                mtrRightDrive.setPower(0);
                rightReached = true;
            } else {
                mtrRightDrive.setPower(power);
            }

            if(leftReached && rightReached) {
                targetReached = true;
            }
        }

        if(targetReached) {
            mtrLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtrRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Log.v(TAG,"drive: Successfully drove to target of " + distance + " inches");
        } else {
            Log.e(TAG,"drive: Opmode status is: "+((LinearOpMode) opMode).opModeIsActive());
            Log.e(TAG,"drive: OpMode aborted prior to reaching target of " + distance + " inches");
        }

    }


}
