package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.hortonvillerobotics.FileUtils;
import com.hortonvillerobotics.FinalRobotConfiguration;
import com.hortonvillerobotics.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public class InterpreterOpmode extends LinearOpMode {

    final static String TAG = "InterpreterOpmode";

    final static String fileName = "script";

    List<Object[]> methods = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        final Robot rbt = Robot.getInstance(this, new FinalRobotConfiguration());
        rbt.initialize(this, new FinalRobotConfiguration());

        try {
            byte[] data = FileUtils.readFromFile(fileName);
            if (data != null) {
                String fullCommands = new String(data);
                for (String command : fullCommands.split("\n")) {
                    if (command.startsWith("drive(")) {
                        if (command.contains(",")) {

                            String[] split = command.replace("drive(", "")
                                    .replace(")\n", "")
                                    .replace("\r", "")
                                    .split(",");

                            double distance = Double.parseDouble(split[0]);
                            double speed = Double.parseDouble(split[1]);

                            Object[] params = {"drive", distance, speed};
                            methods.add(params);
                        } else {
                            double distance = Double.parseDouble(
                                    command.replace("drive(", "")
                                            .replace(")\n", "")
                                            .replace("\r", "")
                            );
                            Object[] params = {"drive", distance};
                            methods.add(params);
                        }
                    } else if (command.startsWith("turn(")) {
                        if (command.contains(",")) {

                            String[] split = command.replace("turn(", "")
                                    .replace(")\n", "")
                                    .replace("\r", "")
                                    .split(",");

                            double degrees = Double.parseDouble(split[0]);
                            double speed = Double.parseDouble(split[1]);

                            Object[] params = {"turn", degrees, speed};
                            methods.add(params);
                        } else {
                            double degrees = Double.parseDouble(
                                    command.replace("turn(", "")
                                            .replace(")\n", "")
                                            .replace("\r", "")
                            );
                            Object[] params = {"turn", degrees};
                            methods.add(params);
                        }
                    } else if (command.startsWith("owTurn(")) {

                        String[] split = command.replace("owTurn(", "")
                                .replace(")\n", "")
                                .replace("\r", "")
                                .split(",");

                        double degrees = Double.parseDouble(split[0]);
                        double speed = Double.parseDouble(split[1]);

                        Object[] params = {"owTurn", degrees, speed};
                        methods.add(params);

                    }
                }

            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        waitForStart();

        for (Object[] methodData : methods) {
            try {
                if (methodData[0].equals("drive")) {
                    switch (methodData.length) {
                        case 2:
                            rbt.drive((double) methodData[1]);
                            break;
                        case 3:
                            rbt.drive((double) methodData[1], (double) methodData[2]);
                            break;
                        default:
                            Log.e(TAG, "Invalid method in method list");
                    }
                } else if (methodData[0].equals("turn")) {
                    switch (methodData.length) {
                        case 2:
                            rbt.turn((double) methodData[1]);
                            break;
                        case 3:
                            rbt.turn((double) methodData[1], (double) methodData[2]);
                            break;
                        default:
                            Log.e(TAG, "Invalid method in method list: " + methodData[0]);
                    }
                } else if (methodData[0].equals("owTurn")) {
                    switch (methodData.length) {
                        case 3:
                            rbt.owTurn((double) methodData[1], (double) methodData[2]);
                            break;
                        default:
                            Log.e(TAG, "Invalid method in method list");
                    }
                } else {
                    Log.e(TAG, "Invalid method in method list: " + methodData[0]);
                }
            } catch (Exception e) {
                e.printStackTrace();
                Log.e(TAG,"Something just went really wrong");
            }
        }

    }
}
