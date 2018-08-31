package lib_6981;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;

public class Robot {

    //THIS IS SAM

    public static String TAG = "ROBOT";

    private static Robot currInstance;

    public static Robot getInstance() {
        return currInstance == null ? currInstance = new Robot() : currInstance;
    }

    List<String> flags = new CopyOnWriteArrayList<>();

    interface Task {
        void executeTasks();
    }

    private Robot() {
    }

    public void parallelProcess(String endTag, Task... tasks) {
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
                    Log.e(TAG,"parallelProcess: Failed to await latch");
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

    public static void main(String[] args) {
        Robot r = Robot.getInstance();
        for (int j = 0; j < 1000; j++) {
            r.parallelProcess("Breads", (() -> {
//            try{Thread.sleep(0);}catch(Exception e){}
                for (int i = 0; i < 10; i++) {
                    try {
                        Thread.sleep((int) (Math.random() * 100));
                    } catch (Exception e) {
                    }
                    System.out.println(i % 2 == 0 ? "Garlic" : "Wheat");
                }
            }), (() -> {
                for (int i = 1; i <= 10; i++) {
                    try {
                        Thread.sleep((int) (Math.random() * 100));
                    } catch (Exception e) {
                    }
                    System.out.println(i);
                }
            }));

            r.waitForFlag("Breads");

            System.out.println("Iteration " + j + " complete.");
            System.out.println();
        }

        System.out.println("Program Complete");
        System.out.println(r.flags);
    }
}
