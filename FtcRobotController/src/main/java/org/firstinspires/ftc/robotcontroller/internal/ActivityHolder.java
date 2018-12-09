package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;

public class ActivityHolder{

    static FtcRobotControllerActivity activity;

    public static void setActivity(FtcRobotControllerActivity a){
        activity = a;
    }


    public static FtcRobotControllerActivity getActivity(){
        return activity;
    }
}
