package com.hortonvillerobotics;

import android.os.Environment;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

public class FileUtils {
    public static void writeToFile(String fileName, Object contents){
        try {
            File f = new File(Environment.getExternalStorageDirectory() + fileName);
            checkFileExistence(f);
            OutputStream o = new FileOutputStream(f,false);
            o.write(contents instanceof byte[] ? (byte[])contents : contents.toString().getBytes());
            o.flush();
            o.close();
        }catch(Exception e){e.printStackTrace();}
    }

    public static void appendToFile(String fileName, Object contents){
        try{
            File f = new File(Environment.getExternalStorageDirectory() + fileName);
            checkFileExistence(f);
            OutputStream o = new FileOutputStream(f, true);
            byte[] b = contents instanceof byte[] ? (byte[])contents : contents.toString().getBytes();
            o.write(b);
            o.flush();
            o.close();
        }catch(Exception e){e.printStackTrace();}
    }

    public static String readFromFile(String fileName){
        try{
            File f = new File(Environment.getExternalStorageDirectory() + fileName);
            checkFileExistence(f);
            InputStream i = new FileInputStream(f);
            byte[] b = new byte[(int)f.length()];
            i.read(b);
            return new String(b);
        }catch(Exception e){e.printStackTrace();}
        return null;
    }

    private static void checkFileExistence(File f){
        try{
            if(!f.exists()){
                f.createNewFile();
            }
        }catch(Exception e){e.printStackTrace();}
    }

}
