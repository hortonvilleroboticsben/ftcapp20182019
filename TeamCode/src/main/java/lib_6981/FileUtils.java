package lib_6981;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

public class FileUtils {
    public static void writeToFile(String fileName, Object contents){
        try {
            File f = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
            OutputStream o = new FileOutputStream(f);
            o.write(contents.toString().getBytes());
            o.flush();
            o.close();
        }catch(Exception e){e.printStackTrace();}
    }

    public static void appendToFile(String fileName, Object contents){
        try{
            File f = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
            OutputStream o = new FileOutputStream(f);
            byte[] b = contents.toString().getBytes();
            o.write(b, (int)f.length(), b.length);
            o.flush();
            o.close();
        }catch(Exception e){e.printStackTrace();}
    }

    public static String readFromFile(String fileName){
        try{
            File f = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
            InputStream i = new FileInputStream(f);
            byte[] b = new byte[(int)f.length()];
            i.read(b);
            return new String(b);
        }catch(Exception e){e.printStackTrace();}
        return null;
    }

}
