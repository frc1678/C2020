package com.team1678.frc2020.logger;

import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.loops.ILooper;

import java.util.ArrayList;
import java.io.File;

import java.io.FileWriter;

public class LoggingSystem {
    private static LoggingSystem mInstance; 
    //  set original directory path. Will be added to in LoggingSystem() when new directories are created inside /home/lvuser/logs
    public static String mDirectory = "/home/lvuser/logs";
    ArrayList<ILoggable> loggableItems = new ArrayList<ILoggable>();

    ArrayList<FileWriter> loggableFiles = new ArrayList<FileWriter>();

    /* 
        Create a for loop that goes over all the current files and subdirectories in mDirectories.
        If the directory is empty (when the max number is 0), start a new subdirectory at 1.
        Whenever the logging system reboots, function will scan over all the existing files and subdirectories and find the largest one.
        New subdirectory is created by adding one (1) to the max file number.
    */
    private LoggingSystem() {
    }

    public  void LogDirectory() {
        File Directory = new File(mDirectory);
        Integer maxNum = 0;
        if  (! Directory.isDirectory()) {
            Directory.mkdir();
        }
        
        for (final File directoryEntry : Directory.listFiles()) {
            try {
                if (directoryEntry.isDirectory()) {
                    int Num = Integer.parseInt(directoryEntry.getName());
                    if (Num > maxNum) {
                        maxNum = Num;
                    }
                } 
            } catch (Exception e) {
                //  Files that are not numbers are expected and ignored
            }
        }
        maxNum++;
        mDirectory = mDirectory + "/" + maxNum.toString(); 
        File newDirectory = new File(mDirectory);
        newDirectory.mkdir();
    }

    public synchronized static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem();
        }
        return mInstance; 
    }
    //  start function that opens file
    public void register(ILoggable newLoggable, String fileName) {
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(mDirectory + "/" + fileName);
        } catch (Exception e) {
            System.err.println("Couldn't register new file" + fileName);
        }
        ArrayList<String> itemNames = newLoggable.getItemNames();
        loggableFiles.add(fileWriter);
        //  Write names to file
        try {
            for (int h=0; h < itemNames.size(); h++) {
                fileWriter.write(itemNames.get(h));
                if (h!= itemNames.size()) {
                    fileWriter.write(",");
                }
            }
            fileWriter.write("\n");
            //  Adding Loggable to loggableItems list
            loggableItems.add(newLoggable);
        } catch (Exception e) {
            System.err.println("Couldn't write to file");
        }
    }
    //  Logging Function
    //  gets called when main begins logging
    void Log() {
        try{
            for (int i=0; i < loggableItems.size(); i++) {
               ArrayList<ArrayList<Double>> items = loggableItems.get(i).getItems();
               //  get object fileWriter from the list 
               FileWriter fileWriter = loggableFiles.get(i);
               //  write to files
               for (int j=0; j < items.size(); j++) {
                   ArrayList<Double> data = items.get(j);
                   for (int m=0; m < data.size(); m++){
                        fileWriter.write(data.get(m).toString());
                        if (m != data.size()){
                            fileWriter.write(",");
                        }
                    }
                    fileWriter.write("\n");
                }
            }
        } catch (Exception e) {
            System.err.println("Couldn't get object and/or log it");
        }
    }
    //  Close Logging System
    void Close() {
        try {
            //  Get final logs
            Log();
            //  Close files 
            for (int i=0; i< loggableFiles.size(); i++) {
                FileWriter fileWriter = loggableFiles.get(i);
                fileWriter.close();
            }
        } catch (Exception e) {
            System.err.println("Couldn't close file");
        }
    }
    public void registerLoops(ILooper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                LogDirectory();
            }
            @Override 
            public void onLoop(double timestamp) {
                Log();
            }
            @Override 
            public void onStop(double timestamp) {
            }
        });
    }
}
