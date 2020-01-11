/* real logger code
  TODO (EithneA-V)
    1) Create Logging System class
        1.1) Constructor function (base class pointer)
        1.2) Member var that is a list of obj to call when logging
            1.2.1) List of things it can call (and that can call it --> same that it can call)
            1.2.2) Can add/append/register whenever smthng new calls it
        1.3) Get data from list in 2.2
            1.3.1) Writes data to logging file (specify file name for each class)
        1.4) Create LS thread and update regularly
*/
package com.team1678.frc2020.logger;

import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.loops.ILooper;

import java.util.ArrayList;
import java.io.FileWriter;

public class LoggingSystem {
    private static LoggingSystem mInstance; 
    // 1.1
    //  LoggableItems object
    ArrayList<ILoggable> loggable_items = new ArrayList<ILoggable>();

    //  creating a usingFileWriter object per loggable file
    ArrayList<FileWriter> loggable_files = new ArrayList<FileWriter>();
    private LoggingSystem() {
    }
    public synchronized static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem();
        }
        return mInstance; 
    }
    public void register(ILoggable new_loggable, String filename) {  //  start function that opens files
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(filename);
        } catch (Exception e) {}
        ArrayList<String> item_names = new_loggable.get_item_names();
        loggable_files.add(fileWriter);
        // write names to file
        try {
        for (int h=0; h < item_names.size(); h++) {
            fileWriter.write(item_names.get(h));
            fileWriter.write(",");
            }
            fileWriter.write("\n");
            //  adding Loggable to Loggable_items list
            loggable_items.add(new_loggable);
        } catch (Exception e) {}
    }
    void Log() {  //  function that gets called and told when to log by main
        try{
            for (int i=0; i < loggable_items.size(); i++) {
               ArrayList<ArrayList<Double>> items = loggable_items.get(i).get_items();
               //  assertArrayEquals(items[i], item_names[i]);
               //  get object fileWriter from the list 
               FileWriter fileWriter = loggable_files.get(i);
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
        } catch (Exception e) {}  // making compiler happy by trying to catch stuff that could crash 
    }
    void Close() {  //  close file
        try {
            for (int i=0; i< loggable_files.size(); i++) {
                FileWriter fileWriter = loggable_files.get(i);
                fileWriter.close();
            }
        } catch (Exception e) {}
    }
    public void registerLoops(ILooper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
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