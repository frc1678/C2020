/*  While using the reflective system to implement the logging system in subsystems is the cleanest 
    way to approach the implementation, IT IS 3X SLOWER than implementation by hand (about 300 milisec 
    rather than 90 milisec)
*/ 

package com.team1678.frc2020.logger;
import java.util.ArrayList;
import java.lang.reflect.Field;

public class ReflectingLogStorage<T> extends LogStorage<T> {
    private Field[] mVars;
    ArrayList<Double> newRow = new ArrayList<Double>();

    public ReflectingLogStorage(Class<T> typeClass) {
        //  Getting member variables
        mVars = typeClass.getFields();
    }
    public ArrayList<String> getItemNames() {
        //  create array list to the names of the member variables collected
        ArrayList<String> getItemNames = new ArrayList<String>();
            //  For the member variables, get the variable  and add it to the array list
            for (Field variableName : mVars) {
                getItemNames.add(variableName.getName());
            }
        //  Log item names from array list
        return getItemNames;
    }
    //  Collect data from the variables and store in a new array list
    public void Add(T classData) {
        newRow.clear();
        for (Field variableName : mVars) {
            try {
                //  add data from the variables and make sure they're doubles to add them to the array list 
                newRow.add(((Number)variableName.get(classData)).doubleValue());
            } catch (Exception e) {
                System.err.print("Unable to add variable name data to new row as a double value");
            }
        }
        addData(newRow);
    }
}