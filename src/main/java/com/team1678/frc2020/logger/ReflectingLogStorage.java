package com.team1678.frc2020.logger;
import java.util.ArrayList;
import java.lang.reflect.Field;

public class ReflectingLogStorage<T> extends LogStorage {
    private Field[] mVars;

    public ReflectingLogStorage(Class<T> typeClass) {
        mVars = typeClass.getFields(); //  getting member variables
    }
    public ArrayList<String> getItemNames() {
    ArrayList<String> getItemNames = new ArrayList<String>();
        for (Field variableName : mVars) {
            getItemNames.add(variableName.getName());
        }
        return getItemNames;
    }
    public void Add(T classData) {
        ArrayList<Double> newRow = new ArrayList<Double>();
        for (Field variableName : mVars) {
            try {
                newRow.add(((Number)variableName.get(classData)).doubleValue());
            } catch (Exception e) {
                System.err.print("Unable to add variable name data to new row as a double value");
            }
        }
    }
}