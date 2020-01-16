package com.team1678.frc2020.logger;

import java.util.ArrayList;

public class LogStorage implements ILoggable {
    ArrayList<String> mColumns = new ArrayList<String>();
    ArrayList<ArrayList<Double>> mItems = new ArrayList<ArrayList<Double>>();

    @Override
    public synchronized ArrayList<ArrayList<Double>> getItems() {
        ArrayList<ArrayList<Double>> items_copy = new ArrayList<ArrayList<Double>>(mItems);
        mItems.clear(); 
        return (items_copy);
    }

    @Override
    public ArrayList<String> getItemNames() {
        return mColumns;
    }
    public void setHeaders(ArrayList<String> columns) {
        mColumns = columns;
        
    }
    public synchronized void addData(ArrayList<Double> items) {
        mItems.add(items); 

    }

}