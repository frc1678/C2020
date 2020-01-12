package com.team1678.frc2020.logger;

import java.util.ArrayList;

public class LogStorage implements ILoggable {
    ArrayList<String> mColumns = new ArrayList<String>();
    ArrayList<Double> mValues = new ArrayList<Double>();  

    @Override
    public ArrayList<ArrayList<Double>> get_items() {
        return null;
    }

    @Override
    public ArrayList<String> get_item_names() {
        return null;
    }
    public void setHeaders(ArrayList<String> column) {
        mColumns = column;
        
    }
    public void addData(ArrayList<Double> values) {
        mValues = values; 

    }

}