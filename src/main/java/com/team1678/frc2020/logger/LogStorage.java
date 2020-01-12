package com.team1678.frc2020.logger;

import java.util.ArrayList;

public class LogStorage implements ILoggable {
    ArrayList<String> mColumns = new ArrayList<String>();
    ArrayList<ArrayList<Double>> mItems = new ArrayList<ArrayList<Double>>();

    @Override
    public ArrayList<ArrayList<Double>> get_items() {
        ArrayList<ArrayList<Double>> items_copy = new ArrayList<ArrayList<Double>>(mItems);
        mItems.clear(); 
        return (items_copy);
    }

    @Override
    public ArrayList<String> get_item_names() {
        return mColumns;
    }
    public void setHeaders(ArrayList<String> column) {
        mColumns = column;
        
    }
    public void addData(ArrayList<ArrayList<Double>> items) {
        mItems = items; 

    }

}