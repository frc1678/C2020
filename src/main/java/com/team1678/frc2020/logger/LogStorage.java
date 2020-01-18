package com.team1678.frc2020.logger;

import java.util.ArrayList;

public class LogStorage implements ILoggable {
    ArrayList<String> mColumns = new ArrayList<String>();
    ArrayList<ArrayList<Double>> mItems = new ArrayList<ArrayList<Double>>();
    
    /*  Add functionality to contents of ILoggable.

        Get Item:
            Creates an Array List of Array Lists of doubles
            Transfer logged data values of original array list to a new array list
            Print old array list
            Array list is now cleared for further logging use

        Get Item Names:
            Make columns to store the item names so that log files are in a neat grid type thing

            | timestamp | current | demand |  
            |           |         |        |
            |           |         |        |    Logged data goes under the columns
            |           |         |        |
            |           |         |        |
    */
    @Override
    public synchronized ArrayList<ArrayList<Double>> getItems() {
        ArrayList<ArrayList<Double>> items_tmp = mItems;
        mItems = new ArrayList<ArrayList<Double>>();
        return items_tmp;
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
