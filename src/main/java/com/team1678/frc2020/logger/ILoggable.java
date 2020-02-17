package com.team1678.frc2020.logger;
import java.util.ArrayList;

/*  
    See:
         LogStorage for functionality
         TestLoggable for implementation
*/
 
 public interface ILoggable {
     ArrayList<ArrayList<Number>> getItems();
     ArrayList<String> getItemNames();
 }
