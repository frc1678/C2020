package com.team1323.lib.util;

import java.util.ArrayList;
import java.util.List;

import com.team1678.frc2020.Constants;
import com.team1323.lib.geometry.UnwrappablePose2d;
import com.team1323.lib.geometry.UnwrappableRotation2d;
import com.team1323.lib.geometry.UnwrappableTranslation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveInverseKinematics {
	
	public SwerveInverseKinematics(){
		setCenterOfRotation(new UnwrappableTranslation2d());
	}
	
	private final int kNumberOfModules = 4;
	
	private List<UnwrappableTranslation2d> moduleRelativePositions = Constants.kModulePositions;
	private List<UnwrappableTranslation2d> moduleRotationDirections = updateRotationDirections();
			
	private List<UnwrappableTranslation2d> updateRotationDirections(){
		List<UnwrappableTranslation2d> directions = new ArrayList<>(kNumberOfModules);
		for(int i = 0; i < kNumberOfModules; i++){
			directions.add(moduleRelativePositions.get(i).rotateBy(UnwrappableRotation2d.fromDegrees(90)));
		}
		return directions;
	}
	
	public void setCenterOfRotation(UnwrappableTranslation2d center){
		List<UnwrappableTranslation2d> positions = new ArrayList<>(kNumberOfModules);
		double maxMagnitude = 0.0;
		for(int i = 0; i < kNumberOfModules; i++){
			UnwrappableTranslation2d position = Constants.kModulePositions.get(i).translateBy(center.inverse());
			positions.add(position);
			double magnitude = position.norm();
			if(magnitude > maxMagnitude){
				maxMagnitude = magnitude;
			}
		}
		for(int i = 0; i < kNumberOfModules; i++){
			UnwrappableTranslation2d position = positions.get(i);
			positions.set(i, position.scale(1.0/maxMagnitude));
		}
		moduleRelativePositions = positions;
		moduleRotationDirections = updateRotationDirections();
	}
	
	public List<UnwrappableTranslation2d> updateDriveVectors(UnwrappableTranslation2d translationalVector, double rotationalMagnitude, 
	UnwrappablePose2d robotPose, boolean robotCentric){
		SmartDashboard.putNumber("Vector Direction", translationalVector.direction().getDegrees());
		//SmartDashboard.putNumber("Vector Magnitude", translationalVector.norm());
		SmartDashboard.putNumber("Robot Velocity", translationalVector.norm());
		
		if(!robotCentric)
			translationalVector = translationalVector.rotateBy(robotPose.getRotation().inverse());
		List<UnwrappableTranslation2d> driveVectors = new ArrayList<>(kNumberOfModules);
		for(int i = 0; i < kNumberOfModules; i++){
			driveVectors.add(translationalVector.translateBy(moduleRotationDirections.get(i).scale(rotationalMagnitude)));
		}
		double maxMagnitude = 1.0;
		for(UnwrappableTranslation2d t : driveVectors){
			double magnitude = t.norm();
			if(magnitude > maxMagnitude){
				maxMagnitude = magnitude;
			}
		}
		for(int i = 0; i < kNumberOfModules; i++){
			UnwrappableTranslation2d driveVector = driveVectors.get(i);
			driveVectors.set(i, driveVector.scale(1.0/maxMagnitude));
		}
		return driveVectors;
	}
	
}
