package com.team1678.frc2020.controlboard;

public interface IControlBoard extends IDriveControlBoard, IButtonControlBoard {
    boolean getRunIntake();

	boolean getRunOuttake();
}