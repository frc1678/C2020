package com.team1678.frc2020.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.planners.DriveMotionPlanner;
import com.team1678.frc2020.RobotState;
import com.team1678.frc2020.Settings;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1323.lib.math.vectors.VectorField;
import com.team1323.lib.util.SwerveHeadingController;
import com.team1323.lib.util.SwerveInverseKinematics;
import com.team1323.lib.util.SynchronousPIDF;
import com.team1323.lib.util.Util;
import com.team1323.lib.util.VisionCriteria;
import com.team1323.lib.vision.ShooterAimingParameters;
import com.team1323.lib.geometry.UnwrappablePose2d;
import com.team1323.lib.geometry.UnwrappablePose2dWithCurvature;
import com.team1323.lib.geometry.UnwrappableRotation2d;
import com.team1323.lib.geometry.UnwrappableTranslation2d;
import com.team1323.lib.trajectory.TimedView;
import com.team1323.lib.trajectory.Trajectory;
import com.team1323.lib.trajectory.TrajectoryIterator;
import com.team1323.lib.trajectory.timing.TimedState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.vision.AimingParameters;
import com.wpilib.SwerveDriveKinematics;
import com.wpilib.SwerveDriveOdometry;
import com.wpilib.SwerveModuleState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem{
	//Instance declaration
	private static Swerve instance = null;
	public static Swerve getInstance(){
		if(instance == null)
			instance = new Swerve();
		return instance;
	}
	
	//Module declaration
	public SwerveDriveModule frontRight, frontLeft, rearLeft, rearRight;
	List<SwerveDriveModule> modules;
	List<SwerveDriveModule> positionModules;
	
	//Evade maneuver variables
	UnwrappableTranslation2d clockwiseCenter = new UnwrappableTranslation2d();
	UnwrappableTranslation2d counterClockwiseCenter = new UnwrappableTranslation2d();
	boolean evading = false;
	boolean evadingToggled = false;
	public void toggleEvade(){
		evading = !evading;
		evadingToggled = true;
	}
	
	//Heading controller methods
	Pigeon pigeon;
	SwerveHeadingController headingController = new SwerveHeadingController();
	public void temporarilyDisableHeadingController(){
		headingController.temporarilyDisable();
	}
	public double getTargetHeading(){
		return headingController.getTargetHeading();
	}

	//Vision dependencies
	RobotState robotState;
	UnwrappableRotation2d visionTargetHeading = new UnwrappableRotation2d();
	boolean visionUpdatesAllowed = true;
	public void resetVisionUpdates(){
		visionUpdatesAllowed = true;
		visionUpdateCount = 0;
		attemptedVisionUpdates = 0;
		visionVisibleCycles = 0;
		firstVisionCyclePassed = false;
		visionCriteria.reset();
	}
	UnwrappableTranslation2d visionTargetPosition = new UnwrappableTranslation2d();
	public UnwrappableTranslation2d getVisionTargetPosition(){ return visionTargetPosition; }
	int visionUpdateCount = 0;
	int attemptedVisionUpdates = 0;
	int visionVisibleCycles = 0;
	boolean firstVisionCyclePassed = false;
	VisionCriteria visionCriteria = new VisionCriteria();
	double initialVisionDistance = 0.0;
	ShooterAimingParameters latestAim = new ShooterAimingParameters(100.0, new Rotation2d(), 0.0, 0.0);
	UnwrappableTranslation2d latestTargetPosition = new UnwrappableTranslation2d();
	UnwrappableTranslation2d lastVisionEndTranslation = new UnwrappableTranslation2d(-Constants.kRobotProbeExtrusion, 0.0);
	boolean useFixedVisionOrientation = false;
	UnwrappableRotation2d fixedVisionOrientation = UnwrappableRotation2d.fromDegrees(180.0);
	double visionCutoffDistance = Constants.kClosestVisionDistance;

	SynchronousPIDF lateralPID = new SynchronousPIDF(0.05, 0.0, 0.0);
	SynchronousPIDF forwardPID = new SynchronousPIDF(0.02, 0.0, 0.0);
	boolean visionTargetAcquired = false;

	boolean needsToNotifyDrivers = false;
	public boolean needsToNotifyDrivers(){
		if(needsToNotifyDrivers){
			needsToNotifyDrivers = false;
			return true;
		}		
		return false;
	}

	//Name says it all
	TrajectoryGenerator generator;

	//Odometry variables
	UnwrappablePose2d pose;
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;
	public UnwrappablePose2d getPose(){
		return pose;
	}

	//Wpilib odometry
	SwerveDriveOdometry odometry;
	UnwrappablePose2d wpiPose = new UnwrappablePose2d();
	boolean outputWpiPose = false;

	// Module configuration variables (for beginnning of auto)
	boolean modulesReady = false;
	boolean alwaysConfigureModules = false;
	boolean moduleConfigRequested = false;
	public void requireModuleConfiguration(){
		modulesReady = false;
	}
	public void alwaysConfigureModules(){
		alwaysConfigureModules = true;
	}
	UnwrappablePose2d startingPose = Constants.kRobotStartingPose.unwrap();
	public void setStartingPose(UnwrappablePose2d newPose){
		startingPose = newPose;
	}

	//Trajectory variables
	DriveMotionPlanner motionPlanner;
	public double getRemainingProgress(){
		if(motionPlanner != null && getState() == ControlState.TRAJECTORY){
			return motionPlanner.getRemainingProgress();
		}
		return 0.0;
	}
	double rotationScalar;
	double trajectoryStartTime = 0;
	UnwrappableTranslation2d lastTrajectoryVector = new UnwrappableTranslation2d();
	public UnwrappableTranslation2d getLastTrajectoryVector(){ return lastTrajectoryVector; }
	boolean hasStartedFollowing = false;
	boolean hasFinishedPath = false;
	public boolean hasFinishedPath(){
		return hasFinishedPath;
	}
	
	//Experimental
	VectorField vf;
	
	private Swerve(){
		frontRight = new SwerveDriveModule(Constants.FRONT_RIGHT_ROTATION, Constants.FRONT_RIGHT_DRIVE,
				0, Constants.kFrontRightEncoderStartingPos, Constants.kVehicleToModuleZero.unwrap());
		frontLeft = new SwerveDriveModule(Constants.FRONT_LEFT_ROTATION, Constants.FRONT_LEFT_DRIVE,
				1, Constants.kFrontLeftEncoderStartingPos, Constants.kVehicleToModuleOne.unwrap());
		rearLeft = new SwerveDriveModule(Constants.REAR_LEFT_ROTATION, Constants.REAR_LEFT_DRIVE,
				2, Constants.kRearLeftEncoderStartingPos, Constants.kVehicleToModuleTwo.unwrap());
		rearRight = new SwerveDriveModule(Constants.REAR_RIGHT_ROTATION, Constants.REAR_RIGHT_DRIVE,
				3, Constants.kRearRightEncoderStartingPos, Constants.kVehicleToModuleThree.unwrap());
		
		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		positionModules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		
		//rearLeft.disableDriveEncoder();
		
		rearLeft.invertDriveMotor(false);
		frontLeft.invertDriveMotor(false);
		
		modules.forEach((m) -> m.reverseRotationSensor(true));
				
		pigeon = Pigeon.getInstance();
		
		pose = new UnwrappablePose2d();
		distanceTraveled = 0;
		
		motionPlanner = new DriveMotionPlanner();

		robotState = RobotState.getInstance();

		odometry = new SwerveDriveOdometry(new SwerveDriveKinematics(Constants.kVehicleToModuleZero.unwrap(), 
Constants.kVehicleToModuleOne.unwrap(), Constants.kVehicleToModuleTwo.unwrap(), Constants.kVehicleToModuleThree.unwrap()), 
			UnwrappableRotation2d.identity());

		generator = TrajectoryGenerator.getInstance();

		lateralPID.setSetpoint(0.0);
		forwardPID.setSetpoint(0.0);
	}

	//Assigns appropriate directions for scrub factors
	public void setCarpetDirection(boolean standardDirection){
		modules.forEach((m) -> m.setCarpetDirection(standardDirection));
	}
	
	//Teleop driving variables
	private UnwrappableTranslation2d translationalVector = new UnwrappableTranslation2d();
	private double rotationalInput = 0;
	private UnwrappableTranslation2d lastDriveVector = new UnwrappableTranslation2d();
	private final UnwrappableTranslation2d rotationalVector = UnwrappableTranslation2d.identity();
	private double lowPowerScalar = 0.6;
	public void setLowPowerScalar(double scalar){
		lowPowerScalar = scalar;
	}
	private double maxSpeedFactor = 1.0;
	public void setMaxSpeed(double max){
		maxSpeedFactor = max;
	}
	private boolean robotCentric = false;
	
	//Swerve kinematics (exists in a separate class)
	private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
	public void setCenterOfRotation(UnwrappableTranslation2d center){
		inverseKinematics.setCenterOfRotation(center);
	}
	
	//The swerve's various control states
	public enum ControlState{
		NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
		TRAJECTORY, VELOCITY
	}
	private ControlState currentState = ControlState.NEUTRAL;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	
	/**
	 * Main function used to send manual input during teleop.
	 * @param x forward/backward input
	 * @param y left/right input
	 * @param rotate rotational input
	 * @param robotCentric gyro use
	 * @param lowPower scaled down output
	 */
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
		UnwrappableTranslation2d translationalInput = new UnwrappableTranslation2d(x, y);
		double inputMagnitude = translationalInput.norm();
		
		/* Snap the translational input to its nearest pole, if it is within a certain threshold 
		  of it. */
		double threshold = Math.toRadians(10.0);
		if(Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold){
			translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
		}
		
		/* Scale x and y by applying a power to the magnitude of the vector they create, in order
		 to make the controls less sensitive at the lower end. */
		double deadband = 0.25;
		inputMagnitude = Util.scaledDeadband(inputMagnitude, 1.0, deadband);
		final double power = (lowPower) ? 1.75 : 1.5;
		inputMagnitude = Math.pow(inputMagnitude, power);
		translationalInput = UnwrappableTranslation2d.fromPolar(translationalInput.direction(), inputMagnitude);
		
		rotate = Util.scaledDeadband(rotate, 1.0, deadband);
		rotate = Math.pow(Math.abs(rotate), 1.75)*Math.signum(rotate);
		
		translationalInput = translationalInput.scale(maxSpeedFactor);
		rotate *= maxSpeedFactor;
				
		translationalVector = translationalInput;
		
		if(lowPower){
			translationalVector = translationalVector.scale(lowPowerScalar);
			rotate *= lowPowerScalar;
		}else{
			rotate *= 0.8;
		}
		
		if(rotate != 0 && rotationalInput == 0){
			headingController.disable();
		}else if(rotate == 0 && rotationalInput != 0){
			headingController.temporarilyDisable();
		}
		
		rotationalInput = rotate;

		if(translationalInput.norm() != 0){
			if(Superstructure.getInstance().isAimed()) {
				if(Math.abs(translationalInput.direction().distance(visionTargetHeading)) > Math.toRadians(90.0)){
					setState(ControlState.MANUAL);
				}
			}else if(currentState != ControlState.MANUAL){
				setState(ControlState.MANUAL);
			}
		}else if(rotationalInput != 0){
			if(currentState != ControlState.MANUAL && currentState != ControlState.TRAJECTORY){
				setState(ControlState.MANUAL);
			}
		}

		if(inputMagnitude > 0.3)
			lastDriveVector = new UnwrappableTranslation2d(x, y);
		else if(translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0){
			lastDriveVector = rotationalVector;
		}
		
		this.robotCentric = robotCentric;
	}

	//Possible new control method for rotation
	public UnwrappableRotation2d averagedDirection = UnwrappableRotation2d.identity();
	public void resetAveragedDirection(){ averagedDirection = pose.getRotation(); }
	public void setAveragedDirection(double degrees){ averagedDirection = UnwrappableRotation2d.fromDegrees(degrees); }
	public final double rotationDirectionThreshold = Math.toRadians(5.0);
	public final double rotationDivision = 1.0;
	public synchronized void updateControllerDirection(UnwrappableTranslation2d input){
		if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
			UnwrappableRotation2d direction = input.direction();
			double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
			averagedDirection = UnwrappableRotation2d.fromDegrees(roundedDirection);
		}
	}
	
	//Various methods to control the heading controller
	public synchronized void rotate(double goalHeading){
		if(translationalVector.x() == 0 && translationalVector.y() == 0)
			rotateInPlace(goalHeading);
		else
			headingController.setStabilizationTarget(
					Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlace(double goalHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(
				Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlaceAbsolutely(double absoluteHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(absoluteHeading);
	}
	
	public void setPathHeading(double goalHeading){
		headingController.setSnapTarget(
				Util.placeInAppropriate0To360Scope(
						pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void setAbsolutePathHeading(double absoluteHeading){
		headingController.setSnapTarget(absoluteHeading);
	}
	
	/** Sets MotionMagic targets for the drive motors */
	public void setPositionTarget(double directionDegrees, double magnitudeInches){
		setState(ControlState.POSITION);
		modules.forEach((m) -> m.setModuleAngle(directionDegrees));
		modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
	}

	/** Locks drive motors in place with MotionMagic */
	public void lockDrivePosition(){
		modules.forEach((m) -> m.setDrivePositionTarget(0.0));
	}

	/** Puts drive motors into closed-loop velocity mode */
	public void setVelocity(UnwrappableRotation2d direction, double velocityInchesPerSecond){
		setState(ControlState.VELOCITY);
		modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
		modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
	}
	
	/** Configures each module to match its assigned vector */
	public void setDriveOutput(List<UnwrappableTranslation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
    		}
    	}
	}

	public void setDriveOutput(List<UnwrappableTranslation2d> driveVectors, double percentOutputOverride){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setDriveOpenLoop(-percentOutputOverride);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setDriveOpenLoop(percentOutputOverride);
    		}
    	}
	}


	/** Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode */
	public void setVelocityDriveOutput(List<UnwrappableTranslation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setVelocitySetpoint(-driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setVelocitySetpoint(driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
    		}
    	}
	}

	public void setVelocityDriveOutput(List<UnwrappableTranslation2d> driveVectors, double velocityOverride){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    			modules.get(i).setVelocitySetpoint(-velocityOverride);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    			modules.get(i).setVelocitySetpoint(velocityOverride);
    		}
    	}
	}

	/** Sets only module angles to match their assigned vectors */
	public void setModuleAngles(List<UnwrappableTranslation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
    		if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
    		}else{
    			modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    		}
    	}
	}

	/** Increases each module's rotational power cap for the beginning of auto */
	public void set10VoltRotationMode(boolean tenVolts){
		modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
	}
	
	/**
	 * @return Whether or not at least one module has reached its MotionMagic setpoint
	 */
	public boolean positionOnTarget(){
		boolean onTarget = false;
		for(SwerveDriveModule m : modules){
			onTarget |= m.drivePositionOnTarget();
		}
		return onTarget;
	}
	
	/**
	 * @return Whether or not all modules have reached their angle setpoints
	 */
	public boolean moduleAnglesOnTarget(){
		boolean onTarget = true;
		for(SwerveDriveModule m : modules){
			onTarget &= m.angleOnTarget();
		}
		return onTarget;
	}

	/**
	 * Sets a trajectory for the robot to follow
	 * @param trajectory 
	 * @param targetHeading Heading that the robot will rotate to during its path following
	 * @param rotationScalar Scalar to increase or decrease the robot's rotation speed
	 * @param followingCenter The point (relative to the robot) that will follow the trajectory
	 */
	public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
		double rotationScalar, UnwrappableTranslation2d followingCenter){
			hasStartedFollowing = false;
			hasFinishedPath = false;
			moduleConfigRequested = false;
			motionPlanner.reset();
			motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
			motionPlanner.setFollowingCenter(followingCenter);
			inverseKinematics.setCenterOfRotation(followingCenter);
			setAbsolutePathHeading(targetHeading);
			this.rotationScalar = rotationScalar;
			trajectoryStartTime = Timer.getFPGATimestamp();
			setState(ControlState.TRAJECTORY);
		}
	
	public synchronized void setTrajectory(Trajectory<TimedState<UnwrappablePose2dWithCurvature>> trajectory, double targetHeading,
			double rotationScalar){
		setTrajectory(trajectory, targetHeading, rotationScalar, UnwrappableTranslation2d.identity());
	}

	public synchronized void setRobotCentricTrajectory(UnwrappableTranslation2d relativeEndPos, double targetHeading){
		setRobotCentricTrajectory(relativeEndPos, targetHeading, 45.0);
	}

	public synchronized void setRobotCentricTrajectory(UnwrappableTranslation2d relativeEndPos, double targetHeading, double defaultVel){
		modulesReady = true;
		UnwrappableTranslation2d endPos = pose.transformBy(UnwrappablePose2d.fromTranslation(relativeEndPos)).getTranslation();
		UnwrappableRotation2d startHeading = endPos.translateBy(pose.getTranslation().inverse()).direction();
		List<UnwrappablePose2d> waypoints = new ArrayList<>();
		waypoints.add(new UnwrappablePose2d(pose.getTranslation(), startHeading));	
		waypoints.add(new UnwrappablePose2d(pose.transformBy(UnwrappablePose2d.fromTranslation(relativeEndPos)).getTranslation(), startHeading));
		Trajectory<TimedState<UnwrappablePose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
		double heading = Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), targetHeading);
		setTrajectory(trajectory, heading, 1.0);
	}


	/****************************************************/
	/* Vector Fields */
	public synchronized void setVectorField(VectorField vf_) {
		vf = vf_;
		setState(ControlState.VECTORIZED);
	}
	
	/** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
	public synchronized void determineEvasionWheels(){
		UnwrappableTranslation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
		List<UnwrappableTranslation2d> wheels = Constants.kModulePositions.stream().map(Translation2d::unwrap).collect(Collectors.toList());;
		clockwiseCenter = wheels.get(0);
		counterClockwiseCenter = wheels.get(wheels.size()-1);
		for(int i = 0; i < wheels.size()-1; i++) {
			UnwrappableTranslation2d cw = wheels.get(i);
			UnwrappableTranslation2d ccw = wheels.get(i+1);
			if(here.isWithinAngle(cw,ccw)) {
				clockwiseCenter = ccw;
				counterClockwiseCenter = cw;
			}
		}
	}
	
	/** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		UnwrappableRotation2d heading = pigeon.getYaw().unwrap();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveDriveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = 100.0;
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for(SwerveDriveModule m : positionModules){
				double deviance = Math.abs(distances[m.moduleID] - averageDistance);
				if(deviance < minDeviance){
					minDeviance = deviance;
					minDevianceIndex = m.moduleID;
				}
				if(deviance <= 0.01){
					modulesToUse.add(m);
				}
			}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		
		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		UnwrappablePose2d updatedPose = new UnwrappablePose2d(new UnwrappableTranslation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	/** Playing around with different methods of odometry. This will require the use of all four modules, however. */
	public synchronized void alternatePoseUpdate(){
		double x = 0.0;
		double y = 0.0;
		UnwrappableRotation2d heading = pigeon.getYaw().unwrap();
		
		double[][] distances = new double[4][2];
		for(SwerveDriveModule m : modules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
			distances[m.moduleID][0] = m.moduleID;
			distances[m.moduleID][1] = distance;
		}
		
		Arrays.sort(distances, new java.util.Comparator<double[]>() {
			public int compare(double[] a, double[] b) {
				return Double.compare(a[1], b[1]);
			}
		});
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		double firstDifference = distances[1][1] - distances[0][1];
		double secondDifference = distances[2][1] - distances[1][1];
		double thirdDifference = distances[3][1] - distances[2][1];
		if(secondDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
		}else if(thirdDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
		}else{
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
			modulesToUse.add(modules.get((int)distances[3][0]));
		}
		
		SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}

		UnwrappablePose2d updatedPose = new UnwrappablePose2d(new UnwrappableTranslation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
		distanceTraveled += deltaPos;
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}

	/** Called every cycle to update the swerve based on its control state */
	public synchronized void updateControlCycle(double timestamp){
		double rotationCorrection = headingController.updateRotationCorrection(pose.getRotation().getUnboundedDegrees(), timestamp);

		switch(currentState){
		case MANUAL:
			if(evading && evadingToggled){
				determineEvasionWheels();
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
				evadingToggled = false;
			}else if(evading){
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
			}else if(evadingToggled){
				inverseKinematics.setCenterOfRotation(UnwrappableTranslation2d.identity());
				evadingToggled = false;
			}
			if(translationalVector.equals(UnwrappableTranslation2d.identity()) && rotationalInput == 0.0){
				if(lastDriveVector.equals(rotationalVector)){
					stop();
				}else{
					setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector,
					rotationCorrection, pose, robotCentric), 0.0);
				}
			}else{
				setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
						rotationalInput + rotationCorrection, pose, robotCentric));
			}
			break;
		case POSITION:
			if(positionOnTarget())
				rotate(headingController.getTargetHeading());
			break;
		case ROTATION:
			setDriveOutput(inverseKinematics.updateDriveVectors(new UnwrappableTranslation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
			break;
		case VECTORIZED:
			Translation2d outputVectorV = vf.getVector(pose.wrap().getTranslation()).scale(0.25);
			SmartDashboard.putNumber("Vector Direction", outputVectorV.direction().getDegrees());
			SmartDashboard.putNumber("Vector Magnitude", outputVectorV.norm());
//			System.out.println(outputVector.x()+" "+outputVector.y());
			setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV.unwrap(), rotationCorrection, getPose(), false));
			break;
		case TRAJECTORY:
			if(!motionPlanner.isDone()){
				UnwrappableTranslation2d driveVector = motionPlanner.update(timestamp, pose);

				if(modulesReady){
					if(!hasStartedFollowing){
						if(moduleConfigRequested){
							zeroSensors(startingPose);
							System.out.println("Position reset for auto");
						}
						hasStartedFollowing = true;
					}
					double rotationInput = Util.deadBand(Util.limit(rotationCorrection*rotationScalar*driveVector.norm(), motionPlanner.getMaxRotationSpeed()), 0.01);
					if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
						driveVector = lastTrajectoryVector;
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
							rotationInput, pose, false), 0.0);
						//System.out.println("Trajectory Vector set: " + driveVector.toString());
					}else{
						setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
							rotationInput, pose, false));
						//System.out.println("Trajectory Vector set: " + driveVector.toString());
					}
				}else if(!moduleConfigRequested){
					//set10VoltRotationMode(true);
					setModuleAngles(inverseKinematics.updateDriveVectors(driveVector, 
						0.0, pose, false));
					moduleConfigRequested = true;
				}

				if(moduleAnglesOnTarget() && !modulesReady){
					set10VoltRotationMode(false);
					modules.forEach((m) -> m.resetLastEncoderReading());
					modulesReady = true;
					System.out.println("Modules Ready");
				}
				
				lastTrajectoryVector = driveVector;
			}else{
				if(!hasFinishedPath){ 
					System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
					hasFinishedPath = true;
					if(alwaysConfigureModules) requireModuleConfiguration();
				}
			}
			break;
		case VELOCITY:

			break;
		case NEUTRAL:
			stop();
			break;
		case DISABLED:
			
			break;
		default:
			break;
		}
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			synchronized(Swerve.this){
				translationalVector = new UnwrappableTranslation2d();
				lastDriveVector = rotationalVector;
				rotationalInput = 0;
				resetAveragedDirection();
				headingController.temporarilyDisable();
				stop();
				outputWpiPose = true;
				lastUpdateTimestamp = timestamp;
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Swerve.this){
				if(modulesReady || (getState() != ControlState.TRAJECTORY)){
					//updatePose(timestamp);
					//alternatePoseUpdate();
					pose = odometry.update(pigeon.getYaw().unwrap(), getModuleStates());
				}
				updateControlCycle(timestamp);
				lastUpdateTimestamp = timestamp;
			}
		}

		@Override
		public void onStop(double timestamp) {
			synchronized(Swerve.this){
				translationalVector = new UnwrappableTranslation2d();
				rotationalInput = 0;
				outputWpiPose = false;
				stop();
			}
		}
		
	};
	public void setNominalDriveOutput(double voltage){
		modules.forEach((m) -> m.setNominalDriveOutput(voltage));
	}
	
	/** Sets the maximum rotation speed opf the modules, based on the robot's velocity */
	public void setMaxRotationSpeed(){
		double currentDriveSpeed = translationalVector.norm() * Constants.kSwerveMaxSpeedInchesPerSecond;
		double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed / 
				((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
		modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.size()]; 
		for(int i = 0; i < modules.size(); i++) {
			states[i] = modules.get(i).getState();
		}
		return states;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		modules.forEach((m) -> m.readPeriodicInputs());
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	/** Puts all rotation and drive motors into open-loop mode */
	public synchronized void disable(){
		modules.forEach((m) -> m.disable());
		setState(ControlState.DISABLED);
	}

	@Override
	public synchronized void stop() {
		setState(ControlState.NEUTRAL);
		modules.forEach((m) -> m.stop());
	}

	@Override
	public synchronized void zeroSensors() {
		zeroSensors(Constants.kRobotStartingPose.unwrap());
	}
	
	/** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
	public synchronized void zeroSensors(UnwrappablePose2d startingPose){
		pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
		modules.forEach((m) -> m.zeroSensors(startingPose));
		pose = startingPose;
		odometry.resetPosition(startingPose, startingPose.getRotation());
		distanceTraveled = 0;
	}
	
	public synchronized void resetPosition(UnwrappablePose2d newPose){
		pose = new UnwrappablePose2d(newPose.getTranslation(), pose.getRotation());
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(newPose, pose.getRotation());
		distanceTraveled = 0;
	}
	
	public synchronized void setXCoordinate(double x){
		pose.getTranslation().setX(x);
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(pose, pose.getRotation());
		System.out.println("X coordinate reset to: " + pose.getTranslation().x());
	}
	
	public synchronized void setYCoordinate(double y){
		pose.getTranslation().setY(y);
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(pose, pose.getRotation());
		System.out.println("Y coordinate reset to: " + pose.getTranslation().y());
	}

	@Override
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
		// testing the wpi odometry
		if(outputWpiPose)
			SmartDashboard.putNumberArray("Path Pose", new double[]{wpiPose.getTranslation().x(), wpiPose.getTranslation().y(), wpiPose.getRotation().getUnboundedDegrees()});
		if(Settings.debugSwerve()){
			SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
			SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
			SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees());
			SmartDashboard.putString("Heading Controller", headingController.getState().toString());
			SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
			SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
			SmartDashboard.putNumber("Robot Velocity", currentVelocity);
			SmartDashboard.putString("Swerve State", currentState.toString());
			SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
			SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());
		}
	}

	@Override
	public boolean checkSystem() {
		// TODO Auto-generated method stub
		return false;
	}
}
