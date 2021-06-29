/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import static frc.robot.simulation.SimConstants.blueGoalPose;
import static frc.robot.simulation.SimConstants.redGoalPose;

/*
Subsystem for interacting with the Limelight and OpenSight vision systems
 */

public class Vision extends SubsystemBase {
    // Variables for calculating distance
    private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
    private final double LIMELIGHT_MOUNT_ANGLE = 32; // Angle that the Limelight is mounted at
    private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

    private final double MIN_TARGET_DISTANCE = 1;
    private final double INNER_PORT_SLOPE = 1;
    private final double INNER_PORT_OFFSET = 1;

    private final double HORIZONTAL_TARGET_PIXEL_WIDTH = 1;
    private final double HORIZONTAL_TARGET_PIXEL_THRESHOLD = 1;
    private final double VERTICAL_TARGET_PIXEL_WIDTH = 1;
    private final double VERTICAL_TARGET_PIXEL_THRESHOLD = 1;

    // NetworkTables for reading vision data
    PhotonCamera photonCamera;
    SimVisionSystem simPhotonCamera;

    // Subsystems that will be controlled based on vision data
    private final DriveTrain m_driveTrain;
    private final Turret m_turret;
    double[] distances = new double[5];
    double[] counts = new double[5];
    int index = 0;
    // Filters to prevent target values from oscillating too much
    SlewRateLimiter targetXFilter = new SlewRateLimiter(20);
    SlewRateLimiter innerTargetXFilter = new SlewRateLimiter(20);
    UsbCamera camera;
    private boolean resetPose;
    private double lastValidTargetTime;
    private boolean validTarget;

    public Vision(DriveTrain driveTrain, Turret turret) {
        m_driveTrain = driveTrain;
        m_turret = turret;

		if(RobotBase.isReal()) {
//		camera = CameraServer.getInstance().startAutomaticCapture();
			camera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/video0");
			camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
			camera.setExposureManual(25);
			camera.setResolution(320, 240);
			camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
		}

//        PortForwarder.add(5800, "10.42.1.11", 5800);
//        PortForwarder.add(5801, "10.42.1.11", 5801);
//        PortForwarder.add(5805, "10.42.1.11", 5805);

        // Init vision NetworkTables
        photonCamera = new PhotonCamera("photonvision");


        double TARGET_HEIGHT_METERS = Units.inchesToMeters(81.19);
        double TARGET_WIDTH = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
        double TARGET_HEIGHT = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

        simPhotonCamera = new SimVisionSystem(
                "photonvision",
                75.76079874010732,
                30,
                new Transform2d(),
                Units.feetToMeters(2),
                Units.feetToMeters(25),
                320,
                240,
                10);
        simPhotonCamera.addSimVisionTarget(
                new SimVisionTarget(redGoalPose,
                        TARGET_HEIGHT_METERS,
                        TARGET_WIDTH,
                        TARGET_HEIGHT)
        );
        simPhotonCamera.addSimVisionTarget(
                new SimVisionTarget(blueGoalPose,
                        TARGET_HEIGHT_METERS,
                        TARGET_WIDTH,
                        TARGET_HEIGHT)
        );
        setPipeline(0);

        //initShuffleboard();
    }

    private void updateValidTarget() {
        // Determine whether the limelight has detected a valid target and not a random reflection
        // If the target is seen for a specific amount of time it is marked as valid
        if(hasTarget()) {
            setLastValidTargetTime();
        }
        if((Timer.getFPGATimestamp() - lastValidTargetTime) < 3) {
            ledsOn();
            validTarget = true;
        } else {
            ledsOff();
            validTarget = false;
        }
    }

    public boolean getValidTarget() {
        return validTarget;
    }

    public void setLastValidTargetTime() {
        lastValidTargetTime = Timer.getFPGATimestamp();
    }

    // Limelight interaction functions
    public double getTargetY() {
        if(hasTarget())
            return photonCamera.getLatestResult().getBestTarget().getYaw();
        else
            return 0;
    }

    public double getTargetX() {
        if(hasTarget())
            return photonCamera.getLatestResult().getBestTarget().getPitch();
        else
            return 0;
    }

    public double getFilteredTargetX() {
        return targetXFilter.calculate(getTargetX());
    }

    public double getSmartTargetX() {
        if(getTargetDistance() > MIN_TARGET_DISTANCE) {
            double xDistance = Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getX());
            double yDistance = Math.abs(Units.metersToFeet(m_driveTrain.getRobotPose().getTranslation().getY()));

            double maxYDistance = INNER_PORT_SLOPE * xDistance + INNER_PORT_OFFSET;

            if(yDistance < maxYDistance) {
                xDistance += 29.25 / 12.0;
                return innerTargetXFilter.calculate(Math.signum(getFilteredTargetX()) * Units.radiansToDegrees(Math.atan(xDistance / yDistance)));
            }
        }

        return getFilteredTargetX();
    }

    private void resetPoseByVision() {

    }

    // More Limelight interaction functions

    public boolean hasTarget() {
        return photonCamera.hasTargets();
    }

    public Transform2d getCameraToTarget() {
        if(photonCamera.hasTargets())
            return photonCamera.getLatestResult().getBestTarget().getCameraToTarget();
        else
            return new Transform2d();
    }

    public double getTargetArea() {
        return photonCamera.getLatestResult().getBestTarget().getArea();
    }

    public double getTargetSkew() {
        return photonCamera.getLatestResult().getBestTarget().getSkew();
    }

    public double getPipelineLatency() {
        return photonCamera.getLatestResult().getLatencyMillis();
    }

    public int getPipeline() {
        return photonCamera.getPipelineIndex();
    }

    public void setPipeline(int pipeline) {
        photonCamera.setPipelineIndex(pipeline);
    }

    public void ledsOn() {
        photonCamera.setLED(LEDMode.kOn);
    }
    public void ledsOff() {
        photonCamera.setLED(LEDMode.kOff);
    }

    // Calculate target distance based on field dimensions and the angle from the Limelight to the target
    public double getTargetDistance() {
        if (RobotBase.isReal()) {
            double angleToTarget = getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();

            double inches = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + angleToTarget));
            distances[index++ % distances.length] = inches / 12.0;
    
            return computeMode(distances);
        } else {
            return Units.metersToFeet(m_turret.getIdealTargetDistance());
        }
        
    }

	public double getAngleToTarget() {
        if (RobotBase.isReal()) {
            return getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();
        } else {
            return m_turret.getIdealTurretAngle();
        }
    }
    
    // For Shoot on the Move, gets horizontal angle on field to target
    public double getHorizontalAngleToTarget() {
        return getTargetX();
            // TODO: Figure out what to add/subtract if we're zoomed in
    }

    // Used to find the most common value to provide accurate target data
    private double computeMode(double[] data) {
        // Compute mode
        this.counts = new double[data.length];
        for(int i = 0; i < data.length; i++) {
            for(double datum : data) {
                if(data[i] == datum) {
                    this.counts[i]++;
                }
            }
        }

        int highestIndex = 0;
        double previousHigh = 0;
        for(int i = 0; i < this.counts.length; i++) {
            if(this.counts[i] > previousHigh) {
                highestIndex = i;
                previousHigh = this.counts[i];
            }
        }

        return data[highestIndex]; // Final distance in feet
    }

    // Read ball position data from OpenSight (Raspberry Pi)
    public double getPowerCellX() {
        // TODO: Calculate degrees from pixels?
        // return openSight.getEntry("found-x").getDouble(0) * 5.839; // 5.839 pixels per degree
        return 0;
    }

    public boolean hasPowerCell() {
        return false;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this :: getValidTarget);
        Shuffleboard.getTab("Turret").addNumber("Vision Target X", this :: getFilteredTargetX);

    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight Target X", getTargetX());
        SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
        SmartDashboard.putNumber("Limelight Pipeline", getPipeline());

        SmartDashboardTab.putBoolean("Turret", "Vision Valid Output", getValidTarget());
        SmartDashboardTab.putNumber("Turret", "Vision Target X", getFilteredTargetX());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        updateValidTarget();

        //resetPoseByVision();
    }

    @Override
    public void simulationPeriodic() {
        simPhotonCamera.processFrame(m_driveTrain.getRobotPose());
    }
}
