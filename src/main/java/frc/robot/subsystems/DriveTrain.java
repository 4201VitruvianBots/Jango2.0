/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class
DriveTrain extends SubsystemBase {
    // Set up constants
    private double gearRatioLow = 1 / 14.14;
    private double gearRatioHigh = 1 / 7.49;
    private double wheelDiameter = 0.5;

    // ???
    private double kS = 0.19;
    private double kV = 2.23;
    private double kA = 0.0289;

    // ???
    public double kP = 1.33;
    public double kI = 0;
    public double kD = 0;

    public int controlMode = 0;

    // Set up motors
    private TalonFX[] driveMotors = {
            new TalonFX(Constants.leftFrontDriveMotor),
            new TalonFX(Constants.leftRearDriveMotor),
            new TalonFX(Constants.rightFrontDriveMotor),
            new TalonFX(Constants.rightRearDriveMotor)
    };

    private boolean[] brakeMode = {
            true,
            false,
            true,
            false
    };

    //Set up shifters
    DoubleSolenoid driveTrainShifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);
    private AHRS navX = new AHRS(SerialPort.Port.kMXP);

    // Set up spatial tools
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    DifferentialDriveOdometry odometry;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    // ???
    PIDController leftPIDController = new PIDController(kP, kI, kD);
    PIDController rightPIDController = new PIDController(kP, kI, kD);

    PowerDistributionPanel m_pdp;

    public DriveTrain(PowerDistributionPanel pdp) {
        // Set up DriveTrain motors
        for (TalonFX motor : driveMotors) {
            motor.configFactoryDefault();
//            motor.configVoltageCompSaturation(12);
//            motor.enableVoltageCompensation(true);
            // motor.configGetSupplyCurrentLimit(30);
            // motor.configPeakCurrentLimit(40);
            // motor.configPeakCurrentDuration(1000);
            // motor.enableCurrentLimit(true);
            motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            motor.configOpenloopRamp(0.1);
            motor.configClosedloopRamp(0.1);
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configForwardSoftLimitEnable(false);
            motor.configReverseSoftLimitEnable(false);
        }

        // Configure motors
        driveMotors[0].setInverted(true);
        driveMotors[1].setInverted(true);
        driveMotors[2].setInverted(false);
        driveMotors[3].setInverted(false);

        driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotors[0].setSensorPhase(false);
        driveMotors[2].setSensorPhase(false);

        driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
        driveMotors[1].setNeutralMode(NeutralMode.Brake);
        driveMotors[3].setNeutralMode(NeutralMode.Brake);

        driveMotors[1].configOpenloopRamp(0);
        driveMotors[3].configOpenloopRamp(0);

        m_pdp = pdp;
        //initShuffleboardValues();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    // Self-explanatory functions

    public int getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getAngle() {
        return navX.getAngle();
    }

    public double getHeading() {
        return Math.IEEEremainder(-navX.getAngle(), 360);
    }

    public void resetAngle() {
        navX.zeroYaw();
    }

    public double getWheelDistanceMeters(int sensorIndex) {
        double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;

        return (driveMotors[sensorIndex].getSelectedSensorPosition() / 2048.0) * gearRatio * Math.PI * Units.feetToMeters(wheelDiameter);
    }

    // ???
    public double getMotorInputCurrent(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

//    if(rightPWM > 1.0) {
//      leftPWM -= rightPWM - 1.0;
//      rightPWM = 1.0;
//    } else if(rightPWM < -1.0) {
//      leftPWM -= rightPWM + 1.0;
//      rightPWM = -1.0;
//    } else if(leftPWM > 1.0) {
//      rightPWM -= leftPWM - 1.0;
//      leftPWM = 1.0;
//    } else if(leftPWM < -1.0) {
//      rightPWM -= leftPWM + 1.0;
//      leftPWM = -1.0;
//    }

        // Normalization
        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
        if (magnitude > 1.0) {
            leftPWM *= 1.0 / magnitude;
            rightPWM *= 1.0 / magnitude;
        }

        setMotorPercentOutput(leftPWM, rightPWM);
    }

    public void setMotorTankDrive(double leftOutput, double rightOutput) {
        setMotorPercentOutput(leftOutput, rightOutput);
    }

    public void setVoltageOutput(double leftVoltage, double rightVoltage) {
        SmartDashboardTab.putNumber("DriveTrain", "Left Voltage", leftVoltage);
        SmartDashboardTab.putNumber("DriveTrain", "Right Voltage", rightVoltage);
        setMotorPercentOutput(leftVoltage / m_pdp.getVoltage(), rightVoltage / m_pdp.getVoltage());
    }

    private void setMotorPercentOutput(double leftOutput, double rightOutput) {
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
    }

    // Make motors neutral
    public void setDriveTrainNeutralMode(int mode) {
        switch (mode) {
            case 2:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Coast);
                for (var brakeMode : brakeMode)
                    brakeMode = false;
                break;
            case 1:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Brake);
                for (var brakeMode : brakeMode)
                    brakeMode = true;
                break;
            case 0:
            default:
                driveMotors[0].setNeutralMode(NeutralMode.Brake);
                driveMotors[1].setNeutralMode(NeutralMode.Coast);
                driveMotors[2].setNeutralMode(NeutralMode.Brake);
                driveMotors[3].setNeutralMode(NeutralMode.Coast);
                brakeMode[0] = true;
                brakeMode[1] = false;
                brakeMode[2] = true;
                brakeMode[3] = false;
                break;
        }
    }

    public boolean getDriveShifterStatus() {
        return (driveTrainShifters.get() == DoubleSolenoid.Value.kForward) ? true : false;
    }

    public void setDriveShifterStatus(boolean state) {
        driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;

        double leftMetersPerSecond = (driveMotors[0].getSelectedSensorVelocity() * 10.0 / 2048) * gearRatio * Math.PI * Units.feetToMeters(wheelDiameter);
        double rightMetersPerSecond = (driveMotors[2].getSelectedSensorVelocity() * 10.0 / 2048) * gearRatio * Math.PI * Units.feetToMeters(wheelDiameter);

        // getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveKinematics getDriveTrainKinematics() {
        return kinematics;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        odometry.resetPosition(pose, rotation);
    }

    private void initShuffleboardValues() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Drive Train").addNumber("Left Encoder", () -> getEncoderCount(0));
        Shuffleboard.getTab("Drive Train").addNumber("Right Encoder", () -> getEncoderCount(2));
        Shuffleboard.getTab("Drive Train").addNumber("xCoordinate", () ->
                Units.metersToFeet(getRobotPose().getTranslation().getX()));
        Shuffleboard.getTab("Drive Train").addNumber("yCoordinate", () ->
                Units.metersToFeet(getRobotPose().getTranslation().getY()));
        Shuffleboard.getTab("Drive Train").addNumber("Angle", () ->
                getRobotPose().getRotation().getDegrees());
        Shuffleboard.getTab("Drive Train").addNumber("leftSpeed", () ->
                Units.metersToFeet(getSpeeds().leftMetersPerSecond));
        Shuffleboard.getTab("Drive Train").addNumber("rightSpeed", () ->
                Units.metersToFeet(getSpeeds().rightMetersPerSecond));

        Shuffleboard.getTab("Turret").addNumber("Robot Angle", navX::getAngle);
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
        SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
        SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
                Units.metersToFeet(getRobotPose().getTranslation().getX()));
        SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
                Units.metersToFeet(getRobotPose().getTranslation().getY()));
        SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPose().getRotation().getDegrees());
        SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
                Units.metersToFeet(getSpeeds().leftMetersPerSecond));
        SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
                Units.metersToFeet(getSpeeds().rightMetersPerSecond));

        SmartDashboardTab.putNumber("Turret", "Robot Angle", getAngle());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));

        updateSmartDashboard();
    }
}
