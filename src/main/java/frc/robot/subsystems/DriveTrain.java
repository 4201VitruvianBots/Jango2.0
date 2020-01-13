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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class
DriveTrain extends SubsystemBase {
  private double gearRatio = 1;
  private double wheelDiameter = 0.5;
  private double ticksPerMeter = Units.inchesToMeters(6 * Math.PI) / 4096;

  private double kS = 1.35;
  private double kV = 1.04;
  private double kA = 0.182;

  public double kP = 8.02;
  public double kI = 0;
  public double kD = 0;

  public int controlMode = 0;

  private TalonSRX[] driveMotors = {
        new TalonSRX(Constants.leftFrontDriveMotor),
        new TalonSRX(Constants.leftRearDriveMotor),
        new TalonSRX(Constants.rightFrontDriveMotor),
        new TalonSRX(Constants.rightRearDriveMotor)
  };

  DoubleSolenoid driveTrainShifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);
  public AHRS navX = new AHRS(SerialPort.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  PIDController leftPIDController = new PIDController(kP, kI, kD);
  PIDController rightPIDController = new PIDController(kP, kI, kD);

  Pose2d pose;

  public DriveTrain() {

    for (TalonSRX motor : driveMotors) {
      motor.configFactoryDefault();
      motor.configVoltageCompSaturation(12);
      motor.enableVoltageCompensation(true);
      motor.configContinuousCurrentLimit(30);
      motor.configPeakCurrentLimit(40);
      motor.configPeakCurrentDuration(1000);
      motor.enableCurrentLimit(true);
      motor.configOpenloopRamp(0.1);
      motor.configClosedloopRamp(0.1);
      motor.setNeutralMode(NeutralMode.Coast);
      motor.configForwardSoftLimitEnable(false);
      motor.configReverseSoftLimitEnable(false);
    }

    driveMotors[0].setInverted(true);
    driveMotors[1].setInverted(true);
    driveMotors[2].setInverted(false);
    driveMotors[3].setInverted(false);

    driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    driveMotors[0].setSensorPhase(false);
    driveMotors[2].setSensorPhase(false);

    driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
    driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());

  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    // getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
    return new DifferentialDriveWheelSpeeds(
 (driveMotors[0].getSelectedSensorVelocity() * 10 / 4096) * Math.PI * Units.feetToMeters(wheelDiameter), //divide by gear ratio to make sure we have wheel speed
(driveMotors[2].getSelectedSensorVelocity() * 10 / 4096) * Math.PI * Units.feetToMeters(wheelDiameter) //divide by gear ratio to make sure we have wheel speed
    );
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public int getEncoderCount(int sensorIndex) {
    return driveMotors[sensorIndex].getSelectedSensorPosition();
  }

  public double getWheelDistanceMeters(int sensorIndex) {
    return driveMotors[sensorIndex].getSelectedSensorPosition() * ticksPerMeter;
  }

  public void resetEncoderCounts() {
    driveMotors[0].setSelectedSensorPosition(0);
    driveMotors[2].setSelectedSensorPosition(0);
  }

  public void setMotorArcadeDrive(double throttle, double turn) {
    double leftPWM = throttle + turn;
    double rightPWM = throttle - turn;

    if(rightPWM > 1.0) {
      leftPWM -= rightPWM - 1.0;
      rightPWM = 1.0;
    } else if(rightPWM < -1.0) {
      leftPWM -= rightPWM + 1.0;
      rightPWM = -1.0;
    } else if(leftPWM > 1.0) {
      rightPWM -= leftPWM - 1.0;
      leftPWM = 1.0;
    } else if(leftPWM < -1.0) {
      rightPWM -= leftPWM + 1.0;
      leftPWM = -1.0;
    }

    setMotorPercentOutput(leftPWM, rightPWM);
  }

  public void setMotorTankDrive(double leftOutput, double rightOutput) {
    setMotorPercentOutput(leftOutput, rightOutput);
  }

  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    setMotorPercentOutput(leftVoltage/12, rightVoltage/12);
  }

  private void setMotorPercentOutput(double leftOutput, double rightOutput) {
    driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
    driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
  }

  public boolean getDriveShifterStatus() {
    return (driveTrainShifters.get() == DoubleSolenoid.Value.kForward) ? true : false;
  }

  public void setDriveShifterStatus(boolean state) {
    driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void resetOdometry(Pose2d pose, Rotation2d rotation){
    odometry.resetPosition(pose, rotation);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Left Encoder", getEncoderCount(0));
    SmartDashboard.putNumber("Right Encoder", getEncoderCount(2));

    SmartDashboard.putNumber("xCoordinate", Units.metersToFeet(getRobotPose().getTranslation().getX()));
    SmartDashboard.putNumber("yCoordinate", Units.metersToFeet(getRobotPose().getTranslation().getY()));
    SmartDashboard.putNumber("angle", getRobotPose().getRotation().getDegrees());
    SmartDashboard.putNumber("leftSpeed", Units.metersToFeet(getSpeeds().leftMetersPerSecond));
    SmartDashboard.putNumber("rightSpeed", Units.metersToFeet(getSpeeds().rightMetersPerSecond));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getWheelDistanceMeters(0), getWheelDistanceMeters(2));

    updateSmartDashboard();
  }

  public Pose2d getRobotPose(){
    return pose;
  }

  public DifferentialDriveKinematics getDriveTrainKinematics() {
    return kinematics;
  }
}