/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Vision;

/*
Susbsystem for interacting with the robot's indexer
 */

/*
Motor directions:

-Two motors: front and back motor
  Front motor: always runs up (clockwise)
  Back motor: runs down (counterclockwise)

-Moving balls up:
  Front motor spins up (clockwise)
  Back motor spins up (clockwise)
 */

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // Setup indexer motor controller (SparkMax)
  CANSparkMax front = new CANSparkMax(Constants.indexerMotorFront, MotorType.kBrushless);
  CANSparkMax back = new CANSparkMax(Constants.indexerMotorBack, MotorType.kBrushless);
  CANEncoder encoderFront = front.getEncoder();
  CANEncoder encoderBack = back.getEncoder();

  VictorSPX kicker = new VictorSPX(Constants.kickerMotor);

  // Indexer sensors setup
  DigitalInput indexerTopSensor = new DigitalInput(Constants.indexerTopSensor);
  DigitalInput indexerBottomSensor = new DigitalInput(Constants.indexerBottomSensor);

  private double targetSetpoint;

  private double gearRatio = 1.0 / 27.0;

  private int controlMode = 1;

  private double ballCount = 0;
  private double prevBallCount = 0;

  public Indexer() {
    // Motor and PID controller setup
    front.restoreFactoryDefaults();
    back.restoreFactoryDefaults();
    front.setInverted(false);
    back.setInverted(true);

    front.setIdleMode(IdleMode.kBrake);
    back.setIdleMode(IdleMode.kBrake);

    kicker.configFactoryDefault();
    kicker.setInverted(true);
  }

  public void toggleControlMode() {
    if(controlMode == 0)
      controlMode = 1;
    else
      controlMode = 0;
  }

  public int getControlMode() {
    return controlMode;
  }

  public double getIntakeBallCount() {  }

  public boolean getIndexerBottomSensor(){
    return !indexerBottomSensor.get();
  }

  public boolean getIndexerTopSensor(){
    return !indexerTopSensor.get();
  }

  public void setKickerOutput(double output) {
    kicker.set(ControlMode.PercentOutput, output);
  }

  public void setIndexerOutput(double output) {
    front.set(output);
    back.set(output);
  }

  // Detect whether a new ball has been picked up
  // There is a new ball if the intake sensor is blocked and was not blocked before
  boolean pTripped = false;
  public boolean newBall() {
    boolean returnVal;
    if(pTripped == false && getIntakeSensor()){
      returnVal = true;
    }
    else 
      returnVal = false;
    if(getIntakeSensor())
      pTripped = true;
    else
      pTripped = false;
    return returnVal;
  }

  public void setRPM(double rpm) {
    double setpoint = rpm / gearRatio;
    SmartDashboard.putNumber("Indexer Setpoint", setpoint);
  }

  public boolean getIndexerState(){
    if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      indexerState = IndexerStates.INDEXER_TWO_BALLS;
    else
      indexerState = IndexerStates.INDEXER_ONE_BALL;
  }

  public void IndexerBandStates(boolean getIndexerState) {
  //If neither sensors are triggered move 1 down and 1 up
  //If bottom sensor is triggered move both up
  //If both sensors are triggered move 1 down 1 up
  //If shoot button is pressed both up 
  switch(indexerState){
   case INDEXER_TWO_BALLS:
   front.setOutput(output)
  }
  }
  private void initShuffleboard() {
    // Unstable. Don''t use until WPILib fixes this
    Shuffleboard.getTab("Indexer").addBoolean("Intake Sensor", this::getIntakeSensor);
    Shuffleboard.getTab("Indexer").addBoolean("Indexer Bottom Sensor", this::getIndexerBottomSensor);
    Shuffleboard.getTab("Indexer").addBoolean("Indexer Top Sensor", this::getIndexerTopSensor);
  }

  private void updateSmartDashboard(){
    SmartDashboardTab.putBoolean("Indexer","Intake Sensor", getIntakeSensor());
    SmartDashboardTab.putBoolean("Indexer","Indexer Bottom Sensor", getIndexerBottomSensor());
    SmartDashboardTab.putBoolean("Indexer","Indexer Top Sensor", getIndexerTopSensor());
    SmartDashboardTab.putNumber("Indexer", "Indexer Control Mode", controlMode);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    //updatePIDValues();
  }
}
