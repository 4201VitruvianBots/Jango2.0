/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public boolean practiceField = true;
  public boolean isColor = false;
  public int semiRotations = 0;
  private int colorID;
  public ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
  public TalonSRX motor = new TalonSRX(Constants.controlPanel);
  
  public ColorSensor() {
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public Color getColor() {
    return sensor.getColor();
  }

  public double getIR() {
    return sensor.getIR();
  }

  public int getProximity() {
    return sensor.getProximity();
  }

  public int panelColor(){ // none = 0; red = 1; green = 2; blue = 3; yellow = 4
    if(practiceField){
      if(getColor().red > getColor().green && getColor().green * 1.8 > getColor().red){
        return 1;
      }
      if(getColor().red * 2.26 < getColor().green && getColor().blue * 2.16 < getColor().green){
        return 2;
      }
      if(getColor().blue * 1.25 > getColor().green && getColor().blue * 1.02 < getColor().green && getColor().blue > getColor().red * 1.89){
        return 3;
      }
      if(getColor().red * 1.83 > getColor().green && getColor().red * 1.78 < getColor().green && getColor().red > getColor().blue * 1.86){
        return 4;
      }
      else
        return 0;
    }
    else {
      if(getColor().red > getColor().blue * 3 && getColor().red > getColor().green * 1.33){
        return 1;
      }
      else if(getColor().green > getColor().red * 2.75 && getColor().green > getColor().blue * 1.8){
        return 2;
      }
      else if(getColor().blue < getColor().green * 1.15 && getColor().green < getColor().blue * 1.15 && getColor().blue > getColor().red * 2.5){
        return 3;
      }
      else if(getColor().green < getColor().red * 1.8 && getColor().green > getColor().red * 1.65){
        return 4;
      }
      else
        return 0;
    }
  }

  public void resetRotationControlVars(){
    isColor = true;
    semiRotations = 0;
    colorID = panelColor();
  }

  public boolean rotationControlComplete(){
    if(panelColor() == colorID && isColor == false){
      isColor = true;
      semiRotations++;
    } else if(panelColor() != colorID && isColor){
      isColor = false;
    }
    if(semiRotations >= 6 && isColor == false){
      return true;
    } else
      return false;
  }

  public void setOutput(double output){
    motor.set(ControlMode.PercentOutput, output);
  }

  public int getFMSColor() {
    String message = DriverStation.getInstance().getGameSpecificMessage();
    switch(message){
      case "R":
        return 1;
      case "G":
        return 2;
      case "B":
        return 3;
      case "Y":
        return 4;
      default:
        return -1;
    }
  }
  
  public void updateSmartDashboard() {
    String colorName = "Not Close Enough";
    SmartDashboard.putNumber("Red", getColor().red);
    SmartDashboard.putNumber("Green", getColor().green);
    SmartDashboard.putNumber("Blue", getColor().blue);
    SmartDashboard.putNumber("IR", getIR());
    SmartDashboard.putNumber("Proximity", getProximity());
    switch(panelColor()){
      case 1:
        colorName = "Red";
        break;
      case 2:
        colorName = "Green";
        break;
      case 3:
        colorName = "Blue";
        break;
      case 4:
        colorName = "Yellow";
        break;
    }
    SmartDashboard.putString("Panel Color", colorName);
    SmartDashboard.putBoolean("Rotation Control Complete", rotationControlComplete());
    SmartDashboard.putNumber("Semi-Rotations", semiRotations);
    //SmartDashboard.putString("Color", getColorString());
  }

  private void updateSmartDashboard() {
    SmartDashboardTab.putNumber("ColorSensor","Red", getColor().red);
    SmartDashboardTab.putNumber("ColorSensor","Green", getColor().green);
    SmartDashboardTab.putNumber("ColorSensor","Blue", getColor().blue);
    SmartDashboardTab.putNumber("ColorSensor","IR", getIR());
    SmartDashboardTab.putNumber("ColorSensor","Poximity", getProximity());
    SmartDashboardTab.putNumber("ColorSensor","Panel Color", panelColor());
    SmartDashboardTab.putBoolean("ColorSensor","Rotation Control Complete", rotationControlComplete());
    SmartDashboardTab.putNumber("ColorSensor","Semi Rotations", semiRotations);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
