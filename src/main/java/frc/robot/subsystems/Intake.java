/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */
  private TalonFX intakeMotor =  new TalonFX(Constants.intakeMotor);
  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
  DoubleSolenoid intakePiston2 = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
  private boolean intaking = false;

  public Intake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(false);

  }
  public boolean getIntakingState() {
    return intaking;
  }

  public void setIntakingState(boolean state) {
    intaking = state;
  }
  public boolean getIntakePistonExtendStatus(){

    if ((intakePiston.get() == DoubleSolenoid.Value.kForward) && (intakePiston2.get() == DoubleSolenoid.Value.kForward)) {
      return true;
    } else {
      return false;
    }
  }

  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    intakePiston2.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
  public void setIntakePercentOutput(double value){
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
  private void updateSmartDashboard() {
    SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
    SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
