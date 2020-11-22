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
import frc.robot.subsystems.Vision;

public class Intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */
  private TalonFX intakeMotor =  new TalonFX(Constants.intakeMotor);
  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
  DoubleSolenoid intakePiston2 = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
  private boolean intaking = false;
  private Vision m_vision;

  public Intake(Vision vision) {
    //Just makes sure that their are no previous settings on the motor and that its not inverted.  
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(false);

    m_vision = vision;
  }

  public boolean getIntakingState() {
    return intaking;
  }

  //Telling the code if the intake is actually intaking or not.
  public void setIntakingState(boolean state) {
    intaking = state;
  }

  public boolean getIntakePistonExtendStatus(){
    // We use an if statement because two returns wouldn't make sense since one of them would never run.
    if ((intakePiston.get() == DoubleSolenoid.Value.kForward) && (intakePiston2.get() == DoubleSolenoid.Value.kForward)) {
      return true;
    } else {
      return false;
    }
  }

  //For this command it is setting when the piston is up its false, and when the piston its down its true.
  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    intakePiston2.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  //Percent output is the amount of electricity that goes directly to the motor from the main battery.
  public void setIntakePercentOutput(double value){
    intakeMotor.set(ControlMode.PercentOutput, value);
  }

  public double getPowercellCount() {
    return m_vision.getPowercellCount();
  }

  //Updates the smart dashboard regularly for ease of access to the robot's codition.
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
