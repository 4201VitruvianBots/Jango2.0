/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class SetArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetArcadeDrive(DriveTrain subsystem) {
    m_driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickY = (Math.abs(RobotContainer.getLeftJoystickY()) > 0.05) ? RobotContainer.getLeftJoystickY() : 0;
    double joystickX = (Math.abs(RobotContainer.getRightJoystickX()) > 0.05) ? RobotContainer.getRightJoystickX() : 0;

//        double throttle = 0.5 * (joystickY + Math.pow(joystickY, 3));
//        throttle = throttle < 0 ? Math.max( -0.7, throttle) : throttle;
//        double turn = 0.25 *(joystickX + Math.pow(joystickX, 3));
    double throttle = joystickY;
    throttle = throttle < 0 ? Math.max(-0.7, throttle) : throttle;
    double turn = (m_driveTrain.getDriveShifterStatus() ? 0.5 : 0.35) * joystickX;

    m_driveTrain.setMotorArcadeDrive(throttle, turn);
//    if (Robot.climber.climbMode == 1) {
//      double operatorThrottle = Math.abs(Robot.m_oi.getXBoxRightY()) > 0.05 ? Robot.m_oi.getXBoxRightY() : 0;
//      Robot.driveTrain.setClimbMotorPercentOutput(Math.min(throttle + operatorThrottle, 0.5));
////            Robot.driveTrain.setClimbMotorCurrentOutput(30 * Math.min(throttle + operatorThrottle, 0.5));
//      throttle = Math.max(Math.min(throttle, 0.25), -0.25);
//      turn = Math.max(Math.min(turn, 0.4), -0.4);
//      Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
//    } else {
//      if (Robot.elevator.controlMode == 1)
//        throttle = Robot.elevator.getHeight() > 30 ? Math.min(Math.max(throttle, -0.4), 0.5) : throttle;
//      Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
//    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
