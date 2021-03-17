/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;

/**
 * An example command that uses an example subsystem.
 */
public class TestPathFollowing extends CommandBase {
    private static final double m_period = 0.02;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private Trajectory trajectory;
    private Notifier m_notifier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */
    public TestPathFollowing(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetEncoderCounts();
        m_driveTrain.resetAngle();
        m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());

        var startPosition = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
        var stopPosition = new Pose2d(Units.feetToMeters(6), Units.feetToMeters(0), Rotation2d.fromDegrees(0));

        var trajectoryWaypoints = new ArrayList<Pose2d>();
        trajectoryWaypoints.add(startPosition);
        trajectoryWaypoints.add(stopPosition);


        var trajectoryConstraints = new DifferentialDriveKinematicsConstraint(m_driveTrain.getDriveTrainKinematics(),
                3);


        var trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(4));

        trajectoryConfig.setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, trajectoryConfig);

        RamseteCommand followTrajectory = new RamseteCommand(
                trajectory,
                m_driveTrain :: getRobotPose,
                new RamseteController(),
                m_driveTrain.getFeedforward(),
                m_driveTrain.getDriveTrainKinematics(),
                m_driveTrain :: getSpeeds,
                m_driveTrain.getLeftPIDController(),
                m_driveTrain.getRightPIDController(),
                m_driveTrain :: setVoltageOutput,
                m_driveTrain
        );
        CommandScheduler.getInstance().schedule(followTrajectory);
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
