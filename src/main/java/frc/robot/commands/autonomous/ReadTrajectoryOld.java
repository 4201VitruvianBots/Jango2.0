/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
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
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;

/**
 * An example command that uses an example subsystem.
 */
public class ReadTrajectoryOld extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final String m_filename;
    ArrayList<Pose2d> trajectoryWaypoints = new ArrayList<Pose2d>();
    private Trajectory trajectory;
    private boolean m_isInverted = false;
    private DifferentialDriveKinematicsConstraint m_kinematicsConstraint;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */

    public ReadTrajectoryOld(DriveTrain driveTrain, String filename, boolean isInverted, DifferentialDriveKinematicsConstraint kinematicsConstraint) {
        m_driveTrain = driveTrain;
        m_isInverted = isInverted;
        m_filename = filename;
        m_kinematicsConstraint = kinematicsConstraint;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    public ReadTrajectoryOld(DriveTrain driveTrain, String filename, boolean isInverted) {
        m_driveTrain = driveTrain;
        m_isInverted = isInverted;
        m_filename = filename;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    public ReadTrajectoryOld(DriveTrain driveTrain, String filename) {
        m_driveTrain = driveTrain;
        m_filename = filename;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        trajectoryWaypoints = new ArrayList<Pose2d>();
        m_driveTrain.resetAngle();
        m_driveTrain.setDriveTrainNeutralMode(1);
        m_driveTrain.resetOdometry(new Pose2d(), new Rotation2d());
        m_driveTrain.resetEncoderCounts();
        // Start position for all trajectories will be where the robot is currently
        var startPosition = new Pose2d(m_driveTrain.getRobotPose().getTranslation().getX(),
                m_driveTrain.getRobotPose().getTranslation().getY(),
                Rotation2d.fromDegrees(m_driveTrain.getHeading()));

        String filePath = Filesystem.getDeployDirectory().getAbsolutePath() + "/Trajectories/" + m_filename + ".csv";
        var fileTrajectory = TrajectoryUtils.readCsvTrajectory(filePath);

        // All points we generate assume we start from (0,0). Take those points and shift it based on your starting position
        for(Pose2d point : fileTrajectory) {
//      System.out.printf("X: %.2f\tY: %.2f\n", point.getTranslation().getX(), point.getTranslation().getY());
            if(point.getTranslation().getX() == 0 && point.getTranslation().getY() == 0)
                continue;

            trajectoryWaypoints.add(new Pose2d(startPosition.getTranslation().getX() + startPosition.getTranslation().getX(),
                    startPosition.getTranslation().getY() + startPosition.getTranslation().getY(),
                    point.getRotation()));
        }

        var trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(2));

        if(m_kinematicsConstraint != null)
            trajectoryConfig.addConstraint(m_kinematicsConstraint);

        trajectoryConfig.setReversed(m_isInverted);

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
