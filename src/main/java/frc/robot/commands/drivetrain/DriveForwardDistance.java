package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class DriveForwardDistance extends SequentialCommandGroup {
    public DriveForwardDistance(DriveTrain driveTrain, FieldSim fieldSim, double distance) { // Distance in meters
        Pose2d startPosition = new Pose2d();
        Pose2d endPosition = new Pose2d(distance, 0, new Rotation2d());
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(4), Units.feetToMeters(4));
        configA.setReversed(false);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(),
                configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(),
                driveTrain.getDriveTrainKinematics(), 10));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPosition, List.of(), endPosition, configA);

        var driveForwardCommand = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, trajectory);

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition), new SetDriveNeutralMode(driveTrain, 0),
                driveForwardCommand);
    }
}
