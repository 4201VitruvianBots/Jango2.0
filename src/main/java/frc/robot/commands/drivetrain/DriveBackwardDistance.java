package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.lang.reflect.Field;
import java.util.ArrayList;

public class DriveBackwardDistance extends SequentialCommandGroup {
    public DriveBackwardDistance(DriveTrain driveTrain, FieldSim fieldSim, double distance) { // Distance in meters
        Pose2d startPosition = new Pose2d();
        Pose2d endPosition = new Pose2d(-distance, 0, new Rotation2d());
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(10));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        ArrayList<Pose2d> driveBackwardPath = new ArrayList();
        driveBackwardPath.add(startPosition);
        driveBackwardPath.add(endPosition);
        var driveBackwardCommand = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, driveBackwardPath, configA);

        addCommands(
                new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain,0),
                driveBackwardCommand
            );
    }
}

