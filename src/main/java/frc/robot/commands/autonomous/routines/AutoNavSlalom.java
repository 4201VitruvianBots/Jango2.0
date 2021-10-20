package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.SetDriveNeutralMode;
import frc.robot.commands.drivetrain.SetDriveShifters;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(DriveTrain driveTrain, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                //new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(60))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                //new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-45))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(86), new Rotation2d(Units.degreesToRadians(150))),
                //new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(225))),
                new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                //new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(120))),
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(150)))    
        };
        
        Pose2d startPosition = waypoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(10));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.5));

        int numberOfCones = SimConstants.autoNavSlalomCones.length;
            for(int i = 0; i < numberOfCones; i++) {
            var constraint = new EllipticalRegionConstraint(
                    SimConstants.autoNavSlalomCones[i],
                    Units.inchesToMeters(20),
                    Units.inchesToMeters(20),
                    new Rotation2d(),
                    new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), 0)
            );
                //configA.addConstraint(constraint);
        }

        addCommands(new SetDriveShifters(driveTrain, Constants.DriveConstants.inSlowGear),
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        for(int i = 0; i < waypoints.length - 1; i++) {
                if (i != 0) {
                        configA.setEndVelocity(configA.getMaxVelocity());
                        configA.setStartVelocity(configA.getMaxVelocity());
                }
                if (i == waypoints.length - 2) {
                        configA.setEndVelocity(0);
                }
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                List.of(),
                waypoints[i + 1],
                configA);


            var command = TrajectoryUtils.generateVitruvianRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

