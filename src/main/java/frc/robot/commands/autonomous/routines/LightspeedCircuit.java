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

public class LightspeedCircuit extends SequentialCommandGroup {
    public LightspeedCircuit(DriveTrain driveTrain, FieldSim fieldSim) {
        Pose2d[] waypoints = {
            new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(95), new Rotation2d(Units.degreesToRadians(70))),
            new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-60))), 
            new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))), 
            new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(165))),
            new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(120), new Rotation2d(Units.degreesToRadians(90))),
            new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(195), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))), 
            new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-60))), 
            new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))), 
            new Pose2d(Units.inchesToMeters(170), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(Units.inchesToMeters(115), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-100))),
            
        };
        
        Pose2d startPosition = waypoints[0];

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(10));
        configA.setReversed(false);
        //configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        configA.addConstraint(new CentripetalAccelerationConstraint(2.25));

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


            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}

