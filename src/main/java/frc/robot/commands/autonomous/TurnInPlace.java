package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurnInPlace extends CommandBase {
    private DriveTrain m_driveTrain;
    private double m_setpoint;

    private final double kS = 0.19;
    private final double kV = 2.23;
    private final double kA = 0.0289;

    public double kP = 0.09;
    public double kI = 0;
    public double kD = 0.011;

    private double degreesTolerance = 3;
    private double degreesPerSecondTolerance = 3;
    private double kDt = 0.02;

    private PIDController m_controller = new PIDController(kP, kI, kD, kDt);
    private SimpleMotorFeedforward m_feedforward;
    private double m_startTime;
    private double m_timeout = 2;

    double distancePerAngle;
    double maxVel;  // 2.167780360340067 m/s
    double lastTimestamp;
    double lastAngle;

    boolean latch = true;
    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */
    public TurnInPlace(DriveTrain driveTrain, double targetAngle) {
        m_driveTrain = driveTrain;
        m_setpoint = targetAngle;

        m_feedforward = m_driveTrain.getFeedforward();

        m_controller.setSetpoint(targetAngle);
        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(degreesTolerance, degreesPerSecondTolerance);
        m_controller.setIntegratorRange(-30.0, 30.0);

        SmartDashboard.putNumber("AA_P", kP);
        SmartDashboard.putNumber("AB_I", kI);
        SmartDashboard.putNumber("AC_D", kD);

        distancePerAngle = Constants.DriveConstants.kTrackwidthMeters * Math.PI / 360;

        m_driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        m_startTime = lastTimestamp = Timer.getFPGATimestamp();
        lastAngle = m_driveTrain.getHeading();
        latch = true;
    }

    @Override
    public void execute() {
        double pid_output = -m_controller.calculate(m_driveTrain.getHeading());
//        double output = (pid_output + ff_output);
        m_driveTrain.setMotorArcadeDrive(0, pid_output);
//        SmartDashboard.putNumber("PID Output",pid_output);
//        SmartDashboard.putNumber("PID Setpoint",m_controller.getSetpoint());
////        SmartDashboard.putNumber("PID Goal", m_controller.getGoal().position);
//        SmartDashboard.putNumber("PID Position Error", m_controller.getPositionError());
//        SmartDashboard.putNumber("PID Velocity Error", m_controller.getVelocityError());
//        SmartDashboard.putNumber("PID Output", output);
//
//        m_controller.setP(SmartDashboard.getNumber("AA_P", kP));
//        m_controller.setI(SmartDashboard.getNumber("AB_I", kI));
//        m_controller.setD(SmartDashboard.getNumber("AC_D", kD));
//
////        var deltaA = m_driveTrain.getAngle() - lastAngle;
////        var deltaT = Timer.getFPGATimestamp() - lastTimestamp;
////        var vel = (deltaA / deltaT) * distancePerAngle;
////        maxVel = vel > maxVel ? vel : maxVel;
////        SmartDashboard.putNumber("Max Rotational Vel", maxVel);
////        lastAngle = m_driveTrain.getAngle();
////        lastTimestamp = Timer.getFPGATimestamp();
        if(Math.abs(m_driveTrain.getHeading()) >= m_setpoint  && latch) {
            // 0.417952 seconds
            SmartDashboard.putNumber("MinTime:", Timer.getFPGATimestamp() - lastTimestamp);
            latch = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("RunTime:", Timer.getFPGATimestamp() - m_startTime);
        m_driveTrain.setVoltageOutput(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
