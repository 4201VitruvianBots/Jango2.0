package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;

public class SmartdashboardCommand extends CommandBase {
    String m_smartDashboardKey;

    Sendable m_sendableData;
    double m_numberData;
    String m_stringData;
    boolean m_booleanData;
    int type;

    public SmartdashboardCommand(String smartDashboardKey,Object data) {
        m_smartDashboardKey = smartDashboardKey;
        if(data instanceof Sendable) {
            m_sendableData  = (Sendable) data;
            type = 4;
        } else if(data instanceof Number) {
            m_numberData  = ((Number) data).doubleValue();
            type = 3;
        } else if(data instanceof Boolean) {
            m_booleanData = ((Boolean) data).booleanValue();
            type = 2;
        } else if(data instanceof String) {
            m_stringData = data.toString();
            type = 1;
        } else {
            try {
                m_stringData = data.toString();
                type = 0;
            } catch (Exception e) {
                System.out.println("SmartdashboardCommand Error: object cannot be added to SmartDashboard");
            }
        }
    }

    @Override
    public void initialize() {
         switch(type) {
             case 4:
                 SmartDashboard.putData(m_smartDashboardKey, m_sendableData);
                 break;
             case 3:
                 SmartDashboard.putNumber(m_smartDashboardKey, m_numberData);
                 break;
             case 2:
                 SmartDashboard.putBoolean(m_smartDashboardKey, m_booleanData);
                 break;
             case 1:
                 SmartDashboard.putString(m_smartDashboardKey, m_stringData);
                 break;
             default:
             case 0:
                 try {
                     SmartDashboard.putString(m_smartDashboardKey, m_stringData);
                 } catch (Exception e) {
                     System.out.println("SmartdashboardCommand Error: object cannot be added to SmartDashboard");
                 }
                 break;
         }
    }



    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
