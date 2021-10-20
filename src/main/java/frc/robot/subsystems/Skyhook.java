/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/*
Subsystem for controlling climber
 */

public class Skyhook extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final TalonSRX skyhookMotor = new TalonSRX(Constants.skyhookMotor);

    public Skyhook() {
        skyhookMotor.configFactoryDefault();
        skyhookMotor.setNeutralMode(NeutralMode.Brake);
        skyhookMotor.setInverted(true);
    }

    public void setSkyhook(double value) {
        skyhookMotor.set(ControlMode.PercentOutput, value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}