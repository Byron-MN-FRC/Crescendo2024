// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS



/**
 *
 */
public class Climb extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax climbR;
// private PWMSparkMax climbL;
private DigitalInput climbLswitch;
private DigitalInput climbRswitch;


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

private CANSparkMax climbL;



    /**
    *
    */
    public Climb() {

    climbLswitch = new DigitalInput(5);
    addChild("climbLswitch", climbLswitch);

    climbRswitch = new DigitalInput(4);
    addChild("climbRswitch", climbRswitch);

    climbR = new CANSparkMax(23, MotorType.kBrushless);
        climbR.setInverted(false);
        climbR.setIdleMode(IdleMode.kBrake);
        climbR.burnFlash();
        climbR.setSmartCurrentLimit(25);

    climbL = new CANSparkMax(30, MotorType.kBrushless);

        climbL.setInverted(true);
        climbL.setIdleMode(IdleMode.kBrake);
        climbL.burnFlash();
    climbL.setSmartCurrentLimit(25);
    

    // climbL = new PWMSparkMax(6);
    // addChild("sparkMaxL",climbL);
    // climbL.setInverted(true);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Right Climbing", isClimbRTriggered());
        SmartDashboard.putBoolean("Left Climbing", isClimbLTriggered());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    

    public void startClimb() {
        if (!isClimbLTriggered()) climbL.set(0);
            else climbL.set(Math.abs(RobotContainer.getInstance().accessory.getLeftY()));

        if (!isClimbRTriggered()) climbR.set(0);
            else climbR.set(Math.abs(RobotContainer.getInstance().accessory.getRightY()));
     }
    
    public void reverseClimb(){
        if (!isClimbLTriggered()) climbL.set(0);
            else climbL.set(-Math.abs(RobotContainer.getInstance().accessory.getLeftY()));

        if (!isClimbRTriggered()) climbR.set(0);
            else climbR.set(-Math.abs(RobotContainer.getInstance().accessory.getRightY()));
    }

    boolean isClimbLTriggered() {
        return climbLswitch.get();
    }

    boolean isClimbRTriggered() {
        return climbRswitch.get();
    }

    public void stopClimb() {
        climbL.set(0);
        climbR.set(0);
    }
}

