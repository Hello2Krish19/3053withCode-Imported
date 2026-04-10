package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeRollers extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.IntakeRollersID);


    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0.3447, 0, 0.0025)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    //don't change gears for SA comp //acutal gear ratio: 22:18 TT
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("rollers", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(60))//For flywheel, stator current lm can be 60A/80A //talon fx can handle up to 260A
                    .withMotorInverted(true)
                    .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01)) //thanks 3561!
                    .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                    .withControlMode(ControlMode.CLOSED_LOOP);
                    //.withVoltageCompensation(Volts.of(12));

    private final SmartMotorController motor =
            new TalonFXWrapper(intakeMotor, DCMotor.getKrakenX60(2), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

    private final FlyWheel rollers = new FlyWheel(flywheelConfig);

    public IntakeRollers() {
    }

    public AngularVelocity getRPM() {
        return rollers.getSpeed();
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return rollers.setSpeed(velocity);
    }

    public void setVelocitySetpoint(AngularVelocity velocity)
    {
        rollers.setMechanismVelocitySetpoint(velocity);
    }

    public Command setDutyCycle(double dutyCycle) {
        return rollers.set(dutyCycle);
    }

    public Command stopCommand() {
        return rollers.set(0);
    }

    public void periodic() {
        rollers.updateTelemetry();
    }

    public void simulationPeriodic() {
        rollers.simIterate();
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
         rollers.setDutyCycleSetpoint(0);
    }

}