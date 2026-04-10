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

public class Conveyor extends SubsystemBase {

    private final TalonFX conveyorMotor = new TalonFX(Constants.conveyorMotorID);



    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(0.3447, 0, 0.0025)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                    //don't change gears for SA comp //acutal gear ratio: 22:18 TT
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("conveyor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(60))//For flywheel, stator current lm can be 60A/80A //talon fx can handle up to 260A
                    .withMotorInverted(true)
                    .withFeedforward(new SimpleMotorFeedforward(0.17, 0.117, 0.01)) //thanks 3561!
                    .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                    .withControlMode(ControlMode.CLOSED_LOOP);
                    //.withVoltageCompensation(Volts.of(12));

    private final SmartMotorController motor =
            new TalonFXWrapper(conveyorMotor, DCMotor.getKrakenX60(2), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(Inches.of(4))
                    .withMass(Pounds.of(1))
                    .withTelemetry("conveyor", TelemetryVerbosity.HIGH);

    private final FlyWheel conveyor = new FlyWheel(flywheelConfig);

    public Conveyor() {
    }

    public AngularVelocity getRPM() {
        return conveyor.getSpeed();
    }

    public Command setVelocityommand(AngularVelocity velocity) {
        return conveyor.setSpeed(velocity);
    }

    public void setVelocitySetpoint(AngularVelocity velocity)
    {
        conveyor.setMechanismVelocitySetpoint(velocity);
    }

    public Command setDutyCycle(double dutyCycle) {
        return conveyor.set(dutyCycle);
    }

    public Command stopCommand() {
        return conveyor.set(0);
    }

    public void periodic() {
        conveyor.updateTelemetry();
    }

    public void simulationPeriodic() {
        conveyor.simIterate();
    }

    public void setDutyCycleSetpoint(double dutyCycle) {
         conveyor.setDutyCycleSetpoint(0);
    }

}