//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.kBasicMotor;

import edu.wpi.first.wpilibj2.command.Command;

public final class BasicMotor extends Subsystem1507 {

    private final Motor1507 motor;
    private final BaseStatusSignal[] motorSignals;

    public BasicMotor() {
        super("BasicMotor");

        motor = new Motor1507(
            "BasicMotor",
            Motor1507.Type.FX,
            12,
            kBasicMotor.CONFIG
        );

        motorSignals = motor.getSignals();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorSignals);
    }

    public void runForward() {
        motor.runDuty(kBasicMotor.DUTY_FORWARD);
    }

    public void runReverse() {
        motor.runDuty(kBasicMotor.DUTY_REVERSE);
    }

    public void stop() {
        motor.stop();
    }

    // ============================================================
    // Commands
    // ============================================================

    public Command runForwardCommand() {
        return new CommandBuilder(this)
            .named("BasicMotor.runForward")
            .onExecute(this::runForward)
            .onEnd(this::stop);
    }

    public Command runReverseCommand() {
        return new CommandBuilder(this)
            .named("BasicMotor.runReverse")
            .onExecute(this::runReverse)
            .onEnd(this::stop);
    }

    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("BasicMotor.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
