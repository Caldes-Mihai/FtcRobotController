package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.cache.CacheableCRServo;
import org.firstinspires.ftc.teamcode.cache.CacheableMotor;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double SLIDERS_POS = 0;
    public static double Kg = 0;
    private final CacheableMotor slider1;
    private final CacheableMotor slider2;
    private final CacheableCRServo holder;
    private final GamepadEx gamepad;
    private final PIDController pidController = new PIDController(Kp, Ki, Kd);
    private double factor = 1;

    public OuttakeSubsystem(CacheableMotor slider1, CacheableMotor slider2, CacheableCRServo holder) {
        this(slider1, slider2, holder, null);
    }

    public OuttakeSubsystem(CacheableMotor slider1, CacheableMotor slider2, CacheableCRServo holder, GamepadEx gamepad) {
        //TODO: Change motors depending on which one we will use
        this.slider1 = slider1;
        this.slider2 = slider2;
        this.holder = holder;
        this.gamepad = gamepad;
        slider1.setInverted(true);
    }


    @Override
    public void periodic() {
        slider1.set(pidController.calculate(slider1.getCurrentPosition()) + Kg);
        slider2.set(pidController.calculate(slider2.getCurrentPosition()) + Kg); //TODO: maybe swap slider2 position with slider1 position
    }

    public void extend() {
        pidController.setSetPoint(SLIDERS_POS);
    }

    public void retract() {
        pidController.setSetPoint(0);
    }


    public void setReversed(boolean isReversed) {
        factor = isReversed ? -1 : 1;
    }

    public void activateHolder() {
        holder.setPower(factor);
    }

    public void deactivateHolder() {
        holder.setPower(0);
    }

    public void handle() {
        if (this.isExtended() && gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            this.setReversed(true);
            this.activateHolder();
        } else if (!gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER))
            this.deactivateHolder();
        else if (this.isRetracted() && gamepad.getButton(GamepadKeys.Button.X)) {
            this.setReversed(false);
            this.activateHolder();
        } else if (!gamepad.getButton(GamepadKeys.Button.X))
            this.deactivateHolder();
        //PIDF LOOPS KROOKYBOTS TUTORIAL PEBTRU PID, NU CRED CA TREBUIE SA FOLOSESTI F
    }

    public boolean isExtended() {
        return slider1.getCurrentPosition() == SLIDERS_POS && slider2.getCurrentPosition() == SLIDERS_POS;
    }

    public boolean isRetracted() {
        return slider1.getCurrentPosition() == 0 && slider2.getCurrentPosition() == 0;
    }
}