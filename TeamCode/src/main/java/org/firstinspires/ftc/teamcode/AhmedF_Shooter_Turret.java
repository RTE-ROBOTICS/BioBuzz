package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "BIOBUZZ Auto Turret")
public class AhmedF_Shooter_Turret extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotorEx turret;
    private DcMotorEx hood;
    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private static final double TURRET_TICKS_PER_REV = 537.7;
    private static final double TURRET_GEAR_RATIO = 1.0;
    private static final double RAMP_TICKS_PER_REV = 537.7;
    private static final double RAMP_GEAR_RATIO = 3.0;
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double TURRET_MIN = -80.0;
    private static final double TURRET_MAX = 80.0;
    private static final double RAMP_MIN = 10.0;
    private static final double RAMP_MAX = 45.0;
    private static final double TURRET_KP = 0.018;
    private static final double TURRET_KI = 0.00015;
    private static final double TURRET_KD = 0.0012;
    private static final double RAMP_KP = 0.025;
    private static final double RAMP_KI = 0.0001;
    private static final double RAMP_KD = 0.001;
    private double turretIntegral = 0;
    private double turretLastError = 0;
    private double hoodIntegral = 0;
    private double hoodLastError = 0;
    private long lastLoopTime;
    private boolean autoAim = true;
    private boolean lastAimButton = false;
    private boolean lastFireButton = false;
    private final ElapsedTime fireTimer = new ElapsedTime();
    private static final double AIM_TOLERANCE = 1.0;
    private static final double RAMP_TOLERANCE = 1.0;
    private static final double MIN_DISTANCE = 12.0;
    private static final double MAX_DISTANCE = 100.0;
    double dt;
    double cameraHeight =
            0.40;
    double targetHeight =
            1.20;
    double cameraAngle =
            0.0;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );
        turret = hardwareMap.get(
                DcMotorEx.class,
                "turret"
        );
        hood = hardwareMap.get(
                DcMotorEx.class,
                "hood"
        );
        flywheelLeft = hardwareMap.get(
                DcMotorEx.class,
                "flywheelLeft"
        );
        flywheelRight = hardwareMap.get(
                DcMotorEx.class,
                "flywheelRight"
        );
        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        hood.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        flywheelLeft.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
        flywheelRight.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
        turret.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );
        turret.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
        hood.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );
        hood.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        lastLoopTime = System.nanoTime();
        waitForStart();
        while (opModeIsActive()) {
            double now = System.nanoTime();
            dt = (now - lastLoopTime) / 1e9;
            if (dt <= 0) {
                dt = 0.01;
            }

            //Used wasPressed instead of the boolean algorithm
            if (gamepad1.rightBumperWasPressed()) {
                autoAim = !autoAim;
            }

            if (autoAim) {
                autoAim();
            } else {
                manualTurret();
            }

            manualHoodOverride();

            boolean fireButton = gamepad1.right_trigger > 0.5;
            if (fireButton && !lastFireButton) {
                fireTimer.reset();
            }
            lastFireButton = fireButton;

            if (fireButton) {
                runShooter();
            } else {
                stopShooter();
            }

            if (gamepad1.left_bumper) {
                runShooter();
            }
            if (gamepad1.a) {
                stopShooter();
            }
            if (gamepad1.b) {
                autoAim = true;
            }

            sleep(5);
        }
        limelight.stop();
        turret.setPower(0);
        hood.setPower(0);
        stopShooter();
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        double tx = result.getTx();
        double turretAngle = getTurretAngle();

        double targetAngle = turretAngle + tx;

        targetAngle = clamp(
                targetAngle,
                TURRET_MIN,
                TURRET_MAX
        );

        double error = targetAngle - turretAngle;
        turretIntegral += error * dt;
        turretIntegral = clamp(
                turretIntegral,
                -50,
                50
        );

        double derivative =
                (error - turretLastError) / dt;
        double power =
                TURRET_KP * error
                        + TURRET_KI * turretIntegral
                        + TURRET_KD * derivative;

        //Removed Power clamping beacuse the motor driver already clamps at 1 and we need max turning speed

        if (Math.abs(error) < 0.15) {
            power = 0;
        }

        turret.setPower(power);
        turretLastError = error;
        double distance = getDistance(result);
        if (distance > 0) {
            double hoodTarget =
                    calculateHoodAngle(distance);
            runHoodPID(hoodTarget);
        }
    }

    private void manualTurret() {
        double power =
                -gamepad1.left_stick_x * 0.5;
        turret.setPower(power);
        turretIntegral = 0;
        turretLastError = 0;
    }
    private void manualHoodOverride() {
        double input =
                -gamepad1.right_stick_y;
        if (Math.abs(input) > 0.08) {
            hood.setPower(
                    Math.pow(input, 2) // Changed the input from a multiplication to an exponential scaling to decrease motor sensitivity without limiting the power at 0.35
            );
            hoodIntegral = 0;
            hoodLastError = 0;
        }
    }
    private void runHoodPID(
            double targetAngle
    ) {
        double currentAngle =
                getHoodAngle();
        double error =
                targetAngle - currentAngle;
        hoodIntegral += error * dt;
        hoodIntegral = clamp(
                hoodIntegral,
                -50,
                50
        );
        double derivative =
                (error - hoodLastError) / dt;
        double power =
                RAMP_KP * error
                        + RAMP_KI * hoodIntegral
                        + RAMP_KD * derivative;
        // REMVOED the clamp. I also think that we are going to use a servo for the hood instead
        // Removed the power requirement on the hood

        hood.setPower(power);
        hoodLastError = error;
    }
    private void runShooter() {
        LLResult result =
                limelight.getLatestResult();
        double distance = 0;
        if (result != null && result.isValid()) {
            distance = getDistance(result);
        }
        double rpm =
                calculateRPM(distance);
        double ticksPerSecond =
                rpm *
                        FLYWHEEL_TICKS_PER_REV /
                        60.0;
        flywheelLeft.setVelocity(
                ticksPerSecond
        );
        flywheelRight.setVelocity(
                ticksPerSecond
        );
    }
    private void stopShooter() {
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
    }
    private double getDistance(
            LLResult result
    ) {
        double ty =
                result.getTy();
        double angle =
                Math.toRadians(
                        cameraAngle + ty
                );
        double heightDifference =
                targetHeight -
                        cameraHeight;
        if (Math.abs(
                Math.tan(angle)
        ) < 0.001) {
            return MAX_DISTANCE;
        }
        double distance =
                heightDifference /
                        Math.tan(angle);
        return Math.abs(distance);
    }
    private double calculateRPM(
            double distance
    ) {
        distance =
                clamp(
                        distance,
                        MIN_DISTANCE,
                        MAX_DISTANCE
                );
        if (distance <= 24) {
            return 3000;
        }
        if (distance <= 36) {
            return interpolate(
                    distance,
                    24,
                    36,
                    3000,
                    3500
            );
        }
        if (distance <= 48) {
            return interpolate(
                    distance,
                    36,
                    48,
                    3500,
                    4000
            );
        }
        if (distance <= 60) {
            return interpolate(
                    distance,
                    48,
                    60,
                    4000,
                    4500
            );
        }
        if (distance <= 72) {
            return interpolate(
                    distance,
                    60,
                    72,
                    4500,
                    5000
            );
        }
        return 5300;
    }
    private double calculateHoodAngle(
            double distance
    ) {
        distance =
                clamp(
                        distance,
                        MIN_DISTANCE,
                        MAX_DISTANCE
                );
        if (distance <= 24) {
            return 38;
        }
        if (distance <= 36) {
            return interpolate(
                    distance,
                    24,
                    36,
                    38,
                    33
            );
        }
        if (distance <= 48) {
            return interpolate(
                    distance,
                    36,
                    48,
                    33,
                    29
            );
        }
        if (distance <= 60) {
            return interpolate(
                    distance,
                    48,
                    60,
                    29,
                    25
            );
        }
        if (distance <= 72) {
            return interpolate(
                    distance,
                    60,
                    72,
                    25,
                    22
            );
        }
        return 20;
    }
    private double getTurretAngle() {
        return (
                turret.getCurrentPosition()
                        / TURRET_TICKS_PER_REV
        ) * 360.0
                / TURRET_GEAR_RATIO;
    }
    private double getHoodAngle() {
        return (
                hood.getCurrentPosition()
                        / RAMP_TICKS_PER_REV
        ) * 360.0
                / RAMP_GEAR_RATIO;
    }
    private double interpolate(
            double x,
            double x1,
            double x2,
            double y1,
            double y2
    ) {
        return y1 +
                (x - x1) *
                        (y2 - y1) /
                        (x2 - x1);
    }
    private double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }
}