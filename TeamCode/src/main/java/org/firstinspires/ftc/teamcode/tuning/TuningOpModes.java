package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = PinpointDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(PinpointDrive.class)) {
                dvf = hardwareMap -> {
                    PinpointDrive pd = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

                    List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                    List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                    parEncs.add(new PinpointEncoder(pd.pinpoint,false, pd.leftBack));
                    perpEncs.add(new PinpointEncoder(pd.pinpoint,true, pd.leftBack));

                    return new DriveView(
                            DriveType.MECANUM,
                            MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.maxWheelVel,
                            MecanumDrive.PARAMS.minProfileAccel,
                            MecanumDrive.PARAMS.maxProfileAccel,
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(
                                    pd.leftFront,
                                    pd.leftBack
                            ),
                            Arrays.asList(
                                    pd.rightFront,
                                    pd.rightBack
                            ),
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            pd.lazyImu,
                            pd.voltageSensor,
                            () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                    MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick)
                    );
                };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
