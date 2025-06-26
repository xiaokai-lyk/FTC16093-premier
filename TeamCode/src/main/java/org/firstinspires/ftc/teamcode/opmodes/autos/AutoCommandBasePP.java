package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoDriveCommandPP;
//import org.firstinspires.ftc.teamcode.subsystems.Climber;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
//import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStructure;
//import org.firstinspires.ftc.teamcode.subsystems.Vision;

import lombok.Getter;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
public abstract class AutoCommandBasePP extends LinearOpMode {
//    protected LiftClaw liftClaw;
//    protected Lift lift;
//    protected SlideSuperStructure slide;
    protected Follower follower;
//    protected Climber climb;
//    protected Vision vision;
    protected Pose currentPose = new Pose();

    public static long handoff_slide2LiftCloseDelayMs = 150;
    public static long handoff_liftClose2OpenIntakeDelayMs = 100;
    public static int liftClawScoreThreshold = 37;

    @Getter private static Pose autoEndPose = new Pose();

    protected void initialize() {
        initialize(true);
    }

    protected void initialize(boolean telemetryInDashboard) {
        if (telemetryInDashboard) {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        CommandScheduler.getInstance().reset();
        // Subsystems Initialized
//        lift = new Lift(hardwareMap, telemetry);
//        liftClaw = new LiftClaw(hardwareMap);
//        climb = new Climber(hardwareMap);
//        slide = new SlideSuperStructure(hardwareMap, telemetry);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        vision = new Vision(hardwareMap, telemetry);
    }

//    protected Command upLiftToBasket() {
//        return new ParallelCommandGroup(
//                new InstantCommand(() -> lift.setGoal(Lift.Goal.HIGH_BASKET)),
//                new WaitUntilCommand(() -> lift.getCurrentPosition() > liftClawScoreThreshold)
//                        .andThen(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, 0)));
//    }

    protected Command followPath(Path path) {
        return new AutoDriveCommandPP(follower, path);
    }

    protected Command followPathChain(PathChain pathChain, boolean holdPoint) {
        return new AutoDriveCommandPP(follower, pathChain, holdPoint);
    }

//    protected Command autoSamplePickCommand(Pose goalPose) {
//        AtomicReference<Double> turnServoSupplier = new AtomicReference<>();
//        AtomicReference<Double> slideExtensionSupplier = new AtomicReference<>();
//        SampleAutoAlignCommandPP sampleAutoAlignCommand =
//                new SampleAutoAlignCommandPP(
//                        follower, vision, telemetry, turnServoSupplier, slideExtensionSupplier);
//        return new SequentialCommandGroup(
//                sampleAutoAlignCommand.alongWith(
//                        new WaitUntilCommand(() -> !sampleAutoAlignCommand.isInitializing())
//                                .andThen(
//                                        slide.aimCommand(turnServoSupplier),
//                                        new InstantCommand(() -> slide.forwardSlideExtension(slideExtensionSupplier)))),
//                new WaitCommand(50),
//                slide.grabCommand());
//    }

//    protected Command stowArmFromBasket() {
//        return new SequentialCommandGroup(
//                liftClaw.openClawCommand(),
//                new WaitCommand(200),
//                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 100),
//                new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
//    }

//    protected Command slowHandoff() {
//        return slowHandoff(slide, liftClaw).beforeStarting(() -> slide.setAutoTurnControl(false));
//    }

//    public static Command slowHandoff(SlideSuperStructure slide, LiftClaw liftClaw) {
//        return slide
//                .slowHandoffCommand()
//                .beforeStarting(liftClaw::openClaw)
//                .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
//                .andThen(liftClaw.closeClawCommand())
//                .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
//                .andThen(new InstantCommand(slide::openIntakeClaw));
//    }

//    protected Command fastHandoff() {
//        return fastHandoff(slide, liftClaw).beforeStarting(() -> slide.setAutoTurnControl(false));
//    }

//    public static Command fastHandoff(SlideSuperStructure slide, LiftClaw liftClaw) {
//        return new SequentialCommandGroup(
//                liftClaw.openClawCommand(),
//                slide.fastHandoffCommand().andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs)),
//                liftClaw.closeClawCommand(),
//                new WaitCommand(handoff_liftClose2OpenIntakeDelayMs),
//                new InstantCommand(slide::openIntakeClaw));
//    }

    public Command wait(Follower follower, long ms) {
        return new ParallelDeadlineGroup(
                new WaitCommand(ms), new RunCommand(follower::update).interruptOn(this::isStopRequested));
    }

//    public Command initializeCommand() {
//        return new ParallelCommandGroup(
//                liftClaw.closeClawCommand(),
//                new InstantCommand(slide::slideArmUp),
//                new InstantCommand(slide::wristUp),
//                new InstantCommand(slide::openIntakeClaw));
//    }

    public Command autoFinish() {
        return new ParallelCommandGroup(
//                slide.manualResetCommand().withTimeout(1000),
//                lift.manualResetCommand().withTimeout(1000),
//                liftClaw.openClawCommand(),
                new InstantCommand(() -> autoEndPose = follower.getPose()));
    }

//    public Command upLiftToHang() {
//        return new SequentialCommandGroup(
//                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_CHAMBER, 200),
//                new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)));
//    }

    /**
     * Gets the command to run in auto, this should be implemented in each auto.
     *
     * @return The command to run.
     */
    public abstract Command runAutoCommand();

    /**
     * Gets the robot starting pose in field coordinate or its respective coordinates.
     *
     * @return The start pose following Pedro Pathing's coordinate system.
     */
    public abstract Pose getStartPose();

    public abstract void initializeSuperStructure();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeSuperStructure();

        follower.setStartingPose(getStartPose());
        telemetry.addData("Init Complete", "Start Now");
        Command toRun = runAutoCommand().andThen(autoFinish());
        waitForStart();

        CommandScheduler.getInstance().schedule(toRun);

        while (opModeIsActive() && !isStopRequested()) {
            currentPose = follower.getPose();
//            lift.periodicAsync();
            CommandScheduler.getInstance().run();
//            telemetry.addData("Y Velocity", follower.getPoseVelocity().getY());
            telemetry.update();
        }

        if (isStopRequested()) {
//            follower.resetHeading();
        }
    }
}