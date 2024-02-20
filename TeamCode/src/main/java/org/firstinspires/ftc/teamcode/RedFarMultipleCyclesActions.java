package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Red Far Multiple Cycles Actions"/*, preselectTeleOp = "1 Manual Control"*/)
public class RedFarMultipleCyclesActions extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action BoardTrajFinal;
    Action Park;
    Action DriveBackToStack1;
    Action DriveBackToStack2;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(-54.5, -11.5, Math.toRadians(180));

        speedUpVelocityConstraint = new TranslationalVelConstraint(90.0); //TODO Need to add a speed-up Velocity constraint to some of the trajectories
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-70.0, 70.0);    //TODO need to determine is an acceleration constraint on some trajectories would be useful

        /* Initialize the Robot */
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();
        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();//make decisions
            RedLeftPurplePixelDecision();
        }

        DriveToStack = drive.actionBuilder(deliverToFloorPose)
                .splineToLinearHeading(stackPose, Math.toRadians(180))
                .build();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        /* Drive to Floor Position */
                        new ParallelAction(
                                lockPixels(),
                                FloorTraj),
                        /* Deliver the Purple Pixel */
                        dropOnLine(), //TODO -- takes too long, need to see if can split up and make parts of it parallel
                        resetArm(),
                        //slidesDown(),
                        new ParallelAction(
                                slidesDown(),
                                servoIntake(),
                                DriveToStack)
                )
        );

        /* Pick up a White Pixel from the stack */
        control.AutoPickupRoutine(); //TODO -- takes too long, need to see if can split up pickup part to be in parallel

        /* Drive to the board while moving arm up to scoring position after crossing the half-way point */
        drive.updatePoseEstimate();
        RedBoardDecision(); // updates BoardTraj2
        Actions.runBlocking(
            new ParallelAction(
                    BoardTraj2,
                    halfwayTrigger() //TODO -- need to split this logic up and remove sleeps, could split up BoardTraj2 to make simpler logic
                    )
        );

        /* Use AprilTags to Align Perfectly to the Board */
        control.TagCorrection();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y + control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());

        /* release pixels on the board using the distance sensor to know when to stop */
        control.StopNearBoardAuto(true);

        /* BACK UP FROM BOARD slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .lineToX(46)
                .build());

        // **********************************************************
        // ******    Begin Logic to get an extra 2 White Pixels *****
        // **********************************************************
        /* move arm to reset position while strafing to the side, before driving back to the stack of white pixels */
        drive.updatePoseEstimate();
        DriveBackToStack1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(45, -11.5), Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        resetArm(), //TODO could we move the slidesDown up into this parallelAction to be safer?
                        DriveBackToStack1
                )
        );

        /* move slides down and drive back to stack */
        drive.updatePoseEstimate();
        DriveBackToStack2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        slidesDown(),
                        servoIntake(),
                        DriveBackToStack2
                )
        );

        //grab 2 more white pixels
        control.AutoPickupRoutine();  //TODO -- takes too long, need to see if can split up pickup part to be in parallel

        //TODO Need to drive to Position 1 so that don't interfere with alliance partner, and to save time
        //drive to center position
        drive.updatePoseEstimate();
        BoardTrajFinal = drive.actionBuilder(drive.pose)
                .setTangent(0)
                //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        BoardTrajFinal,
                        halfwayTrigger()
                )
        );

        //deliver two white pixels
        control.StopNearBoardAuto(true);

        /* BACK UP FROM BOARD (2nd time) slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(46)
                        .build());

        /* Park the Robot, and Reset the Arm and slides */
        drive.updatePoseEstimate();
        Park = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                Park,
                                resetArm(),
                                servoStop()
                        ),
                        slidesDown()
                        //servoStop()
                )
        );
    }

    public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47.5,-28,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))

        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47.5,-38,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47.5,-33,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
    }

    public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                    .build();
        }
    }

    public Action lockPixels(){return new LockPixels();}
    public class LockPixels implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.GrabPixels();
                control.ReleaseLeft();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action dropOnLine(){return new DropOnLine();}
    public class DropOnLine implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DropOnLine(); //TODO split up this logic, it has a bunch of sleeps in it, should do some of this in parallel with driving
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action resetArm(){return new ResetArm();}
    public class ResetArm implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ResetArmAutoNoSlides();
                initialized = true;
            }
            packet.put("ResetArm", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesDown(){return new SlidesDown();}
    public class SlidesDown implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                //control.SlidesDown();
                initialized = true;
            }
            packet.put("Slides Down", 0);
            boolean slidesAllDown = control.SlidesDownInParallel();
            return !slidesAllDown;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoIntake(){return new ServoIntake();}
    public class ServoIntake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoIntake();
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesToAuto(){return new SlidesToAuto();}
    public class SlidesToAuto implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SlidesToAuto();
                initialized = true;
            }
            packet.put("SlidesToAuto", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action deliverPixelToBoardPos(){return new DeliverPixelToBoardPos();}
    public class DeliverPixelToBoardPos implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DeliverPixelToBoardPos(); // contains a sleep, therefore this action cannot be in parallel
                initialized = true;
            }
            packet.put("DeliverPixelToBoardPos", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action stopNearBoardAuto(){return new StopNearBoardAuto();}
    public class StopNearBoardAuto implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.StopNearBoardAuto(true); // contains a sleep, therefore this action cannot be in parallel
                initialized = true;
            }
            packet.put("StopNearBoardAuto", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger(){return new HalfwayTrigger();}
    public class HalfwayTrigger implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.pose.position.x >= 12) {
                moveArm = true;
                control.SlidesToAuto(); //TODO there is an action above that does just this, should use that to help split
                sleep(150);  //TODO Need to split this logic up and use a SleepAction instead
                control.DeliverPixelToBoardPos(); //TODO Need to split this one up too, has a sleep buried in it
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoStop(){return new ServoStop();}
    public class ServoStop implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoStop();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;
            }
    }
}

