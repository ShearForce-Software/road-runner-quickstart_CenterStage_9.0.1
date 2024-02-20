package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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
    Pose2d stackPose = new Pose2d(-54.5, -11.5, Math.toRadians(180));
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action BoardTrajFinal;
    Action Park;
    Action DriveBackToStack1;
    Action DriveBackToStack2;
    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
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
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        /* Drive to Floor Position */
                        new ParallelAction(
                                lockPixels(),
                                FloorTraj),
                        /* Deliver the Purple Pixel */
                        dropOnLine(),
                        resetArm(),
                        //slidesDown(),
                        new ParallelAction(
                                slidesDown(),
                                servoIntake(),
                                DriveToStack)
                )
        );

        /* Pick up a White Pixel */
        control.AutoPickupRoutine();
        drive.updatePoseEstimate();
        RedBoardDecision();

        //Drive to the board while moving arm up halfway
        Actions.runBlocking(
            new ParallelAction(
                    BoardTraj2,
                    halfwayTrigger()
                    )
        );

        /* Use AprilTags to Align Perfectly to the Board */
        control.TagCorrection();
        sleep(150);
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y + control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());

        //release pixels
        control.StopNearBoardAuto(true);
        drive.updatePoseEstimate();

        //BACK UP FROM BOARD
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .lineToX(46)
                .build());
        drive.updatePoseEstimate();

        //move arm to return position while strafing
        DriveBackToStack1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(45, -11.5), Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        resetArm(),
                        DriveBackToStack1
                )
        );

        //move slides down then drive back to stack
        drive.updatePoseEstimate();
        DriveBackToStack2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        slidesDown(),
                        DriveBackToStack2
                )
        );

        //grab 2 more white pixels
        control.AutoPickupRoutine();
        drive.updatePoseEstimate();

        //drive to center position
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
        drive.updatePoseEstimate();
        //BACK UP FROM BOARD
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(46)
                        .build());
        drive.updatePoseEstimate();
        //define park traj
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
                control.DropOnLine();
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
    public Action resetArmSlides(){return new ResetArmSlides();}
    public class ResetArmSlides implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ResetArmAuto();
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
    public Action autoPickup(){return new AutoPickup();}
    public class AutoPickup implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.AutoPickupRoutine();
                initialized = true;
            }
            packet.put("AutoPickup", 0);
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
                control.DeliverPixelToBoardPos();
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
                control.StopNearBoardAuto(true);
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
                control.SlidesToAuto();
                sleep(150);
                control.DeliverPixelToBoardPos();
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

