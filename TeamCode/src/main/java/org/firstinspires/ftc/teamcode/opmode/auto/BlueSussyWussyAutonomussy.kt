package org.firstinspires.ftc.teamcode.opmode.auto

import com.amarcolini.joos.command.*
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline

@Autonomous
@JoosConfig
class BlueSussyWussyAutonomussy : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(true)

    companion object {
        var startPose = Pose2d(16.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(10.0, 35.0, (180).deg)
        var rightPlacePose = Pose2d(50.5, 32.0, 0.deg)
        var centerPlopPose = Pose2d(16.0, 41.0, (-90).deg)
        var centerPlacePose = Pose2d(50.5, 39.0, 0.deg)
        var leftPlopPose = Pose2d(25.0, 43.5, (-90).deg)
        var leftPlacePose = Pose2d(50.5, 46.0, 0.deg)
        var parkPose = Pose2d(52.0, 15.0, 0.deg)

        var exitTangent = (120).deg
        var exitPose = Pose2d(18.0, 60.0, 0.deg)
        var stackPose = Pose2d(-59.0, 37.0, 0.deg)
        var crossPose = Pose2d(-36.0, 60.0, 0.deg)
        var stackPlacePose = Pose2d(47.0, 39.0, 0.deg)
        var leftStackTangent = (90).deg

        var stackHigh = 0.51
    }

    private var stackPos = stackHigh

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.openFrontCameraAsync {
            robot.frontCamera.setPipeline(pipeline)
            schedule(true) {
                telem.addData("prop location", pipeline.lastKnownLocation)
            }
        }
    }

    override fun preStart() {
        cancelAll()
        robot.outtake.ready()
        robot.drive.poseEstimate = startPose

        val propPosition = pipeline.lastKnownLocation
        val (plopPose, placePose) = when (propPosition) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }
        val purplePlopCommand = robot.drive.pathCommandBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .build()
        val yellowPlaceCommand = robot.drive.pathCommandBuilder(plopPose)
            .setTangent { it + 180.deg }
            .splineToSplineHeading(placePose, placePose.heading)
            .and(WaitCommand(1.0) then robot.outtake.extend())
            .setFollower(robot.drive.slowFollower)
            .forward(3.0).withTimeout(1.0)
            .resetFollower()
            .then {
                robot.outtake.releaseRight()
                robot.outtake.releaseLeft()
            }.wait(0.5)
            .build()

        val cycleCommand = robot.drive.pathCommandBuilder(placePose)
            .setTangent(exitTangent)
            .splineToSplineHeading(exitPose, exitPose.heading + 180.deg)
            .splineTo(crossPose.vec(), crossPose.heading + 180.deg)
            .splineToSplineHeading(stackPose + Pose2d(5.0), leftStackTangent)
            .and(
                (robot.outtake.reset() and robot.verticalExtension.goToPosition(0.0)) wait 1.0 then Command.select(
                    robot.intake
                ) {
                    robot.intake.waitForServoPosition(
                        stackPos
                    )
                }
                    .then {
                        stackPos += 0.1
                        robot.intake.servoState = Intake.ServoState.STACK
                        robot.intake.motorState = Intake.MotorState.ACTIVE
                    }
            )
            .setFollower(robot.drive.slowFollower)
            .setTangent(stackPose.heading + 180.deg)
            .splineToSplineHeading(
                stackPose,
                stackPose.heading + 180.deg
            ).race(WaitCommand(2.0))
            .strafeRight(2.0)
            .race(
                (
                        Command.emptyCommand().waitUntil {
                            robot.intake.numPixels == 2
                        }.race(
                            Command.select
                            {
                                WaitCommand(0.4) then robot.intake.waitForServoPosition(robot.intake.servo.position + 0.01)
                            }.repeatForever()
                        )
                        ).withTimeout(1.5)
            )
            .resetFollower()
            .forward(5.0)
            .setTangent(if (true || propPosition == PropPipeline.PropLocation.Left) (leftStackTangent) else stackPose.heading)
            .splineToSplineHeading(Pose2d(crossPose.vec(), exitPose.heading), crossPose.heading)
            .splineToConstantHeading(exitPose.vec(), crossPose.heading)
            .splineToSplineHeading(stackPlacePose, stackPlacePose.heading)
            .and(
                SequentialCommand(
                    WaitCommand(0.5),
                    robot.transfer(),
                    FunctionalCommand(isFinished =
                    { robot.drive.poseEstimate.x > 10.0 }),
                    (robot.verticalExtension.goToPosition(400.0) and robot.outtake.extend())
                )
            )
            .setFollower(robot.drive.slowFollower)
            .forward(3.0).withTimeout(2.0)
            .resetFollower()
            .then {
                robot.outtake.releaseLeft()
                robot.outtake.releaseRight()
            }.wait(0.5)
            .build()

        val parkCommand = robot.drive.pathCommandBuilder(leftPlacePose - Pose2d(3.0))
            .lineToSplineHeading(parkPose)
            .and(robot.outtake.reset() and robot.verticalExtension.goToPosition(0.0))
            .build()

        SequentialCommand(
            true,
            purplePlopCommand,
            yellowPlaceCommand,
            cycleCommand,
            parkCommand
        ).thenStopOpMode().schedule()
    }
}