package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.drive.DriveSignal
import com.amarcolini.joos.followers.HolonomicGVFFollower
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.kinematics.Kinematics
import com.amarcolini.joos.path.Path
import com.amarcolini.joos.path.PathBuilder
import com.amarcolini.joos.util.*
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot
import kotlin.math.PI

@JoosConfig
@TeleOp
class GVFTest : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val follower by onInit {
        HolonomicGVFFollower(
            robot.drive.constraints.maxVel,
            robot.drive.constraints.maxAccel,
            robot.drive.constraints.maxAccel * 0.8,
            robot.drive.constraints.maxAngVel,
            robot.drive.constraints.maxAngAccel,
            Pose2d(0.5, 0.5, 5.deg),
            kN, kOmega, kX, kY, pidCoefficients,
        )
    }
    private val forwardPath by onInit {
        PathBuilder(Pose2d())
            .lineToSplineHeading(Pose2d(48.0, 24.0, (-180).deg)).build()
    }

    companion object {
        var kN = 0.25
        var kOmega = 20.0
        var kX = 0.25
        var kY = 0.25
        var pidCoefficients = PIDCoefficients(8.0, 0.0, 0.2)
        var avoidanceDistance = 5.0
        var l = 0.1
    }

    override fun preInit() {
        robot.outtake.armServo.position = 0.8
    }

    override fun preStart() {
//        val backwardPath = PathBuilder(forwardPath.end(), true)
//            .splineTo(Vector2d(), 180.deg)
//            .build()

        val obstacleRadius = 2.0
//        val avoidanceDistance = 5.0
        val robotRadius = Vector2d(18.0, 16.0).norm() * 0.5
        val obstacle = CircularGVF(Vector2d(24.0, 12.0), obstacleRadius + robotRadius + avoidanceDistance, kN)
        val gvf = CompositeGVF(
            PathGVF(forwardPath, kN),
            listOf(GVFObstacle(obstacle, avoidanceDistance, {
                GVFObstacle.defaultMapFunction(it, l, l)
            }, {
                GVFObstacle.defaultMapFunction(it, l, l)
            }))
        )

        val poseHistory = ArrayList<Pose2d>()
        robot.drive.dashboardEnabled = true
        schedule(true) {
            follower.kN = kN
//            follower.kOmega = kOmega
            telem.drawSampledPath(forwardPath, "green")
            if (follower.isFollowing()) poseHistory.add(robot.drive.poseEstimate)
            telem.drawPoseHistory(poseHistory, "blue")
            telem.fieldOverlay().setAlpha(0.5)
            telem.fieldOverlay().setStroke("yellow")
            telem.fieldOverlay().strokeCircle(obstacle.center.x, obstacle.center.y, obstacle.radius)
            telem.fieldOverlay().setAlpha(1.0)
            telem.fieldOverlay().setStroke("red")
            telem.fieldOverlay().strokeCircle(obstacle.center.x, obstacle.center.y, obstacleRadius)
            telem.addData("error", follower.lastError)
        }

        followGVF(gvf).schedule()
//        followPath(forwardPath).runForever().schedule()

//        SequentialCommand(
//            false,
//            followPath(forwardPath),
//            followPath(backwardPath)
//        ).repeatForever().schedule()
    }

    private fun followPath(path: Path): Command =
        FunctionalCommand(init = {
            follower.followPath(path)
        }, execute = {
            val pose = robot.drive.poseEstimate
            val velocity = robot.drive.poseVelocity?.let { Kinematics.fieldToRobotVelocity(pose, it) }
            robot.drive.setDriveSignal(follower.update(pose, velocity))
        }, isFinished = {
            !follower.isFollowing()
        }, end = {
            robot.drive.setDriveSignal(DriveSignal())
        }, requirements = setOf(robot.drive)
        )

    private fun followGVF(gvf: FollowableGVF): Command =
        FunctionalCommand(init = {
            follower.followGVF(gvf)
        }, execute = {
            val pose = robot.drive.poseEstimate
            val velocity = robot.drive.poseVelocity?.let { Kinematics.fieldToRobotVelocity(pose, it) }
            robot.drive.setDriveSignal(follower.update(pose, velocity))
        }, isFinished = {
            !follower.isFollowing()
        }, end = {
            robot.drive.setDriveSignal(DriveSignal())
        }, requirements = setOf(robot.drive)
        )
}

@TeleOp
@JoosConfig
class HeadingTuning : CommandOpMode() {
    companion object {
        val coeffs = PIDCoefficients()
    }

    private val robot by robot<CSRobot>()

    override fun preInit() {
        val controller = PIDController(coeffs)
        controller.setInputBounds(-PI, PI)

        (Command.of {
            controller.targetPosition = 0.0
        }.wait(3.0) then Command.of {
            controller.targetPosition =
                (Angle.halfCircle + Angle.quarterCircle * (Math.random() - 0.5)).normDelta().radians
        }.wait(3.0)).repeatForever().schedule()

        schedule(true) {
            val heading = robot.drive.poseEstimate.heading.radians
            val targetHeading = controller.targetPosition
            telem.addData("heading", heading)
                .addData("targetHeading", targetHeading)
            robot.drive.setDriveSignal(
                DriveSignal(
                    Pose2d(
                        heading = controller.update(heading).rad
                    )
                )
            )
        }
    }

}