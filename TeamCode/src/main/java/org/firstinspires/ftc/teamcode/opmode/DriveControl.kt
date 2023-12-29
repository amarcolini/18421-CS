package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import kotlin.math.pow

@JoosConfig
@TeleOp(name = "DriveControl", group = "Official")
class DriveControl : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var intakePosition = 0.0
    }

    override fun preInit() {
        robot.outtake.reset()
        robot.pixelPlopper.prime()

        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()

            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x.rad
                )
            )

            val leftTrigger = gamepad.p1.left_trigger.value
            val rightTrigger = gamepad.p1.right_trigger.value
            robot.verticalExtension.motors.setPower(
                (rightTrigger - leftTrigger).pow(3).toDouble() * 1.0 + 0.1
            )

            robot.intake.servo.position = intakePosition
        }

        map(gamepad.p1 { (y or triangle)::justActivated }, Command.of {
            if (robot.outtake.isExtended) robot.outtake.resetArm()
            else robot.outtake.extend()
        })

        map(gamepad.p1.left_bumper::justActivated, robot.outtake::releaseLeft)
        map(gamepad.p1.right_bumper::justActivated, robot.outtake::releaseRight)

        map(gamepad.p1 { (b or circle)::justActivated }, robot.outtake::ready)

        map(gamepad.p1 { (a or cross)::justActivated }) {
            robot.intake.motorState = when (robot.intake.motorState) {
                Intake.MotorState.ACTIVE -> Intake.MotorState.STOP
                else -> Intake.MotorState.ACTIVE
            }
        }

        map(gamepad.p1.dpad_up::justActivated, robot.droneLauncher::launch)
        map(gamepad.p1.dpad_down::justActivated, robot.droneLauncher::reset)

        map(gamepad.p1.dpad_left::justActivated) {
            if (robot.pixelPlopper.isPrimed) robot.pixelPlopper.open()
            else robot.pixelPlopper.prime()
        }
    }
}