package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.CommandScheduler.gamepad
import com.amarcolini.joos.gamepad.GamepadEx
import com.amarcolini.joos.gamepad.MultipleGamepad
import com.amarcolini.joos.gamepad.Toggleable
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.function.BooleanSupplier
import kotlin.math.pow
import kotlin.reflect.KProperty0

@TeleOp(name = "DriveControl", group = "Official")
class DriveControl : CommandOpMode() {
    private val robot by robot<CSRobot>()

    override fun preInit() {
        robot.outtake.reset()

        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()

            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x
                )
            )

            val leftTrigger = gamepad.p1.left_trigger.value
            val rightTrigger = gamepad.p1.right_trigger.value

            robot.verticalExtension.motors.setPower(
                (rightTrigger - leftTrigger).pow(3).toDouble() * 1.0
            )
        }

        map(gamepad.p1 { (y or triangle)::justActivated }, Command.of {
            if (robot.outtake.isExtended) robot.outtake.resetArm()
            else robot.outtake.extend()
        })

        map(gamepad.p1.left_bumper::justActivated, robot.outtake::releaseLeft)
        map(gamepad.p1.right_bumper::justActivated, robot.outtake::releaseRight)

        map(gamepad.p1 { (b or circle)::justActivated }, robot.outtake::ready)

        map(gamepad.p1.dpad_up::justActivated, robot.droneLauncher::launch)
        map(gamepad.p1.dpad_down::justActivated, robot.droneLauncher::reset)
    }
}