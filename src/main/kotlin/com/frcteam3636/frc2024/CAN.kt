package com.frcteam3636.frc2024

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

// This module contains one enum for each (device type, manufacturer) pair we use.

enum class REVMotorControllerId(val num: Int) {
    FrontLeftTurningMotor(5),
    BackLeftTurningMotor(6),
    BackRightTurningMotor(7),
    FrontRightTurningMotor(8),

    FrontLeftDrivingMotor(1),
    BackLeftDrivingMotor(2),
    BackRightDrivingMotor(3),
    FrontRightDrivingMotor(4),

    IndexerMotor(13),

    IntakeMotor(21),
}

fun CANSparkMax(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) =
    CANSparkMax(id.num, type)

fun CANSparkFlex(id: REVMotorControllerId, type: CANSparkLowLevel.MotorType) =
    CANSparkFlex(id.num, type)

enum class CTREMotorControllerId(val num: Int, val bus: String) {
    FrontLeftDrivingMotor(1, "*"),
    BackLeftDrivingMotor(2, "*"),
    BackRightDrivingMotor(3, "*"),
    FrontRightDrivingMotor(4, "*"),
    RightArmMotor(10, "*"),
    LeftArmMotor(11, "*"),
    WristMotor(12, "*"),
}

fun TalonFX(id: CTREMotorControllerId) = TalonFX(id.num, id.bus)