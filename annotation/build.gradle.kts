val kspVersion: String by project

plugins {
    kotlin("jvm")
}

group = "org.team9432.lib"

dependencies {
    implementation("com.squareup:kotlinpoet:1.14.2")
    implementation("com.squareup:kotlinpoet-ksp:1.14.2")
    implementation("com.google.devtools.ksp:symbol-processing-api:$kspVersion")

    implementation("org.littletonrobotics.akit.junction:junction-core:3.2.1")
}

repositories {
    maven {
        url = uri("https://frcmaven.wpi.edu/artifactory/release/")
    }
}
