pluginManagement {
    val kotlinVersion: String by settings
    val kspVersion: String by settings
    val frcYear: String by settings

    plugins {
        id("com.google.devtools.ksp") version kspVersion
        kotlin("jvm") version kotlinVersion
    }

    repositories {
        mavenLocal()
        gradlePluginPortal()

        val frcHome = if (System.getProperty("os.name").lowercase().contains("windows")) {
            file(System.getenv("PUBLIC") ?: "C:\\Users\\Public")
        } else {
            file(System.getProperty("user.home"))
        }
            .resolve("wpilib")
            .resolve(frcYear)

        maven {
            name = "frcHome"
            url = uri(frcHome.resolve("maven"))
        }
    }
}

include("annotation")
