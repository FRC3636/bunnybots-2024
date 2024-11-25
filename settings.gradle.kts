import org.gradle.internal.os.OperatingSystem

pluginManagement {
    repositories {
        mavenLocal()
        gradlePluginPortal()

        val frcYear: String by settings

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
