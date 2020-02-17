[![](https://jitpack.io/v/net.thefletcher/SparkMaxSim-java.svg)](https://jitpack.io/#net.thefletcher/SparkMaxSim-java)

# SparkMaxSim

This is a redistribution of the SparkMax-java library provided by [REV Robotics](https://revrobotics.com) for the [FIRST Robotics Competition](https://firstinspires.org/robotics/frc).

This version adds support for the Desktop Simulation features of [WPILib](https://wpilib.org) and also adds support for the Sendable APIs.

## Adding the dependency

SparkMaxSim is dependent on components from the `"vendordep"` [provided by REV Robotics](http://www.revrobotics.com/sparkmax-software/#java-api) therefore you must ensure that vendor library is installed first.

Once done, you can add SparkMaxSim to your robot project as a dependency with only a couple lines to your project’s build.gradle.

SparkMaxSim is distributed using jitpack, so to add the dependency, you need the following gradle repository entry:

```gradle
repositories {
  maven { url 'https://jitpack.io' }
}
```

Now that you've told gradle *how* to find the package, we need to tell it *what* package to use.  To do this, just add the following to the dependencies list:

```gradle
implementation "net.thefletcher:SparkMaxSim-java:RELEASE_TAG"
```
where RELEASE_TAG is the release version listed in your project's `vendordeps/revrobotics.json` file (e.g. 1.5.1).

## Using SparkMaxSim

Usage is simple.  All of the classes are effectively copy/pasted and modified versions of what REV provides, so the class names remained the same.  To avoid name collisions, the classes live under the `net.thefletcher.revrobotics` package instead.

For example, to instantiate a CANSparkMax object, you would use `import net.thefletcher.revrobotics.CANSparkMax;` instead of `import com.revrobotics.CANSparkMax;`

## Caveats

1. I've made a few "housekeeping" changes as well, including moving all of the enum types from inner classes to discrete classes under the `net.thefletcher.revrobotics.enums` package.  They behave identically, so you should have no issue using them, other than having to add/change a few `import` statements.
1. Due to how REV has built the APIs and how the simulation functionality works, you may run into a few oddities when using this library.
  1. `CANSparkMax::getEncoder()` will create a `new` CANEncoder object every time it's called.  This breaks the simulation functionality and will lead to console errors and/or unexpected behavior ONLY in simulation mode.
  1. This applies to `getAlternateEncoder()`, `getAnalog()`, `getPIDController`, `getForwardLimitSwitch()` and `getReverseLimitSwitch()` as well
1. The `Sendable` functionality is very much at the [MVP](https://en.wikipedia.org/wiki/Minimum_viable_product) stage, so it may not show everything properly.  Please send bug reports/feature requests as appropriate.
1. The sensor objects do not update input values in Sim mode.  If you think about it, this makes perfect sense given that there's no actual hardware to measure.  If someone has a good solution to this, I'd love to hear it.

## Motivation

You're probably wondering why bother with this. Why put the effort in? Why not simply extend the classes?

Well, that's a complicated answer that starts simple.  Basically, I really like the new Sim GUI, but don't like that none of the vendor libraries currently support it.  It seemed easy enough, just extend the classes, override the handful of methods required, and go about my day, right? Wrong.

Unfortunately, REV made a some design decisions that caused me a few headaches.
1. This library is really just a java-specific wrapper that delegates to JNI calls to do all the heavy-lifting.  This means I have very little insight into what the actual logic is.  I could think of a few reasons why this was done, so no more than a minor inconvenience to me.
1. There are a couple instances of private variables that get instantiated in the constructor of a class that is extended by another class.  This means I either call `super()` or let them remain null.  Given the presence of a JNI call in that super constructor, I can't call it without getting HAL/CAN errors in the console; and the variables are used elsewhere, so I can't leave them uninstantiated.  This was the biggest reason for the copy/paste approach.
1. As mentioned above, nearly every single method is a JNI-wrapper, so I would need to extend EVERY method.
