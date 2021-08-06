# Car Brake Example

This example illustrates a deadline violation in a car brake that can be triggered either by the car's automatic braking assistant or the driver.
The hypothetical braking assistant would receive a frame from the car's camera and process it to identify potentially dangerous situations.
This processing takes about 10ms in our hypothetical example.
Upon identifying a condition requiring intervention, the braking mechanism triggers.

At the same time, the driver may brake at any given time.

## Actual result

Currently, the braking mechanism can miss its deadline when hitting the brake, indicating a delayed braking.

## Variant that fixes this

The CarBrake2.lf variant, if made federated, decouples the vision system from the handling of brake pedal actions
in a way that makes it impossible for the vision system to have any effect on the ability of the other component
to make deadlines. The price for this decoupling is added nondeterminacy because the physical connection
reassigns time stamps based on the current physical clock.
