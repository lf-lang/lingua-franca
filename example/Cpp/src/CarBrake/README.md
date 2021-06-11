# Car Brake Example

This example illustrates a deadline violation in a car brake that can be triggered either by the car's automatic braking assistant or the driver.
The hypothetical braking assistant would receive a frame from the car's camera and process it to identify potentially dangerous situations.
This processing takes about 10ms in our hypothetical example.
Upon identifying a condition requiring intervention, the braking mechanism triggers.

At the same time, the driver may brake at any given time.

## Actual result

Currently, the braking mechanism can miss its deadline when hitting the brake, indicating a delayed braking.

