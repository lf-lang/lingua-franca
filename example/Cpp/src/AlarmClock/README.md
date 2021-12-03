# Lingua Franca AlarmClock

This is medium sized learning project which solves various interesting problems.

You can find the project here: [here]( https://github.com/revol-xut/lf-alarm-clock )

## Goal

The goal of this project is a working alarm clock using the scheduling and timing features of lingua franca.
The clock is designed to be run on a raspberry pi connected to speakers. You can interact with the clock over an http
api. 

## What you will find

- **Networking** how to interface with the outside world using `physical actions` and crow an http server for cpp.
- **Sharing State** When you have two reactors and want one reactor to access the state of the other.
- **Cancelation of scheduled Events** if you have events already scheduled for some tag but want to cancel the execution.

