# Piano
This is a pygame-based piano demo for the Python target of Lingua Franca. 
The contents of the demo is mostly adapted from: https://github.com/bspaans/python-mingus/tree/master/mingus_examples/pygame-piano

## Installation
### MacOs:
```bash
$ brew install sdl2 sdl2_gfx sdl2_image sdl2_mixer sdl2_net sdl2_ttf
$ brew install Caskroom/cask/xquartz
$ brew install fluid-synth
$ python3 -m pip install mingus
$ python3 -m pip install pygame
```
Then, go to ```System Preferences -> Security & Privacy -> Privacy -> Input Monitoring``` and check Terminal so that the Terminal application can read keystrokes.

### Linux:
```bash
 $ sudo apt install libsdl2-2* libsdl2-image-2*
 $ sudo apt install fluidsynth
 $ pip3 install mingus
 $ pip3 install pygame
```


### Windows: