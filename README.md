# Ωhmware
kernel and operating system for ohmbre eurorack synth hardware

### Building

```
git checkout https://github.com/ohmbre/ohmware.git
cd ohmware
source setupenv
```
make sure you've attached a 4GB+ microSD card and that it's not mounted already, and run:
```
sdformat /dev/<SD writing device name>
ohmsweetohm
```

Congrats if that took less than two hours.  Now that you've built your own operating system, insert the card 
into the ohmbre and it should boot up to a bash shell. If you'd like some synthesizer with your computer, 
head on over [this way](https://github.com/ohmbre/ohmstudio) and build the rest.
