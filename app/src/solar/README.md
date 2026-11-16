# Solar System module

The rocket asks the Solar System where things are through `SolarSystem` in `app/src/shared/SolarSystem.h`. `CircularSolarSystem` answers it with simple circles for now; the real orbits go there.

It needs to answer:

- `positionAt(body, time)`: where a planet is at a time
- `velocityAt(body, time)`: how fast it's moving then
- `dominantBody(position, time)`: whose gravity a spot is in

Positions are metres from the Sun, times are seconds since launch, and sizes are one tenth of real (`addBody` already scales them). Work from a formula of time rather than stepping forces each frame, so any time can be asked for.