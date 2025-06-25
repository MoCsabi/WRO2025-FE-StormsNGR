# Ideas and principles
As our second year competing in Future Engineers, this was easily the hardest, most complicated project we have ever completed, and we are very happy with the final results. We like to think this was possible due to our core principles, and some of our great ideas. Here's a non-exhaustive list or our principles and ideas that we believe may be helpful for anyone working on something similar to us.

## Principles
- **Reliability first**
- **Keep it simple**
- **Trigonometry is your friend**
- **Don't give up :)**
## Ideas
- In the open challenge we can just go around in the outer lane. The path won't be the most optimized, but this makes solving the challenge so much simpler
- In the obstacle challenge we can break down the mat into 4 straight sections and 90° turns. The movement of the robot can also be simplified  into a handful of scenarios
- Small robot --> better turning and easier parking. Height isn't a problem as long as the center of gravity is low
- The 360° 2D LiDAR is an amazing sensor. If provided with a wide enough FOV it can be used to measure the distance from the side walls, or detect the precise position of traffic signs. It gives us more than enough information to always know where the robot is
- Reliable gyro - the basis for movement. Going straight for more than 1 meter and only deviating from the path 1-2 centimeters while this accuracy is constant through all 3 laps is very useful, and we rely on this accuracy a lot in our solutions
- Color detection. We struggled a lot with this. As a general principle, going closer to the traffic signs to detect its color is a good idea.
- EMI. If you can't find what's causing and issue, look into electro magnetic interference. Look out for cables carrying analog or data signals running parallel to cables carrying high frequency high amperes signals such as motor's DC cables.
- Going faster isn't as simple as increasing the speed. The whole dynamics of the robot is likely going to be different, such as turning and stopping, at faster speed, due to factors like slippage. These all have to be accounted for manually.
- Environmental light disturbance during color detection can be minimized by using our own light source. This of course only works if we are close enough to the object. Ideally the angle of the light should be the same as the angle of the camera and definitely not perpendicular to the object, since then the reflection could blind the camera
- Using a default color when color detection fails. So if the camera doesn't detect **any** colors we just assume the default color, which should be set to the color the camera is more likely to not recognize. This doesn't mean we don't try to detect both colors, but it does improve detection accuracy percentage at the cost of nothing, we only have to perfect one color's detection
- Lego is a solid solution. It becomes especially useful when mixed with third party solutions, such as metal lego compatible parts, carbon fiber axles, ball bearings and 3D printing, or even glue, while still being easy to assemble. This is especially useful if you are (like us) not that familiar with CNC machines or welding
- A good framework is extremely worth it. This is something we carried on from our past RoboMission experiences. A solid framework can make solving the challenges much easier and trivializes implementing next year's small rule change

## Conclusion
We believe these principles are well reflected in our various solutions to challenges that arose during the preparations for the world championship. We truly hope these can be helpful for someone.