# targetLead
inspired by silent hunter u-boats

Takes inputs through GUI:
  distance to target(m)
  bearing to target (degrees from positive x axis)
  target heading    (degrees from positive x axis)
  target speed      (m/s)
 
 calculates lead infront of the target assuming the projectile and target have a constant velocity (speed and direction)
 result is given as an angle in degrees from the positive x axis.
 
 
 It uses an equation in the form of
  f(t) = s^2 - (d.x/t + v.x)^2 - (d.y/t + v.y)^2
  and is solved for t using newtons method (finding the root(s) of the equation)
