# 🌱 sprout - A small collision engine demo.
With _sprout 🌱_ objective is to find a compromise between performances and relatively physically realistic collisions detection and response.

### Collision detection
Currently _sprout 🌱_ performs collision detection in two steps :
  - a broad phase during which the collisions are detected using circles that encloses in the convex polygon shape
  - a narrow phase during which the collisions detected during the broad phase are examined to determine if the convex polygon shapes effectively collides and what is the projection vector to use to resolve the collision

The narrow phase is based on the separate-axis theorem.

### Collision response
Some adjustements are still needed in case of the collision between two moveable objects which currently causes them to bounce.

### TODO list
- [ ] Bounce
- [ ] Static friction
- [ ] Dynamic friction

_300 moving boxes with gravity inside a giant box made of 4 walls._
![A box made of 4 walls with 300 small boxes inside](https://raw.githubusercontent.com/Qu3tzal/sprout/master/sproutdemo.png)
