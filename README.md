# RAS 598 - mobile-robotics: Team Palantir

This repository is for the final project of the team **Palantir** for the course RAS598 Mobile Robotics. 
our mission is to build a two-robot predator-prey game using TurtleBot4 in a bounded indoor arena. 
One robot acts as a **wolf** patrolling a designated territory, the other as a 
**rabbit** searching the arena for randomly generated AR carrots. Both robots 
use **YOLO-based visual detection** to recognize each other and carrot targets.

When the rabbit detects the wolf, it turns 180° and flees — changing direction 
at the territory boundary until it escapes. The wolf chases the rabbit when 
detected but stops at its boundary. The game ends when the wolf closes within 
**0.4 m** of the rabbit.

## Team Members
- Aldrick peter Thomas 
- Shao-Chi Cheng (Brian)
- Chach Chaimongkol