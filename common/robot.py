
from common.movable_entity import MovableEntity
class Robot(MovableEntity):

    def __str__(self) -> str:
        return "R[%s]: x:%f, y:%f, theta:%f"%(self.id,self.x,self.y,self.theta)
    

