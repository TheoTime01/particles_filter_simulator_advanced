from enum import Enum

class ENUM_STATUS_UI(Enum):
    NEW_GOAL:str = "NEW_GOAL" 
    NAV_PENDING:str ="NAV_PENDING"
    CANCEL_GOAL:str ="CANCEL_GOAL"
    NO_GOAL:str ="NO_GOAL"