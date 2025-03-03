from enum import Enum, auto
from collections import namedtuple

class Lane(Enum):
    ONE = auto()
    TWO = auto()
    
class StopPlace(Enum):
    STOP_LINE = auto()
    SPEED_BUMP = auto()

class Constant:
    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise Exception('변수에 값을 할당할 수 없습니다.')
        self.__dict__[name] = value

    def __delattr__(self, name):
        if name in self.__dict__:
            raise Exception('변수를 삭제할 수 없습니다.')
        
const = Constant()
const.STOP_PLACE_POS = [
    (935600.6919080958, 1915973.5190349538, StopPlace.STOP_LINE), 
    (935652.7680651817, 1916083.9379778637, StopPlace.STOP_LINE), 
    (935648.7732787153, 1916216.714338656,  StopPlace.STOP_LINE)
]

# 이 값은 optimal_velocity 에서 가져다 쓰는 함수에 따라 다름
const.STOP_THRESHOLD_DIST = 10.0
