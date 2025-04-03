
from enum import Enum

from template_observer.dead_reconing import DeadReconing
from template_observer.luenberger import Luenberger


class Observers(str, Enum):
    LUENBERGER = Luenberger
    DEAD_RECONING = DeadReconing
