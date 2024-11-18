from enum import Enum
from typing import TypedDict


class GeneratorType(Enum):
    DEFAULT = 'default'
    RANDOM = 'random'
    GRID = 'grid'
    BAYESIAN = 'bayesian'


a = GeneratorType.DEFAULT
b = GeneratorType('default')

print(a)
print(b)

print(a == b)


class Trial(TypedDict):
    name: str
    map: str
    robot: str
    generator: GeneratorType
    runs: int


t = Trial(name='test', map='map', robot='robot', generator=GeneratorType.DEFAULT, runs=1)
print(t)

t.update({'new_param': 1000})
print(t)
