from dataclasses import dataclass

@dataclass
class TestClass:
    name: str
    value: int = 0
    is_true: bool = False

def modify_test(t: TestClass):
    t.value = 5
    t.is_true = True

test = TestClass('test')
print(test)

modify_test(test)
print(test)