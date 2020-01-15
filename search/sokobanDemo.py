import os #for time functions

from search import * #for search engines

from sokoban import * #for Sokoban specific classes and problems

#######################################################

counter = 0

for i in PROBLEMS:
    print("===>Sokoban Problem #:",counter)
    print(SokobanState.state_string(i))
    counter = counter +1



a=PROBLEMS[3]
print(SokobanState.state_string(a))
print("Width,x =", a.width)
print("Height,y =", a.height)
