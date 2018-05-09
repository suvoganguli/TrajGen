import setting
import subfile

setting.init()          # Call only once
subfile.stuff()         # Do stuff with global var
print setting.myList[0] # Check the result