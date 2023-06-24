#import os
#os.system('cmd /c "python"')
#os.system('cmd /c "pip --version"')

#import os
#os.system('dir c:\\')
'''
import subprocess

cmd = "cmd"
print("yes")

subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
'''

import os
returned_value = os.system("start cmd /k pip list && pip list")
#os.system("start cmd /k cd .. $$ cmd /k pip list")
print("yes")


