#!/usr/bin/env python3

import json
import sys

def flatten_json(nested_json):
    out = {}

    def flatten(x, name=''):
        if type(x) is dict:
            for a in x:
                flatten(x[a], name + a + '.')
        elif type(x) is list:
            i = 0
            for a in x:
                flatten(a, name + str(i) + '.')
                i += 1
        else:
            out[name[:-1]] = x

    flatten(nested_json)
    return out

data = json.load(open(sys.argv[1]))

print("#include <ODriveArduino.h>")
print("")

print("void configureOdrive( ODriveArduino& odrive ) {")
for key, value in flatten_json(data).items():
    if value == True:
        value = "true"
    elif value == False:
        value = "false"
    print("    odrive.setProperty(\"{}\", {});".format(key, value))
print("}")