#!/usr/bin/python
from risk_tools import RiskExtractor
import sys

if __name__ == "__main__":
    print sys.argv
    if len(sys.argv) != 3:
        print "args filename[string]  service_required[bool]"
        sys.exit()
    flag =  eval(sys.argv[2])
    print (flag)
    RiskExtractor(file_name=sys.argv[1], service_required =flag)
