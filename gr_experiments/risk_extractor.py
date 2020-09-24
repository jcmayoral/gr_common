#!/usr/bin/python
from risk_tools import RiskExtractor
import sys

if __name__ == "__main__":
    print sys.argv
    if len(sys.argv) != 2:
        print "add filename"
        sys.exit()
    RiskExtractor(file_name=sys.argv[1])
