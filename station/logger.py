# The purpose of this module is to define a set of functions that perform specific logging functions.
from datetime import *

def debug(message):
	with open('debug.log', 'a') as log:
		t = datetime.now();
		s = str(t) + "  >> " + str(message) + "\n";
		log.write(s);

	
