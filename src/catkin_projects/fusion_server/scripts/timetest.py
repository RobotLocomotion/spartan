import datetime
a = datetime.datetime.now()
string = "%s:%s.%s" % (a.hour, a.minute, a.second)
print string

now = datetime.datetime.now()
print "%0.2d:%0.2d:%0.2d" % (now.hour, now.minute, now.second)

string =  "%0.4d-%0.2d-%0.2d-%0.2d-%0.2d-%0.2d" % (now.year, now.month, now.day, now.hour, now.minute, now.second)

print string

import time
unique_name = time.strftime("%Y%m%d-%H%M%S")
print unique_name