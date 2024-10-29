"""Running config file is necessary to import heat package. Currently, heat package is not installed in the interpreter."""

import config
import heat as ht
#print(heat.__file__)  # This will show where Python is importing `heat` from

print(ht.arange(10))