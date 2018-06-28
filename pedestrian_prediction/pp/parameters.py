import sys

# XXX: Figure this crazy import mess another day.
# I'm guessing this is because hardmax.occupancy
# which is part of inference.hardmax, is importing
# this module.
from .mdp import euclid
from .mdp import hardmax

if 'pp.mdp.euclid' in sys.modules:
    val_euclid = sys.modules['pp.mdp.euclid']
else:
    val_euclid = sys.modules['pedestrian_prediction.pp.mdp.euclid']

if 'pp.mdp.hardmax' in sys.modules:
    val_hardmax = sys.modules['pp.mdp.hardmax']
else:
    val_hardmax = sys.modules['pedestrian_prediction.pp.mdp.hardmax']

val_default = val_hardmax

#from .inference import hardmax
import inference
if 'pp.inference.hardmax' in sys.modules:
    inf_default = sys.modules['pp.inference.hardmax']
else:
    inf_default = sys.modules['pedestrian_prediction.pp.inference.hardmax']
