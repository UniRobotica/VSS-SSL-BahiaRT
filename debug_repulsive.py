from debug import debug, measures
from random import randint
from strategy import univector_field

pot_f = univector_field.RepulsivePointField(
    measures.centerPosition(),
    env='simulation'
)

if __name__ == "__main__":
    
    debug.debug('Repulsive Field', pot_f, None)