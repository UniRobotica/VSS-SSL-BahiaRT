from debug import debug, measures
from random import randint
from algorithms import univector_field

pot_f = univector_field.HyperbolicField(
    measures.centerPosition(),
    cw=True,
    env='simulation'
)

if __name__ == "__main__":
    
    debug.debug('Hyperbolic Field', pot_f, None)