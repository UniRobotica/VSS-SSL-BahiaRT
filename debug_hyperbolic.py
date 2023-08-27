from debug import debug, measures
from random import randint
from strategy import potential_field

pot_f = potential_field.HyperbolicField(
    measures.centerPosition(),
    cw=True,
    env='simulation'
)

if __name__ == "__main__":
    
    debug.debug('Move To Goal', pot_f, None)