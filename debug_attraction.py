from debug import debug, measures
from random import randint
from algorithms import univector_field

pot_f = univector_field.AttractionField(
    measures.centerPosition()
)

if __name__ == "__main__":
    
    debug.debug('Attraction Field', pot_f, None)