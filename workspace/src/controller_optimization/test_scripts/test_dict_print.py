import json

def pretty_print_dict(d, level=2):
    """ Pretty print dictionary with a specified depth. """
    def _pretty_print(d, level, current_level=0):
        if current_level > level:
            return d
        if isinstance(d, dict):
            return {
                k: _pretty_print(v, level, current_level + 1)
                for k, v in d.items()
            }
        elif isinstance(d, list):
            return [_pretty_print(i, level, current_level + 1) for i in d]
        else:
            return d

    return json.dumps(_pretty_print(d, level), indent=4)


import logging

# Configure the logger
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Example dictionary
data = {
    'name': 'John Doe',
    'age': 30,
    'languages': ['Python', 'C++', 'JavaScript'],
    'address': {
        'street': '123 Main St',
        'city': 'Anytown',
        'zip': '12345',
        'additional_info': {
            'apartment': 'Apt 101',
            'landmark': 'Near the park'
        }
    }
}

# Log the dictionary with pretty printing and depth control
logging.info("Pretty-printed dictionary with depth 2:\n%s", pretty_print_dict(data, level=2))
