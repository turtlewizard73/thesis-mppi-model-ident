import os
from util_functions import reformat_yaml

input_file = os.path.join(
    os.path.dirname(__file__),
    'controller_server.yaml'
)

output_file = os.path.join(
    os.path.dirname(__file__),
    'controller_server_reformatted2.yaml'
)

reformat_yaml(input_file, output_file)
