"""
SYS ID JSON Parser

This script will:

- Read SYS ID JSON file
- Replace 0.0 right position value with valid left position value
- Replace 0.0 right velocity value with valid right velocity value
"""


import json



# Change these variables to change the JSON file
# ------------------------------

DIRECTORY = 'Characterization'

# JSON data input
JSON_DATA = f'{DIRECTORY}/sysid_data20220224-153404.json'

# JSON data output
JSON_OUTPUT = f'{DIRECTORY}/new_data.json'

# ------------------------------



# Get the existing JSON data as a Python dictionary
with open(JSON_DATA, 'r') as f:
    file_dict = json.load(f)

# For each key in the JSON
for data in file_dict:

    # If it is a list
    if type(file_dict[data]) == list:

        for sys_id in file_dict[data]:

            # Replace right position with left position
            left_position = sys_id[3]
            sys_id[4] = left_position

            # Replace right velocity with left velocity
            left_velocity = sys_id[5]
            sys_id[6] = left_velocity


# Write the new JSON data
with open(JSON_OUTPUT, 'w') as f:
    json.dump(file_dict, f)