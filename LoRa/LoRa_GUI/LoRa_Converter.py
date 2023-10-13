import re

# Used when sending multiple values in a string and to format them correctly
# Ex sending 13.5 and 2 -> "13[99]5[1337]2" -> 13.5 & 2
def parse_hex_value(hex_str):
    # Extract the first two hex digits and convert to decimal
    id_part = int(hex_str[:2], 16)

    # Extract the next three hex digits and convert to an integer
    integer_part = int(hex_str[2:5], 16)

    # Extract the last hex digit and convert to an integer
    decimal_digit = int(hex_str[5:], 16)

    return id_part, integer_part, decimal_digit
    

