import re

def remove_chars(input_str):
    new_str = ""
    for char in input_str:
        char_ascii_value = ord(char)
        if 48 <= char_ascii_value <= 57:
            new_str += char
    return new_str

def extract_data_after_5th_comma(input_str):
    comma_count = 0
    start_index = 0

    for i, char in enumerate(input_str):
        if char == ',':
            comma_count += 1
            if comma_count == 5:
                start_index = i + 1
                break

    extracted_data = input_str[start_index:]
    return extracted_data

def hex_string_to_int(hex_value):
    if len(hex_value) != 6:
        return -1  # Return an error code

    id_hex = hex_value[0:2]
    int_hex = hex_value[2:5]
    decimal_hex = hex_value[5:]

    id = int(id_hex, 16)
    int_value = int(int_hex, 16)
    decimal_value = int(decimal_hex, 16)

    result = (id << 16) | (int_value << 8) | decimal_value
    return result
