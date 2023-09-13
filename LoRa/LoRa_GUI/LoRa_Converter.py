import re

# Used when sending multiple values in a string and to format them correctly
# Ex sending 13.5 and 2 -> "13[99]5[1337]2" -> 13.5 & 2
class LoRaConverter():

    def __init__(self, define_comma_value="99", define_new_value="1337") -> None:
        self.comma_value_identifier = define_comma_value
        self.new_value_identifier = define_new_value


    def parse_value(self, input_data):
        data = input_data
        
        if type(data) != str:
            data = str(data)

        parsed_value = ""
        for char in data:
            if char == ".":
                parsed_value = parsed_value + self.comma_value_identifier
            else:
                parsed_value = parsed_value + char
        
        parsed_value = parsed_value + self.new_value_identifier
        return parsed_value
    

    def unparse_value(self, data: str) -> list:
        seperated_values = data.split(self.new_value_identifier)

        for i in range(len(seperated_values)):
            seperated_values[i] = re.sub(self.comma_value_identifier, ".", seperated_values[i])

        return seperated_values


if __name__ == "__main__":
    parser = LoRaConverter()
    val1 = parser.parse_value("133.7")
    val2 = parser.parse_value("55.1")
    val3 = parser.parse_value("55")
    tot_val = val1 + val2 + val3 + "99"
    print(tot_val)
    print(parser.unparse_value(tot_val))
