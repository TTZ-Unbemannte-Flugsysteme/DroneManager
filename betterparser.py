import argparse


class ArgumentParserError(Exception):
    pass


class ArgParser(argparse.ArgumentParser):
    def error(self, message):
        if "invalid choice" in message:
            raise ValueError(message)
        elif "arguments are required" in message:
            raise ValueError(message)
        elif "unrecognized argument" in message:
            raise ValueError(message)
        elif "invalid" in message:  # Likely an invalid argument, i.e. a string instead of float
            raise ValueError(message)
        else:
            raise ArgumentParserError(message)
