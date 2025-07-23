import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--input_string', type=str, required=True)
args = parser.parse_args()
print(args.input_string)
# to run code from terminal -> python print_input.py --input_string test