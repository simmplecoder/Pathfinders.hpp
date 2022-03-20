import json
import click
import os


@click.command()
@click.option('--input', type=click.Path(dir_okay=False, file_okay=True), help='input json file to analyze')
@click.option('--output', default='analysis.txt', help='path to output file to write data into')
def analyze(input, output):
    with open(input, 'r') as f:
        contents = json.load(f)
        total_time = int(contents['total_time_ns'])
        nanoseconds_in_second = 1000000000
        print(f'total time is {total_time / nanoseconds_in_second} seconds')


if __name__ == '__main__':
    analyze()
