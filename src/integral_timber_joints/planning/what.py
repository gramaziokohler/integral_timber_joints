import argparse

"""An interactive tool to explore a process file"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--design_dir', default='210419_HyparHut',
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--problem', default='shelter_process.json',  # twelve_pieces_process.json
                        help='The name of the problem to solve')
    parser.add_argument('--problem_subdir', default='results',
                        help='subdir of the process file. Popular use: `.` or `results`')

    args = parser.parse_args()
    print('Arguments:', args)


if __name__ == '__main__':
    main()

    # TODO Implementation
