# Andrea Ghiotto 2118418, David Petrovic 2092073
# Learning From Network project
# Implementation of the script necessary to modify datasets for the temporal locality-aware sampling algorithm

def spaceToTab(line):
    numbers = line.strip().split(' ')[:2]
    numbers_with_tab = '\t'.join(numbers)
    return numbers_with_tab + '\n'

def convertFile(input_file, output_file):
    with open(input_file, 'r') as file_input:
        lines_input = file_input.readlines()

    with open(output_file, 'w') as file_output:
        for line_input in lines_input:
            line_output = spaceToTab(line_input)
            file_output.write(line_output)

    print(f"Done! Check the file named {output_file}.")

if __name__ == "__main__":
  
    files_input = ['second.txt', 'third.txt', 'fifth.txt']

    for file_input in files_input:
        
        file_output = file_input.replace('.txt', '_output.txt')

        convertFile(file_input, file_output)