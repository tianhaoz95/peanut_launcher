import subprocess

def get_terminal_output(command):
    command_list = command.split(' ')
    output = subprocess.check_output(command_list)
    return output

def launch_terminal_command(command, new_terminal=False):
    if new_terminal:
        input_command = 'start ' + command
        p = subprocess.Popen(input_command, shell=True)
    else:
        input_command = command.split(' ')
        p = subprocess.Popen(input_command)
    return None
