import subprocess
from raven import Client
from .config import sentry_api

logger = Client(sentry_api)

def get_terminal_output(command):
    command_list = command.split(' ')
    output = None
    try:
        output = subprocess.check_output(command_list)
    except:
        logger.captureException()
    return output

def launch_terminal_command(command, new_terminal=False):
    if new_terminal:
        input_command = 'start ' + command
        try:
            p = subprocess.Popen(input_command, shell=True)
        except:
            logger.captureException()
    else:
        input_command = command.split(' ')
        try:
            p = subprocess.Popen(input_command)
        except:
            logger.captureException()
    return None
