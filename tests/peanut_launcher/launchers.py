import abc, six, time
from utils.val_utils import is_valid_output_checker
from utils.tm_utils import get_terminal_output, launch_terminal_command

@six.add_metaclass(abc.ABCMeta)
class BaseLauncher():
    @abc.abstractmethod
    def launch(self):
        pass

@six.add_metaclass(abc.ABCMeta)
class BlockingLauncher(BaseLauncher):
    def __init__(self, interval=0.1, retry=0):
        self.interval = interval
        self.retry = retry

    @abc.abstractmethod
    def launch_finished(self):
        pass

    @abc.abstractmethod
    def launch_process(self):
        pass

    def launch(self):
        if self.launch_finished():
            return {'status': 'success', 'detail': 'N/A'}
        self.launch_process()
        retry_cnt = 0
        while not self.launch_finished():
            time.sleep(self.interval)
            retry_cnt = retry_cnt + 1
            if self.retry != 0:
                if retry_cnt > self.retry:
                    return {'status': 'fail', 'detail': 'retry limit reached'}
        return {'status': 'success', 'detail': 'N/A'}

class CommandLauncher(BlockingLauncher):
    def __init__(self, launch_command, check_command, output_checker,
                        new_terminal=False, interval=0.1, retry=0):
        super(CommandLauncher, self).__init__(interval, retry)
        if not is_valid_output_checker(output_checker):
            raise ValueError('output_checker should take in a string and output a bool')
        self.launch_command = launch_command
        self.check_command = check_command
        self.output_checker = output_checker
        self.new_terminal = new_terminal

    def launch_finished(self):
        output = get_terminal_output(self.check_command)
        return self.output_checker(output)

    def launch_process(self):
        res = launch_terminal_command(self.launch_command, self.new_terminal)
