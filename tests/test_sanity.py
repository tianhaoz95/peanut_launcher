import inspect
import pytest
import time
import errno
import os
import signal
from functools import wraps
from peanut_launcher import supervisor
from peanut_launcher import launchers

class TimeoutError(Exception):
    pass

def timeout(seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)

        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(seconds)
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result

        return wraps(func)(wrapper)

    return decorator

class Launcher():
    def launch(self):
        return {'status': 'success', 'detail': 'test'}

class FailLauncher():
    def launch(self):
        return {'status': 'fail', 'detail': 'test'}

def test_init_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()

def test_add_launcher_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    test_launcher = Launcher()
    s.add_launcher('test_launcher', test_launcher)

def test_exec_launcher_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    test_launcher = Launcher()
    s.add_launcher('test_launcher', test_launcher)
    s.exec_launcher('test_launcher')

def test_launch_all_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    l1 = Launcher()
    l2 = Launcher()
    l3 = Launcher()
    s.add_launcher('l1', l1)
    s.add_launcher('l2', l2)
    s.add_launcher('l3', l3)
    status, info = s.launch_all()

def test_launch_all_success_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    l1 = Launcher()
    l2 = Launcher()
    l3 = Launcher()
    s.add_launcher('l1', l1)
    s.add_launcher('l2', l2)
    s.add_launcher('l3', l3)
    status, info = s.launch_all()
    assert(status == 'success')

def test_launch_all_fail_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    l1 = Launcher()
    l2 = Launcher()
    l3 = FailLauncher()
    s.add_launcher('l1', l1)
    s.add_launcher('l2', l2)
    s.add_launcher('l3', l3)
    status, info = s.launch_all()
    assert(status == 'fail')

def test_init_fail_base_launcher():
    with pytest.raises(Exception):
        bl = launchers.BaseLauncher()

class BaseLauncher(launchers.BaseLauncher):
    def launch(self):
        return {'status': 'success', 'detail': 'N/A'}

class BaseLauncherFail(launchers.BaseLauncher):
    def launch(self):
        return {'status': 'fail', 'detail': 'N/A'}

def test_init_success_base_launcher():
    tbl = BaseLauncher()

def test_base_launcher_success_with_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    tbl1 = BaseLauncher()
    tbl2 = BaseLauncher()
    tbl3 = BaseLauncher()
    s.add_launcher('tbl1', tbl1)
    s.add_launcher('tbl2', tbl2)
    s.add_launcher('tbl3', tbl3)
    status, info = s.launch_all()
    assert(status == 'success')

def test_base_launcher_fail_with_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    tbl1 = BaseLauncher()
    tbl2 = BaseLauncher()
    tbl3 = BaseLauncherFail()
    s.add_launcher('tbl1', tbl1)
    s.add_launcher('tbl2', tbl2)
    s.add_launcher('tbl3', tbl3)
    status, info = s.launch_all()
    assert(status == 'fail')

def test_init_fail_blocking_launcher():
    with pytest.raises(Exception):
        bl = launchers.BlockingLauncher()

class BlockingLauncher(launchers.BlockingLauncher):
    def launch_finished(self):
        return True

    def launch_process(self):
        print('launching')

class BlockingLauncherInitCheck(launchers.BlockingLauncher):
    def launch_finished(self):
        return True

    def launch_process(self):
        while True:
            time.sleep(1)

class BlockingLauncherDelay(launchers.BlockingLauncher):
    def __init__(self):
        super(BlockingLauncherDelay, self).__init__()
        self.cnt = 0

    def launch_finished(self):
        if self.cnt == 5:
            return True
        else:
            time.sleep(0.1)
            self.cnt = self.cnt + 1

    def launch_process(self):
        print('launching')

def test_init_success_blocking_launcher():
    tbl = BlockingLauncher()

def test_launch_blocking_launcher():
    tbl = BlockingLauncher()
    tbl.launch()

@timeout(3)
def test_init_check_blocking_launcher():
    tbl = BlockingLauncherInitCheck()
    tbl.launch()

@timeout(3)
def test_async_launching_blocking_launcher():
    tbl = BlockingLauncherDelay()
    tbl.launch()
    assert(tbl.cnt == 5)

@timeout(5)
def test_async_launching_with_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    tbl1 = BlockingLauncherDelay()
    tbl2 = BlockingLauncherDelay()
    tbl3 = BlockingLauncherDelay()
    tbl4 = BlockingLauncherDelay()
    tbl5 = BlockingLauncherDelay()
    tbl6 = BlockingLauncherDelay()
    tbl7 = BlockingLauncherDelay()
    tbl8 = BlockingLauncherDelay()
    tbl9 = BlockingLauncherDelay()
    s.add_launcher('tbl1', tbl1)
    s.add_launcher('tbl2', tbl2)
    s.add_launcher('tbl3', tbl3)
    s.add_launcher('tbl4', tbl4)
    s.add_launcher('tbl5', tbl5)
    s.add_launcher('tbl6', tbl6)
    s.add_launcher('tbl7', tbl7)
    s.add_launcher('tbl8', tbl8)
    s.add_launcher('tbl9', tbl9)
    status, info = s.launch_all()
    assert(tbl1.cnt == 5)
    assert(tbl2.cnt == 5)
    assert(tbl3.cnt == 5)

@timeout(3)
def test_async_launching_blocking_launcher_stress():
    s = supervisor.MultiThreadSupervisor()
    tbl_list = []
    for i in range(500):
        tbl = BlockingLauncherDelay()
        tbl_list.append(tbl)
        s.add_launcher('tbl' + str(i), tbl)
    status, info = s.launch_all()
    for tbl in tbl_list:
        assert(tbl.cnt == 5)

def ls_output_checker(output):
    return True

@timeout(3)
def test_dummy_ls_command_launcher():
    tcl = launchers.CommandLauncher('ls', 'ls', ls_output_checker)
    tcl.launch()

@timeout(3)
def test_dummy_ls_command_launcher_with_multi_thread_supervisor():
    s = supervisor.MultiThreadSupervisor()
    tcl1 = launchers.CommandLauncher('ls', 'ls', ls_output_checker)
    tcl2 = launchers.CommandLauncher('ls', 'ls', ls_output_checker)
    tcl3 = launchers.CommandLauncher('ls', 'ls', ls_output_checker)
    s.add_launcher('tcl1', tcl1)
    s.add_launcher('tcl2', tcl2)
    s.add_launcher('tcl3', tcl3)
    status, info = s.launch_all()
