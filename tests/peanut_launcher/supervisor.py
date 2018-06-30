import inspect
import threading
from raven import Client
from utils.config import sentry_api

class MultiThreadSupervisor():
    def __init__(self):
        self.launcher_dict = {}
        self.report_dict = {}
        self.logger = Client(sentry_api)

    def add_launcher(self, launcher_id, launcher):
        if not inspect.ismethod(launcher.launch):
            raise ValueError('Your launcher does not have method launch')
        if launcher_id in self.launcher_dict:
            print('Warning: ', launcher_id, ' exist')
        self.launcher_dict[launcher_id] = launcher
        self.report_dict[launcher_id] = {'status': 'not_launched', 'detail': 'N/A'}

    def exec_launcher(self, launcher_id):
        report = {'status': 'not_launched', 'detail': 'N/A'}
        if launcher_id not in self.launcher_dict:
            raise ValueError('launcher ' + launcher_id + ' not exist')
        launcher = self.launcher_dict[launcher_id]
        try:
            launch_report = launcher.launch()
            if 'status' not in launch_report:
                raise ValueError('launch report must contain status: success or fail')
            if 'detail' not in launch_report:
                raise ValueError('launch report must contain detail: string')
            report['status'] = launch_report['status']
            report['detail'] = launch_report['detail']
        except ValueError as e:
            report['status'] = 'fail'
            report['detail'] = str(e)
        except:
            report['status'] = 'fail'
            report['detail'] = 'unknown error'
            self.logger.captureException()
        self.report_dict[launcher_id] = report

    def launch_all(self):
        t_list = []
        for launcher_id in self.launcher_dict:
            t = threading.Thread(target=self.exec_launcher, args=[launcher_id])
            t_list.append(t)
        for t in t_list:
            t.start()
        for t in t_list:
            t.join()
        fail_cnt = 0
        noop_cnt = 0
        success_cnt = 0
        total_cnt = len(self.launcher_dict)
        for launcher_id, report in self.report_dict.items():
            if report['status'] == 'fail':
                fail_cnt = fail_cnt + 1
            if report['status'] == 'success':
                success_cnt = success_cnt + 1
            if report['status'] == 'not_launched':
                noop_cnt = noop_cnt + 1
        if fail_cnt == 0 and noop_cnt == 0 and success_cnt == total_cnt:
            print('All launching finished')
            return 'success', None
        else:
            print('Warning: launching not finished')
            print('success: ', success_cnt)
            print('fail: ', fail_cnt)
            print('not launched: ', noop_cnt)
            return 'fail', self.report_dict
