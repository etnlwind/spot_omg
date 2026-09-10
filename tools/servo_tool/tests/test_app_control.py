import unittest
from unittest.mock import patch, Mock
from servo import app_control
from servo.cli import _land_before_update
from servo.console import ConsoleResponse, ConsoleError

class AppControlTests(unittest.TestCase):
    def test_lease_restores_connected_robot(self):
        with patch.object(app_control, 'request', side_effect=[{'ready': True, 'target':'robot'}, {'ready':False}, {'ready':True}]) as request:
            with app_control.robot_ble_lease('phone'):
                self.assertEqual(request.call_count, 2)
            self.assertEqual([c.args[1] for c in request.call_args_list], ['status','disconnect','connect'])

    def test_failed_operation_does_not_reconnect(self):
        with patch.object(app_control, 'request', side_effect=[{'ready':True,'target':'robot'}, {'ready':False}]) as request:
            with self.assertRaises(RuntimeError):
                with app_control.robot_ble_lease('phone'):
                    raise RuntimeError('upload failed')
            self.assertEqual(request.call_count,2)

    def test_disconnected_app_stays_disconnected(self):
        with patch.object(app_control, 'request', side_effect=[{'ready':False,'target':'robot'},{'ready':False}]) as request:
            with app_control.robot_ble_lease('phone'): pass
            self.assertEqual(request.call_count,2)

    def test_landing_failures_do_not_block_update(self):
        for outcome in [ConsoleResponse('landing',('ERROR: bus fault',),(), 'error'),
                        ConsoleResponse('landing',('STOPPED:',),(), 'stopped'), ConsoleError('timeout')]:
            with self.subTest(outcome=outcome), patch('servo.cli.Stm32Console') as cls:
                if isinstance(outcome,Exception): cls.return_value.send.side_effect=outcome
                else: cls.return_value.send.return_value=outcome
                _land_before_update(Mock(),'test')
                self.assertEqual(cls.return_value.send.call_args.args[0],'landing')

    def test_landing_success_checks_pose(self):
        with patch('servo.cli.Stm32Console') as cls:
            cls.return_value.send.side_effect=[ConsoleResponse('landing',('OK landing',),(),'ok'), ConsoleResponse('syncstate',('$SPOTSTATE pose=landing safety=ok',),(),'info')]
            _land_before_update(Mock(),'test')
            self.assertEqual([c.args[0] for c in cls.return_value.send.call_args_list],['landing','syncstate'])
