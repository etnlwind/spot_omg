import unittest
from unittest.mock import patch, Mock
from servo import app_control
from servo.cli import _land_before_update
from servo.console import ConsoleResponse, ConsoleError

class AppControlTests(unittest.TestCase):
    def test_windows_direct_ble_does_not_require_iphone_control(self):
        from servo.cli import main
        with patch.object(app_control.sys, 'platform', 'win32'), patch('servo.cli._main', return_value=0) as run:
            self.assertEqual(main(['--no-app-control','--via','ble','console','send','syncstate']),0)
            run.assert_called_once()
            with self.assertRaisesRegex(RuntimeError, 'requires macOS'):
                app_control.request('phone','status')

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

    def test_landing_failures_block_update(self):
        for outcome in [ConsoleResponse('landing',('ERROR: bus fault',),(), 'error'),
                        ConsoleResponse('landing',('STOPPED:',),(), 'stopped'), ConsoleError('timeout')]:
            with self.subTest(outcome=outcome), patch('servo.cli.Stm32Console') as cls:
                if isinstance(outcome,Exception): cls.return_value.send.side_effect=outcome
                else: cls.return_value.send.return_value=outcome
                with self.assertRaisesRegex(RuntimeError, 'Refusing firmware update'):
                    _land_before_update(Mock(),'test')
                self.assertEqual(cls.return_value.send.call_args.args[0],'landing')

    def test_landing_success_checks_pose(self):
        with patch('servo.cli.Stm32Console') as cls:
            cls.return_value.send.side_effect=[ConsoleResponse('landing',('OK landing',),(),'ok'), ConsoleResponse('syncstate',('$SPOTSTATE pose=landing torque=on safety=ok',),(),'info')]
            _land_before_update(Mock(),'test')
            self.assertEqual([c.args[0] for c in cls.return_value.send.call_args_list],['landing','syncstate'])

    def test_landing_acknowledgement_alone_does_not_allow_update(self):
        for state in ('$SPOTSTATE pose=stand torque=on safety=ok',
                      '$SPOTSTATE pose=landing torque=off safety=ok',
                      '$SPOTSTATE pose=landing torque=on safety=fault',
                      '$SPOTSTATE pose=landing', ''):
            with self.subTest(state=state), patch('servo.cli.Stm32Console') as cls:
                cls.return_value.send.side_effect = [
                    ConsoleResponse('landing', ('OK',), (), 'ok'),
                    ConsoleResponse('syncstate', (state,), (), 'info')]
                with self.assertRaisesRegex(RuntimeError, 'Refusing firmware update'):
                    _land_before_update(Mock(), 'test')

    def test_landing_sync_failure_blocks_update_without_motion(self):
        with patch('servo.cli.Stm32Console') as cls:
            cls.return_value.sync.side_effect = ConsoleError('timeout')
            with self.assertRaisesRegex(RuntimeError, 'Refusing firmware update'):
                _land_before_update(Mock(), 'test')
            cls.return_value.send.assert_not_called()
