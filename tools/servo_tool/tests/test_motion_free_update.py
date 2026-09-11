from unittest.mock import Mock,patch
import pytest
from servo.cli import _confirm_update_torque_off

@pytest.mark.parametrize('text,accepted',[
 ('$SPOTSTATE pose=custom torque=off safety=fault',True),
 ('$SPOTSTATE pose=stand torque=on safety=ok',False),
 ('$SPOTSTATE torque=unknown',False),
 ('unrelated torque=off',False),
])
def test_no_pose_motion_when_torque_off_is_verified(text,accepted):
    console=Mock();console.send.return_value=Mock(ok=True,lines=[text])
    with patch('servo.cli.Stm32Console',return_value=console):
        if accepted:_confirm_update_torque_off(Mock(),'robot')
        else:
            with pytest.raises(RuntimeError):_confirm_update_torque_off(Mock(),'robot')
    console.send.assert_called_once_with('syncstate',timeout=15.)
