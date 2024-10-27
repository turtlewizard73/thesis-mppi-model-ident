# Ros comms
from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import ChangeState, GetState


class ControllerServerHandle:
    def __init__(self, logger, controller_server_name: str = 'controller_server'):
        self.logger = logger
        self.controller_server_name = controller_server_name

        self.get_state_cli_ = self.node_.create_client(
            GetState,
            f"/{self.controller_server_name}/get_state",
            callback_group=self.cb_group_)

        self.change_state_cli_ = self.node_.create_client(
            ChangeState,
            f"/{self.controller_server_name}/set_state",
            callback_group=self.cb_group_)

    def _set_transition(self, transition: Transition) -> None:
        pass

    def activate(self) -> None:
        self.logger.info('Activating controller server.')
        self._set_transition(Transition.TRANSITION_ACTIVATE)
        self.logger.info('Controller server activated.')

    def deactivate(self) -> None:
        self.logger.info('Deactivating controller server.')
        self._set_transition(Transition.TRANSITION_DEACTIVATE)
        self.logger.info('Controller server deactivated.')

