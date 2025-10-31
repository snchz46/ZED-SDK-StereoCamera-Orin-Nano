"""Plugins de condiciones para misiones reactivas."""

class BaseCondition:
    """Interfaz de condiciones."""

    def evaluate(self, context: dict) -> bool:
        raise NotImplementedError
