"""Condition plugins for reactive missions."""

class BaseCondition:
    """Condition interface."""

    def evaluate(self, context: dict) -> bool:
        raise NotImplementedError
